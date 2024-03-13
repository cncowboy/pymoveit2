#!/usr/bin/env python3
"""
Example of moving to a pose goal.
- ros2 run pymoveit2 ex_pose_goal.py --ros-args -p position:="[0.25, 0.0, 1.0]" -p quat_xyzw:="[0.0, 0.0, 0.0, 1.0]" -p cartesian:=False
- ros2 run pymoveit2 ex_pose_goal.py --ros-args -p position:="[0.25, 0.0, 1.0]" -p quat_xyzw:="[0.0, 0.0, 0.0, 1.0]" -p cartesian:=False -p synchronous:=False -p cancel_after_secs:=1.0
- ros2 run pymoveit2 ex_pose_goal.py --ros-args -p position:="[0.25, 0.0, 1.0]" -p quat_xyzw:="[0.0, 0.0, 0.0, 1.0]" -p cartesian:=False -p synchronous:=False -p cancel_after_secs:=0.0
"""

from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2 import MoveIt2, MoveIt2State
#from pymoveit2.robots import panda as robot
from pymoveit2.robots import ur5 as robot
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetPlanningScene
from moveit_msgs.msg import PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import Plane


def main():
    rclpy.init()

    # Create node for this example
    node = Node("ex_pose_goal")

    # Declare parameters for position and orientation
    node.declare_parameter("position", [0.5, 0.0, 0.25])
    node.declare_parameter("quat_xyzw", [1.0, 0.0, 0.0, 0.0])
    node.declare_parameter("cartesian", False)
    node.declare_parameter("synchronous", True)
    # If non-positive, don't cancel. Only used if synchronous is False
    node.declare_parameter("cancel_after_secs", 0.0)
    # Planner ID
    node.declare_parameter("planner_id", "RRTConnectkConfigDefault")

    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()

    # Create MoveIt 2 interface
    moveit2 = MoveIt2(
        node=node,
        joint_names=robot.joint_names(),
        base_link_name=robot.base_link_name(),
        end_effector_name=robot.end_effector_name(),
        group_name=robot.MOVE_GROUP_ARM,
        callback_group=callback_group,
    )
    moveit2.planner_id = (
        node.get_parameter("planner_id").get_parameter_value().string_value
    )

    # Spin the node in background thread(s) and wait a bit for initialization
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    node.create_rate(1.0).sleep()

    # Scale down velocity and acceleration of joints (percentage of maximum)
    moveit2.max_velocity = 0.5
    moveit2.max_acceleration = 0.5

    # Get parameters
    position = node.get_parameter("position").get_parameter_value().double_array_value
    quat_xyzw = node.get_parameter("quat_xyzw").get_parameter_value().double_array_value
    cartesian = node.get_parameter("cartesian").get_parameter_value().bool_value
    synchronous = node.get_parameter("synchronous").get_parameter_value().bool_value
    cancel_after_secs = (
        node.get_parameter("cancel_after_secs").get_parameter_value().double_value
    )

    # Move to pose
    node.get_logger().info(
        f"Moving to {{position: {list(position)}, quat_xyzw: {list(quat_xyzw)}}}"
    )
    try:
        add_ground_plane(node)  
        moveit2.move_to_pose(position=position, quat_xyzw=quat_xyzw, cartesian=cartesian)
        if synchronous:
            # Note: the same functionality can be achieved by setting
            # `synchronous:=false` and `cancel_after_secs` to a negative value.
            moveit2.wait_until_executed()
        else:
            # Wait for the request to get accepted (i.e., for execution to start)
            print("Current State: " + str(moveit2.query_state()))
            rate = node.create_rate(10)
            while moveit2.query_state() != MoveIt2State.EXECUTING:
                rate.sleep()

            # Get the future
            print("Current State: " + str(moveit2.query_state()))
            future = moveit2.get_execution_future()

            # Cancel the goal
            if cancel_after_secs > 0.0:
                # Sleep for the specified time
                sleep_time = node.create_rate(cancel_after_secs)
                sleep_time.sleep()
                # Cancel the goal
                print("Cancelling goal")
                moveit2.cancel_execution()

            # Wait until the future is done
            while not future.done():
                rate.sleep()

            # Print the result
            print("Result status: " + str(future.result().status))
            print("Result error code: " + str(future.result().result.error_code))

    except Exception as err:
        node.get_logger().info(f'Exception occured. {err}')

    node.get_logger().info(f'Movement completed')   
    rclpy.shutdown()
    executor_thread.join()
    exit(0)

'''
def reset_collision_objects(node):
    # Create a PlanningScene message
    planning_scene = PlanningScene()
    # Set the operation to REMOVE or CLEAR to remove or clear all collision objects respectively
    planning_scene.world.collision_objects.clear()
    planning_scene.is_diff = True
    # Publish the PlanningScene to update the collision objects in the planning scene
    publisher = node.create_publisher(PlanningScene, "planning_scene", 10)
    publisher.publish(planning_scene)
    # Wait for the update to be applied
    apply_scene_service = node.create_client(ApplyPlanningScene, "apply_planning_scene")
    while not apply_scene_service.wait_for_service(timeout_sec=1.0):
        node.get_logger().info("Waiting for the /apply_planning_scene service...")
    request = ApplyPlanningScene.Request()
    #apply_scene_service.call(request)
'''

def get_planning_scene(node):

    # Create a client for the GetPlanningScene service
    get_scene_service = node.create_client(GetPlanningScene, "get_planning_scene")

    # Wait for the service to be available
    while not get_scene_service.wait_for_service(timeout_sec=1.0):
        node.get_logger().info("Waiting for the get_planning_scene service...")

    # Create a request object
    request = GetPlanningScene.Request()

    # Set the desired scene components to be returned
    request.components.components = request.components.WORLD_OBJECT_NAMES
    # Call the GetPlanningScene service
    future = get_scene_service.call(request)

    if future is not None:
        if future.scene is not None:
            scene = future.scene
            for obj in scene.world.collision_objects:
                print("Collision Object ID:", obj.id)
                print("Collision Object Type:", obj.type)
                print("Collision Object Plane:", obj.planes)
                print("Collision Object Plane Pose:", obj.plane_poses)
                print("-----------------------------")
        return future.scene

    return None

def add_ground_plane(node):

    # Create a CollisionObject message
    collision_object = CollisionObject()
    collision_object.id = "ground_plane"
    collision_object.header.frame_id = "world"

    # Define the ground plane as a box shape
    ground_plane = Plane()
    ground_plane.coef = [0.0, 0.0, 1.0, 0.0]

    # Set the ground plane's pose
    ground_plane_pose = Pose()
    ground_plane_pose.position.z = -0.005  # Adjust the height of the ground plane

    collision_object.planes.append(ground_plane)
    collision_object.plane_poses.append(ground_plane_pose)

    # Create a PlanningScene message
    scene = PlanningScene()
    scene.world.collision_objects.append(collision_object)
    scene.is_diff = True

    publisher_ = node.create_publisher(PlanningScene, 'planning_scene', 10)
    publisher_.publish(scene)

if __name__ == "__main__":
    main()
