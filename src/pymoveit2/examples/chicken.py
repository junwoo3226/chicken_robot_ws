#!/usr/bin/env python3
"""
Chicken frying motion with UR5 using MoveIt2.
Run using:
python3 chickenFried.py --ros-args --remap __ns:=/arm1 -p cartesian:=True
"""

from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2 import MoveIt2
from pymoveit2.robots import ur5 as robot
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from moveit_msgs.msg import PlanningScene


def main():
    rclpy.init()

    # Create node
    node = Node("chicken_frying_node")

    # Declare parameters for cartesian motion and namespace
    node.declare_parameter("cartesian", False)

    # Create callback group for parallel execution
    callback_group = ReentrantCallbackGroup()

    # Create MoveIt2 interface
    moveit2 = MoveIt2(
        node=node,
        joint_names=robot.joint_names(),
        base_link_name=robot.base_link_name(),
        end_effector_name=robot.end_effector_name(),
        group_name=robot.MOVE_GROUP_ARM,
        callback_group=callback_group
    )

    # Spin the node in a background thread
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    # Get parameters
    cartesian = node.get_parameter("cartesian").get_parameter_value().bool_value

    # Add ground plane
    add_ground_plane(node)

    # Execute chicken frying motion
    chicken_frying_motion(node, moveit2, cartesian)

    # Shutdown
    rclpy.shutdown()
    executor_thread.join()
    exit(0)


def add_ground_plane(node):
    """
    Add a ground plane as a collision object in the planning scene.
    """
    collision_object = CollisionObject()
    collision_object.id = "ground_plane"
    collision_object.header.frame_id = "world"

    # Define the ground plane as a box
    ground_plane = SolidPrimitive()
    ground_plane.type = SolidPrimitive.BOX
    ground_plane.dimensions = [10.0, 10.0, 0.01]  # Length, Width, Height

    # Set the pose of the ground plane
    ground_plane_pose = Pose()
    ground_plane_pose.position.z = -0.005  # Slightly below the robot base

    collision_object.primitives.append(ground_plane)
    collision_object.primitive_poses.append(ground_plane_pose)

    # Publish the collision object to the planning scene
    scene = PlanningScene()
    scene.world.collision_objects.append(collision_object)
    scene.is_diff = True

    publisher = node.create_publisher(PlanningScene, 'planning_scene', 10)
    publisher.publish(scene)


def chicken_frying_motion(node, moveit2, cartesian):
    """
    Execute a simplified chicken frying motion in 3 steps.
    """
    try:
        # Step 1: Move to initial position
        node.get_logger().info("Step 1: Moving to initial position")
        moveit2.move_to_pose(
            position=[0.3, 0.2, 0.4],
            quat_xyzw=[0.0, 0.0, 0.0, 1.0],
            cartesian=cartesian,
        )
        moveit2.wait_until_executed()

        # Step 2: Perform up and down motion (frying)
        node.get_logger().info("Step 2: Performing frying motion (up/down)")
        for _ in range(3):  # Repeat up/down motion 3 times
            moveit2.move_to_pose(
                position=[0.3, 0.2, 0.2],
                quat_xyzw=[0.0, 0.0, 0.0, 1.0],
                cartesian=cartesian,
            )
            moveit2.wait_until_executed()
            moveit2.move_to_pose(
                position=[0.3, 0.2, 0.4],
                quat_xyzw=[0.0, 0.0, 0.0, 1.0],
                cartesian=cartesian,
            )
            moveit2.wait_until_executed()

        # Step 3: Rotate left and return to initial position
        node.get_logger().info("Step 3: Rotating left and returning to initial position")
        moveit2.move_to_pose(
            position=[0.3, -0.2, 0.4],  # Slightly to the left
            quat_xyzw=[0.0, 0.0, 0.0, 1.0],
            cartesian=cartesian,
        )
        moveit2.wait_until_executed()

        moveit2.move_to_pose(
            position=[0.3, 0.2, 0.4],  # Back to initial position
            quat_xyzw=[0.0, 0.0, 0.0, 1.0],
            cartesian=cartesian,
        )
        moveit2.wait_until_executed()

        node.get_logger().info("Simplified chicken frying motion completed")

    except Exception as err:
        node.get_logger().error(f"Error during chicken frying motion: {err}")


if __name__ == "__main__":
    main()