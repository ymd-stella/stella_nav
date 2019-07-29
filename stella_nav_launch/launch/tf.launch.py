#!/usr/bin/env python3
from launch import LaunchDescription
import launch_ros.actions
import os.path
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    urdf_path = os.path.join(
        get_package_share_directory("turtlebot3_description"),
        "urdf",
        "turtlebot3_burger.urdf",
    )
    return LaunchDescription([
        launch_ros.actions.Node(
            package="tf2_ros",
            node_executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"]),
        launch_ros.actions.Node(
            package="robot_state_publisher",
            node_executable="robot_state_publisher",
            arguments=[urdf_path]),
    ])


