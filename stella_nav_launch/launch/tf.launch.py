#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
import os.path
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="True")
    turtlebot3_model = "burger"
    urdf_path = os.path.join(
        get_package_share_directory("turtlebot3_description"),
        "urdf",
        "turtlebot3" + turtlebot3_model + ".urdf",
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="True",
            description="Use simulation (Gazebo) clock if true"
        ),
        Node(
            package="tf2_ros",
            node_executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
            parameters=[{"use_sim_time": use_sim_time}],
            output="screen"),
        Node(
            package="robot_state_publisher",
            node_executable="robot_state_publisher",
            node_name="robot_state_publisher",
            parameters=[{"use_sim_time": use_sim_time}],
            arguments=[urdf_path],
            output="screen"),
    ])


