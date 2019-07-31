#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument
import os.path
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="True")
    turtlebot3_model = "burger"
    world_file_name = get_package_share_directory("turtlebot3_gazebo") + "/worlds/turtlebot3_worlds/" + turtlebot3_model + ".model"
    gzserver = ExecuteProcess(
        cmd=["gzserver", "--verbose", world_file_name,
             "-s", "libgazebo_ros_init.so"],
        output="screen")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="True",
            description="Use simulation (Gazebo) clock if true"
        ),
        gzserver,
        # spawn_model,
    ])
