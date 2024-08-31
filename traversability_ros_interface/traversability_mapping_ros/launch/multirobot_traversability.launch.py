#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, FindExecutable, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    # Setup project paths
    pkg_project_gazebo = get_package_share_directory("traversability_mapping_ros")

    traversabilities = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_project_gazebo, "launch", "global_traversability_mapping.launch.py")
            ),
            launch_arguments={"robot_ns": f"robot_{i}/"}.items(),
        )
        for i in range(0, 1)
    ]

    return LaunchDescription(
        traversabilities
    )