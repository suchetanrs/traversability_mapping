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

def generate_launch_description():

#---------------------------------------------

    #Essential_paths
    traversability_mapping_ros_pkg = get_package_share_directory('traversability_mapping_ros')

    name_argument = DeclareLaunchArgument(
        "robot_ns",
        default_value="robot_0",
        description="Robot namespace",
    )

    namespace = LaunchConfiguration("robot_ns")
    
#---------------------------------------------

    def all_nodes_launch(context):
        rviz_file = LaunchConfiguration('rviz_file')

        declare_rviz_file_cmd = DeclareLaunchArgument(
            'rviz_file',
            default_value=os.path.join(traversability_mapping_ros_pkg, 'rviz', 'traversability.rviz'),
            description='Full path to the ROS2 parameters file to use for all launched nodes')

        rviz_node = Node(
            package='rviz2', 
            executable='rviz2', 
            name="rviz2", 
            output='screen',
            arguments=[["-d"], [LaunchConfiguration("rviz_file")]])
        
        return [declare_rviz_file_cmd, rviz_node]

    opaque_function = OpaqueFunction(function=all_nodes_launch)
#---------------------------------------------

    return LaunchDescription([
        name_argument,
        opaque_function
    ])