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
        params_file = LaunchConfiguration('params_file')

        declare_params_file_cmd = DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(traversability_mapping_ros_pkg, 'params', 'traversability_ros_params.yaml'),
            description='Full path to the ROS2 parameters file to use for all launched nodes')

        traversability_mapping_ros = Node(
            package='traversability_mapping_ros',
            executable='local_traversability',
            namespace=namespace,
            output='screen',
            parameters=[params_file])
        
        return [declare_params_file_cmd, traversability_mapping_ros]

    opaque_function = OpaqueFunction(function=all_nodes_launch)
#---------------------------------------------

    return LaunchDescription([
        name_argument,
        opaque_function
    ])