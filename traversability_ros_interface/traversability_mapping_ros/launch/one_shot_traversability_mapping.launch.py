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
from launch.substitutions import EnvironmentVariable

def generate_launch_description():

#---------------------------------------------

    #Essential_paths
    traversability_mapping_ros_pkg = get_package_share_directory('traversability_mapping_ros')
    traversability_pkg = get_package_share_directory('traversability_mapping')

    name_argument = DeclareLaunchArgument(
        "robot_ns",
        default_value=EnvironmentVariable("ROBOT_NAMESPACE", default_value=""),
        description="Robot namespace",
    )

    namespace = LaunchConfiguration("robot_ns")
    use_gt_pose = LaunchConfiguration('use_gt_pose')
    
#---------------------------------------------

    def all_nodes_launch(context):
        ros_params_file = LaunchConfiguration('ros_params_file')

        declare_ros_params_file_cmd = DeclareLaunchArgument(
            'ros_params_file',
            default_value=os.path.join(traversability_mapping_ros_pkg, 'params', 'traversability_one_shot_params.yaml'),
            description='Full path to the ROS2 parameters file to use for all launched nodes')
        
        traversability_params_file = LaunchConfiguration('traversability_params_file')
        
        declare_traversability_params_file_cmd = DeclareLaunchArgument(
            'traversability_params_file',
            default_value=os.path.join(traversability_pkg, 'params', 'traversabilityParamsOneShot.yaml'),
            description='Full path to the traversability parameters file to use for all launched nodes')

        declare_use_gt_pose_cmd = DeclareLaunchArgument(
            'use_gt_pose', default_value='True',
            description='Use ground truth poses if True')

        traversability_mapping_ros = Node(
            package='traversability_mapping_ros',
            executable='one_shot_traversability',
            # prefix=["gdbserver localhost:3000"],
            # prefix='heaptrack --output /tmp/heaptrack_global_trav.gz',
            namespace=namespace,
            output='screen',
            parameters=[ros_params_file, {"parameter_file_path": traversability_params_file}])
        
        threshold_traversability_ros = Node(
            package='traversability_mapping_ros',
            executable='threshold_occupancy',
            # prefix='heaptrack --output /tmp/heaptrack_threshold.gz',
            namespace=namespace,
            output='screen',
            parameters=[ros_params_file])
        
        return [declare_ros_params_file_cmd, declare_traversability_params_file_cmd, declare_use_gt_pose_cmd, traversability_mapping_ros, threshold_traversability_ros]

    opaque_function = OpaqueFunction(function=all_nodes_launch)
#---------------------------------------------

    return LaunchDescription([
        name_argument,
        opaque_function
    ])