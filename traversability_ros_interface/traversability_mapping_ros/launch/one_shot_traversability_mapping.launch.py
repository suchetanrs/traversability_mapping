#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import EnvironmentVariable


def generate_launch_description():

    traversability_mapping_ros_pkg = get_package_share_directory('traversability_mapping_ros')
    traversability_pkg = get_package_share_directory('traversability_mapping')

    name_argument = DeclareLaunchArgument(
        "robot_ns",
        default_value=EnvironmentVariable("ROBOT_NAMESPACE", default_value=""),
        description="Robot namespace",
    )
    namespace = LaunchConfiguration("robot_ns")

    def all_nodes_launch(context):
        traversability_params_file = LaunchConfiguration('traversability_params_file')
        declare_traversability_params_file_cmd = DeclareLaunchArgument(
            'traversability_params_file',
            default_value=os.path.join(traversability_pkg, 'params', 'traversabilityParamsOneShot.yaml'),
            description='Full path to the (one-shot) traversability core parameters file')

        declare_ply_file_cmd = DeclareLaunchArgument(
            'ply_file',
            # default_value='/root/other_ws/benchmarking/cinema_scene_ramp_lidar.ply',
            # default_value='/root/other_ws/benchmarking/full_global_cloud_recorded_data.ply',
            default_value='/root/other_ws/benchmarking/pcl_mesh_gt.ply',
            # default_value='/root/other_ws/benchmarking/pcl_all_surfaces_full_gt.ply',
            description='Full path to the input PLY cloud (already expressed in the map frame)')
        ply_file = LaunchConfiguration('ply_file')

        declare_map_frame_cmd = DeclareLaunchArgument(
            'map_frame', default_value='map',
            description='Frame the PLY cloud lives in; stamped on the published maps')
        map_frame = LaunchConfiguration('map_frame')

        # One-shot node: reads the PLY, ingests it as a single keyframe, and publishes
        # the latched grid_map + occupancy once the (only) bin completes. It needs NO
        # TF, odom, or keyframe simulator -- the cloud is already a full map.
        one_shot_traversability = Node(
            package='traversability_mapping_ros',
            executable='one_shot_traversability',
            namespace=namespace,
            output='screen',
            parameters=[{
                'ply_file_path': ply_file,
                'map_frame': map_frame,
                'parameter_file_path': traversability_params_file,
            }])

        return [declare_traversability_params_file_cmd, declare_ply_file_cmd,
                declare_map_frame_cmd, one_shot_traversability]

    return LaunchDescription([
        name_argument,
        OpaqueFunction(function=all_nodes_launch)
    ])
