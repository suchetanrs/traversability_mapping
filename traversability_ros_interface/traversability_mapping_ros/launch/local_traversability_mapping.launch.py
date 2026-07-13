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
        params_file = LaunchConfiguration('params_file')
        declare_params_file_cmd = DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(traversability_mapping_ros_pkg, 'params', 'traversability_gt_ros_params.yaml'),
            description='Full path to the ROS2 parameters file to use for all launched nodes')

        traversability_params_file = LaunchConfiguration('traversability_params_file')
        declare_traversability_params_file_cmd = DeclareLaunchArgument(
            'traversability_params_file',
            default_value=os.path.join(traversability_pkg, 'params', 'traversabilityParamsLocal.yaml'),
            description='Full path to the (local) traversability core parameters file')

        # ONLY the local node. It consumes the raw cloud (pointcloud_topic_name) + pose
        # (odom_topic_name) straight from the sensor/sim, and the static base<-lidar
        # extrinsic from robot_state_publisher/URDF -- it does NOT need the keyframe
        # simulator. Do NOT launch slam_keyframe_sim_with_drift here: it broadcasts
        # odom->base_footprint and map->odom TF (and keyframe additions), so running a
        # second copy alongside the global launch double-broadcasts those transforms and
        # corrupts BOTH maps. The upstream bringup (or the global launch) provides the
        # single source of those topics + TF.
        local_traversability = Node(
            package='traversability_mapping_ros',
            executable='local_traversability',
            namespace=namespace,
            output='screen',
            parameters=[params_file, {"parameter_file_path": traversability_params_file}])

        return [declare_params_file_cmd, declare_traversability_params_file_cmd,
                local_traversability]

    return LaunchDescription([
        name_argument,
        OpaqueFunction(function=all_nodes_launch)
    ])
