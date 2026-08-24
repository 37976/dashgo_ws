#!/usr/bin/env python3
"""
slam_mapping.launch.py — SLAM mapping subsystem for DashGo.

Launches slam_toolbox (async online SLAM) + slam_controller bridge.
Use this via dashgo_nav_real.launch.py with use_slam:=true.
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory("nav_slam")

    # --- Arguments ---
    slam_params_file = LaunchConfiguration("slam_params_file")
    declare_slam_params = DeclareLaunchArgument(
        "slam_params_file",
        default_value=os.path.join(pkg_share, "config", "slam_toolbox_params.yaml"),
        description="Path to slam_toolbox parameter file",
    )

    scan_topic = LaunchConfiguration("scan_topic")
    declare_scan = DeclareLaunchArgument(
        "scan_topic",
        default_value="/scan",
        description="LiDAR scan topic for SLAM",
    )

    slam_map_topic = LaunchConfiguration("slam_map_topic")
    declare_slam_map = DeclareLaunchArgument(
        "slam_map_topic",
        default_value="/slam_map",
        description="Topic for SLAM map output (remapped from /map)",
    )

    combined_grid_topic = LaunchConfiguration("combined_grid_topic")
    declare_combined = DeclareLaunchArgument(
        "combined_grid_topic",
        default_value="/combined_grid",
        description="Topic to relay SLAM map to",
    )

    # --- slam_toolbox node ---
    slam_toolbox_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[slam_params_file],
        remappings=[
            # Avoid conflict with static_map_server's /map
            ("/map", slam_map_topic),
        ],
    )

    # --- slam_controller bridge node ---
    slam_controller_node = Node(
        package="nav_slam",
        executable="slam_controller",
        name="slam_controller",
        output="screen",
        parameters=[
            {
                "slam_map_topic": slam_map_topic,
                "combined_grid_topic": combined_grid_topic,
            }
        ],
    )

    return LaunchDescription([
        declare_slam_params,
        declare_scan,
        declare_slam_map,
        declare_combined,
        slam_toolbox_node,
        slam_controller_node,
    ])
