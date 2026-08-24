#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_launch = os.path.join(
        get_package_share_directory("dashgo_driver_ros2"),
        "launch",
        "dashgo_robot.launch.py",
    )
    nav_launch = os.path.join(
        get_package_share_directory("dashgo_driver_ros2"),
        "launch",
        "dashgo_nav_real.launch.py",
    )

    start_robot = LaunchConfiguration("start_robot")
    start_nav = LaunchConfiguration("start_nav")
    start_nav_rviz = LaunchConfiguration("start_nav_rviz")
    start_lidar = LaunchConfiguration("start_lidar")
    start_d435 = LaunchConfiguration("start_d435")
    base_frame = LaunchConfiguration("base_frame")
    laser_frame = LaunchConfiguration("laser_frame")
    laser_z = LaunchConfiguration("laser_z")
    d435_serial_no = LaunchConfiguration("d435_serial_no")
    top_k = LaunchConfiguration("top_k")
    detection_threshold = LaunchConfiguration("detection_threshold")
    min_score = LaunchConfiguration("min_score")
    depth_min_m = LaunchConfiguration("depth_min_m")
    depth_max_m = LaunchConfiguration("depth_max_m")
    xfeat_weights_path = LaunchConfiguration("xfeat_weights_path")
    match_min_cossim = LaunchConfiguration("match_min_cossim")
    min_pnp_points = LaunchConfiguration("min_pnp_points")
    min_inliers = LaunchConfiguration("min_inliers")
    pnp_reproj_error = LaunchConfiguration("pnp_reproj_error")
    pnp_iterations = LaunchConfiguration("pnp_iterations")
    odom_topic = LaunchConfiguration("odom_topic")
    delta_odom_topic = LaunchConfiguration("delta_odom_topic")
    odom_frame = LaunchConfiguration("odom_frame")
    camera_frame = LaunchConfiguration("camera_frame")
    fused_odom_topic = LaunchConfiguration("fused_odom_topic")
    correction_gain_xy = LaunchConfiguration("correction_gain_xy")
    correction_gain_yaw = LaunchConfiguration("correction_gain_yaw")
    max_delta_translation_diff_m = LaunchConfiguration("max_delta_translation_diff_m")
    max_delta_yaw_diff_deg = LaunchConfiguration("max_delta_yaw_diff_deg")
    use_slam = LaunchConfiguration("use_slam")

    return LaunchDescription([
        DeclareLaunchArgument("start_robot", default_value="true"),
        DeclareLaunchArgument("start_nav", default_value="true"),
        DeclareLaunchArgument("start_nav_rviz", default_value="true"),
        DeclareLaunchArgument("start_lidar", default_value="true"),
        DeclareLaunchArgument("start_d435", default_value="true"),
        DeclareLaunchArgument("base_frame", default_value="base_footprint"),
        DeclareLaunchArgument("laser_frame", default_value="laser"),
        DeclareLaunchArgument("laser_z", default_value="0.52"),
        DeclareLaunchArgument("d435_serial_no", default_value=""),
        DeclareLaunchArgument("top_k", default_value="768"),
        DeclareLaunchArgument("detection_threshold", default_value="0.05"),
        DeclareLaunchArgument("min_score", default_value="0.0"),
        DeclareLaunchArgument("depth_min_m", default_value="0.2"),
        DeclareLaunchArgument("depth_max_m", default_value="3.0"),
        DeclareLaunchArgument("xfeat_weights_path", default_value=""),
        DeclareLaunchArgument("match_min_cossim", default_value="0.65"),
        DeclareLaunchArgument("min_pnp_points", default_value="6"),
        DeclareLaunchArgument("min_inliers", default_value="4"),
        DeclareLaunchArgument("pnp_reproj_error", default_value="8.0"),
        DeclareLaunchArgument("pnp_iterations", default_value="200"),
        DeclareLaunchArgument("odom_topic", default_value="/xfeat/odom"),
        DeclareLaunchArgument("delta_odom_topic", default_value="/xfeat/delta_odom"),
        DeclareLaunchArgument("odom_frame", default_value="xfeat_odom"),
        DeclareLaunchArgument("camera_frame", default_value="camera_link"),
        DeclareLaunchArgument("fused_odom_topic", default_value="/localized_odom"),
        DeclareLaunchArgument("correction_gain_xy", default_value="0.15"),
        DeclareLaunchArgument("correction_gain_yaw", default_value="0.10"),
        DeclareLaunchArgument("max_delta_translation_diff_m", default_value="0.20"),
        DeclareLaunchArgument("max_delta_yaw_diff_deg", default_value="20.0"),
        DeclareLaunchArgument("use_slam", default_value="false"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_launch),
            launch_arguments={
                "start_lidar": start_lidar,
                "start_d435": start_d435,
                "start_t265": "false",
                "driver_baud": "115200",
                "lidar_baud": "1000000",
                "base_frame": base_frame,
                "laser_frame": laser_frame,
                "laser_z": laser_z,
                "publish_laser_tf": "false",
                "publish_sonar_tf": "false",
                "d435_serial_no": d435_serial_no,
            }.items(),
        ),
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package="dashgo_xfeat_bringup",
                    executable="xfeat_rgbd_odometry",
                    name="xfeat_rgbd_odometry",
                    output="screen",
                    parameters=[{
                        "rgb_topic": "/camera/camera/color/image_raw",
                        "depth_topic": "/camera/camera/aligned_depth_to_color/image_raw",
                        "camera_info_topic": "/camera/camera/color/camera_info",
                        "xfeat_weights_path": xfeat_weights_path,
                        "top_k": top_k,
                        "detection_threshold": detection_threshold,
                        "min_score": min_score,
                        "min_depth_m": depth_min_m,
                        "max_depth_m": depth_max_m,
                        "sync_queue_size": 10,
                        "odom_topic": odom_topic,
                        "delta_odom_topic": delta_odom_topic,
                        "odom_frame": odom_frame,
                        "base_frame": "camera_link",
                        "camera_frame": camera_frame,
                        "publish_tf": False,
                        "match_min_cossim": match_min_cossim,
                        "min_pnp_points": min_pnp_points,
                        "min_inliers": min_inliers,
                        "pnp_reproj_error": pnp_reproj_error,
                        "pnp_iterations": pnp_iterations,
                    }],
                ),
                Node(
                    package="dashgo_xfeat_bringup",
                    executable="odom_fusion_node",
                    name="odom_fusion_node",
                    output="screen",
                    parameters=[{
                        "base_odom_topic": "/odom",
                        "xfeat_delta_topic": delta_odom_topic,
                        "output_odom_topic": fused_odom_topic,
                        "correction_gain_xy": correction_gain_xy,
                        "correction_gain_yaw": correction_gain_yaw,
                        "max_delta_translation_diff_m": max_delta_translation_diff_m,
                        "max_delta_yaw_diff_deg": max_delta_yaw_diff_deg,
                    }],
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(nav_launch),
                    launch_arguments={
                        "start_robot": "false",
                        "start_nav": start_nav,
                        "start_nav_rviz": start_nav_rviz,
                        "start_lidar": "false",
                        "start_d435": "false",
                        "publish_robot_model": "true",
                        "map_odom_topic": fused_odom_topic,
                        "control_odom_topic": "/odom_in_map",
                        "use_slam": use_slam,
                    }.items(),
                ),
            ],
        ),
    ])
