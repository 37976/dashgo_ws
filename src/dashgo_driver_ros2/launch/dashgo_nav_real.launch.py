import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    driver_share = get_package_share_directory("dashgo_driver_ros2")
    nav_share_dir = get_package_share_directory("nav_slam")
    default_nav_map = os.path.join(nav_share_dir, "map", "gpt.yaml")
    default_nav_rviz = os.path.join(nav_share_dir, "config", "rviz.rviz")
    default_robot_urdf = os.path.join(driver_share, "urdf", "dashgo_visual.urdf")

    robot_launch = os.path.join(driver_share, "launch", "dashgo_robot.launch.py")
    with open(default_robot_urdf, "r", encoding="utf-8") as urdf_file:
        robot_description = urdf_file.read()

    start_robot = LaunchConfiguration("start_robot")
    start_nav = LaunchConfiguration("start_nav")
    start_nav_rviz = LaunchConfiguration("start_nav_rviz")
    start_lidar = LaunchConfiguration("start_lidar")
    start_d435 = LaunchConfiguration("start_d435")
    publish_robot_model = LaunchConfiguration("publish_robot_model")
    driver_port = LaunchConfiguration("driver_port")
    lidar_port = LaunchConfiguration("lidar_port")
    nav_map_yaml = LaunchConfiguration("nav_map_yaml")
    base_frame = LaunchConfiguration("base_frame")
    laser_frame = LaunchConfiguration("laser_frame")
    laser_z = LaunchConfiguration("laser_z")
    points_frame = LaunchConfiguration("points_frame")
    scan_min_range = LaunchConfiguration("scan_min_range")

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_robot", default_value="true"),
            DeclareLaunchArgument("start_nav", default_value="true"),
            DeclareLaunchArgument("start_nav_rviz", default_value="true"),
            DeclareLaunchArgument("start_lidar", default_value="true"),
            DeclareLaunchArgument("start_d435", default_value="false"),
            DeclareLaunchArgument("publish_robot_model", default_value="true"),
            DeclareLaunchArgument("driver_port", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument(
                "lidar_port",
                default_value=(
                    "/dev/serial/by-id/"
                    "usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_"
                    "d8a678826473ed11a4766aeefdf7b791-if00-port0"
                ),
            ),
            DeclareLaunchArgument("nav_map_yaml", default_value=default_nav_map),
            DeclareLaunchArgument("base_frame", default_value="base_footprint"),
            DeclareLaunchArgument("laser_frame", default_value="laser"),
            DeclareLaunchArgument("laser_z", default_value="0.52"),
            DeclareLaunchArgument("points_frame", default_value="base_link"),
            DeclareLaunchArgument("scan_min_range", default_value="0.30"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(robot_launch),
                condition=IfCondition(start_robot),
                launch_arguments={
                    "start_lidar": start_lidar,
                    "start_d435": start_d435,
                    "start_t265": "false",
                    "driver_port": driver_port,
                    "driver_baud": "115200",
                    "lidar_port": lidar_port,
                    "lidar_baud": "1000000",
                    "base_frame": base_frame,
                    "laser_frame": laser_frame,
                    "laser_z": laser_z,
                    "publish_laser_tf": "false",
                    "publish_sonar_tf": "false",
                }.items(),
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                condition=IfCondition(publish_robot_model),
                output="screen",
                parameters=[
                    {
                        "robot_description": robot_description,
                        "publish_frequency": 30.0,
                    }
                ],
            ),
            Node(
                package="dashgo_driver_ros2",
                executable="scan_to_points_node",
                name="scan_to_points",
                condition=IfCondition(start_nav),
                output="screen",
                parameters=[
                    {
                        "scan_topic": "/scan",
                        "filtered_scan_topic": "/scan_filtered",
                        "points_topic": "/points_raw",
                        "output_frame": points_frame,
                        "laser_height": laser_z,
                        "x_offset": 0.0,
                        "y_offset": 0.0,
                        "min_valid_range": scan_min_range,
                    }
                ],
            ),
            Node(
                package="nav2_voronoi_planner",
                executable="voronoi_node",
                name="voronoi",
                condition=IfCondition(start_nav),
                output="screen",
                parameters=[
                    {
                        "trunk_safety_penalty_scale": 0.06,
                        "direct_connect_distance": 0.60,
                    }
                ],
            ),
            Node(
                package="nav_slam",
                executable="map_pub",
                name="map_pub",
                condition=IfCondition(start_nav),
                output="screen",
                parameters=[
                    {
                        "use_static_map": False,
                        "static_map_yaml": nav_map_yaml,
                        "grid_width": 40.0,
                        "grid_height": 40.0,
                        "resolution": 0.05,
                        "dynamic_obstacle_timeout": 0.8,
                        "accumulate_pointcloud_obstacles": False,
                        "min_height": 0.0,
                        "max_height": 1.0,
                        "obstacle_radius": 0.08,
                        "projection_gap_fill_cells": 1,
                    }
                ],
            ),
            Node(
                package="nav_slam",
                executable="odom_map_tf",
                name="odom_map_tf",
                condition=IfCondition(start_nav),
                output="screen",
            ),
            Node(
                package="nav_slam",
                executable="points_pub_map",
                name="points_pub_map",
                condition=IfCondition(start_nav),
                output="screen",
                parameters=[{"frame_id": "map"}],
            ),
            Node(
                package="nav_slam",
                executable="start_nav",
                name="start_nav",
                condition=IfCondition(start_nav),
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="nav_rviz2",
                condition=IfCondition(start_nav_rviz),
                output="screen",
                arguments=["-d", default_nav_rviz],
            ),
        ]
    )
