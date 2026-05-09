import glob
import os

from ament_index_python.packages import get_package_share_directory
from dashgo_driver_ros2.device_resolver import resolve_serial_ports
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def resolve_qr_popup_script():
    matches = glob.glob(
        os.path.join(
            get_package_share_directory("dashgo_web_control"),
            "..",
            "..",
            "lib",
            "python*",
            "site-packages",
            "dashgo_web_control",
            "show_web_qr_popup.py",
        )
    )
    source_path = os.path.expanduser(
        "~/project/dashgo_ws/src/dashgo_web_control/dashgo_web_control/show_web_qr_popup.py"
    )
    
    if matches:
        return matches[0]

    if os.path.exists(source_path):
        return source_path

    raise FileNotFoundError("Unable to locate show_web_qr_popup.py")


def resolve_hotspot_script():
    matches = glob.glob(
        os.path.join(
            get_package_share_directory("dashgo_web_control"),
            "..",
            "..",
            "lib",
            "python*",
            "site-packages",
            "dashgo_web_control",
            "hotspot_manager.py",
        )
    )
    source_path = os.path.expanduser(
        "~/project/dashgo_ws/src/dashgo_web_control/dashgo_web_control/hotspot_manager.py"
    )

    if matches:
        return matches[0]

    if os.path.exists(source_path):
        return source_path

    raise FileNotFoundError("Unable to locate hotspot_manager.py")


def generate_launch_description():
    resolved_ports = resolve_serial_ports()
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
    start_web_ui = LaunchConfiguration("start_web_ui")
    start_hotspot = LaunchConfiguration("start_hotspot")
    publish_robot_model = LaunchConfiguration("publish_robot_model")
    driver_port = LaunchConfiguration("driver_port")
    lidar_port = LaunchConfiguration("lidar_port")
    nav_map_yaml = LaunchConfiguration("nav_map_yaml")
    base_frame = LaunchConfiguration("base_frame")
    laser_frame = LaunchConfiguration("laser_frame")
    laser_z = LaunchConfiguration("laser_z")
    points_frame = LaunchConfiguration("points_frame")
    scan_min_range = LaunchConfiguration("scan_min_range")
    web_host = LaunchConfiguration("web_host")
    web_port = LaunchConfiguration("web_port")
    web_image_topic = LaunchConfiguration("web_image_topic")
    web_map_publish_hz = LaunchConfiguration("web_map_publish_hz")
    web_camera_publish_hz = LaunchConfiguration("web_camera_publish_hz")
    web_camera_max_width = LaunchConfiguration("web_camera_max_width")
    hotspot_connection_name = LaunchConfiguration("hotspot_connection_name")
    hotspot_ssid = LaunchConfiguration("hotspot_ssid")
    hotspot_password = LaunchConfiguration("hotspot_password")
    hotspot_ifname = LaunchConfiguration("hotspot_ifname")
    map_odom_topic = LaunchConfiguration("map_odom_topic")
    control_odom_topic = LaunchConfiguration("control_odom_topic")

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_robot", default_value="true"),
            DeclareLaunchArgument("start_nav", default_value="true"),
            DeclareLaunchArgument("start_nav_rviz", default_value="true"),
            DeclareLaunchArgument("start_lidar", default_value="true"),
            DeclareLaunchArgument("start_d435", default_value="true"),
            DeclareLaunchArgument("start_web_ui", default_value="true"),
            DeclareLaunchArgument("start_hotspot", default_value="false"),
            DeclareLaunchArgument("publish_robot_model", default_value="true"),
            DeclareLaunchArgument("driver_port", default_value=resolved_ports["driver_port"]),
            DeclareLaunchArgument(
                "lidar_port",
                default_value=resolved_ports["lidar_port"],
            ),
            DeclareLaunchArgument("nav_map_yaml", default_value=default_nav_map),
            DeclareLaunchArgument("base_frame", default_value="base_footprint"),
            DeclareLaunchArgument("laser_frame", default_value="laser"),
            DeclareLaunchArgument("laser_z", default_value="0.52"),
            DeclareLaunchArgument("points_frame", default_value="base_link"),
            DeclareLaunchArgument("scan_min_range", default_value="0.30"),
            DeclareLaunchArgument("web_host", default_value="0.0.0.0"),
            DeclareLaunchArgument("web_port", default_value="8080"),
            DeclareLaunchArgument("web_image_topic", default_value="/camera/camera/color/image_raw"),
            DeclareLaunchArgument("web_map_publish_hz", default_value="2.0"),
            DeclareLaunchArgument("web_camera_publish_hz", default_value="12.0"),
            DeclareLaunchArgument("web_camera_max_width", default_value="320"),
            DeclareLaunchArgument("hotspot_connection_name", default_value="dashgo-hotspot"),
            DeclareLaunchArgument("hotspot_ssid", default_value="Dashgo-Robot"),
            DeclareLaunchArgument("hotspot_password", default_value="dashgo12345"),
            DeclareLaunchArgument("hotspot_ifname", default_value=""),
            DeclareLaunchArgument("map_odom_topic", default_value="/odom"),
            DeclareLaunchArgument("control_odom_topic", default_value="/odom"),
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
            LogInfo(msg=f"Dashgo auto-detected base port default: {resolved_ports['driver_port']}"),
            LogInfo(msg=f"Dashgo auto-detected lidar port default: {resolved_ports['lidar_port']}"),
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
                        "robot_radius": 0.20,
                        "occ_threshold": 15,
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
                        "clear_radius": 0.20,
                        "projection_gap_fill_cells": 1,
                        "odom_topic": map_odom_topic,
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
                parameters=[{"frame_id": "map"}, {"odom_topic": map_odom_topic}],
            ),
            Node(
                package="nav_slam",
                executable="start_nav",
                name="start_nav",
                condition=IfCondition(start_nav),
                output="screen",
                parameters=[{"odom_topic": control_odom_topic}],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="nav_rviz2",
                condition=IfCondition(start_nav_rviz),
                output="screen",
                arguments=["-d", default_nav_rviz],
            ),
            Node(
                package="dashgo_web_control",
                executable="web_control_node",
                name="dashgo_web_control",
                condition=IfCondition(start_web_ui),
                output="screen",
                parameters=[
                    {
                        "host": web_host,
                        "port": web_port,
                        "image_topic": web_image_topic,
                        "map_publish_hz": web_map_publish_hz,
                        "camera_publish_hz": web_camera_publish_hz,
                        "camera_max_width": web_camera_max_width,
                        "robot_radius": 0.20,
                    }
                ],
            ),
            TimerAction(
                period=0.3,
                condition=IfCondition(start_hotspot),
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "python3",
                            resolve_hotspot_script(),
                            "--connection-name",
                            hotspot_connection_name,
                            "--ssid",
                            hotspot_ssid,
                            "--password",
                            hotspot_password,
                            "--ifname",
                            hotspot_ifname,
                        ],
                        output="screen",
                    ),
                ],
            ),
            TimerAction(
                period=2.3,
                condition=IfCondition(start_web_ui),
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "python3",
                            resolve_qr_popup_script(),
                        ],
                        additional_env={
                            "DASHGO_WEB_UI_HOST": web_host,
                            "DASHGO_WEB_UI_PORT": web_port,
                            "DASHGO_HOTSPOT_ENABLED": start_hotspot,
                            "DASHGO_HOTSPOT_SSID": hotspot_ssid,
                            "DASHGO_HOTSPOT_PASSWORD": hotspot_password,
                        },
                        output="screen",
                    ),
                ],
            ),
        ]
    )
