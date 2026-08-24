"""
启动底盘 + D435 相机 + Web 远程控制 + 热点 + ros2 bag 录制。

用法:
  ros2 launch dashgo_rgbd_recorder record_bag.launch.py \
    output_dir:=room_static_01 \
    bag_name:=room_static_01

生成的 .db3 bag 位于: <output_dir>/raw/<bag_name>/
"""

import glob
import os
import shutil

from ament_index_python.packages import get_package_share_directory
from dashgo_driver_ros2.device_resolver import resolve_serial_ports
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resolve_hotspot_script():
    matches = glob.glob(
        os.path.join(
            get_package_share_directory("dashgo_web_control"),
            "..", "..", "lib", "python*", "site-packages",
            "dashgo_web_control", "hotspot_manager.py",
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


def _resolve_qr_popup_script():
    matches = glob.glob(
        os.path.join(
            get_package_share_directory("dashgo_web_control"),
            "..", "..", "lib", "python*", "site-packages",
            "dashgo_web_control", "show_web_qr_popup.py",
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


def _setup_and_record(context, *args, **kwargs):
    """创建输出目录、复制配置，并返回 ros2 bag record 进程。"""
    output_dir = os.path.expanduser(
        LaunchConfiguration("output_dir").perform(context)
    )
    bag_name = LaunchConfiguration("bag_name").perform(context)

    raw_dir = os.path.join(output_dir, "raw")
    os.makedirs(raw_dir, exist_ok=True)

    pkg_share = get_package_share_directory("dashgo_rgbd_recorder")
    src_config = os.path.join(pkg_share, "config", "recording_config.yaml")
    dst_config = os.path.join(output_dir, "recording_config.yaml")
    if os.path.exists(src_config) and not os.path.exists(dst_config):
        shutil.copy2(src_config, dst_config)

    bag_output = os.path.join(raw_dir, bag_name)

    # 如果上次录制的 bag 目录还在，先删除（ros2 bag record 不允许覆盖）
    if os.path.exists(bag_output):
        shutil.rmtree(bag_output)
        print(f"[INFO] Removed existing bag: {bag_output}")

    return [
        LogInfo(msg=f"=== D435 RGB-D Recording ==="),
        LogInfo(msg=f"Output dir : {os.path.abspath(output_dir)}"),
        LogInfo(msg=f"Bag output : {os.path.abspath(bag_output)}"),
        LogInfo(msg=f"Config     : {os.path.abspath(dst_config)}"),
        LogInfo(msg=f"Topics     : color + aligned_depth + camera_info"),
        LogInfo(msg=f"Press Ctrl+C to stop recording."),
        ExecuteProcess(
            cmd=[
                "ros2", "bag", "record",
                "-o", bag_output,
                "/camera/camera/color/image_raw",
                "/camera/camera/aligned_depth_to_color/image_raw",
                "/camera/camera/color/camera_info",
            ],
            output="screen",
            name="ros2_bag_record",
        ),
    ]


def generate_launch_description():
    # ---- 串口自动检测（与原版 dashgo_nav_real 一致）----
    resolved_ports = resolve_serial_ports()

    # ---- 录制参数 ----
    output_dir = LaunchConfiguration("output_dir")
    bag_name = LaunchConfiguration("bag_name")

    # ---- 底盘参数（与原版一致）----
    driver_port = LaunchConfiguration("driver_port")
    lidar_port = LaunchConfiguration("lidar_port")

    # ---- Web 控制参数（与原版一致）----
    start_web_ui = LaunchConfiguration("start_web_ui")
    start_hotspot = LaunchConfiguration("start_hotspot")
    web_host = LaunchConfiguration("web_host")
    web_port = LaunchConfiguration("web_port")
    web_image_topic = LaunchConfiguration("web_image_topic")
    web_map_publish_hz = LaunchConfiguration("web_map_publish_hz")
    web_camera_publish_hz = LaunchConfiguration("web_camera_publish_hz")
    web_camera_max_width = LaunchConfiguration("web_camera_max_width")

    # ---- 热点参数（与原版一致）----
    hotspot_connection_name = LaunchConfiguration("hotspot_connection_name")
    hotspot_ssid = LaunchConfiguration("hotspot_ssid")
    hotspot_password = LaunchConfiguration("hotspot_password")
    hotspot_ifname = LaunchConfiguration("hotspot_ifname")

    # ---- 路径 ----
    driver_share = get_package_share_directory("dashgo_driver_ros2")
    robot_launch = os.path.join(driver_share, "launch", "dashgo_robot.launch.py")

    return LaunchDescription([
        # ========== 录制参数 ==========
        DeclareLaunchArgument("output_dir",
            default_value=os.path.join(os.path.expanduser("~"), "project", "room_static_01"),
            description="数据集输出目录"),
        DeclareLaunchArgument("bag_name",
            default_value="room_static_01",
            description="bag 文件名"),

        # ========== 底盘参数（与原版 dashgo_nav_real 完全一致）==========
        DeclareLaunchArgument("driver_port",
            default_value=resolved_ports["driver_port"]),
        DeclareLaunchArgument("lidar_port",
            default_value=resolved_ports["lidar_port"]),

        # ========== Web 控制参数 ==========
        DeclareLaunchArgument("start_web_ui", default_value="true"),
        DeclareLaunchArgument("start_hotspot", default_value="true"),
        DeclareLaunchArgument("web_host", default_value="0.0.0.0"),
        DeclareLaunchArgument("web_port", default_value="8080"),
        DeclareLaunchArgument("web_image_topic",
            default_value="/camera/camera/color/image_raw"),
        DeclareLaunchArgument("web_map_publish_hz", default_value="2.0"),
        DeclareLaunchArgument("web_camera_publish_hz", default_value="12.0"),
        DeclareLaunchArgument("web_camera_max_width", default_value="320"),

        # ========== 热点参数 ==========
        DeclareLaunchArgument("hotspot_connection_name",
            default_value="dashgo-hotspot"),
        DeclareLaunchArgument("hotspot_ssid", default_value="Dashgo-Robot"),
        DeclareLaunchArgument("hotspot_password", default_value="dashgo12345"),
        DeclareLaunchArgument("hotspot_ifname", default_value=""),

        # ========== 底盘 + 雷达（与原版 dashgo_nav_real 完全一致）==========
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_launch),
            launch_arguments={
                "start_lidar": "false",          # 录制 RGB-D 不需要雷达
                "start_d435": "false",           # 下面单独启动 D435
                "start_t265": "false",
                "driver_port": driver_port,
                "driver_baud": "115200",
                "lidar_port": lidar_port,
                "lidar_baud": "1000000",
                "base_frame": "base_footprint",
                "laser_frame": "laser",
                "laser_z": "0.52",
                "publish_laser_tf": "false",
                "publish_sonar_tf": "false",
            }.items(),
        ),

        LogInfo(
            msg=f"Dashgo auto-detected base port default: {resolved_ports['driver_port']}"
        ),
        LogInfo(
            msg=f"Dashgo auto-detected lidar port default: {resolved_ports['lidar_port']}"
        ),

        # ========== D435 相机 ==========
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("dashgo_realsense_ros2"),
                    "launch", "d435.launch.py",
                )
            ),
            launch_arguments={
                "camera_name": "camera",
                "camera_namespace": "camera",
                "color_profile": "640,480,30",
                "depth_profile": "640,480,30",
                "align_depth": "true",
                "publish_model": "false",
                "use_rviz": "false",
            }.items(),
        ),

        # ========== ros2 bag 录制 ==========
        OpaqueFunction(function=_setup_and_record),

        # ========== Web 远程控制（与原版 dashgo_nav_real 一致）==========
        Node(
            package="dashgo_web_control",
            executable="web_control_node",
            name="dashgo_web_control",
            condition=IfCondition(start_web_ui),
            output="screen",
            parameters=[{
                "host": web_host,
                "port": web_port,
                "image_topic": web_image_topic,
                "map_publish_hz": web_map_publish_hz,
                "camera_publish_hz": web_camera_publish_hz,
                "camera_max_width": web_camera_max_width,
                "robot_radius": 0.20,
            }],
        ),

        # ========== WiFi 热点（与原版一致）==========
        TimerAction(
            period=0.3,
            condition=IfCondition(start_hotspot),
            actions=[
                ExecuteProcess(
                    cmd=[
                        "python3",
                        _resolve_hotspot_script(),
                        "--connection-name", hotspot_connection_name,
                        "--ssid", hotspot_ssid,
                        "--password", hotspot_password,
                        "--ifname", hotspot_ifname,
                    ],
                    output="screen",
                ),
            ],
        ),

        # ========== QR 码弹窗（与原版一致）==========
        TimerAction(
            period=2.3,
            condition=IfCondition(start_web_ui),
            actions=[
                ExecuteProcess(
                    cmd=["python3", _resolve_qr_popup_script()],
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
    ])
