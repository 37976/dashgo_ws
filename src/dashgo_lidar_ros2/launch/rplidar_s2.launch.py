import glob
import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def read_udev_properties(device_path):
    result = subprocess.run(
        ["udevadm", "info", "-q", "property", "-n", device_path],
        check=False,
        capture_output=True,
        text=True,
    )
    props = {}
    if result.returncode != 0:
        return props
    for line in result.stdout.splitlines():
        if "=" not in line:
            continue
        key, value = line.strip().split("=", 1)
        props[key] = value
    return props


def resolve_lidar_port():
    candidates = []
    seen_real_paths = set()

    for symlink in sorted(glob.glob("/dev/serial/by-id/*")):
        real_path = os.path.realpath(symlink)
        if not real_path.startswith("/dev/"):
            continue
        seen_real_paths.add(real_path)
        candidates.append((symlink, read_udev_properties(real_path)))

    for pattern in ("/dev/ttyUSB*", "/dev/ttyACM*"):
        for device_path in sorted(glob.glob(pattern)):
            real_path = os.path.realpath(device_path)
            if real_path in seen_real_paths:
                continue
            seen_real_paths.add(real_path)
            candidates.append((device_path, read_udev_properties(real_path)))

    ranked = []
    for path, props in candidates:
        score = 1000
        if props.get("ID_VENDOR_ID") == "10c4" and props.get("ID_MODEL_ID") == "ea60":
            score = 0
        elif props.get("ID_USB_DRIVER") == "cp210x":
            score = 10
        elif "CP210" in props.get("ID_MODEL", ""):
            score = 20
        if path.startswith("/dev/serial/by-id/"):
            score -= 1
        ranked.append((score, path))

    if not ranked:
        return "/dev/ttyUSB0"

    ranked.sort(key=lambda item: (item[0], item[1]))
    best_score, best_path = ranked[0]
    if best_score >= 1000:
        return "/dev/ttyUSB0"
    return best_path


def generate_launch_description():
    pkg_share = get_package_share_directory("dashgo_lidar_ros2")
    default_params = os.path.join(pkg_share, "config", "rplidar_s2.yaml")
    resolved_lidar_port = resolve_lidar_port()

    params_file = LaunchConfiguration("params_file")
    serial_port = LaunchConfiguration("serial_port")
    publish_laser_tf = LaunchConfiguration("publish_laser_tf")
    base_frame = LaunchConfiguration("base_frame")
    laser_frame = LaunchConfiguration("laser_frame")
    laser_z = LaunchConfiguration("laser_z")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Path to the lidar parameter file.",
            ),
            DeclareLaunchArgument(
                "serial_port",
                default_value=resolved_lidar_port,
                description="Serial port for the lidar.",
            ),
            DeclareLaunchArgument(
                "publish_laser_tf",
                default_value="true",
                description="Publish the static transform from base_link to laser.",
            ),
            DeclareLaunchArgument(
                "base_frame",
                default_value="base_link",
                description="Parent frame for the lidar static transform.",
            ),
            DeclareLaunchArgument(
                "laser_frame",
                default_value="laser",
                description="Laser frame id.",
            ),
            DeclareLaunchArgument(
                "laser_z",
                default_value="0.18",
                description="Laser height above base frame in meters.",
            ),
            Node(
                package="sllidar_ros2",
                executable="sllidar_node",
                name="sllidar_node",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "serial_port": serial_port,
                        "frame_id": laser_frame,
                    },
                ],
            ),
            LogInfo(msg=f"Dashgo auto-detected lidar port default: {resolved_lidar_port}"),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_laser_tf",
                condition=IfCondition(publish_laser_tf),
                arguments=[
                    "--x", "0.0",
                    "--y", "0.0",
                    "--z", laser_z,
                    "--roll", "0.0",
                    "--pitch", "0.0",
                    "--yaw", "0.0",
                    "--frame-id", base_frame,
                    "--child-frame-id", laser_frame,
                ],
            ),
        ]
    )
