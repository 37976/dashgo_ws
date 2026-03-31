import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    driver_params = LaunchConfiguration("driver_params")
    lidar_params = LaunchConfiguration("lidar_params")
    start_lidar = LaunchConfiguration("start_lidar")
    start_d435 = LaunchConfiguration("start_d435")
    start_t265 = LaunchConfiguration("start_t265")
    publish_sonar_tf = LaunchConfiguration("publish_sonar_tf")
    publish_laser_tf = LaunchConfiguration("publish_laser_tf")
    base_frame = LaunchConfiguration("base_frame")
    laser_frame = LaunchConfiguration("laser_frame")
    laser_z = LaunchConfiguration("laser_z")
    d435_serial_no = LaunchConfiguration("d435_serial_no")
    t265_serial_no = LaunchConfiguration("t265_serial_no")

    driver_share = get_package_share_directory("dashgo_driver_ros2")
    lidar_share = get_package_share_directory("dashgo_lidar_ros2")
    realsense_share = get_package_share_directory("dashgo_realsense_ros2")

    default_driver_params = os.path.join(driver_share, "config", "my_dashgo_params.yaml")
    default_lidar_params = os.path.join(lidar_share, "config", "rplidar_s2.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "driver_params",
                default_value=default_driver_params,
                description="Path to the base driver parameter file.",
            ),
            DeclareLaunchArgument(
                "lidar_params",
                default_value=default_lidar_params,
                description="Path to the lidar parameter file.",
            ),
            DeclareLaunchArgument(
                "start_lidar",
                default_value="true",
                description="Start the lidar wrapper.",
            ),
            DeclareLaunchArgument(
                "start_d435",
                default_value="true",
                description="Start the RealSense D435 wrapper.",
            ),
            DeclareLaunchArgument(
                "start_t265",
                default_value="false",
                description="Start the RealSense T265 wrapper.",
            ),
            DeclareLaunchArgument(
                "publish_sonar_tf",
                default_value="true",
                description="Publish base to sonar static transforms.",
            ),
            DeclareLaunchArgument(
                "publish_laser_tf",
                default_value="true",
                description="Publish base to laser static transform.",
            ),
            DeclareLaunchArgument(
                "base_frame",
                default_value="base_footprint",
                description="Base frame used for driver and sensor static transforms.",
            ),
            DeclareLaunchArgument(
                "laser_frame",
                default_value="laser",
                description="Laser frame id.",
            ),
            DeclareLaunchArgument(
                "laser_z",
                default_value="0.18",
                description="Laser height above the base frame.",
            ),
            DeclareLaunchArgument(
                "d435_serial_no",
                default_value="",
                description="Optional serial number for the D435 camera.",
            ),
            DeclareLaunchArgument(
                "t265_serial_no",
                default_value="",
                description="Optional serial number for the T265 camera.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(driver_share, "launch", "dashgo_bringup.launch.py")
                ),
                launch_arguments={
                    "params_file": driver_params,
                    "publish_sonar_tf": publish_sonar_tf,
                    "base_frame": base_frame,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(lidar_share, "launch", "rplidar_s2.launch.py")
                ),
                condition=IfCondition(start_lidar),
                launch_arguments={
                    "params_file": lidar_params,
                    "publish_laser_tf": publish_laser_tf,
                    "base_frame": base_frame,
                    "laser_frame": laser_frame,
                    "laser_z": laser_z,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(realsense_share, "launch", "d435.launch.py")
                ),
                condition=IfCondition(start_d435),
                launch_arguments={
                    "serial_no": d435_serial_no,
                    "camera_name": "camera",
                    "camera_namespace": "camera",
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(realsense_share, "launch", "t265.launch.py")
                ),
                condition=IfCondition(start_t265),
                launch_arguments={
                    "serial_no": t265_serial_no,
                    "camera_name": "t265",
                    "camera_namespace": "t265",
                }.items(),
            ),
        ]
    )
