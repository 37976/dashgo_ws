from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    serial_no = LaunchConfiguration("serial_no")
    camera_name = LaunchConfiguration("camera_name")
    camera_namespace = LaunchConfiguration("camera_namespace")

    try:
        rs_launch = get_package_share_directory("realsense2_camera") + "/launch/rs_launch.py"
    except PackageNotFoundError:
        return LaunchDescription(
            [
                LogInfo(
                    msg="realsense2_camera is not installed. Skipping T265 bringup."
                )
            ]
        )

    return LaunchDescription(
        [
            DeclareLaunchArgument("serial_no", default_value=""),
            DeclareLaunchArgument("camera_name", default_value="camera"),
            DeclareLaunchArgument("camera_namespace", default_value="camera"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(rs_launch),
                launch_arguments={
                    "serial_no": serial_no,
                    "camera_name": camera_name,
                    "camera_namespace": camera_namespace,
                    "device_type": "t265",
                    "enable_fisheye1": "false",
                    "enable_fisheye2": "false",
                    "enable_gyro": "true",
                    "enable_accel": "true",
                    "enable_pose": "true",
                    "gyro_fps": "0",
                    "accel_fps": "0",
                    "publish_tf": "true",
                    "publish_odom_tf": "true",
                }.items(),
            ),
        ]
    )
