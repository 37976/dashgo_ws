from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    serial_no = LaunchConfiguration("serial_no")
    camera_name = LaunchConfiguration("camera_name")
    camera_namespace = LaunchConfiguration("camera_namespace")
    enable_fisheye1 = LaunchConfiguration("enable_fisheye1")
    enable_fisheye2 = LaunchConfiguration("enable_fisheye2")
    enable_gyro = LaunchConfiguration("enable_gyro")
    enable_accel = LaunchConfiguration("enable_accel")
    enable_pose = LaunchConfiguration("enable_pose")
    gyro_fps = LaunchConfiguration("gyro_fps")
    accel_fps = LaunchConfiguration("accel_fps")
    unite_imu_method = LaunchConfiguration("unite_imu_method")

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
            DeclareLaunchArgument("enable_fisheye1", default_value="false"),
            DeclareLaunchArgument("enable_fisheye2", default_value="false"),
            DeclareLaunchArgument("enable_gyro", default_value="true"),
            DeclareLaunchArgument("enable_accel", default_value="true"),
            DeclareLaunchArgument("enable_pose", default_value="true"),
            DeclareLaunchArgument("gyro_fps", default_value="0"),
            DeclareLaunchArgument("accel_fps", default_value="0"),
            DeclareLaunchArgument("unite_imu_method", default_value="0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(rs_launch),
                launch_arguments={
                    "serial_no": serial_no,
                    "camera_name": camera_name,
                    "camera_namespace": camera_namespace,
                    "device_type": "t265",
                    "enable_fisheye1": enable_fisheye1,
                    "enable_fisheye2": enable_fisheye2,
                    "enable_gyro": enable_gyro,
                    "enable_accel": enable_accel,
                    "enable_pose": enable_pose,
                    "gyro_fps": gyro_fps,
                    "accel_fps": accel_fps,
                    "unite_imu_method": unite_imu_method,
                    "publish_tf": "true",
                    "publish_odom_tf": "true",
                }.items(),
            ),
        ]
    )
