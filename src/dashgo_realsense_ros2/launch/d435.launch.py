from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_no = LaunchConfiguration("serial_no")
    camera_name = LaunchConfiguration("camera_name")
    camera_namespace = LaunchConfiguration("camera_namespace")
    publish_model = LaunchConfiguration("publish_model")
    use_rviz = LaunchConfiguration("use_rviz")
    color_profile = LaunchConfiguration("color_profile")
    depth_profile = LaunchConfiguration("depth_profile")
    align_depth = LaunchConfiguration("align_depth")

    try:
        rs_launch = get_package_share_directory("realsense2_camera") + "/launch/rs_launch.py"
        realsense_description_share = get_package_share_directory("realsense2_description")
    except PackageNotFoundError:
        return LaunchDescription(
            [
                LogInfo(
                    msg=(
                        "realsense2_camera or realsense2_description is not installed. "
                        "Skipping D435 bringup."
                    )
                )
            ]
        )

    xacro_file = realsense_description_share + "/urdf/test_d435_camera.urdf.xacro"
    rviz_file = realsense_description_share + "/rviz/urdf.rviz"

    robot_description = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            xacro_file,
            " ",
            "use_nominal_extrinsics:=false",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("serial_no", default_value=""),
            DeclareLaunchArgument("camera_name", default_value="camera"),
            DeclareLaunchArgument("camera_namespace", default_value="camera"),
            DeclareLaunchArgument("publish_model", default_value="false"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument("color_profile", default_value="640,480,30"),
            DeclareLaunchArgument("depth_profile", default_value="640,480,30"),
            DeclareLaunchArgument("align_depth", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(rs_launch),
                launch_arguments={
                    "serial_no": serial_no,
                    "camera_name": camera_name,
                    "camera_namespace": camera_namespace,
                    "enable_color": "true",
                    "enable_depth": "true",
                    "enable_infra1": "false",
                    "enable_infra2": "false",
                    "enable_gyro": "false",
                    "enable_accel": "false",
                    "rgb_camera.color_profile": color_profile,
                    "depth_module.depth_profile": depth_profile,
                    "enable_pointcloud": "false",
                    "align_depth.enable": align_depth,
                    "initial_reset": "true",
                    "publish_tf": "true",
                    "publish_odom_tf": "false",
                }.items(),
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="realsense_state_publisher",
                condition=IfCondition(publish_model),
                parameters=[{"robot_description": robot_description}],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="realsense_rviz",
                condition=IfCondition(use_rviz),
                arguments=["-d", rviz_file],
            ),
        ]
    )
