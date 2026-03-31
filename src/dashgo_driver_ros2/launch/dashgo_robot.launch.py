import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def static_tf_node(
    name,
    parent_frame,
    child_frame,
    x,
    y,
    z,
    roll,
    pitch,
    yaw,
    enabled,
):
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=name,
        condition=IfCondition(enabled),
        arguments=[
            "--x", str(x),
            "--y", str(y),
            "--z", str(z),
            "--roll", str(roll),
            "--pitch", str(pitch),
            "--yaw", str(yaw),
            "--frame-id", parent_frame,
            "--child-frame-id", child_frame,
        ],
    )


def generate_launch_description():
    driver_share = get_package_share_directory("dashgo_driver_ros2")
    lidar_share = get_package_share_directory("dashgo_lidar_ros2")
    realsense_share = get_package_share_directory("dashgo_realsense_ros2")

    default_driver_params = os.path.join(driver_share, "config", "my_dashgo_params.yaml")
    default_lidar_params = os.path.join(lidar_share, "config", "rplidar_s2.yaml")

    driver_params = LaunchConfiguration("driver_params")
    lidar_params = LaunchConfiguration("lidar_params")
    driver_port = LaunchConfiguration("driver_port")
    driver_baud = LaunchConfiguration("driver_baud")
    lidar_port = LaunchConfiguration("lidar_port")
    lidar_baud = LaunchConfiguration("lidar_baud")
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
                "driver_port",
                default_value="/dev/ttyUSB0",
                description="Serial port for the Dashgo base controller.",
            ),
            DeclareLaunchArgument(
                "driver_baud",
                default_value="115200",
                description="Baud rate for the Dashgo base controller.",
            ),
            DeclareLaunchArgument(
                "lidar_port",
                default_value=(
                    "/dev/serial/by-id/"
                    "usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_"
                    "d8a678826473ed11a4766aeefdf7b791-if00-port0"
                ),
                description="Serial port for the lidar.",
            ),
            DeclareLaunchArgument(
                "lidar_baud",
                default_value="1000000",
                description="Baud rate for the lidar.",
            ),
            DeclareLaunchArgument(
                "start_lidar",
                default_value="true",
                description="Start the lidar node.",
            ),
            DeclareLaunchArgument(
                "start_d435",
                default_value="true",
                description="Start the D435 camera wrapper.",
            ),
            DeclareLaunchArgument(
                "start_t265",
                default_value="false",
                description="Start the T265 camera wrapper.",
            ),
            DeclareLaunchArgument(
                "publish_sonar_tf",
                default_value="true",
                description="Publish static transforms for sonar frames.",
            ),
            DeclareLaunchArgument(
                "publish_laser_tf",
                default_value="true",
                description="Publish the static transform from base frame to laser.",
            ),
            DeclareLaunchArgument(
                "base_frame",
                default_value="base_footprint",
                description="Base frame for robot-mounted sensors.",
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
            Node(
                package="dashgo_driver_ros2",
                executable="dashgo_driver_node",
                name="dashgo_driver",
                output="screen",
                parameters=[
                    driver_params,
                    {
                        "port": driver_port,
                        "baud": driver_baud,
                        "base_frame": base_frame,
                    },
                ],
            ),
            static_tf_node(
                "base_link_to_sonar0",
                base_frame,
                "sonar0",
                0.18,
                0.10,
                0.115,
                0.524,
                0.0,
                0.0,
                publish_sonar_tf,
            ),
            static_tf_node(
                "base_link_to_sonar1",
                base_frame,
                "sonar1",
                0.20,
                0.0,
                0.115,
                0.0,
                0.0,
                0.0,
                publish_sonar_tf,
            ),
            static_tf_node(
                "base_link_to_sonar2",
                base_frame,
                "sonar2",
                0.18,
                -0.10,
                0.115,
                -0.524,
                0.0,
                0.0,
                publish_sonar_tf,
            ),
            static_tf_node(
                "base_link_to_sonar3",
                base_frame,
                "sonar3",
                -0.20,
                0.0,
                0.115,
                3.14,
                0.0,
                0.0,
                publish_sonar_tf,
            ),
            Node(
                package="sllidar_ros2",
                executable="sllidar_node",
                name="sllidar_node",
                condition=IfCondition(start_lidar),
                output="screen",
                parameters=[
                    lidar_params,
                    {
                        "serial_port": lidar_port,
                        "serial_baudrate": lidar_baud,
                        "frame_id": laser_frame,
                    },
                ],
            ),
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
