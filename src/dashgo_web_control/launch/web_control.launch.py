import glob
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
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
    if not matches:
        raise FileNotFoundError("Unable to locate show_web_qr_popup.py")
    return matches[0]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("host", default_value="0.0.0.0"),
            DeclareLaunchArgument("port", default_value="8080"),
            DeclareLaunchArgument("image_topic", default_value="/camera/camera/color/image_raw"),
            Node(
                package="dashgo_web_control",
                executable="web_control_node",
                name="dashgo_web_control",
                output="screen",
                parameters=[
                    {
                        "host": LaunchConfiguration("host"),
                        "port": LaunchConfiguration("port"),
                        "image_topic": LaunchConfiguration("image_topic"),
                    }
                ],
            ),
            TimerAction(
                period=1.5,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "python3",
                            resolve_qr_popup_script(),
                        ],
                        additional_env={
                            "DASHGO_WEB_UI_HOST": LaunchConfiguration("host"),
                            "DASHGO_WEB_UI_PORT": LaunchConfiguration("port"),
                        },
                        output="screen",
                    ),
                ],
            ),
        ]
    )
