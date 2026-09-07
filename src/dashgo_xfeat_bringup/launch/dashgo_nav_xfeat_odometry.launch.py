#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Compatibility entry point for the standard Dashgo real navigation launch."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    nav_launch = os.path.join(
        get_package_share_directory("dashgo_driver_ros2"),
        "launch",
        "dashgo_nav_real.launch.py",
    )

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(nav_launch)),
    ])
