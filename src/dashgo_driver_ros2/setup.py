from setuptools import setup

package_name = "dashgo_driver_ros2"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{package_name}"],
        ),
        (f"share/{package_name}", ["package.xml"]),
        (
            f"share/{package_name}/launch",
            [
                "launch/dashgo_driver.launch.py",
                "launch/dashgo_bringup.launch.py",
                "launch/dashgo_full_bringup.launch.py",
                "launch/dashgo_robot.launch.py",
                "launch/dashgo_nav_real.launch.py",
            ],
        ),
        (
            f"share/{package_name}/config",
            ["config/my_dashgo_params.yaml"],
        ),
    ],
    install_requires=["setuptools", "pyserial"],
    zip_safe=True,
    maintainer="xu",
    maintainer_email="xu@example.com",
    description="ROS 2 driver package for Dashgo base controller.",
    license="GPL-2.0-or-later",
    entry_points={
        "console_scripts": [
            "dashgo_driver_node = dashgo_driver_ros2.dashgo_driver_node:main",
            "scan_to_points_node = dashgo_driver_ros2.scan_to_points_node:main",
        ],
    },
)
