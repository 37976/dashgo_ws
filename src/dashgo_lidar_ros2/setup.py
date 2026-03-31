from setuptools import setup

package_name = "dashgo_lidar_ros2"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (
            f"share/{package_name}/launch",
            ["launch/rplidar_s2.launch.py"],
        ),
        (
            f"share/{package_name}/config",
            ["config/rplidar_s2.yaml"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="xu",
    maintainer_email="xu@example.com",
    description="ROS 2 lidar wrapper package for Dashgo using SLAMTEC sllidar_ros2.",
    license="BSD-2-Clause",
)
