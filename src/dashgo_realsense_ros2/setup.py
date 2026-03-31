from setuptools import setup

package_name = "dashgo_realsense_ros2"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (
            f"share/{package_name}/launch",
            [
                "launch/d435.launch.py",
                "launch/t265.launch.py",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="xu",
    maintainer_email="xu@example.com",
    description="ROS 2 RealSense wrapper package for Dashgo using realsense2_camera.",
    license="Apache-2.0",
)
