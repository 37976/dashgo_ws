from glob import glob
import os

from setuptools import find_packages, setup


package_name = "dashgo_xfeat_bringup"


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="boxing",
    maintainer_email="clibang2022@163.com",
    description="Standalone XFeat bringup package for the Dashgo real robot workspace.",
    license="TODO: License declaration",
    entry_points={
        "console_scripts": [
            "xfeat_rtabmap_bridge = dashgo_xfeat_bringup.xfeat_rtabmap_bridge:main",
            "xfeat_rgbd_odometry = dashgo_xfeat_bringup.xfeat_rgbd_odometry:main",
            "odom_fusion_node = dashgo_xfeat_bringup.odom_fusion_node:main",
        ],
    },
)
