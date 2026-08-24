from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'dashgo_rgbd_recorder'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='boxing/喵了个水蓝蓝',
    maintainer_email='clibang2022@163.com',
    description='D435 RGB-D dataset recording and export for ORB-SLAM3',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'export_dataset = dashgo_rgbd_recorder.export_dataset:main',
        ],
    },
)
