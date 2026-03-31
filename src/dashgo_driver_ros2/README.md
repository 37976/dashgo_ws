# dashgo_driver_ros2

ROS 2 Humble base driver package for the Dashgo mobile base.

## Features

- Serial communication with the Dashgo controller board
- `cmd_vel` subscription
- Wheel PID polling and motor command output
- `odom` publication
- Optional `odom -> base_frame` TF publication
- Sonar static TF launch entries for compatibility with the original ROS 1 package

## Package layout

- `dashgo_driver_ros2/dashgo_driver_node.py`: main driver node
- `config/my_dashgo_params.yaml`: runtime parameters
- `launch/dashgo_driver.launch.py`: driver-only launch
- `launch/dashgo_bringup.launch.py`: driver plus sonar static TFs

## Build

```bash
cd /home/xu/project/dashgo_ws
source /opt/ros/humble/setup.bash
colcon build --merge-install
source install/setup.bash
```

## Run

```bash
ros2 launch dashgo_driver_ros2 dashgo_bringup.launch.py
```

or

```bash
ros2 run dashgo_driver_ros2 dashgo_driver_node --ros-args --params-file src/dashgo_driver_ros2/config/my_dashgo_params.yaml
```

## Required environment

- ROS 2 Humble
- `python3-serial`
- Accessible controller device such as `/dev/dashgo` or `/dev/ttyUSB0`

## Main parameters

- `port`
- `baud`
- `timeout`
- `cmd_vel_topic`
- `odom_topic`
- `odom_frame`
- `base_frame`
- `publish_odom_tf`
- `wheel_diameter`
- `wheel_track`
- `encoder_resolution`
- `gear_reduction`
- `motors_reversed`

## Current scope

This package currently focuses on the mobile base controller. Sonar and IMU data transport are not implemented yet even though compatibility parameters are kept in the config file.
