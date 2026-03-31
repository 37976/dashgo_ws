# dashgo_realsense_ros2

ROS 2 wrapper package for Dashgo RealSense integration.

This package reuses the official ROS 2 `realsense2_camera` launch files and exposes
Dashgo-oriented wrapper launches for D435-class cameras and T265.

## Build

```bash
cd /home/xu/project/dashgo_ws
source /opt/ros/humble/setup.bash
colcon build --merge-install
source install/setup.bash
```

## Run

```bash
ros2 launch dashgo_realsense_ros2 d435.launch.py
```

or

```bash
ros2 launch dashgo_realsense_ros2 t265.launch.py
```

## Notes

- Install `realsense2_camera` and `realsense2_description` first
- These wrappers intentionally reuse the upstream ROS 2 driver instead of porting the old ROS 1 nodelet implementation
