# dashgo_lidar_ros2

ROS 2 wrapper package for Dashgo lidar integration.

This package reuses the official `sllidar_ros2` driver and provides a Dashgo-oriented
launch file and default configuration for the RPLIDAR S2.

## Build

```bash
cd /home/xu/project/dashgo_ws
source /opt/ros/humble/setup.bash
colcon build --merge-install
source install/setup.bash
```

## Run

```bash
ros2 launch dashgo_lidar_ros2 rplidar_s2.launch.py
```

## Notes

- Install the external driver package first: `sllidar_ros2`
- Default serial port: `/dev/rplidar`
- Default baudrate for S2: `1000000`
- The wrapper only publishes `base_link -> laser` static TF by default
