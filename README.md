# dashgo_ws

基于 ROS 2 Humble 的 Dashgo 真机工作区，包含底盘驱动、RPLIDAR S2、RealSense 相机，以及本地化后的 Voronoi 导航栈。

当前这套工程面向真实机器人联调，不是 Gazebo 仿真工程。

## 功能概览

- 底盘串口驱动
- RPLIDAR S2 接入
- RealSense D435 / T265 启动封装
- 真机一体化启动
- 本地化导航包
- `/scan -> /points_raw` 桥接，便于对接当前导航链路

## 工作区结构

```text
src/
├── dashgo_driver_ros2        # 底盘驱动与总启动
├── dashgo_lidar_ros2         # 雷达封装
├── dashgo_realsense_ros2     # RealSense 封装
├── sllidar_ros2              # 雷达底层驱动
├── nav_slam                  # 本地化导航节点
├── nav2_voronoi_planner      # Voronoi 规划器
└── dynamicvoronoi            # Voronoi 基础库
```

## 环境要求

- Ubuntu 22.04
- ROS 2 Humble
- `python3-serial`
- 可访问的底盘串口设备
- 可访问的雷达串口设备

相机可选依赖：

```bash
sudo apt install ros-humble-realsense2-camera ros-humble-realsense2-description
```

## 首次使用前建议

### 1. 加入串口权限组

```bash
sudo usermod -aG dialout $USER
```

执行后重新登录终端或重启。

### 2. 避免 `brltty` 抢占底盘串口

如果底盘是 `CH340/HL-340`，系统可能被 `brltty` 抢走 `ttyUSB` 设备。

```bash
sudo systemctl stop brltty.service brltty-udev.service
sudo systemctl mask brltty.service brltty-udev.service
```

### 3. 编译工作区

```bash
cd ~/dashgo_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## 常用环境命令

```bash
cd ~/dashgo_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOG_DIR=/tmp/roslogs
```

## 启动方式

### 底盘单独启动

```bash
cd ~/dashgo_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run dashgo_driver_ros2 dashgo_driver_node --ros-args -p port:=/dev/ttyUSB0 -p baud:=115200
```

### 雷达单独启动

```bash
cd ~/dashgo_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch dashgo_lidar_ros2 rplidar_s2.launch.py
```

### 底盘 + 雷达 + 相机

```bash
cd ~/dashgo_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOG_DIR=/tmp/roslogs
ros2 launch dashgo_driver_ros2 dashgo_robot.launch.py
```

### 底盘 + 雷达

```bash
cd ~/dashgo_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOG_DIR=/tmp/roslogs
ros2 launch dashgo_driver_ros2 dashgo_robot.launch.py start_d435:=false
```

### 真机导航

```bash
cd ~/dashgo_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOG_DIR=/tmp/roslogs
ros2 launch dashgo_driver_ros2 dashgo_nav_real.launch.py
```

### 真机导航，不启动相机

```bash
cd ~/dashgo_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOG_DIR=/tmp/roslogs
ros2 launch dashgo_driver_ros2 dashgo_nav_real.launch.py start_d435:=false
```

### 真机导航，不启动 RViz

```bash
cd ~/dashgo_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOG_DIR=/tmp/roslogs
ros2 launch dashgo_driver_ros2 dashgo_nav_real.launch.py start_d435:=false start_nav_rviz:=false
```

## 常用检查命令

```bash
ros2 topic list
ros2 topic echo /scan --once
ros2 topic echo /odom --once
ros2 topic echo /cmd_vel --once
ros2 topic echo /points_raw --once
ros2 topic echo /combined_grid --once
ros2 topic echo /path --once
```

## 话题链路说明

真实机器人导航当前链路如下：

- 雷达发布 `/scan`
- `scan_to_points_node` 将 `/scan` 转成 `/points_raw`
- `nav_slam` 使用 `/points_raw`、`/odom`、静态地图进行处理
- `start_nav` 发布 `/cmd_vel`
- `dashgo_driver_ros2` 接收 `/cmd_vel` 控制底盘

## 当前限制

- 这套导航当前更偏“里程计 + 点云/栅格 + 自定义规划控制”
- 不是标准 Nav2 的 `amcl` / `slam_toolbox` 全套定位导航方案
- 如果后续需要标准雷达定位，需要继续补定位模块

## 相关入口文件

- [真实机器人总启动](./src/dashgo_driver_ros2/launch/dashgo_nav_real.launch.py)
- [底盘总启动](./src/dashgo_driver_ros2/launch/dashgo_robot.launch.py)
- [底盘驱动](./src/dashgo_driver_ros2/dashgo_driver_ros2/dashgo_driver_node.py)
- [雷达桥接](./src/dashgo_driver_ros2/dashgo_driver_ros2/scan_to_points_node.py)
- [导航地图](./src/nav_slam/map/gpt.yaml)

## 备注

- 雷达在 RViz 中显示时，`LaserScan` 的 QoS 建议设为 `Best Effort`
- 若 RViz 出现 TF 时间外推，可先将 `Fixed Frame` 设为 `base_footprint` 或 `laser`
