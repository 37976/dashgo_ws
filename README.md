# dashgo_ws

基于 ROS 2 Humble 的 Dashgo 真机工作区，包含底盘驱动、RPLIDAR S2、RealSense 相机，以及本地化后的 Voronoi 导航栈。

当前这套工程面向真实机器人联调，不是 Gazebo 仿真工程。

## 功能概览

- 底盘串口驱动
- RPLIDAR S2 接入
- RealSense D435 / T265 启动封装
- 真机一体化启动
- 手机网页控制面板
- 本地化导航包
- `/scan -> /points_raw` 桥接，便于对接当前导航链路

## 工作区结构

```text
src/
├── dashgo_driver_ros2        # 底盘驱动与总启动
├── dashgo_lidar_ros2         # 雷达封装
├── dashgo_realsense_ros2     # RealSense 封装
├── dashgo_web_control        # 手机网页控制面板
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

### 4. 接线建议

这套真机对 USB 接线比较敏感，实测有如下经验：

- 底盘、雷达、D435 不要都接在同一个无源扩展口或同一个小 Hub 上
- 如果雷达启动后一瞬间掉线，或报 `RPLidar internal error detected`，优先检查是不是和底盘或相机接在同一路 USB 扩展上
- 如果启动后一直没有 `/odom`，除了检查串口权限，也要检查底盘是否和雷达共用了同一个扩展接口
- 更稳妥的做法是：底盘、雷达、相机尽量分散到不同物理接口；如果必须扩展，优先用带独立供电的 USB Hub

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
ros2 run dashgo_driver_ros2 dashgo_driver_node --ros-args -p port:=/dev/ttyUSB1 -p baud:=115200
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

### 真机导航（默认不启动相机，默认启动网页控制）

```bash
cd ~/dashgo_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOG_DIR=/tmp/roslogs
ros2 launch dashgo_driver_ros2 dashgo_nav_real.launch.py
```

### 真机导航，启动相机

```bash
cd ~/dashgo_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOG_DIR=/tmp/roslogs
ros2 launch dashgo_driver_ros2 dashgo_nav_real.launch.py start_d435:=true
```

### 真机导航，不启动 RViz

```bash
cd ~/dashgo_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOG_DIR=/tmp/roslogs
ros2 launch dashgo_driver_ros2 dashgo_nav_real.launch.py start_d435:=false start_nav_rviz:=false
```

### 单独启动手机网页控制

```bash
cd ~/dashgo_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch dashgo_web_control web_control.launch.py
```

### 真机导航，同时开启机器人固定热点

这台机器当前无线网卡是 `wlp13s0`，如需让机器人自己发热点，推荐显式指定：

```bash
cd ~/dashgo_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOG_DIR=/tmp/roslogs
ros2 launch dashgo_driver_ros2 dashgo_nav_real.launch.py start_hotspot:=true hotspot_ifname:=wlp13s0
```

如需自定义热点名和密码：

```bash
ros2 launch dashgo_driver_ros2 dashgo_nav_real.launch.py \
  start_hotspot:=true \
  hotspot_ifname:=wlp13s0 \
  hotspot_ssid:=Dashgo-Robot \
  hotspot_password:=dashgo12345
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
ros2 topic echo /control_mode --once
```

## 话题链路说明

真实机器人导航当前链路如下：

- 雷达发布 `/scan`
- `scan_to_points_node` 将 `/scan` 转成 `/points_raw`
- `nav_slam` 使用 `/points_raw`、`/odom`、静态地图进行处理
- `start_nav` 发布 `/cmd_vel`
- `dashgo_driver_ros2` 接收 `/cmd_vel` 控制底盘
- `dashgo_web_control` 通过网页提供模式切换、手动遥控和目标下发

## 当前限制

- 这套导航当前更偏“里程计 + 点云/栅格 + 自定义规划控制”
- 不是标准 Nav2 的 `amcl` / `slam_toolbox` 全套定位导航方案
- 如果后续需要标准雷达定位，需要继续补定位模块

## 手机网页控制说明

默认使用 `dashgo_nav_real.launch.py` 时，会同时启动网页控制节点，默认地址为：

```text
http://<机器人IP>:8080
```

当前网页控制逻辑如下：

- 页面分为“导航页面”和“手动页面”，通过顶部按钮切换
- 导航页面保留地图、居中/刷新、选点与确认导航
- 手动页面只保留相机、摇杆和基础状态
- 导航不会因为误触立即开始，必须先“选点导航”，再点击“确认导航”
- 手动模式下才能通过摇杆发送速度指令，导航模式下摇杆会被禁用
- 地图支持鼠标滚轮缩放、双指缩放，以及拖拽平移
- 网页地图会叠加显示雷达点

### 二维码弹窗

启动网页控制后，桌面环境下会自动弹出二维码窗口。

- 默认会显示“打开网页”二维码
- 如果启动时带了 `start_hotspot:=true`，弹窗会显示两个二维码
- 左侧二维码用于手机连接机器人热点
- 右侧二维码用于打开控制网页

如果没有图形界面，程序会自动退回到终端打印网页二维码。

### 机器人固定热点说明

当前已经支持由机器人 Ubuntu 22 自己创建固定热点。

- 热点功能默认关闭，避免一启动就切走当前网络
- 启动参数 `start_hotspot:=true` 后，会先自动打开 Wi-Fi，再拉起热点
- 默认连接名为 `dashgo-hotspot`
- 默认热点名为 `Dashgo-Robot`
- 默认密码为 `dashgo12345`
- 若你手动关闭过热点，下次重新启动 launch 时会自动重新打开 Wi-Fi 并恢复热点

注意：

- 手机通常需要先扫“连接热点”二维码加入 Wi-Fi，再扫“打开网页”二维码进入控制页
- 不能稳定地用一个二维码同时完成“自动连 Wi-Fi + 自动打开网页”两件事
- 热点启动依赖 NetworkManager 的 `nmcli`

如果修改了 `src/dashgo_web_control/web/` 下的前端文件，记得重新编译：

```bash
cd ~/dashgo_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select dashgo_web_control
source install/setup.bash
```

## 相关入口文件

- [真实机器人总启动](./src/dashgo_driver_ros2/launch/dashgo_nav_real.launch.py)
- [底盘总启动](./src/dashgo_driver_ros2/launch/dashgo_robot.launch.py)
- [底盘驱动](./src/dashgo_driver_ros2/dashgo_driver_ros2/dashgo_driver_node.py)
- [雷达桥接](./src/dashgo_driver_ros2/dashgo_driver_ros2/scan_to_points_node.py)
- [网页控制节点](./src/dashgo_web_control/dashgo_web_control/web_control_node.py)
- [网页控制前端](./src/dashgo_web_control/web/index.html)
- [导航地图](./src/nav_slam/map/gpt.yaml)

## 备注

- 雷达在 RViz 中显示时，`LaserScan` 的 QoS 建议设为 `Best Effort`
- 若 RViz 出现 TF 时间外推，可先将 `Fixed Frame` 设为 `base_footprint` 或 `laser`
- `dashgo_nav_real.launch.py` 当前默认 `start_d435:=false`、`start_web_ui:=true`
