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

## 快速启动

默认推荐把机器人固定热点一起打开，这样手机直接连接机器人热点，再扫码进入网页控制。

当前 `dashgo_robot.launch.py` 和 `dashgo_nav_real.launch.py` 已支持自动识别底盘串口和雷达串口。

- 底盘优先识别 CH340/CH341 控制板
- 雷达优先识别 CP210x 串口设备
- 优先使用 `/dev/serial/by-id/` 这类更稳定的路径
- 换电脑或 `ttyUSB0/1` 顺序变化时，常用总启动命令通常不需要改
- 如果机器上同时插了多块类似 USB 串口设备，仍然可以手动传 `driver_port:=...` 或 `lidar_port:=...`

### 推荐命令

```bash
cd ~/dashgo_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOG_DIR=/tmp/roslogs
ros2 launch dashgo_driver_ros2 dashgo_nav_real.launch.py start_hotspot:=true
```

这条命令会同时启动：

- 底盘
- 雷达
- D435 相机
- 导航节点
- 网页控制
- 机器人固定热点
- 二维码弹窗

### 常用变体

```bash
# 不启动相机
ros2 launch dashgo_driver_ros2 dashgo_nav_real.launch.py start_hotspot:=true start_d435:=false
# 不启动 RViz
ros2 launch dashgo_driver_ros2 dashgo_nav_real.launch.py start_hotspot:=true start_nav_rviz:=false
# 不启动相机，也不启动 RViz
ros2 launch dashgo_driver_ros2 dashgo_nav_real.launch.py start_hotspot:=true start_d435:=false start_nav_rviz:=false
# 这次不想开机器人热点
ros2 launch dashgo_driver_ros2 dashgo_nav_real.launch.py
```

## 其他启动方式

以下命令默认沿用上面的“常用环境命令”环境。

### XFeat 导航与调试

当前 Dashgo 这边可直接使用的 XFeat 相关命令：

```bash
# Dashgo 真机导航 + XFeat 辅助修正
ros2 launch dashgo_xfeat_bringup dashgo_nav_xfeat_odometry.launch.py

# 只起 D435 + XFeat 原生 RGB-D 里程计
ros2 launch dashgo_xfeat_bringup real_d435_only_xfeat_odometry.launch.py
```

说明：

- `dashgo_nav_xfeat_odometry.launch.py`
  - 底盘 `/odom` 仍然是主里程计
  - `XFeat` 输出 `/xfeat/delta_odom`
  - `odom_fusion_node` 输出 `/localized_odom`
  - 终端状态会打印：
    - `base_only`
    - `fused`
    - `rejected`
- `real_d435_only_xfeat_odometry.launch.py`
  - 只用于单独验证 D435 + XFeat 原生视觉里程计
  - 不带整机导航链路

如果要看融合调试表格，默认位置是：

```bash
/home/xu/xfeat_pose/odom_fusion_debug.csv
```

### 底盘单独启动

```bash
ros2 launch dashgo_driver_ros2 dashgo_robot.launch.py start_lidar:=false start_d435:=false
```

### 雷达单独启动

```bash
ros2 launch dashgo_lidar_ros2 rplidar_s2.launch.py
```

说明：

- 这条“单独启动雷达”现在也支持自动识别雷达串口
- 日常仍然更推荐使用 `dashgo_robot.launch.py` 或 `dashgo_nav_real.launch.py` 做整机启动

### 单独启动手机网页控制

```bash
ros2 launch dashgo_web_control web_control.launch.py
```

### 单独启动手机网页控制，同时开启固定热点

```bash
ros2 launch dashgo_web_control web_control.launch.py start_hotspot:=true
```

### 自定义热点名和密码

```bash
ros2 launch dashgo_driver_ros2 dashgo_nav_real.launch.py \
  start_hotspot:=true \
  hotspot_ssid:=Dashgo-Robot \
  hotspot_password:=dashgo12345
```

如果你换了电脑，或者这块移动硬盘插到另一台机器上，网卡名可能会从 `wlp13s0` 变成别的值。这种情况下通常不用改命令，因为程序会自动找可用 Wi-Fi 网卡。

只有在下面两种情况，才建议手动指定 `hotspot_ifname`：

- 机器上同时有多块无线网卡，自动选择到了错误网卡
- 你明确知道要固定使用某一块无线网卡

手动指定示例：

```bash
ros2 launch dashgo_driver_ros2 dashgo_nav_real.launch.py start_hotspot:=true hotspot_ifname:=wlp13s0
```

## 常用检查命令

```bash
ros2 topic list
ros2 topic echo /scan --once
ros2 topic echo /odom --once
ros2 topic echo /cmd_vel --once
ros2 topic echo /imu_angle --once
ros2 topic echo /imu_angles_raw --once
ros2 topic echo /imu/data --once
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
- `dashgo_driver_ros2` 额外发布底盘 IMU 角度 `/imu_angle`、原始双值 `/imu_angles_raw`，以及标准 IMU `/imu/data`
- 默认会把 IMU 的航向角融合进 `/odom` 的朝向，导航链路直接通过现有 `/odom` 受益
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
- 网页相机默认读取 `/camera/camera/color/image_raw`
- D435 相机本身不是靠固定 `/dev/videoX` 启动，而是由 `realsense2_camera` 自动发现设备

### 二维码弹窗

启动网页控制后，桌面环境下会自动弹出二维码窗口。

- README 默认推荐使用 `start_hotspot:=true`
- 带 `start_hotspot:=true` 启动时，弹窗会显示两个二维码
- 左侧二维码用于手机连接机器人热点
- 右侧二维码用于打开控制网页
- 不带热点启动时，只显示“打开网页”二维码

如果没有图形界面，程序会自动退回到终端打印网页二维码。

### 机器人固定热点说明

当前已经支持由机器人 Ubuntu 22 自己创建固定热点。

- README 默认把固定热点作为推荐启动方式
- 代码默认值仍然是 `start_hotspot:=false`，所以命令里要显式带上 `start_hotspot:=true`
- 启动参数 `start_hotspot:=true` 后，会先自动打开 Wi-Fi，再拉起热点
- 默认会自动选择当前可用的无线网卡，不依赖固定网卡名
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
- `dashgo_nav_real.launch.py` 当前默认 `start_d435:=true`、`start_web_ui:=true`
