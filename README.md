# dashgo_ws

基于 ROS 2 Humble 的 Dashgo 真机工作区，主要用于 **激光建图、已有地图上的定位导航，以及定位精度测试**。硬件主链路是 Dashgo 底盘与 RPLIDAR S2；当前导航入口不启动 D435、T265 或 XFeat 视觉里程计。

## 1. 环境与编译

使用 Ubuntu 22.04、ROS 2 Humble，底盘和雷达需要串口访问权限。以下命令按本机工作区路径编写；迁移工作区后请调整路径。

```bash
cd /home/xu/project/dashgo_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

建图需要 `slam_toolbox`，可单独安装：

```bash
sudo apt install ros-humble-slam-toolbox
```

每个新终端运行：

```bash
cd /home/xu/project/dashgo_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOG_DIR=/tmp/roslogs
```

底盘和雷达串口会自动识别，识别有误时可通过 `driver_port:=...`、`lidar_port:=...` 指定 `/dev/serial/by-id/` 路径。串口无权限时执行 `sudo usermod -aG dialout "$USER"`，然后重新登录。

## 2. 激光建图

### 启动

```bash
ros2 launch dashgo_driver_ros2 dashgo_nav_real.launch.py use_slam:=true
```

该模式启动底盘、雷达、`slam_toolbox`、建图桥接、网页控制和 RViz，跳过静态地图服务与 ORB 定位节点。`slam_toolbox` 使用 `/scan` 和里程计 TF，发布 `/slam_map` 与 `map → odom`。

当前建图配置为 0.05 m 栅格、2 s 地图更新周期、12 m 最大激光距离，并开启回环检测。参数在 `src/nav_slam/config/slam_toolbox_params.yaml` 中；修改后重新编译 `nav_slam` 并重启。

### 操作与保存

1. 打开机器人控制网页 `http://<机器人IP>:8080`，进入“建图模式”，点击“开始建图”。
2. 使用建图页面摇杆低速移动，覆盖走廊、转角和房间，并回到走过的区域检查闭环。此时 `slam_controller` 将 `/slam_map` 转发到 `/combined_grid` 供网页显示。
3. 停稳后点击“保存地图”，以保存返回消息中的实际路径为准。
4. 结束建图后用 `Ctrl+C` 停止启动进程，再按下一节加载保存的地图进行定位导航。

也可以在另一个已加载环境的终端操作：

```bash
# 切换到建图显示模式
ros2 topic pub --once /control_mode std_msgs/msg/String "{data: mapping}"

# 保存当前地图，检查返回的 success 与实际路径
ros2 service call /slam_controller/save_map std_srvs/srv/Trigger '{}'
```

**保存行为：** 当前后端固定保存为 `dashgo_slam_map.pgm` 和 `dashgo_slam_map.yaml`，同目录重复保存会覆盖。网页“地图名称”尚未传递到实际保存逻辑。默认保存目录根据运行中的 Python 包路径推导，可能位于源码或安装目录；需要保留多个版本时，将返回路径中的 PGM、YAML 一起复制到独立目录。修改文件名时同步修改 YAML 的 `image` 字段。

网页“开始/停止建图”切换控制模式和地图转发，不会启动或关闭 `slam_toolbox`，也不会清空已有建图结果。必须先用 `use_slam:=true` 启动建图进程。

## 3. 已有地图定位与导航

### 启动

```bash
ros2 launch dashgo_driver_ros2 dashgo_nav_real.launch.py
```

默认加载安装包内的 `nav_slam/map/2dashgo_slam_map.yaml`。使用新建地图时显式指定 YAML 的绝对路径，例如：

```bash
ros2 launch dashgo_driver_ros2 dashgo_nav_real.launch.py \
  nav_map_yaml:=/home/xu/project/dashgo_ws/src/nav_slam/map/dashgo_slam_map.yaml
```

上例适用于地图确实保存在该位置的情况；否则换成保存服务返回位置对应的 YAML 路径。YAML 引用的 PGM 必须存在。

启动后先保持机器人静止，观察初始定位结果，确认 RViz 中机器人位置、朝向及激光与墙体一致，再在网页导航页面“选点导航”并“确认导航”。需要遥控时切换手动模式，需要中断运动时使用暂停。

### 当前定位导航链路

```text
底盘 /odom ── odom_tf_bridge ── odom → base_footprint
雷达 /scan ── scan_to_points ── /scan_filtered
静态地图 YAML + PGM ── static_map_server ── /map

/scan_filtered + 静态地图
  ├─ lidar_global_localize ── 初始匹配 /lidar_global/match_pose
  └─ orb_map_matcher ──────── 持续匹配 /orb/match_pose
                ↓
       map_odom_corrector ── map → odom
                ↓
/odom ── odom_to_map_relay ── /odom_in_map

/scan_filtered ── laser_scan_to_points ── /mapokk
静态地图 + 实时障碍物 ── map_pub ── /combined_grid
/combined_grid + /odom_in_map ── voronoi_node ── /path
/path + /odom_in_map + /combined_grid ── start_nav ── /cmd_vel
```

`map_odom_corrector` 统一发布 `map → odom`，根据初始定位与持续 ORB 观测进行校正，包含观测时间对齐、一致性判断及校正速度限制。当前不再采用“初始定位后永久锁定静态 TF”的说明。

规划采用 Voronoi，跟踪由 `start_nav` 实现，属于自定义导航链路。ORB 在这里用于激光栅格与静态地图匹配，不依赖相机图像。建图模式下 `map → odom` 由 `slam_toolbox` 提供。

### 常用参数

| 参数 | 默认值 | 用途 |
| --- | --- | --- |
| `use_slam` | `false` | `true` 时启用在线建图 |
| `nav_map_yaml` | 包内 `2dashgo_slam_map.yaml` | 定位导航使用的静态地图 |
| `use_global_localize` | `true` | 初始定位和地图坐标校正 |
| `use_continuous_orb` | `true` | 持续 ORB 地图匹配 |
| `orb_match_period_sec` | `2.0` | 持续匹配周期，秒 |
| `orb_required_consistent_matches` | `2` | 校正所需的一致观测数 |
| `use_pointcloud_obstacles` | `true` | 将实时激光障碍物叠加到导航栅格 |
| `use_dynamic_obstacle_points` | `false` | 额外动态障碍物点输入 |
| `goal_relocalization_enabled` | `false` | 到达目标后触发重定位 |
| `map_odom_topic` | `/odom` | TF 与地图校正使用的里程计 |
| `control_odom_topic` | `/odom_in_map` | 地图坐标系下的控制位姿 |
| `start_nav_rviz` | `true` | 启动 RViz |
| `start_web_ui` | `true` | 启动网页控制 |
| `start_hotspot` | `true` | 启动机器人 Wi-Fi 热点 |

```bash
# 使用现有网络，不创建热点；无图形界面时关闭 RViz
ros2 launch dashgo_driver_ros2 dashgo_nav_real.launch.py \
  start_hotspot:=false start_nav_rviz:=false

# 对照测试：保留初始定位，关闭持续 ORB 更新
ros2 launch dashgo_driver_ros2 dashgo_nav_real.launch.py use_continuous_orb:=false

# 查看启动参数
ros2 launch dashgo_driver_ros2 dashgo_nav_real.launch.py --show-args
```

`use_static_map:=false` 仅控制 `map_pub` 是否使用静态底图，**不等于开启 SLAM**，也不会自动关闭静态地图服务或全局定位。建图使用 `use_slam:=true`。

旧入口 `dashgo_xfeat_bringup/dashgo_nav_xfeat_odometry.launch.py` 当前只转发到上述导航入口，不启动视觉融合。单独的 XFeat、RealSense 和 RGB-D 录制工具仍保留在对应包中。

## 4. 测试与验收

### 4.1 话题和 TF 检查

在启动建图或定位导航之后，于另一个终端执行：

```bash
ros2 node list
ros2 topic hz /scan
ros2 topic hz /odom
```

`topic hz` 持续运行，每项检查后按 `Ctrl+C` 再执行下一项。

```bash
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo map base_footprint
```

TF 检查同样按 `Ctrl+C` 结束。定位导航时还应检查：

```bash
ros2 topic echo /odom_in_map --once
ros2 topic echo /orb/match_event --once
ros2 topic echo /control_mode --once
ros2 topic info /combined_grid
ros2 topic info /path
```

建图时检查 `/slam_map`；网页开始建图后检查 `/combined_grid` 是否更新。定位时检查静态地图与激光是否对齐。ORB 匹配事件仅代表匹配节点的观测情况，最终是否采用校正还需结合 `map_odom_corrector` 日志和实际 TF 判断。

### 4.2 瓷砖坐标定位精度测试

现有脚本 `calibration/record_tile_localization_accuracy.py` 采集 `/odom_in_map`，利用标定 YAML 将地图位姿转换为瓷砖坐标，并与手工测量的真实位置、朝向比较。

测试前确认：

- 已在静态地图上完成定位，机器人保持静止。
- 使用与本次地图、物理瓷砖原点及坐标轴一致的标定文件；地图或原点改变后需要重新标定。
- 真实位置以机器人底盘中心为准，输入的是瓷砖编号，朝向单位为度。现有标定的瓷砖边长为 0.604 m，具体以 YAML 为准。

```bash
# 示例：真实位置 (2, 3) 块瓷砖，朝向 90°；请换成实际测量值
python3 calibration/record_tile_localization_accuracy.py 2 3 90 \
  --duration 10 \
  --calibration calibration/tile_origin_20260908_151549.yaml \
  --output calibration/localization_test.csv \
  --note "静止定位，第1轮"
```

也可不传位置参数，按提示交互输入：

```bash
python3 calibration/record_tile_localization_accuracy.py \
  --calibration calibration/tile_origin_20260908_151549.yaml
```

未指定 `--calibration` 时按修改时间选择最新 `tile_origin_*.yaml`；对照测试应显式指定文件，避免误用。标定内 `source_map_yaml` 必须指向存在的地图。脚本会核对 `/map` 的分辨率与原点 x/y，但不比较完整地图内容，仍需自行确认地图版本。

每次运行向 CSV 追加一个测试点，包含位置误差、朝向误差、采样标准差与极差，以及相同标定文件名下累计的位置/角度 MAE、RMSE 和最大误差。未指定输出时保存为 `calibration/orb_tile_localization_evaluation_<标定文件名去扩展名>.csv`。

建议在不同位置、朝向重复测试，分别记录初始定位、行驶后停稳和关闭持续 ORB 的结果，并为不同配置使用不同 CSV。静止采样波动小只说明输出稳定，绝对精度仍要看与真实位置的偏差。仓库已有 CSV 是历史测量记录，不代表本次运行结果。

### 4.3 导航实机测试

| 测试项目 | 操作 | 记录内容 |
| --- | --- | --- |
| 建图闭环 | 绕环境一圈回到起点并保存 | 墙体重影、地图完整性、保存路径及重新加载结果 |
| 初始定位 | 在多个已知位置和朝向重新启动 | 成功/失败、定位耗时、位置与角度误差 |
| 定点导航 | 测试直线、转角、窄通道目标 | 成功率、耗时、终点误差 |
| 障碍物响应 | 在路径前方放置障碍物 | 栅格更新、路径变化、停车或绕行情况 |
| 模式切换 | 导航中暂停，再切换手动 | 是否停止、是否残留运动指令 |
| 重复往返 | 在同一组目标之间多轮行驶 | 定位漂移、失败位置、恢复情况 |

需要保留复现数据时，可在导航期间录制：

```bash
ros2 bag record -o /tmp/dashgo_nav_test \
  /scan /scan_filtered /odom /odom_in_map /tf /tf_static \
  /map /combined_grid /path /cmd_vel /control_mode \
  /lidar_global/match_pose /orb/match_pose /orb/match_event
```

使用未存在的输出目录；结束后按 `Ctrl+C`。建图记录可另外加入 `/slam_map`。同时保存启动命令、地图和标定版本，便于比较结果。

### 4.4 离线回归测试

以下测试不需要启动底盘，但需要已加载 ROS 环境及 Python 依赖。在工作区根目录执行：

```bash
PYTHONPATH="$PWD/src/nav_slam:$PWD/src/kidnapped_robot_finder:$PWD/src/dashgo_xfeat_bringup:$PYTHONPATH" \
python3 -m pytest -q \
  src/nav_slam/test/test_map_odom_corrector_math.py \
  src/nav_slam/test/test_laser_scan_deskew_math.py \
  src/nav_slam/test/test_orb_map_matcher_gate.py \
  src/kidnapped_robot_finder/test/test_kidnap_solver_coordinates.py \
  src/kidnapped_robot_finder/test/test_scanner_unknown_space.py \
  src/dashgo_xfeat_bringup/test/test_imu_yaw_fusion.py
```

覆盖地图校正数学逻辑、激光去畸变插值、ORB 门限、地图坐标转换、未知栅格处理和独立融合模块的 IMU 角度判断。通过这些测试不能替代实机建图与导航验收。

## 5. 常见问题

- **建图页面没有地图：** 确认启动命令带 `use_slam:=true`，收到 `/slam_map`，并已点击“开始建图”进入 `mapping` 模式。
- **地图已保存但导航还是旧地图：** 默认加载的是 `2dashgo_slam_map.yaml`；保存新图不会自动替换导航地图，请重启并显式传 `nav_map_yaml`。
- **定位错误或激光与墙体不对齐：** 核对地图版本、初始匹配结果、雷达安装 TF；重复结构和特征稀疏区域可能出现歧义。
- **没有 `/odom` 或雷达掉线：** 检查串口权限、自动识别结果及 USB 供电，必要时手动指定设备路径。
- **网页相机为空：** 当前导航入口不启动相机，不影响激光建图与定位导航。
- **不需要热点：** 传 `start_hotspot:=false`。默认热点为 `Dashgo-Robot`，密码为 `dashgo12345`，可用 `hotspot_ssid`、`hotspot_password` 修改；热点依赖 NetworkManager 的 `nmcli`。
- **RViz 看不到激光：** 将 LaserScan 的 Reliability 设为 Best Effort；地图显示使用 `map` 坐标系并确认相关 TF 存在。

## 6. 主要代码位置

| 路径 | 内容 |
| --- | --- |
| `src/dashgo_driver_ros2/launch/dashgo_nav_real.launch.py` | 建图与定位导航统一入口 |
| `src/nav_slam/launch/slam_mapping.launch.py` | slam_toolbox 建图子系统 |
| `src/nav_slam/config/slam_toolbox_params.yaml` | 建图参数 |
| `src/nav_slam/nav_slam/slam_controller.py` | 地图转发与保存服务 |
| `src/nav_slam/nav_slam/lidar_global_localize.py` | 初始全局定位 |
| `src/nav_slam/nav_slam/orb_map_matcher.py` | 持续 ORB 匹配 |
| `src/nav_slam/nav_slam/map_odom_corrector.py` | map → odom 统一校正 |
| `src/nav_slam/nav_slam/map_pub.py` | 静态地图与实时障碍物合成 |
| `src/nav2_voronoi_planner` | Voronoi 路径规划 |
| `src/nav_slam/nav_slam/start_nav.py` | 路径跟踪控制 |
| `src/dashgo_web_control` | 网页建图、遥控与导航 |
| `calibration` | 瓷砖坐标标定、定位精度采集脚本与历史结果 |
