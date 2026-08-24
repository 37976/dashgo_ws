# dashgo_rgbd_recorder

D435 RGB-D 数据集录制与导出工具包，用于 ORB-SLAM3 三维场景重建。

## 功能概述

| 功能 | 说明 |
|------|------|
| **在线录制** | 启动 D435 相机 + ros2 bag 录制，保存原始 RGB-D 数据 |
| **Web 远程控制** | 手机扫码遥控机器人，采集时无需跟在机器人旁边 |
| **WiFi 热点** | 自动创建热点，手机连接即可操控 |
| **离线导出** | 从 bag 导出 TUM 格式 RGB-D 数据集（rgb/\*.png + depth/\*.png） |
| **自动元数据** | 提取相机内参、生成 rgb.txt / depth.txt / associate.txt |
| **一键配置** | 根据真实内参自动生成 ORB_SLAM3_D435.yaml |

## 依赖

- ROS2 Humble
- `realsense2_camera`（D435 驱动）
- `dashgo_realsense_ros2`（D435 启动封装）
- `dashgo_web_control`（Web 远程控制 + 热点管理）
- `rosbag2_py`、`cv_bridge`、`python3-opencv`、`python3-yaml`

## 快速开始

### 1. 录制

```bash
# 完整启动：相机 + Web 控制 + 热点 + 录制（默认）
ros2 launch dashgo_rgbd_recorder record_bag.launch.py \
  output_dir:=room_static_01 \
  bag_name:=room_static_01

ros2 run dashgo_rgbd_recorder export_dataset   --bag /home/xu/project/dashgo_ws/room_static_01/raw/room_static_01   --output /home/xu/project/dashgo_ws/room_static_01

# 仅录制，不启动 Web 和热点
ros2 launch dashgo_rgbd_recorder record_bag.launch.py \
  output_dir:=room_static_01 \
  start_web_ui:=false \
  start_hotspot:=false
```

录制过程中请控制机器人**缓慢平稳移动**，包含平移和转弯，**尽量最后回到起点形成闭环**（有利于 SLAM 回环检测）。bag 文件自动保存到 `room_static_01/raw/` 目录。

录制话题：

| 话题 | 内容 |
|------|------|
| `/camera/camera/color/image_raw` | 彩色图像 |
| `/camera/camera/aligned_depth_to_color/image_raw` | 对齐到彩色的深度图 |
| `/camera/camera/color/camera_info` | 相机内参 |

### 2. 离线导出数据集

```bash
# 从 bag 导出 TUM 格式数据集
ros2 run dashgo_rgbd_recorder export_dataset \
  --bag room_static_01/raw/room_static_01 \
  --output room_static_01

# 可选参数
#   --depth-scale 1000.0   深度缩放因子（默认 1000.0，即 mm 单位）
#   --max-diff 0.02         RGB-D 帧关联最大时间差（默认 0.02s）
#   --color-topic ...       彩色话题名（默认 /camera/camera/color/image_raw）
#   --depth-topic ...       深度话题名（默认 /camera/camera/aligned_depth_to_color/image_raw）
```

### 3. 运行 ORB-SLAM3

```bash
cd /path/to/ORB_SLAM3
./Examples/RGB-D/rgbd_tum \
  Vocabulary/ORBvoc.txt \
  room_static_01/ORB_SLAM3_D435.yaml \
  room_static_01 \
  room_static_01/associate.txt
```

## 远程控制采集流程

默认启动时自动开启 Web 面板和 WiFi 热点，方便远程遥控：

1. **手机连接热点**
   - SSID: `Dashgo-Robot`（默认）
   - 密码: `dashgo12345`（默认）

2. **扫码或访问**
   - 机器人屏幕上会自动弹出二维码
   - 或手机浏览器直接访问 `http://<机器人IP>:8080`

3. **遥控采集**
   - Web 面板显示实时摄像头画面
   - 虚拟摇杆控制机器人移动
   - 缓慢移动完成场景录制

4. **结束录制**
   - 终端按 `Ctrl+C` 停止录制
   - bag 文件保存在 `raw/` 目录

### Web 控制参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `start_web_ui` | `true` | 是否启动 Web 远程控制 |
| `start_hotspot` | `true` | 是否启动 WiFi 热点 |
| `web_port` | `8080` | Web 服务端口 |
| `web_image_topic` | `/camera/camera/color/image_raw` | 摄像头预览话题 |
| `web_camera_publish_hz` | `12.0` | 摄像头推流频率 |
| `web_camera_max_width` | `320` | 预览图像最大宽度（降低带宽） |
| `hotspot_ssid` | `Dashgo-Robot` | 热点名称 |
| `hotspot_password` | `dashgo12345` | 热点密码 |

## 输出目录结构

```
room_static_01/
├── raw/
│   └── room_static_01/               # 原始 .db3 bag 文件，长期保留
│
├── rgb/
│   ├── 1710000000.000000.png         # 彩色图 (RGB, 640×480)
│   ├── 1710000000.033333.png
│   └── ...
│
├── depth/
│   ├── 1710000000.000000.png         # 深度图 (uint16 单通道, 单位 mm)
│   ├── 1710000000.033333.png
│   └── ...
│
├── rgb.txt                           # RGB 索引：timestamp filename
├── depth.txt                         # 深度索引：timestamp filename
├── associate.txt                     # RGB-D 配对关系（最近邻匹配）
│
├── camera_intrinsics.txt             # 相机内参 (fx, fy, cx, cy, 畸变系数)
├── recording_config.yaml             # 录制参数（分辨率、FPS、depth_scale 等）
├── ORB_SLAM3_D435.yaml               # ORB-SLAM3 运行配置
│
├── groundtruth.txt                   # 外部定位真值（有则放，没有则不创建）
└── README.txt                        # 数据集说明（需手动补充场景描述）
```

## 深度图说明

- 深度 PNG 为 **uint16 单通道**，像素值除以 `depth_scale`（默认 1000.0）得到以米为单位的真实深度
- D435 有效深度范围约 0.3m ~ 3.5m，超出范围的值可能为 0（无效）
- 深度图已通过 `align_depth.enable=true` 对齐到彩色帧坐标系

## 录制建议

1. **移动要慢**：线速度建议 ≤ 0.3 m/s，角速度建议 ≤ 30°/s
2. **包含平移和转弯**：丰富运动模式有助于 SLAM 初始化
3. **回到起点形成闭环**：有利于回环检测，显著提升地图一致性
4. **场景纹理丰富**：避免大面积白墙、纯色地板，确保足够的视觉特征
5. **光照均匀**：避免强光直射或过暗环境
6. **录制时长**：单个序列建议 2-5 分钟

## 注意事项

- 本工具**不产生外部定位真值**（groundtruth.txt），仅用于真实场景建图和跟踪展示
- 没有真值的情况下**不能计算严格的 APE/RPE**（绝对/相对位姿误差）
- 如需定量评估，需要额外配备动捕系统或激光 tracker 提供 ground truth
- ORB-SLAM3 配置文件中的内参从 `camera_info` 话题**自动提取**，每次录制可能略有差异（D435 内参随温度漂移）
