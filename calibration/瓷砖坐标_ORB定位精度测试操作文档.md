# 瓷砖坐标 ORB 定位精度测试操作文档

更新日期：2026-09-08

## 1. 测试目的

本流程用于测试机器人在新静态地图中的 ORB 全局定位精度。

测试时，将机器人放在已知的瓷砖坐标和朝向上。记录脚本会采集稳定后的定位位姿，将其转换为瓷砖坐标，并自动计算位置误差、角度误差、波动指标和累计误差指标。

本次使用：

- 静态地图：`2dashgo_slam_map.yaml`
- 原点标定：`tile_origin_20260908_151549.yaml`
- 瓷砖边长：`0.604 m`
- 位姿话题：`/odom_in_map`
- 默认采样时间：`10 秒`

## 2. 坐标约定

瓷砖坐标以已经标定的物理位置为 `(0,0)`。

- 机器人车头朝瓷砖 `+x` 方向时，角度为 `0°`
- 机器人左侧方向为 `+y`
- 机器人右侧方向为 `-y`
- 逆时针旋转为正角度
- 顺时针旋转为负角度
- `+x` 方向一块瓷砖为 `+0.604 m`
- `+y` 方向一块瓷砖为 `+0.604 m`

常用角度：

| 角度 | 车头方向 |
|---:|---|
| `0°` | `+x` |
| `90°` | `+y` |
| `-90°` | `-y` |
| `180°` 或 `-180°` | `-x` |

放置机器人时，必须始终使用机器人底盘中心作为测量点，并让底盘中心对准相应的瓷砖缝交点。不要一部分点使用瓷砖中心、另一部分点使用瓷砖缝交点，否则会引入约半块瓷砖的固定误差。

## 3. 启动前检查

1. 确认机器人已开机，底盘和激光雷达已连接。
2. 确认周围没有人员持续走动，也没有临时物体挡住主要墙面。
3. 确认当前要进行的是定位测试，不是建图。不要同时启动建图模式。
4. 如果要通过机器人热点操作：
   - 热点名：`Dashgo-Robot`
   - 密码：`dashgo12345`

## 4. 启动新地图定位

打开一个终端，执行：

```bash
source /opt/ros/humble/setup.bash
source /home/xu/project/dashgo_ws/install/setup.bash

ros2 launch dashgo_driver_ros2 dashgo_nav_real.launch.py \
  use_slam:=false \
  use_global_localize:=true \
  use_continuous_orb:=true \
  start_hotspot:=true \
  nav_map_yaml:=/home/xu/project/dashgo_ws/install/nav_slam/share/nav_slam/map/2dashgo_slam_map.yaml
```

启动后不要关闭这个终端。

正常情况下应能看到以下节点：

- `static_map_server`
- `lidar_global_localize`
- `orb_map_matcher`
- `map_odom_corrector`
- `odom_to_map_relay`

如需检查，可在另一个终端执行：

```bash
source /opt/ros/humble/setup.bash
source /home/xu/project/dashgo_ws/install/setup.bash
ros2 node list
```

## 5. 放置机器人并等待定位稳定

1. 将机器人移动到准备测试的瓷砖坐标。
2. 将机器人车头对准准备测试的真实角度。
3. 停止触碰机器人，保持完全静止。
4. 等待 ORB 完成定位，建议至少等待 `10～20 秒`。
5. 观察启动终端中的 ORB 输出。出现“匹配成功”后，后续较低 F1 被静止门控拒绝属于正常现象，系统会保持已经接受的最高 F1 位姿。

也可以查看当前全局位姿是否存在：

```bash
source /opt/ros/humble/setup.bash
source /home/xu/project/dashgo_ws/install/setup.bash
ros2 topic echo --once /odom_in_map
```

只有机器人已经静止、ORB 已完成定位时，才能运行记录脚本。

## 6. 运行精度记录脚本

### 6.1 交互输入方式

打开另一个终端，执行：

```bash
source /opt/ros/humble/setup.bash
source /home/xu/project/dashgo_ws/install/setup.bash

python3 /home/xu/project/dashgo_ws/calibration/record_tile_localization_accuracy.py
```

脚本会依次询问：

```text
当前真实瓷砖 x 编号：
当前真实瓷砖 y 编号：
当前真实朝向（度）：
```

例如，机器人位于瓷砖坐标 `(1,-2)`，车头为 `-90°`，依次输入：

```text
1
-2
-90
```

输入完成后，脚本会采集 `10 秒`。采样期间不要移动机器人。

### 6.2 直接输入方式

也可以把坐标直接写在命令后面：

```bash
python3 /home/xu/project/dashgo_ws/calibration/record_tile_localization_accuracy.py 1 -2 -90
```

三个数字的顺序固定为：

```text
瓷砖x编号  瓷砖y编号  真实角度
```

### 6.3 添加备注

例如记录“第二次重复测量”：

```bash
python3 /home/xu/project/dashgo_ws/calibration/record_tile_localization_accuracy.py \
  1 -2 -90 \
  --note "第二次重复测量"
```

### 6.4 修改采样时间

默认采样 `10 秒`。如需采样 `20 秒`：

```bash
python3 /home/xu/project/dashgo_ws/calibration/record_tile_localization_accuracy.py \
  1 -2 -90 \
  --duration 20
```

## 7. 查看本次结果

每次测量结束后，终端会直接显示：

- 换算后的瓷砖定位坐标
- x 方向误差
- y 方向误差
- 平面位置误差
- 相当于多少块瓷砖
- 航向误差
- 当前累计位置 RMSE
- 当前累计角度 RMSE

输出示例：

```text
记录完成
定位结果：tile=(0.701, -1.065), yaw=-90.91°
本点误差：dx=+0.097 m, dy=+0.143 m, 位置=0.173 m (0.287 块), 角度=-0.91°
累计 3 点：位置 RMSE=0.139 m，角度 RMSE=0.99°
```

## 8. 表格文件

正式结果会自动追加到：

```text
/home/xu/project/dashgo_ws/calibration/orb_tile_localization_evaluation_tile_origin_20260908_151549.csv
```

该文件可以直接用 LibreOffice Calc 或 Excel 打开。脚本采用 UTF-8 BOM 编码，中文备注可以正常显示。

每运行一次脚本，就追加一行，不会覆盖之前的测试点。

### 8.1 单点误差字段

| 字段 | 含义 |
|---|---|
| `error_x_m` | x 方向定位误差 |
| `error_y_m` | y 方向定位误差 |
| `position_error_m` | 平面位置误差 |
| `position_error_tiles` | 位置误差相当于多少块瓷砖 |
| `yaw_error_deg` | 航向误差 |
| `orb_reference_f1` | 静止门控保留的 ORB 参考 F1 |

### 8.2 波动指标

| 字段 | 含义 |
|---|---|
| `x_std_m` | 采样期间 x 的标准差 |
| `y_std_m` | 采样期间 y 的标准差 |
| `yaw_std_deg` | 采样期间角度标准差 |
| `x_range_m` | 采样期间 x 最大值与最小值之差 |
| `y_range_m` | 采样期间 y 最大值与最小值之差 |
| `yaw_range_deg` | 采样期间角度最大波动范围 |

波动小只表示定位输出稳定，不代表绝对定位误差一定小。绝对精度要看 `position_error_m` 和 `yaw_error_deg`。

### 8.3 累计误差指标

| 字段 | 含义 |
|---|---|
| `cumulative_position_mae_m` | 所有已测点的位置平均绝对误差 |
| `cumulative_position_rmse_m` | 所有已测点的位置均方根误差 |
| `cumulative_position_max_m` | 所有已测点的最大位置误差 |
| `cumulative_yaw_mae_deg` | 所有已测点的角度平均绝对误差 |
| `cumulative_yaw_rmse_deg` | 所有已测点的角度均方根误差 |
| `cumulative_yaw_max_deg` | 所有已测点的最大绝对角度误差 |

## 9. 推荐测试顺序

为了让连续 ORB 的局部搜索能够跟上位置变化，建议每次只移动到相邻或较近的瓷砖，不要一次把机器人搬到很远的位置。

可以按下面的顺序复测：

| 测试顺序 | 瓷砖 x | 瓷砖 y | 真实角度 |
|---:|---:|---:|---:|
| 1 | 0 | 0 | 0° |
| 2 | 1 | -1 | 0° |
| 3 | 0 | -2 | 0° |
| 4 | 1 | -2 | -90° |
| 5 | 1 | -3 | -90° |

注意：`(0,0,0°)` 是建立坐标转换所用的标定参考点，在该点测得的误差不能单独代表系统泛化精度。判断新地图是否改善，应重点比较其他瓷砖位置，并尽量增加不同距离和不同角度的测试点。

每个位置的完整操作循环：

1. 移动机器人到下一个真实瓷砖坐标。
2. 对准真实角度。
3. 保持静止。
4. 等待 ORB 定位稳定。
5. 运行一次记录脚本。
6. 查看终端结果。
7. 再移动到下一个测试点。

建议每个坐标重复测量 `3 次`，这样可以区分偶然波动和固定偏差。

## 10. 新建一份独立测试表

如果不想继续追加到默认表格，可以指定新的输出文件：

```bash
python3 /home/xu/project/dashgo_ws/calibration/record_tile_localization_accuracy.py \
  1 -2 -90 \
  --output /home/xu/project/dashgo_ws/calibration/orb_accuracy_repeat_01.csv
```

同一份 CSV 必须始终使用同一个标定文件和同一种表头格式。

## 11. 常见错误处理

### 错误：未收到 `/odom_in_map`

原因：定位程序未启动，或全局位姿还没有发布。

处理：

1. 确认启动定位的终端仍在运行。
2. 执行 `ros2 node list`，检查定位节点。
3. 等待 ORB 首次定位成功后重新运行脚本。

### 错误：未收到 `/map`

原因：静态地图服务器没有正常启动。

处理：停止当前程序，重新执行第 4 节的新地图定位命令。

### 错误：当前地图与标定所用地图不一致

原因：当前加载了旧地图，或使用了错误的原点标定文件。

处理：重新使用第 4 节命令，明确指定：

```text
2dashgo_slam_map.yaml
```

不要绕过地图一致性检查，否则计算出的瓷砖误差没有意义。

### ORB 一直匹配失败或 F1 很低

处理：

1. 确认雷达没有被人、纸箱或桌布遮挡。
2. 确认机器人所在区域已完整包含在新地图中。
3. 保持机器人静止，多等待几轮。
4. 如果机器人一次被搬动超过约一米，先重新启动定位，让系统重新进行初始全局定位。

### 表格表头不一致

原因：把新脚本的输出指定到了昨天的旧格式 CSV。

处理：使用脚本默认输出文件，或者通过 `--output` 指定一个不存在的新 CSV 文件。

## 12. 结束测试

完成全部测试后：

1. 确认最后一次脚本已经显示“记录完成”。
2. 打开 CSV 检查记录行数。
3. 回到启动定位的终端。
4. 按 `Ctrl+C` 停止实机导航程序。

不要在记录脚本仍显示“采集 10 秒”时关闭定位程序。
