#!/usr/bin/env python3
"""
离线导出 ROS2 bag → TUM RGB-D 数据集。

从 ros2 bag (.db3) 中读取 D435 录制的 color/depth/camera_info 话题，
逐帧导出 rgb/*.png + depth/*.png (uint16)，生成 TUM 格式的索引文件、
相机内参文件、ORB-SLAM3 YAML 和 README.txt。

用法:
  # 从 bag 导出（bag 将被复制到 output/raw/）
  ros2 run dashgo_rgbd_recorder export_dataset \
    --bag room_static_01/raw/room_static_01.db3 \
    --output room_static_01

  # 自定义深度缩放因子
  ros2 run dashgo_rgbd_recorder export_dataset \
    --bag room_static_01/raw/room_static_01.db3 \
    --output room_static_01 \
    --depth-scale 1000.0

  # 指定话题名（若 bag 中话题非默认值）
  ros2 run dashgo_rgbd_recorder export_dataset \
    --bag room_static_01/raw/room_static_01.db3 \
    --output room_static_01 \
    --color-topic /camera/camera/color/image_raw \
    --depth-topic /camera/camera/aligned_depth_to_color/image_raw
"""

import argparse
import os
import shutil
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml

# ROS2
import rclpy
from rclpy.serialization import deserialize_message

from rosbag2_py import (
    SequentialReader,
    StorageOptions,
    ConverterOptions,
)

from cv_bridge import CvBridge

from sensor_msgs.msg import CameraInfo, Image

# 包内工具
from dashgo_rgbd_recorder.utils import (
    stamp_to_sec,
    stamp_to_tum_string,
    format_tum_line,
    extract_intrinsics,
    intrinsics_to_string,
    associate_frames,
    ensure_dir,
)


def read_bag_metadata(bag_path: str):
    """
    读取 bag 中所有话题的类型映射和总消息数。

    返回:
        type_map: {topic_name: topic_type}
        topic_count: {topic_name: count}
    """
    reader = SequentialReader()
    storage_options = StorageOptions(uri=bag_path, storage_id="sqlite3")
    converter_options = ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader.open(storage_options, converter_options)

    type_map = {}
    for tm in reader.get_all_topics_and_types():
        type_map[tm.name] = tm.type

    # 统计每个话题的消息数
    topic_count = {}
    while reader.has_next():
        topic, _, _ = reader.read_next()
        topic_count[topic] = topic_count.get(topic, 0) + 1

    return type_map, topic_count


def export_dataset(
    bag_path: str,
    output_dir: str,
    depth_scale: float = 1000.0,
    color_topic: str = "/camera/camera/color/image_raw",
    depth_topic: str = "/camera/camera/aligned_depth_to_color/image_raw",
    camera_info_topic: str = "/camera/camera/color/camera_info",
    max_timestamp_diff: float = 0.02,
):
    """
    核心导出逻辑。

    参数:
        bag_path: ROS2 .db3 bag 文件路径
        output_dir: 数据集输出根目录
        depth_scale: 深度缩放因子 (像素值 / depth_scale = 米)
        color_topic: bag 中的彩色图像话题
        depth_topic: bag 中的深度图像话题
        camera_info_topic: bag 中的相机信息话题
        max_timestamp_diff: RGB-D 关联最大时间差 (秒)
    """
    # ---- 0. 检查输入 ----
    _bag_path = bag_path  # 保存原始路径，用于后续复制

    # ros2 bag 是目录（metadata.yaml + .db3），也可能直接传入 .db3 文件
    if os.path.isdir(bag_path) and os.path.isfile(os.path.join(bag_path, "metadata.yaml")):
        pass  # 目录格式的 bag，rosbag2_py 直接支持
    elif os.path.isfile(bag_path) and bag_path.endswith(".db3"):
        pass  # 单个 .db3 文件
    else:
        print(f"[ERROR] 无效的 bag 路径: {bag_path}")
        print("  ros2 bag 通常是一个目录（内含 metadata.yaml + .db3 文件）")
        print("  示例: ros2 run dashgo_rgbd_recorder export_dataset --bag room_static_01/raw/room_static_01 --output room_static_01")
        sys.exit(1)

    # ---- 1. 创建输出目录 ----
    rgb_dir = os.path.join(output_dir, "rgb")
    depth_dir = os.path.join(output_dir, "depth")
    raw_dir = os.path.join(output_dir, "raw")
    ensure_dir(rgb_dir)
    ensure_dir(depth_dir)
    ensure_dir(raw_dir)

    # ---- 2. 复制 bag 到 raw/ ----
    bag_basename = os.path.basename(os.path.normpath(bag_path))
    dst_bag_name = bag_basename  # 保持原始名称，不强行改后缀
    dst_bag_path = os.path.join(raw_dir, dst_bag_name)

    if os.path.isdir(bag_path):
        if os.path.exists(dst_bag_path):
            print(f"[WARN] 目标已存在，跳过复制: {dst_bag_path}")
        else:
            print(f"[INFO] 复制 bag {bag_path} -> {dst_bag_path}")
            shutil.copytree(bag_path, dst_bag_path)
    else:
        if os.path.exists(dst_bag_path):
            print(f"[WARN] 目标已存在，跳过复制: {dst_bag_path}")
        else:
            print(f"[INFO] 复制 bag {bag_path} -> {dst_bag_path}")
            os.makedirs(dst_bag_path, exist_ok=True)
            shutil.copy2(bag_path, os.path.join(dst_bag_path, os.path.basename(bag_path)))

    # ---- 3. 打开 bag 读取 ----
    print(f"[INFO] 打开 bag: {bag_path}")
    reader = SequentialReader()
    storage_options = StorageOptions(uri=bag_path, storage_id="sqlite3")
    converter_options = ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader.open(storage_options, converter_options)

    # 获取话题类型
    type_map = {}
    for tm in reader.get_all_topics_and_types():
        type_map[tm.name] = tm.type
        print(f"  [INFO] 发现话题: {tm.name} ({tm.type})")

    # 验证话题存在
    for topic, name in [(color_topic, "Color"), (depth_topic, "Depth"), (camera_info_topic, "CameraInfo")]:
        if topic not in type_map:
            print(f"[WARN] bag 中未找到 {name} 话题: {topic}")

    # ---- 4. 遍览消息 ----
    bridge = CvBridge()

    camera_info_msg = None
    rgb_frames = []    # [(ts_float, ts_str, filepath_relative), ...]
    depth_frames = []  # [(ts_float, ts_str, filepath_relative), ...]
    color_count = 0
    depth_count = 0
    total = 0

    print("[INFO] 开始逐帧导出...")
    while reader.has_next():
        topic, data, t = reader.read_next()
        total += 1

        # -- CameraInfo --
        if topic == camera_info_topic:
            if camera_info_msg is None:
                camera_info_msg = deserialize_message(data, CameraInfo)
                intrinsics = extract_intrinsics(camera_info_msg)
                print(f"[INFO] 获取到 CameraInfo: "
                      f"fx={intrinsics['fx']:.3f}, fy={intrinsics['fy']:.3f}, "
                      f"{intrinsics['width']}x{intrinsics['height']}")

        # -- Color Image --
        elif topic == color_topic:
            try:
                img_msg = deserialize_message(data, Image)
                ts_float = stamp_to_sec(img_msg.header.stamp)
                ts_str = stamp_to_tum_string(img_msg.header.stamp)
            except Exception as e:
                print(f"[WARN] 反序列化 color 消息失败: {e}")
                continue

            try:
                cv_image = bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
            except Exception as e:
                print(f"[WARN] cv_bridge 转换 color 失败: {e}")
                continue

            # 保存为 RGB PNG
            rgb_filename = f"{ts_str}.png"
            rgb_path = os.path.join(rgb_dir, rgb_filename)
            cv2.imwrite(rgb_path, cv_image)  # BGR -> 仍存为 BGR...

            # 改为存 RGB：cv2.imwrite 按 BGR 解释数组，所以先转 RGB
            # Actually cv2.imwrite expects BGR, so if we save BGR array it will look correct.
            # But standard TUM datasets use RGB PNG. Let's convert.
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            cv2.imwrite(rgb_path, rgb_image)

            rgb_frames.append((ts_float, ts_str, f"rgb/{rgb_filename}"))
            color_count += 1

        # -- Depth Image --
        elif topic == depth_topic:
            try:
                img_msg = deserialize_message(data, Image)
                ts_float = stamp_to_sec(img_msg.header.stamp)
                ts_str = stamp_to_tum_string(img_msg.header.stamp)
            except Exception as e:
                print(f"[WARN] 反序列化 depth 消息失败: {e}")
                continue

            try:
                # 保持 uint16 原始编码
                cv_depth = bridge.imgmsg_to_cv2(img_msg, desired_encoding="passthrough")
            except Exception as e:
                print(f"[WARN] cv_bridge 转换 depth 失败: {e}")
                continue

            # 确保是 uint16 单通道
            if cv_depth.dtype != np.uint16:
                print(f"[WARN] 深度图非 uint16 (实际: {cv_depth.dtype})，尝试转换...")
                cv_depth = cv_depth.astype(np.uint16)

            depth_filename = f"{ts_str}.png"
            depth_path = os.path.join(depth_dir, depth_filename)
            cv2.imwrite(depth_path, cv_depth)

            depth_frames.append((ts_float, ts_str, f"depth/{depth_filename}"))
            depth_count += 1

        # 进度提示
        if total % 500 == 0:
            print(f"  ... 已处理 {total} 条消息, {color_count} color / {depth_count} depth")

    print(f"[INFO] 导出完成: {color_count} 张彩色图, {depth_count} 张深度图 (共 {total} 条消息)")

    # ---- 5. 生成索引文件 ----
    print("[INFO] 生成索引文件...")

    # rgb.txt
    rgb_txt_path = os.path.join(output_dir, "rgb.txt")
    with open(rgb_txt_path, "w") as f:
        f.write("# RGB images\n")
        f.write("# timestamp filename\n")
        for _, ts_str, rel_path in rgb_frames:
            f.write(format_tum_line(ts_str, rel_path) + "\n")
    print(f"  [OK] {rgb_txt_path} ({len(rgb_frames)} 行)")

    # depth.txt
    depth_txt_path = os.path.join(output_dir, "depth.txt")
    with open(depth_txt_path, "w") as f:
        f.write("# Depth images\n")
        f.write("# timestamp filename\n")
        for _, ts_str, rel_path in depth_frames:
            f.write(format_tum_line(ts_str, rel_path) + "\n")
    print(f"  [OK] {depth_txt_path} ({len(depth_frames)} 行)")

    # associate.txt
    associate_txt_path = os.path.join(output_dir, "associate.txt")
    pairs = associate_frames(
        [(ts, s, p) for ts, s, p in rgb_frames],
        [(ts, s, p) for ts, s, p in depth_frames],
        max_diff=max_timestamp_diff,
    )
    with open(associate_txt_path, "w") as f:
        for rgb_ts, depth_ts in pairs:
            f.write(f"{rgb_ts} rgb/{rgb_ts}.png {depth_ts} depth/{depth_ts}.png\n")
    print(f"  [OK] {associate_txt_path} ({len(pairs)} 对)")

    # ---- 6. 写入相机内参文件 ----
    if camera_info_msg is None:
        print("[WARN] 未获取到 CameraInfo，使用 D435 默认标定值")
        intrinsics = {
            'fx': 606.056, 'fy': 605.041,
            'cx': 329.167, 'cy': 235.982,
            'k1': 0.0, 'k2': 0.0, 'p1': 0.0, 'p2': 0.0, 'k3': 0.0,
            'width': 640, 'height': 480,
        }

    intrinsics_path = os.path.join(output_dir, "camera_intrinsics.txt")
    with open(intrinsics_path, "w") as f:
        f.write("# Camera intrinsics extracted from bag CameraInfo\n")
        f.write(intrinsics_to_string(intrinsics))
    print(f"  [OK] {intrinsics_path}")

    # ---- 7. 复制/更新 recording_config.yaml ----
    # 如果录制时已经复制过，则更新 depth_scale 等实际信息
    config_path = os.path.join(output_dir, "recording_config.yaml")
    recording_config = {
        "camera_model": "Intel RealSense D435",
        "color_width": intrinsics['width'],
        "color_height": intrinsics['height'],
        "depth_width": intrinsics['width'],
        "depth_height": intrinsics['height'],
        "fps": 30,
        "depth_scale": depth_scale,
        "depth_unit": "mm",
        "align_depth": True,
        "color_topic": color_topic,
        "depth_topic": depth_topic,
        "camera_info_topic": camera_info_topic,
        "color_frames": color_count,
        "depth_frames": depth_count,
        "associated_pairs": len(pairs),
        "source_bag": dst_bag_name,
        "notes": "录制时移动要慢、平稳，包含平移和转弯，尽量回到起点形成闭环。",
    }
    with open(config_path, "w") as f:
        yaml.dump(recording_config, f, default_flow_style=False, allow_unicode=True)
    print(f"  [OK] {config_path}")

    # ---- 8. 生成 ORB_SLAM3_D435.yaml ----
    pkg_share = os.environ.get(
        "AMENT_PREFIX_PATH", ""
    ).split(":")[0]
    # 尝试从包安装路径查找模板
    template_paths = [
        os.path.join(
            os.path.dirname(__file__), "..", "..", "config", "ORB_SLAM3_D435_template.yaml"
        ),
        # 安装后的路径
        os.path.join(
            pkg_share, "share", "dashgo_rgbd_recorder", "config", "ORB_SLAM3_D435_template.yaml"
        ),
    ]
    template_path = None
    for tp in template_paths:
        if os.path.exists(os.path.normpath(tp)):
            template_path = os.path.normpath(tp)
            break

    orb_yaml_path = os.path.join(output_dir, "ORB_SLAM3_D435.yaml")
    if template_path and os.path.exists(template_path):
        with open(template_path, "r") as f:
            yaml_content = f.read()

        # 替换模板占位符
        yaml_content = yaml_content.replace("{{FX}}", f"{intrinsics['fx']:.6f}")
        yaml_content = yaml_content.replace("{{FY}}", f"{intrinsics['fy']:.6f}")
        yaml_content = yaml_content.replace("{{CX}}", f"{intrinsics['cx']:.6f}")
        yaml_content = yaml_content.replace("{{CY}}", f"{intrinsics['cy']:.6f}")
        yaml_content = yaml_content.replace("{{WIDTH}}", str(intrinsics['width']))
        yaml_content = yaml_content.replace("{{HEIGHT}}", str(intrinsics['height']))
        yaml_content = yaml_content.replace("{{FPS}}", str(recording_config['fps']))
        yaml_content = yaml_content.replace("{{DEPTH_SCALE}}", f"{depth_scale:.1f}")

        with open(orb_yaml_path, "w") as f:
            f.write(yaml_content)
        print(f"  [OK] {orb_yaml_path}")
    else:
        print("[WARN] 未找到 ORB_SLAM3 模板，跳过 YAML 生成")

    # ---- 9. 生成 README.txt ----
    readme_path = os.path.join(output_dir, "README.txt")
    readme_content = f"""============================================
RGB-D 数据集 — {os.path.basename(output_dir)}
============================================

相机: Intel RealSense D435
分辨率: {intrinsics['width']}x{intrinsics['height']}
FPS: {recording_config['fps']}
深度缩放因子: {depth_scale} (像素值 / {depth_scale} = 米)
彩色帧数: {color_count}
深度帧数: {depth_count}
关联帧对: {len(pairs)}

相机内参:
  fx={intrinsics['fx']:.3f}, fy={intrinsics['fy']:.3f}
  cx={intrinsics['cx']:.3f}, cy={intrinsics['cy']:.3f}
  k1={intrinsics['k1']}, k2={intrinsics['k2']}, p1={intrinsics['p1']}, p2={intrinsics['p2']}, k3={intrinsics['k3']}

原始 bag: raw/{dst_bag_name}

== 拍摄记录 ==
场景描述: [请填写：房间类型、面积、主要物体等]
拍摄路线: [请填写：起点、行走路径、是否回到起点]
动态物体: [请填写：是否有行人、移动的椅子等]
光照条件: [请填写：室内灯光/自然光、是否均匀]
备注: [请填写其他注意事项]

== ORB-SLAM3 运行方式 ==
  cd /path/to/ORB_SLAM3
  ./Examples/RGB-D/rgbd_tum \\
      Vocabulary/ORBvoc.txt \\
      {os.path.basename(output_dir)}/ORB_SLAM3_D435.yaml \\
      {os.path.basename(output_dir)} \\
      {os.path.basename(output_dir)}/associate.txt

== 注意事项 ==
- 本数据集没有外部定位真值（groundtruth.txt），只能做真实场景建图和跟踪展示
- 不能计算严格的 APE/RPE
- 深度 PNG 为 uint16 单通道，值除以 {depth_scale} 得到米
- RGB PNG 为 3 通道 RGB
"""
    with open(readme_path, "w") as f:
        f.write(readme_content)
    print(f"  [OK] {readme_path}")

    # ---- 10. 总结 ----
    print()
    print("=" * 60)
    print("导出完成！数据集目录结构：")
    print(f"  {output_dir}/")
    print(f"  ├── raw/{dst_bag_name}")
    print(f"  ├── rgb/          ({color_count} 张)")
    print(f"  ├── depth/        ({depth_count} 张)")
    print(f"  ├── rgb.txt       ({len(rgb_frames)} 行)")
    print(f"  ├── depth.txt     ({len(depth_frames)} 行)")
    print(f"  ├── associate.txt ({len(pairs)} 对)")
    print(f"  ├── camera_intrinsics.txt")
    print(f"  ├── recording_config.yaml")
    print(f"  ├── ORB_SLAM3_D435.yaml")
    print(f"  └── README.txt")
    print("=" * 60)


def main():
    parser = argparse.ArgumentParser(
        description="从 ROS2 bag (.db3) 导出 TUM RGB-D 数据集"
    )
    parser.add_argument(
        "--bag", "-b",
        required=True,
        help="ROS2 .db3 bag 路径（目录或文件）",
    )
    parser.add_argument(
        "--output", "-o",
        required=True,
        help="数据集输出目录",
    )
    parser.add_argument(
        "--depth-scale",
        type=float,
        default=1000.0,
        help="深度缩放因子，像素值 / depth_scale = 米（D435 默认 1000.0）",
    )
    parser.add_argument(
        "--color-topic",
        default="/camera/camera/color/image_raw",
        help="bag 中的彩色图像话题名",
    )
    parser.add_argument(
        "--depth-topic",
        default="/camera/camera/aligned_depth_to_color/image_raw",
        help="bag 中的深度图像话题名",
    )
    parser.add_argument(
        "--camera-info-topic",
        default="/camera/camera/color/camera_info",
        help="bag 中的相机信息话题名",
    )
    parser.add_argument(
        "--max-diff",
        type=float,
        default=0.02,
        help="RGB-D 关联最大时间差（秒，默认 0.02）",
    )

    args = parser.parse_args()

    # rosbag2_py 需要初始化 rclpy（虽然是离线工具）
    rclpy.init(args=sys.argv)

    try:
        export_dataset(
            bag_path=args.bag,
            output_dir=args.output,
            depth_scale=args.depth_scale,
            color_topic=args.color_topic,
            depth_topic=args.depth_topic,
            camera_info_topic=args.camera_info_topic,
            max_timestamp_diff=args.max_diff,
        )
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
