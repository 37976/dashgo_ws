#!/usr/bin/env python3
"""Record one tile-coordinate localization measurement and append it to CSV."""

import argparse
import csv
import glob
import json
import math
import os
import statistics
import sys
import time
from datetime import datetime

import rclpy
import yaml
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


FIELDS = [
    "timestamp", "calibration_file", "map_yaml",
    "tile_x_index", "tile_y_index", "tile_size_m",
    "gt_x_m", "gt_y_m", "gt_yaw_deg",
    "sample_topic", "sample_duration_sec", "sample_count",
    "map_x_mean_m", "map_y_mean_m", "map_yaw_mean_deg",
    "estimated_tile_x_m", "estimated_tile_y_m", "estimated_tile_yaw_deg",
    "error_x_m", "error_y_m", "position_error_m",
    "position_error_tiles", "yaw_error_deg",
    "x_std_m", "y_std_m", "yaw_std_deg",
    "x_range_m", "y_range_m", "yaw_range_deg",
    "orb_reference_f1",
    "cumulative_point_count", "cumulative_position_mae_m",
    "cumulative_position_rmse_m", "cumulative_position_max_m",
    "cumulative_yaw_mae_deg", "cumulative_yaw_rmse_deg",
    "cumulative_yaw_max_deg", "note",
]


def wrap_rad(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def wrap_deg(angle):
    return math.degrees(wrap_rad(math.radians(angle)))


def quaternion_to_yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def circular_mean(values):
    return math.atan2(
        statistics.fmean(math.sin(value) for value in values),
        statistics.fmean(math.cos(value) for value in values),
    )


def map_pose_to_tile(map_x, map_y, map_yaw, calibration):
    origin = calibration["map_to_tile_origin"]
    origin_x = float(origin["x_m"])
    origin_y = float(origin["y_m"])
    origin_yaw = float(origin["yaw_rad"])
    dx = map_x - origin_x
    dy = map_y - origin_y
    cos_yaw = math.cos(origin_yaw)
    sin_yaw = math.sin(origin_yaw)
    tile_x = cos_yaw * dx + sin_yaw * dy
    tile_y = -sin_yaw * dx + cos_yaw * dy
    tile_yaw = wrap_rad(map_yaw - origin_yaw)
    return tile_x, tile_y, tile_yaw


def latest_calibration(script_dir):
    candidates = glob.glob(os.path.join(script_dir, "tile_origin_*.yaml"))
    if not candidates:
        raise FileNotFoundError(f"未在 {script_dir} 找到 tile_origin_*.yaml")
    return max(candidates, key=os.path.getmtime)


def load_yaml(path):
    with open(path, "r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def ask_number(value, prompt):
    if value is not None:
        return value
    return float(input(prompt).strip())


class PoseCollector(Node):
    def __init__(self, pose_topic):
        super().__init__("tile_localization_accuracy_recorder")
        self.samples = []
        self.map_msg = None
        self.orb_reference_f1 = None
        self.create_subscription(Odometry, pose_topic, self.pose_callback, 50)
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(OccupancyGrid, "/map", self.map_callback, map_qos)
        self.create_subscription(String, "/orb/match_event", self.event_callback, 10)

    def pose_callback(self, msg):
        pose = msg.pose.pose
        self.samples.append((
            float(pose.position.x),
            float(pose.position.y),
            quaternion_to_yaw(pose.orientation),
        ))

    def map_callback(self, msg):
        self.map_msg = msg

    def event_callback(self, msg):
        try:
            event = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        candidates = [event.get("stationary_reference_f1")]
        if event.get("status") == "matched":
            candidates.append(event.get("f1"))
        for value in candidates:
            if value is None:
                continue
            value = float(value)
            if self.orb_reference_f1 is None or value > self.orb_reference_f1:
                self.orb_reference_f1 = value


def validate_active_map(map_msg, map_yaml_path):
    metadata = load_yaml(map_yaml_path)
    expected_origin = metadata["origin"]
    checks = [
        ("resolution", float(map_msg.info.resolution), float(metadata["resolution"]), 1e-6),
        ("origin.x", float(map_msg.info.origin.position.x), float(expected_origin[0]), 1e-5),
        ("origin.y", float(map_msg.info.origin.position.y), float(expected_origin[1]), 1e-5),
    ]
    mismatches = [
        f"{name}: 当前={actual}, 标定={expected}"
        for name, actual, expected, tolerance in checks
        if abs(actual - expected) > tolerance
    ]
    if mismatches:
        raise RuntimeError(
            "当前 /map 与标定所用地图不一致，已停止记录：\n  "
            + "\n  ".join(mismatches)
        )


def existing_errors(csv_path, calibration_name):
    position_errors = []
    yaw_errors = []
    if not os.path.exists(csv_path) or os.path.getsize(csv_path) == 0:
        return position_errors, yaw_errors
    with open(csv_path, "r", encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames != FIELDS:
            raise RuntimeError(f"已有 CSV 表头与当前脚本不一致：{csv_path}")
        for row in reader:
            if row.get("calibration_file") != calibration_name:
                continue
            position_errors.append(float(row["position_error_m"]))
            yaw_errors.append(abs(float(row["yaw_error_deg"])))
    return position_errors, yaw_errors


def cumulative_metrics(position_errors, yaw_errors):
    return {
        "cumulative_point_count": len(position_errors),
        "cumulative_position_mae_m": statistics.fmean(position_errors),
        "cumulative_position_rmse_m": math.sqrt(
            statistics.fmean(error * error for error in position_errors)
        ),
        "cumulative_position_max_m": max(position_errors),
        "cumulative_yaw_mae_deg": statistics.fmean(yaw_errors),
        "cumulative_yaw_rmse_deg": math.sqrt(
            statistics.fmean(error * error for error in yaw_errors)
        ),
        "cumulative_yaw_max_deg": max(yaw_errors),
    }


def parse_args():
    parser = argparse.ArgumentParser(
        description="采集稳定定位位姿，转换为瓷砖坐标并追加误差 CSV。"
    )
    parser.add_argument("tile_x", nargs="?", type=float, help="真实瓷砖 x 编号")
    parser.add_argument("tile_y", nargs="?", type=float, help="真实瓷砖 y 编号")
    parser.add_argument("yaw_deg", nargs="?", type=float, help="真实朝向角（度）")
    parser.add_argument("--duration", type=float, default=10.0, help="采样秒数，默认 10")
    parser.add_argument("--topic", default="/odom_in_map", help="采样位姿话题")
    parser.add_argument("--calibration", help="标定 YAML；默认使用 calibration 中最新文件")
    parser.add_argument("--output", help="输出 CSV；默认与标定文件同目录")
    parser.add_argument("--note", default="", help="本次记录备注")
    return parser.parse_args()


def main():
    args = parse_args()
    if args.duration <= 0.0:
        raise ValueError("--duration 必须大于 0")

    script_dir = os.path.dirname(os.path.abspath(__file__))
    calibration_path = os.path.abspath(args.calibration or latest_calibration(script_dir))
    calibration = load_yaml(calibration_path)
    map_yaml_path = os.path.abspath(os.path.expanduser(calibration["source_map_yaml"]))
    if not os.path.exists(map_yaml_path):
        raise FileNotFoundError(f"标定引用的地图不存在：{map_yaml_path}")

    tile_size = float(calibration.get("tile_size_m", 0.604))
    tile_x_index = ask_number(args.tile_x, "当前真实瓷砖 x 编号：")
    tile_y_index = ask_number(args.tile_y, "当前真实瓷砖 y 编号：")
    gt_yaw_deg = ask_number(args.yaw_deg, "当前真实朝向（度）：")
    gt_x = tile_x_index * tile_size
    gt_y = tile_y_index * tile_size

    calibration_stem = os.path.splitext(os.path.basename(calibration_path))[0]
    output_path = os.path.abspath(
        args.output
        or os.path.join(
            script_dir,
            f"orb_tile_localization_evaluation_{calibration_stem}.csv",
        )
    )

    print(f"使用标定：{calibration_path}")
    print(f"对应地图：{map_yaml_path}")
    print(
        f"真实位置：tile=({tile_x_index:g}, {tile_y_index:g}), "
        f"meter=({gt_x:.3f}, {gt_y:.3f}), yaw={gt_yaw_deg:g}°"
    )
    print(f"采集 {args.topic} {args.duration:g} 秒，请保持机器人静止……")

    rclpy.init()
    node = PoseCollector(args.topic)
    try:
        deadline = time.monotonic() + args.duration
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if not node.samples:
        raise RuntimeError(f"未收到 {args.topic}，请确认实机定位已经启动")
    if node.map_msg is None:
        raise RuntimeError("未收到 /map，无法确认当前地图与标定地图是否一致")
    validate_active_map(node.map_msg, map_yaml_path)

    xs = [sample[0] for sample in node.samples]
    ys = [sample[1] for sample in node.samples]
    yaws = [sample[2] for sample in node.samples]
    map_x = statistics.fmean(xs)
    map_y = statistics.fmean(ys)
    map_yaw = circular_mean(yaws)
    yaw_spread = [math.degrees(wrap_rad(yaw - map_yaw)) for yaw in yaws]
    tile_x, tile_y, tile_yaw = map_pose_to_tile(map_x, map_y, map_yaw, calibration)
    estimated_yaw_deg = math.degrees(tile_yaw)

    error_x = tile_x - gt_x
    error_y = tile_y - gt_y
    position_error = math.hypot(error_x, error_y)
    yaw_error = wrap_deg(estimated_yaw_deg - gt_yaw_deg)

    calibration_name = os.path.basename(calibration_path)
    previous_position_errors, previous_yaw_errors = existing_errors(
        output_path, calibration_name
    )
    metrics = cumulative_metrics(
        previous_position_errors + [position_error],
        previous_yaw_errors + [abs(yaw_error)],
    )

    row = {
        "timestamp": datetime.now().astimezone().isoformat(timespec="seconds"),
        "calibration_file": calibration_name,
        "map_yaml": map_yaml_path,
        "tile_x_index": tile_x_index,
        "tile_y_index": tile_y_index,
        "tile_size_m": tile_size,
        "gt_x_m": gt_x,
        "gt_y_m": gt_y,
        "gt_yaw_deg": gt_yaw_deg,
        "sample_topic": args.topic,
        "sample_duration_sec": args.duration,
        "sample_count": len(node.samples),
        "map_x_mean_m": map_x,
        "map_y_mean_m": map_y,
        "map_yaw_mean_deg": math.degrees(map_yaw),
        "estimated_tile_x_m": tile_x,
        "estimated_tile_y_m": tile_y,
        "estimated_tile_yaw_deg": estimated_yaw_deg,
        "error_x_m": error_x,
        "error_y_m": error_y,
        "position_error_m": position_error,
        "position_error_tiles": position_error / tile_size,
        "yaw_error_deg": yaw_error,
        "x_std_m": statistics.pstdev(xs),
        "y_std_m": statistics.pstdev(ys),
        "yaw_std_deg": statistics.pstdev(yaw_spread),
        "x_range_m": max(xs) - min(xs),
        "y_range_m": max(ys) - min(ys),
        "yaw_range_deg": max(yaw_spread) - min(yaw_spread),
        "orb_reference_f1": (
            "" if node.orb_reference_f1 is None else node.orb_reference_f1
        ),
        **metrics,
        "note": args.note,
    }

    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    needs_header = not os.path.exists(output_path) or os.path.getsize(output_path) == 0
    with open(output_path, "a", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDS, lineterminator="\n")
        if needs_header:
            writer.writeheader()
        writer.writerow(row)

    print("\n记录完成")
    print(
        f"定位结果：tile=({tile_x:.3f}, {tile_y:.3f}), "
        f"yaw={estimated_yaw_deg:.2f}°"
    )
    print(
        f"本点误差：dx={error_x:+.3f} m, dy={error_y:+.3f} m, "
        f"位置={position_error:.3f} m ({position_error / tile_size:.3f} 块), "
        f"角度={yaw_error:+.2f}°"
    )
    print(
        f"累计 {metrics['cumulative_point_count']} 点："
        f"位置 RMSE={metrics['cumulative_position_rmse_m']:.3f} m，"
        f"角度 RMSE={metrics['cumulative_yaw_rmse_deg']:.2f}°"
    )
    print(f"表格：{output_path}")


if __name__ == "__main__":
    try:
        main()
    except (FileNotFoundError, KeyError, RuntimeError, ValueError) as exc:
        print(f"错误：{exc}", file=sys.stderr)
        raise SystemExit(1)
