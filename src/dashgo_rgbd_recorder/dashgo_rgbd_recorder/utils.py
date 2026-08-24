"""
dashgo_rgbd_recorder 工具函数
- 时间戳格式化与提取
- 相机内参提取
- 文件 I/O 辅助
"""

import os
from typing import Dict, Optional


def stamp_to_sec(stamp) -> float:
    """将 ROS2 Time 消息 (builtin_interfaces.msg.Time) 转为浮点秒。"""
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def stamp_to_tum_string(stamp) -> str:
    """
    TUM RGB-D 格式的时间戳：`seconds.nanoseconds` 保留小数点后 6 位。
    例如 1710000000.033333
    """
    # 纳秒截断到微秒（6 位），TUM 格式要求
    return f"{stamp.sec}.{stamp.nanosec // 1000:06d}"


def format_tum_line(timestamp_str: str, filepath: str) -> str:
    """生成 TUM 格式的索引行：`timestamp path`。"""
    return f"{timestamp_str} {filepath}"


def extract_intrinsics(camera_info_msg) -> Dict:
    """
    从 sensor_msgs/CameraInfo 提取内参信息。

    返回:
        dict: {
            'fx': float, 'fy': float, 'cx': float, 'cy': float,
            'k1': float, 'k2': float, 'p1': float, 'p2': float, 'k3': float,
            'width': int, 'height': int
        }
    """
    k = camera_info_msg.k  # [fx, 0, cx, 0, fy, cy, 0, 0, 1]
    d = camera_info_msg.d  # [k1, k2, p1, p2, k3] or empty

    intrinsics = {
        'fx': k[0],
        'fy': k[4],
        'cx': k[2],
        'cy': k[5],
        'k1': d[0] if len(d) > 0 else 0.0,
        'k2': d[1] if len(d) > 1 else 0.0,
        'p1': d[2] if len(d) > 2 else 0.0,
        'p2': d[3] if len(d) > 3 else 0.0,
        'k3': d[4] if len(d) > 4 else 0.0,
        'width': camera_info_msg.width,
        'height': camera_info_msg.height,
    }
    return intrinsics


def intrinsics_to_string(intrinsics: Dict) -> str:
    """将内参字典格式化为可读字符串。"""
    lines = [
        f"fx = {intrinsics['fx']}",
        f"fy = {intrinsics['fy']}",
        f"cx = {intrinsics['cx']}",
        f"cy = {intrinsics['cy']}",
        f"k1 = {intrinsics['k1']}",
        f"k2 = {intrinsics['k2']}",
        f"p1 = {intrinsics['p1']}",
        f"p2 = {intrinsics['p2']}",
        f"k3 = {intrinsics['k3']}",
        f"width = {intrinsics['width']}",
        f"height = {intrinsics['height']}",
    ]
    return "\n".join(lines) + "\n"


def ensure_dir(path: str) -> None:
    """确保目录存在，不存在则创建。"""
    os.makedirs(path, exist_ok=True)


def associate_frames(
    rgb_stamps: list,
    depth_stamps: list,
    max_diff: float = 0.02,
) -> list:
    """
    基于最近时间戳匹配 RGB 和深度帧，生成 TUM associate.txt 内容。

    参数:
        rgb_stamps: [(ts_float, ts_str, filepath), ...]
        depth_stamps: [(ts_float, ts_str, filepath), ...]
        max_diff: 最大允许时间差（秒）

    返回:
        [(rgb_ts_str, depth_ts_str), ...]  匹配对列表
    """
    pairs = []
    di = 0
    for r_float, r_str, _ in rgb_stamps:
        # 在深度帧中从上次匹配位置开始搜索最近邻
        best_di = di
        best_diff = float('inf')
        for j in range(di, len(depth_stamps)):
            diff = abs(r_float - depth_stamps[j][0])
            if diff < best_diff:
                best_diff = diff
                best_di = j
            else:
                # 时间差重新增大，后续只会更大，停止搜索
                break
        if best_diff <= max_diff:
            pairs.append((r_str, depth_stamps[best_di][1]))
            di = best_di
    return pairs


def parse_stamp_from_filename(filename: str) -> Optional[float]:
    """
    从 TUM 格式文件名中解析时间戳，例如 '1710000000.033333.png'。
    返回浮点秒数，无法解析则返回 None。
    """
    try:
        name = os.path.splitext(os.path.basename(filename))[0]
        return float(name)
    except (ValueError, TypeError):
        return None
