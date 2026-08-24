#!/usr/bin/env python3

import glob
import os
import subprocess


def _read_udev_properties(device_path):
    result = subprocess.run(
        ["udevadm", "info", "-q", "property", "-n", device_path],
        check=False,
        capture_output=True,
        text=True,
    )
    properties = {}
    if result.returncode != 0:
        return properties

    for line in result.stdout.splitlines():
        if "=" not in line:
            continue
        key, value = line.strip().split("=", 1)
        properties[key] = value
    return properties


def _list_serial_candidates():
    candidates = []
    seen_real_paths = set()

    for symlink in sorted(glob.glob("/dev/serial/by-id/*")):
        real_path = os.path.realpath(symlink)
        if not real_path.startswith("/dev/"):
            continue
        seen_real_paths.add(real_path)
        candidates.append(
            {
                "path": symlink,
                "real_path": real_path,
                "properties": _read_udev_properties(real_path),
            }
        )

    for pattern in ("/dev/ttyUSB*", "/dev/ttyACM*"):
        for device_path in sorted(glob.glob(pattern)):
            real_path = os.path.realpath(device_path)
            if real_path in seen_real_paths:
                continue
            seen_real_paths.add(real_path)
            candidates.append(
                {
                    "path": device_path,
                    "real_path": real_path,
                    "properties": _read_udev_properties(real_path),
                }
            )

    return candidates


def _has_by_id_path(candidate):
    return candidate["path"].startswith("/dev/serial/by-id/")


def _score_lidar(candidate):
    props = candidate["properties"]
    score = 1000

    if props.get("ID_VENDOR_ID") == "10c4" and props.get("ID_MODEL_ID") == "ea60":
        score = 0
    elif props.get("ID_USB_DRIVER") == "cp210x":
        score = 10
    elif "CP210" in props.get("ID_MODEL", ""):
        score = 20
    elif "Silicon_Labs" in props.get("ID_VENDOR", ""):
        score = 30

    # 只有匹配到已知设备类型时才给 by-id 路径加分
    if score < 1000 and _has_by_id_path(candidate):
        score -= 1

    return score


def _score_driver(candidate):
    props = candidate["properties"]
    score = 1000

    if props.get("ID_VENDOR_ID") == "1a86" and props.get("ID_MODEL_ID") == "7523":
        score = 0
    elif props.get("ID_USB_DRIVER") == "ch341":
        score = 10
    elif "USB2.0-Serial" in props.get("ID_MODEL", ""):
        score = 20
    elif "QinHeng" in props.get("ID_VENDOR_FROM_DATABASE", ""):
        score = 30

    if score < 1000 and _has_by_id_path(candidate):
        score -= 1

    return score


def _pick_best(candidates, score_fn, excluded_real_paths=None):
    excluded_real_paths = excluded_real_paths or set()
    ranked = []
    for candidate in candidates:
        if candidate["real_path"] in excluded_real_paths:
            continue
        score = score_fn(candidate)
        ranked.append((score, candidate["path"], candidate))

    if not ranked:
        return None

    ranked.sort(key=lambda item: (item[0], item[1]))
    best_score, _best_path, best_candidate = ranked[0]
    if best_score >= 1000:
        return None
    return best_candidate


def resolve_serial_ports():
    candidates = _list_serial_candidates()

    lidar_candidate = _pick_best(candidates, _score_lidar)
    excluded = {lidar_candidate["real_path"]} if lidar_candidate else set()
    driver_candidate = _pick_best(candidates, _score_driver, excluded_real_paths=excluded)

    return {
        "driver_port": driver_candidate["path"] if driver_candidate else "/dev/ttyUSB1",
        "lidar_port": lidar_candidate["path"] if lidar_candidate else "/dev/ttyUSB0",
    }
