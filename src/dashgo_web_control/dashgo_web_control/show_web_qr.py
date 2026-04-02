#!/usr/bin/env python3

import os
import re
import subprocess

from dashgo_web_control._vendor_qrcodegen import QrCode


def detect_best_host(host):
    if host and host not in ("0.0.0.0", "::", ""):
        return host

    env_host = os.environ.get("DASHGO_WEB_UI_IP")
    if env_host:
        return env_host

    try:
        result = subprocess.run(
            ["ip", "-4", "-o", "addr", "show", "up", "scope", "global"],
            check=False,
            capture_output=True,
            text=True,
        )
    except Exception:
        result = None

    candidates = []
    if result and result.stdout:
        pattern = re.compile(r"^\d+:\s+([^ ]+)\s+inet\s+(\d+\.\d+\.\d+\.\d+)/")
        for line in result.stdout.splitlines():
            match = pattern.match(line.strip())
            if not match:
                continue
            if_name, ip_addr = match.groups()
            if ip_addr.startswith("127."):
                continue
            score = interface_priority(if_name)
            candidates.append((score, if_name, ip_addr))

    if candidates:
        candidates.sort(key=lambda item: (item[0], item[1]))
        return candidates[0][2]

    return "127.0.0.1"


def interface_priority(if_name):
    if if_name.startswith(("wl", "wlan", "ap")):
        return 0
    if if_name.startswith(("en", "eth", "usb")):
        return 1
    return 2


def build_web_url(host, port):
    resolved_host = detect_best_host(host)
    return f"http://{resolved_host}:{port}"


def make_matrix(url):
    qr = QrCode.encode_text(url, QrCode.Ecc.MEDIUM)
    size = qr.get_size()
    return [
        [qr.get_module(x, y) for x in range(size)]
        for y in range(size)
    ]


def render_terminal_qr(matrix):
    quiet_zone = 4
    scale_x = 4
    scale_y = 2
    full_width = len(matrix[0]) + quiet_zone * 2
    white_row = " " * (full_width * scale_x)
    lines = []

    for _ in range(quiet_zone * scale_y):
        lines.append(white_row)

    for row in matrix:
        line = [" " * (quiet_zone * scale_x)]
        for module in row:
            line.append(("█" * scale_x) if module else (" " * scale_x))
        line.append(" " * (quiet_zone * scale_x))
        rendered = "".join(line)
        for _ in range(scale_y):
            lines.append(rendered)

    for _ in range(quiet_zone * scale_y):
        lines.append(white_row)

    return "\n".join(lines)


def print_web_qr(host, port):
    url = build_web_url(host, port)
    matrix = make_matrix(url)
    print("")
    print("=" * 60)
    print(f"Dashgo 手机网页地址: {url}")
    print("手机连上同一个热点后，直接扫描下面二维码即可打开：")
    print(render_terminal_qr(matrix))
    print("=" * 60)
    print("")


def main():
    host = os.environ.get("DASHGO_WEB_UI_HOST", "0.0.0.0")
    port = os.environ.get("DASHGO_WEB_UI_PORT", "8080")
    print_web_qr(host, port)


if __name__ == "__main__":
    main()
