#!/usr/bin/env python3

import os
import re
import subprocess
from pathlib import Path


QR_VERSION = 3
QR_SIZE = 29
QR_DATA_CODEWORDS = 55
QR_ECC_CODEWORDS = 15
QR_MASK = 0
QR_FORMAT_BITS = 0b111011111000100


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


def gf_multiply(x, y):
    result = 0
    while y:
      if y & 1:
        result ^= x
      x <<= 1
      if x & 0x100:
        x ^= 0x11D
      y >>= 1
    return result


def gf_pow(x, power):
    result = 1
    for _ in range(power):
        result = gf_multiply(result, x)
    return result


def reed_solomon_generator(degree):
    result = [1]
    for i in range(degree):
        result = polynomial_multiply(result, [1, gf_pow(2, i)])
    return result


def polynomial_multiply(poly_a, poly_b):
    result = [0] * (len(poly_a) + len(poly_b) - 1)
    for i, value_a in enumerate(poly_a):
        for j, value_b in enumerate(poly_b):
            result[i + j] ^= gf_multiply(value_a, value_b)
    return result


def reed_solomon_remainder(data, degree):
    generator = reed_solomon_generator(degree)
    remainder = [0] * degree
    for byte in data:
        factor = byte ^ remainder[0]
        remainder = remainder[1:] + [0]
        for i in range(degree):
            remainder[i] ^= gf_multiply(generator[i + 1], factor)
    return remainder


def append_bits(buffer, value, bit_count):
    for shift in range(bit_count - 1, -1, -1):
        buffer.append((value >> shift) & 1)


def encode_url_to_codewords(url):
    data = url.encode("utf-8")
    if len(data) > 53:
        raise ValueError("URL too long for built-in QR generator")

    bits = []
    append_bits(bits, 0b0100, 4)
    append_bits(bits, len(data), 8)
    for byte in data:
        append_bits(bits, byte, 8)

    capacity = QR_DATA_CODEWORDS * 8
    append_bits(bits, 0, min(4, capacity - len(bits)))
    while len(bits) % 8 != 0:
        bits.append(0)

    codewords = []
    for i in range(0, len(bits), 8):
        value = 0
        for bit in bits[i:i + 8]:
            value = (value << 1) | bit
        codewords.append(value)

    padding = [0xEC, 0x11]
    while len(codewords) < QR_DATA_CODEWORDS:
        codewords.append(padding[(len(codewords) - len(bits) // 8) % 2])

    return codewords + reed_solomon_remainder(codewords, QR_ECC_CODEWORDS)


def make_matrix(url):
    matrix = [[False] * QR_SIZE for _ in range(QR_SIZE)]
    reserved = [[False] * QR_SIZE for _ in range(QR_SIZE)]

    draw_finder(matrix, reserved, 0, 0)
    draw_finder(matrix, reserved, QR_SIZE - 7, 0)
    draw_finder(matrix, reserved, 0, QR_SIZE - 7)
    draw_alignment(matrix, reserved, 22, 22)
    draw_timing(matrix, reserved)
    reserve_format_areas(reserved)

    matrix[QR_SIZE - 8][8] = True
    reserved[QR_SIZE - 8][8] = True

    codewords = encode_url_to_codewords(url)
    data_bits = []
    for byte in codewords:
        append_bits(data_bits, byte, 8)
    place_data_bits(matrix, reserved, data_bits)
    draw_format_bits(matrix)
    return matrix


def draw_finder(matrix, reserved, left, top):
    for dy in range(-1, 8):
        for dx in range(-1, 8):
            x = left + dx
            y = top + dy
            if not (0 <= x < QR_SIZE and 0 <= y < QR_SIZE):
                continue
            reserved[y][x] = True
            if 0 <= dx <= 6 and 0 <= dy <= 6:
                dark = (
                    dx in (0, 6) or
                    dy in (0, 6) or
                    (2 <= dx <= 4 and 2 <= dy <= 4)
                )
                matrix[y][x] = dark
            else:
                matrix[y][x] = False


def draw_alignment(matrix, reserved, center_x, center_y):
    for dy in range(-2, 3):
        for dx in range(-2, 3):
            x = center_x + dx
            y = center_y + dy
            reserved[y][x] = True
            matrix[y][x] = max(abs(dx), abs(dy)) != 1


def draw_timing(matrix, reserved):
    for i in range(8, QR_SIZE - 8):
        value = i % 2 == 0
        if not reserved[6][i]:
            matrix[6][i] = value
            reserved[6][i] = True
        if not reserved[i][6]:
            matrix[i][6] = value
            reserved[i][6] = True


def reserve_format_areas(reserved):
    format_positions = [
        (8, 0), (8, 1), (8, 2), (8, 3), (8, 4), (8, 5), (8, 7),
        (8, 8), (7, 8), (5, 8), (4, 8), (3, 8), (2, 8), (1, 8), (0, 8),
        (QR_SIZE - 1, 8), (QR_SIZE - 2, 8), (QR_SIZE - 3, 8), (QR_SIZE - 4, 8),
        (QR_SIZE - 5, 8), (QR_SIZE - 6, 8), (QR_SIZE - 7, 8), (QR_SIZE - 8, 8),
        (8, QR_SIZE - 7), (8, QR_SIZE - 6), (8, QR_SIZE - 5), (8, QR_SIZE - 4),
        (8, QR_SIZE - 3), (8, QR_SIZE - 2), (8, QR_SIZE - 1),
    ]
    for x, y in format_positions:
        reserved[y][x] = True


def place_data_bits(matrix, reserved, data_bits):
    bit_index = 0
    direction = -1
    x = QR_SIZE - 1
    y = QR_SIZE - 1

    while x > 0:
        if x == 6:
            x -= 1
        for _ in range(QR_SIZE):
            for column in (x, x - 1):
                if not reserved[y][column]:
                    bit = data_bits[bit_index] if bit_index < len(data_bits) else 0
                    if (y + column) % 2 == 0:
                        bit ^= 1
                    matrix[y][column] = bool(bit)
                    bit_index += 1
            y += direction
            if y < 0 or y >= QR_SIZE:
                y -= direction
                direction *= -1
                break
        x -= 2


def draw_format_bits(matrix):
    bits = [(QR_FORMAT_BITS >> i) & 1 for i in range(14, -1, -1)]
    primary = [
        (8, 0), (8, 1), (8, 2), (8, 3), (8, 4), (8, 5), (8, 7),
        (8, 8), (7, 8), (5, 8), (4, 8), (3, 8), (2, 8), (1, 8), (0, 8),
    ]
    secondary = [
        (QR_SIZE - 1, 8), (QR_SIZE - 2, 8), (QR_SIZE - 3, 8), (QR_SIZE - 4, 8),
        (QR_SIZE - 5, 8), (QR_SIZE - 6, 8), (QR_SIZE - 7, 8), (QR_SIZE - 8, 8),
        (8, QR_SIZE - 7), (8, QR_SIZE - 6), (8, QR_SIZE - 5), (8, QR_SIZE - 4),
        (8, QR_SIZE - 3), (8, QR_SIZE - 2), (8, QR_SIZE - 1),
    ]
    for bit, (x, y) in zip(bits, primary):
        matrix[y][x] = bool(bit)
    for bit, (x, y) in zip(bits, secondary):
        matrix[y][x] = bool(bit)


def render_terminal_qr(matrix):
    quiet_zone = 4
    full_size = QR_SIZE + quiet_zone * 2
    lines = []
    white_row = "  " * full_size
    for _ in range(quiet_zone):
        lines.append(white_row)
    for row in matrix:
        line = ["  "] * quiet_zone
        for module in row:
            line.append("██" if module else "  ")
        line.extend(["  "] * quiet_zone)
        lines.append("".join(line))
    for _ in range(quiet_zone):
        lines.append(white_row)
    return "\n".join(lines)


def write_svg(matrix, url):
    output_path = Path("/tmp/dashgo_web_control_qr.svg")
    quiet_zone = 4
    size = QR_SIZE + quiet_zone * 2
    rects = []
    for y, row in enumerate(matrix):
        for x, module in enumerate(row):
            if module:
                rects.append(
                    f'<rect x="{x + quiet_zone}" y="{y + quiet_zone}" width="1" height="1" />'
                )
    svg = (
        f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {size} {size}" '
        f'shape-rendering="crispEdges">\n'
        f'  <rect width="{size}" height="{size}" fill="#ffffff" />\n'
        f'  <g fill="#000000">\n    ' + "\n    ".join(rects) + '\n  </g>\n'
        f'  <desc>{url}</desc>\n'
        '</svg>\n'
    )
    output_path.write_text(svg, encoding="utf-8")
    return str(output_path)


def print_web_qr(host, port):
    url = build_web_url(host, port)
    matrix = make_matrix(url)
    svg_path = write_svg(matrix, url)
    print("")
    print("=" * 60)
    print(f"Dashgo 手机网页地址: {url}")
    print("手机连上同一个热点后，直接扫描下面二维码即可打开：")
    print(render_terminal_qr(matrix))
    print(f"二维码 SVG 已保存到: {svg_path}")
    print("=" * 60)
    print("")


def main():
    host = os.environ.get("DASHGO_WEB_UI_HOST", "0.0.0.0")
    port = os.environ.get("DASHGO_WEB_UI_PORT", "8080")
    print_web_qr(host, port)

