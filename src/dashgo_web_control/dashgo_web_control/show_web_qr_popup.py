#!/usr/bin/env python3

import os
import sys
import tkinter as tk
from pathlib import Path

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from dashgo_web_control.show_web_qr import build_web_url, make_matrix, print_web_qr


def show_popup(host, port):
    url = build_web_url(host, port)
    matrix = make_matrix(url)

    quiet_zone = 4
    scale = 11
    qr_size = len(matrix)
    canvas_size = (qr_size + quiet_zone * 2) * scale

    root = tk.Tk()
    root.title("Dashgo 手机网页二维码")
    root.configure(bg="white")
    root.attributes("-topmost", True)
    root.resizable(False, False)

    title = tk.Label(
        root,
        text="手机连上同一个热点后，扫描二维码打开网页",
        bg="white",
        fg="#10263f",
        font=("DejaVu Sans", 14, "bold"),
        padx=18,
        pady=12,
    )
    title.pack()

    url_label = tk.Label(
        root,
        text=url,
        bg="white",
        fg="#0f76d6",
        font=("DejaVu Sans Mono", 13),
        padx=18,
        pady=4,
    )
    url_label.pack()

    canvas = tk.Canvas(
        root,
        width=canvas_size,
        height=canvas_size,
        bg="white",
        highlightthickness=0,
        bd=0,
    )
    canvas.pack(padx=18, pady=12)

    for y, row in enumerate(matrix):
        for x, module in enumerate(row):
            if not module:
                continue
            left = (x + quiet_zone) * scale
            top = (y + quiet_zone) * scale
            canvas.create_rectangle(
                left,
                top,
                left + scale,
                top + scale,
                fill="black",
                outline="black",
            )

    close_button = tk.Button(
        root,
        text="关闭",
        command=root.destroy,
        bg="#0f76d6",
        fg="white",
        activebackground="#0c66b9",
        activeforeground="white",
        relief="flat",
        padx=18,
        pady=8,
        font=("DejaVu Sans", 12, "bold"),
    )
    close_button.pack(pady=(0, 18))

    root.update_idletasks()
    width = root.winfo_width()
    height = root.winfo_height()
    x = max((root.winfo_screenwidth() - width) // 2, 0)
    y = max((root.winfo_screenheight() - height) // 5, 0)
    root.geometry(f"{width}x{height}+{x}+{y}")
    root.mainloop()


def main():
    host = os.environ.get("DASHGO_WEB_UI_HOST", "0.0.0.0")
    port = os.environ.get("DASHGO_WEB_UI_PORT", "8080")

    if not os.environ.get("DISPLAY"):
        print_web_qr(host, port)
        return

    try:
        show_popup(host, port)
    except tk.TclError as exc:
        print(f"QR popup failed, falling back to terminal output: {exc}")
        print_web_qr(host, port)


if __name__ == "__main__":
    main()
