"""Tiny dependency-free PNG plotting helpers."""

from __future__ import annotations

import math
from pathlib import Path
import struct
import zlib

import numpy as np


COLORS = [
    (31, 119, 180),
    (214, 39, 40),
    (44, 160, 44),
    (148, 103, 189),
    (255, 127, 14),
    (23, 190, 207),
]


class Canvas:
    def __init__(self, width: int = 1000, height: int = 620) -> None:
        self.width = width
        self.height = height
        self.pixels = bytearray([255] * width * height * 3)

    def set_pixel(self, x: int, y: int, color: tuple[int, int, int]) -> None:
        if 0 <= x < self.width and 0 <= y < self.height:
            idx = (y * self.width + x) * 3
            self.pixels[idx : idx + 3] = bytes(color)

    def line(self, x0: int, y0: int, x1: int, y1: int, color: tuple[int, int, int]) -> None:
        dx = abs(x1 - x0)
        sx = 1 if x0 < x1 else -1
        dy = -abs(y1 - y0)
        sy = 1 if y0 < y1 else -1
        err = dx + dy
        while True:
            self.set_pixel(x0, y0, color)
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x0 += sx
            if e2 <= dx:
                err += dx
                y0 += sy

    def rect(self, x0: int, y0: int, x1: int, y1: int, color: tuple[int, int, int]) -> None:
        self.line(x0, y0, x1, y0, color)
        self.line(x1, y0, x1, y1, color)
        self.line(x1, y1, x0, y1, color)
        self.line(x0, y1, x0, y0, color)

    def save_png(self, path: str | Path) -> None:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        raw = bytearray()
        stride = self.width * 3
        for y in range(self.height):
            raw.append(0)
            start = y * stride
            raw.extend(self.pixels[start : start + stride])

        def chunk(kind: bytes, data: bytes) -> bytes:
            return (
                struct.pack(">I", len(data))
                + kind
                + data
                + struct.pack(">I", zlib.crc32(kind + data) & 0xFFFFFFFF)
            )

        png = bytearray(b"\x89PNG\r\n\x1a\n")
        png.extend(chunk(b"IHDR", struct.pack(">IIBBBBB", self.width, self.height, 8, 2, 0, 0, 0)))
        png.extend(chunk(b"IDAT", zlib.compress(bytes(raw), level=6)))
        png.extend(chunk(b"IEND", b""))
        path.write_bytes(bytes(png))


def plot_lines(
    path: str | Path,
    x: np.ndarray,
    series: list[tuple[str, np.ndarray]],
    width: int = 1000,
    height: int = 620,
) -> None:
    canvas = Canvas(width, height)
    left, right, top, bottom = 70, width - 30, 30, height - 60

    x = np.asarray(x, dtype=float)
    finite_y = np.concatenate([np.asarray(values, dtype=float) for _, values in series])
    finite_y = finite_y[np.isfinite(finite_y)]
    y_min = float(np.min(finite_y))
    y_max = float(np.max(finite_y))
    if math.isclose(y_min, y_max):
        y_min -= 1.0
        y_max += 1.0
    y_pad = 0.08 * (y_max - y_min)
    y_min -= y_pad
    y_max += y_pad

    x_min = float(np.min(x))
    x_max = float(np.max(x))
    if math.isclose(x_min, x_max):
        x_min -= 1.0
        x_max += 1.0

    def map_x(value: float) -> int:
        return int(left + (value - x_min) / (x_max - x_min) * (right - left))

    def map_y(value: float) -> int:
        return int(bottom - (value - y_min) / (y_max - y_min) * (bottom - top))

    grid = (230, 230, 230)
    axis = (60, 60, 60)
    for i in range(6):
        gx = left + i * (right - left) // 5
        canvas.line(gx, top, gx, bottom, grid)
        gy = top + i * (bottom - top) // 5
        canvas.line(left, gy, right, gy, grid)
    if y_min < 0.0 < y_max:
        canvas.line(left, map_y(0.0), right, map_y(0.0), (190, 190, 190))
    canvas.rect(left, top, right, bottom, axis)

    for idx, (_, values) in enumerate(series):
        color = COLORS[idx % len(COLORS)]
        values = np.asarray(values, dtype=float)
        last: tuple[int, int] | None = None
        step = max(1, len(x) // 3000)
        for j in range(0, len(x), step):
            px = map_x(float(x[j]))
            py = map_y(float(values[j]))
            if last is not None:
                canvas.line(last[0], last[1], px, py, color)
            last = (px, py)

    # Minimal legend: colored line swatches in the top-left. Text is documented
    # in the example filenames/console output to keep this plotter tiny.
    for idx, _ in enumerate(series):
        y = top + 12 + idx * 14
        canvas.line(left + 12, y, left + 42, y, COLORS[idx % len(COLORS)])

    canvas.save_png(path)

