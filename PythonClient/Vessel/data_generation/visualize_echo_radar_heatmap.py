"""
Render an echo sensor frame as a radar-style heatmap image.

The output is inspired by a classic radar display:
- dark purple background
- elliptical range boundary
- green sensor origin
- echo returns accumulated as a heatmap
"""

import argparse
import json
import math
from pathlib import Path
from typing import Optional, Tuple

import cv2
import numpy as np


BG_COLOR = (42, 20, 54)
HEADER_COLOR = (246, 246, 246)
SUBTEXT_COLOR = (220, 220, 220)
BOUNDARY_COLOR = (205, 205, 205)
SENSOR_COLOR = (56, 255, 56)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Render an echo frame as a radar-style heatmap."
    )
    parser.add_argument(
        "--dataset-dir",
        required=True,
        help="Capture run directory or a parent directory containing capture runs.",
    )
    parser.add_argument(
        "--frame-id",
        default="000000",
        help="Frame id without extension, for example 000000.",
    )
    parser.add_argument(
        "--output",
        default="",
        help="Optional explicit output file path.",
    )
    parser.add_argument(
        "--width",
        type=int,
        default=1280,
        help="Output image width in pixels.",
    )
    parser.add_argument(
        "--height",
        type=int,
        default=720,
        help="Output image height in pixels.",
    )
    parser.add_argument(
        "--range-meters",
        type=float,
        default=0.0,
        help="Radar half-range in meters. 0 means auto.",
    )
    parser.add_argument(
        "--sigma",
        type=float,
        default=18.0,
        help="Gaussian blur sigma for the heatmap.",
    )
    return parser.parse_args()


def resolve_run_dir(path: Path) -> Path:
    if (path / "echo").is_dir():
        return path

    candidates = [
        subdir for subdir in path.iterdir() if subdir.is_dir() and (subdir / "echo").is_dir()
    ]
    if not candidates:
        raise FileNotFoundError(f"No capture run directory found under {path}")
    return max(candidates, key=lambda item: item.stat().st_mtime)


def load_echo_frame(run_dir: Path, frame_id: str) -> Tuple[np.ndarray, np.ndarray, int]:
    echo_npz = np.load(run_dir / "echo" / f"{frame_id}.npz", allow_pickle=True)
    point_cloud = np.asarray(echo_npz["point_cloud"], dtype=np.float32)
    groundtruth = np.asarray(echo_npz["groundtruth"])
    time_stamp = int(echo_npz["time_stamp"])
    return point_cloud, groundtruth, time_stamp


def load_passive_frame(run_dir: Path, frame_id: str) -> int:
    echo_npz = np.load(run_dir / "echo" / f"{frame_id}.npz", allow_pickle=True)
    passive = np.asarray(echo_npz["passive_beacons_point_cloud"], dtype=np.float32)
    if passive.ndim == 2 and passive.shape[1] >= 9:
        return passive.shape[0]
    return 0


def derive_range_meters(run_dir: Path, frame_id: str, points: np.ndarray, explicit: float) -> float:
    if explicit > 0:
        return explicit

    config_path = run_dir / "meta" / "capture_config.json"
    if config_path.exists():
        config = json.loads(config_path.read_text(encoding="utf-8"))
        try:
            return float(config["echo_config"]["DistanceLimit"])
        except Exception:
            pass

    if points.ndim == 2 and points.shape[1] >= 2 and points.size > 0:
        max_abs_xy = float(np.max(np.abs(points[:, :2])))
        return max(10.0, math.ceil(max_abs_xy * 1.2 / 5.0) * 5.0)

    return 50.0


def filter_echo_points(point_cloud: np.ndarray) -> np.ndarray:
    if point_cloud.ndim != 2 or point_cloud.shape[1] < 6:
        return np.empty((0, 6), dtype=np.float32)

    mask = np.isfinite(point_cloud).all(axis=1)
    mask &= np.linalg.norm(point_cloud[:, :2], axis=1) > 1e-4
    return point_cloud[mask]


def draw_dotted_ellipse(
    image: np.ndarray,
    center: Tuple[int, int],
    axes: Tuple[int, int],
    color: Tuple[int, int, int],
    thickness: int = 1,
    segments: int = 200,
    dash_span: int = 3,
) -> None:
    points = []
    cx, cy = center
    rx, ry = axes
    for idx in range(segments + 1):
        theta = 2 * math.pi * idx / segments
        x = int(round(cx + rx * math.cos(theta)))
        y = int(round(cy + ry * math.sin(theta)))
        points.append((x, y))

    for idx in range(0, segments, dash_span * 2):
        start = points[idx]
        end = points[min(idx + dash_span, segments)]
        cv2.line(image, start, end, color, thickness, cv2.LINE_AA)


def project_point_to_radar(
    x_m: float,
    y_m: float,
    range_m: float,
    center_x: int,
    center_y: int,
    axis_x: int,
    axis_y: int,
) -> Tuple[int, int]:
    px = int(round(center_x + (x_m / range_m) * axis_x))
    py = int(round(center_y + (y_m / range_m) * axis_y))
    return px, py


def build_heatmap(
    echo_points: np.ndarray,
    range_m: float,
    width: int,
    height: int,
    center_x: int,
    center_y: int,
    axis_x: int,
    axis_y: int,
    sigma: float,
) -> np.ndarray:
    heatmap = np.zeros((height, width), dtype=np.float32)
    if echo_points.size == 0:
        return heatmap

    ellipse_width_scale = 17.0
    ellipse_height_scale = 7.0
    range_factor = 0.22

    for row in echo_points:
        x_m = float(row[0])
        y_m = float(row[1])
        px, py = project_point_to_radar(
            x_m=x_m,
            y_m=y_m,
            range_m=range_m,
            center_x=center_x,
            center_y=center_y,
            axis_x=axis_x,
            axis_y=axis_y,
        )
        if not (0 <= px < width and 0 <= py < height):
            continue

        dx = px - center_x
        dy = py - center_y
        distance_px = max(1.0, math.hypot(dx, dy))
        tangent = (-dy, dx)
        angle_deg = math.degrees(math.atan2(tangent[1], tangent[0]))

        major_axis = max(6, int(round(distance_px * range_factor * width / 1280 / ellipse_height_scale)))
        minor_axis = max(4, int(round(distance_px * range_factor * width / 1280 / ellipse_width_scale)))

        cv2.ellipse(
            heatmap,
            center=(px, py),
            axes=(minor_axis, major_axis),
            angle=angle_deg,
            startAngle=0,
            endAngle=360,
            color=1.0,
            thickness=-1,
            lineType=cv2.LINE_AA,
        )

    if np.max(heatmap) > 0:
        heatmap = cv2.GaussianBlur(heatmap, (0, 0), sigmaX=sigma, sigmaY=sigma)
        heatmap /= float(np.max(heatmap))

    return heatmap


def render_heatmap(
    echo_points: np.ndarray,
    active_count: int,
    passive_count: int,
    range_m: float,
    time_stamp: int,
    width: int,
    height: int,
    sigma: float,
) -> np.ndarray:
    image = np.full((height, width, 3), BG_COLOR, dtype=np.uint8)

    top_margin = 28
    left_margin = 30
    radar_center_x = width // 2
    radar_center_y = height // 2 - 8
    radar_axis_x = int(width * 0.45)
    radar_axis_y = int(height * 0.45)

    draw_dotted_ellipse(
        image=image,
        center=(radar_center_x, radar_center_y),
        axes=(radar_axis_x, radar_axis_y),
        color=BOUNDARY_COLOR,
        thickness=1,
        segments=220,
        dash_span=3,
    )

    heatmap = build_heatmap(
        echo_points=echo_points,
        range_m=range_m,
        width=width,
        height=height,
        center_x=radar_center_x,
        center_y=radar_center_y,
        axis_x=radar_axis_x,
        axis_y=radar_axis_y,
        sigma=sigma,
    )
    if np.max(heatmap) > 0:
        heatmap_uint8 = (heatmap * 255.0).astype(np.uint8)
        colored = cv2.applyColorMap(heatmap_uint8, cv2.COLORMAP_TURBO)
        alpha = np.clip(heatmap[..., None] * 0.98, 0.0, 0.98).astype(np.float32)
        image = (image.astype(np.float32) * (1.0 - alpha) + colored.astype(np.float32) * alpha).astype(np.uint8)

    cv2.circle(image, (radar_center_x, radar_center_y), 6, SENSOR_COLOR, -1, cv2.LINE_AA)

    cv2.putText(
        image,
        "echo radar heatmap",
        (left_margin, top_margin),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.9,
        HEADER_COLOR,
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        image,
        f"active={active_count} passive={passive_count}",
        (left_margin, top_margin + 28),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        SUBTEXT_COLOR,
        1,
        cv2.LINE_AA,
    )
    cv2.putText(
        image,
        "Echo Heatmap",
        (left_margin, top_margin + 88),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.5,
        HEADER_COLOR,
        3,
        cv2.LINE_AA,
    )

    return image


def main() -> int:
    args = parse_args()
    run_dir = resolve_run_dir(Path(args.dataset_dir).resolve())
    frame_id = args.frame_id

    point_cloud, groundtruth, time_stamp = load_echo_frame(run_dir, frame_id)
    echo_points = filter_echo_points(point_cloud)
    passive_count = load_passive_frame(run_dir, frame_id)
    range_m = derive_range_meters(run_dir, frame_id, echo_points, args.range_meters)

    image = render_heatmap(
        echo_points=echo_points,
        active_count=echo_points.shape[0],
        passive_count=passive_count,
        range_m=range_m,
        time_stamp=time_stamp,
        width=args.width,
        height=args.height,
        sigma=args.sigma,
    )

    output_path = (
        Path(args.output).resolve()
        if args.output
        else run_dir / f"echo_radar_heatmap_{frame_id}.png"
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(output_path), image)

    print(f"Rendered: {output_path}")
    print(f"active_points={echo_points.shape[0]} passive_points={passive_count}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
