"""
Render a 1x4 horizontal view for RGB, segmentation, LiDAR and echo.
"""

import argparse
from pathlib import Path
from typing import Tuple

import cv2
import numpy as np

from visualize_echo_radar_heatmap import (
    derive_range_meters,
    filter_echo_points,
    load_echo_frame,
    load_passive_frame,
    render_heatmap,
    resolve_run_dir,
)
from visualize_lidar_echo import (
    colorize_values,
    filter_lidar_points,
    render_panel,
)


BG_COLOR = (255, 255, 255)
PANEL_BG_COLOR = (255, 255, 255)
HEADER_STRIP_COLOR = (248, 248, 248)
TEXT_COLOR = (32, 32, 32)
SUBTEXT_COLOR = (96, 96, 96)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Render RGB, segmentation, LiDAR and echo into a single 2x2 image."
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
        help="Optional explicit output image path.",
    )
    parser.add_argument(
        "--panel-size",
        type=int,
        default=520,
        help="Square panel size in pixels.",
    )
    parser.add_argument(
        "--gap",
        type=int,
        default=28,
        help="Gap between panels in pixels.",
    )
    parser.add_argument(
        "--margin",
        type=int,
        default=28,
        help="Outer margin in pixels.",
    )
    parser.add_argument(
        "--header-height",
        type=int,
        default=88,
        help="Header height in pixels.",
    )
    parser.add_argument(
        "--range-meters",
        type=float,
        default=0.0,
        help="Shared BEV half-range for LiDAR and echo panels. 0 means auto.",
    )
    return parser.parse_args()


def load_image(path: Path) -> np.ndarray:
    image = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if image is None:
        raise FileNotFoundError(f"Failed to read image: {path}")
    return image


def fit_image_to_square(image: np.ndarray, panel_size: int) -> np.ndarray:
    canvas = np.full((panel_size, panel_size, 3), PANEL_BG_COLOR, dtype=np.uint8)
    height, width = image.shape[:2]
    scale = min(panel_size / width, panel_size / height)
    resized = cv2.resize(
        image,
        (int(round(width * scale)), int(round(height * scale))),
        interpolation=cv2.INTER_AREA,
    )
    y = (panel_size - resized.shape[0]) // 2
    x = (panel_size - resized.shape[1]) // 2
    canvas[y : y + resized.shape[0], x : x + resized.shape[1]] = resized
    return canvas


def decorate_image_panel(
    image: np.ndarray,
    title: str,
    subtitle: str,
    panel_size: int,
) -> np.ndarray:
    panel = fit_image_to_square(image, panel_size)
    overlay = panel.copy()
    cv2.rectangle(overlay, (0, 0), (panel_size, 88), HEADER_STRIP_COLOR, -1)
    panel = cv2.addWeighted(overlay, 0.9, panel, 0.1, 0)

    cv2.putText(
        panel,
        title,
        (24, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.0,
        TEXT_COLOR,
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        panel,
        subtitle,
        (24, 70),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.58,
        SUBTEXT_COLOR,
        1,
        cv2.LINE_AA,
    )
    return panel


def build_lidar_panel(
    run_dir: Path,
    frame_id: str,
    panel_size: int,
    range_meters: float,
) -> np.ndarray:
    lidar_npz = np.load(run_dir / "lidar" / f"{frame_id}.npz", allow_pickle=True)
    lidar_points, lidar_distances = filter_lidar_points(
        np.asarray(lidar_npz["point_cloud"], dtype=np.float32)
    )
    lidar_colors = colorize_values(
        lidar_distances, cv2.COLORMAP_TURBO, (255, 200, 80)
    )
    return render_panel(
        title="LiDAR Points",
        subtitle_lines=[
            f"points={len(lidar_points)} labels={len(lidar_npz['groundtruth'])}",
            f"timestamp={int(lidar_npz['time_stamp'])}",
        ],
        points_xy=lidar_points[:, :2]
        if lidar_points.size
        else np.empty((0, 2), dtype=np.float32),
        point_colors=lidar_colors,
        point_radius=2,
        extent_m=range_meters,
        panel_size=panel_size,
    )


def build_echo_panel(
    run_dir: Path,
    frame_id: str,
    panel_size: int,
    range_meters: float,
) -> np.ndarray:
    point_cloud, _, time_stamp = load_echo_frame(run_dir, frame_id)
    echo_points = filter_echo_points(point_cloud)
    passive_count = load_passive_frame(run_dir, frame_id)
    return render_heatmap(
        echo_points=echo_points,
        active_count=echo_points.shape[0],
        passive_count=passive_count,
        range_m=range_meters,
        time_stamp=time_stamp,
        width=panel_size,
        height=panel_size,
        sigma=18.0,
    )


def main() -> int:
    args = parse_args()
    run_dir = resolve_run_dir(Path(args.dataset_dir).resolve())
    frame_id = args.frame_id

    rgb_path = run_dir / "rgb" / f"{frame_id}.png"
    seg_path = run_dir / "segmentation" / f"{frame_id}.png"

    rgb_image = load_image(rgb_path)
    seg_image = load_image(seg_path)

    echo_points, _, _ = load_echo_frame(run_dir, frame_id)
    range_meters = derive_range_meters(
        run_dir, frame_id, filter_echo_points(echo_points), args.range_meters
    )

    rgb_panel = decorate_image_panel(
        rgb_image,
        title="RGB",
        subtitle=rgb_path.name,
        panel_size=args.panel_size,
    )
    seg_panel = decorate_image_panel(
        seg_image,
        title="Segmentation",
        subtitle=seg_path.name,
        panel_size=args.panel_size,
    )
    lidar_panel = build_lidar_panel(
        run_dir=run_dir,
        frame_id=frame_id,
        panel_size=args.panel_size,
        range_meters=range_meters,
    )
    echo_panel = build_echo_panel(
        run_dir=run_dir,
        frame_id=frame_id,
        panel_size=args.panel_size,
        range_meters=range_meters,
    )

    panel = args.panel_size
    gap = args.gap
    margin = args.margin
    header_height = args.header_height

    canvas_width = margin * 2 + panel * 2 + gap
    canvas_width = margin * 2 + panel * 4 + gap * 3
    canvas_height = header_height + margin * 2 + panel
    canvas = np.full((canvas_height, canvas_width, 3), BG_COLOR, dtype=np.uint8)

    cv2.putText(
        canvas,
        "RGB | Seg | LiDAR | Echo",
        (margin, 42),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.1,
        TEXT_COLOR,
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        canvas,
        f"frame={frame_id}  range={range_meters:.0f}m",
        (margin, 74),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        SUBTEXT_COLOR,
        1,
        cv2.LINE_AA,
    )

    top_y = header_height
    x0 = margin
    x1 = margin + panel + gap
    x2 = margin + (panel + gap) * 2
    x3 = margin + (panel + gap) * 3

    canvas[top_y : top_y + panel, x0 : x0 + panel] = rgb_panel
    canvas[top_y : top_y + panel, x1 : x1 + panel] = seg_panel
    canvas[top_y : top_y + panel, x2 : x2 + panel] = lidar_panel
    canvas[top_y : top_y + panel, x3 : x3 + panel] = echo_panel

    output_path = (
        Path(args.output).resolve()
        if args.output
        else run_dir / f"rgb_seg_lidar_echo_1x4_{frame_id}.png"
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(output_path), canvas)

    print(f"Rendered: {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
