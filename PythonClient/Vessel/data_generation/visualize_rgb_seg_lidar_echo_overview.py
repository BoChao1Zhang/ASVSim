"""
Render a multi-frame overview image for RGB, segmentation, LiDAR and echo.
"""

import argparse
import math
from pathlib import Path
from typing import Dict, List

import cv2
import numpy as np

from visualize_lidar_echo import compute_shared_extent
from visualize_echo_radar_heatmap import resolve_run_dir
from visualize_rgb_seg_lidar_echo_quad import (
    BG_COLOR,
    SUBTEXT_COLOR,
    TEXT_COLOR,
    build_echo_panel,
    build_lidar_panel,
    decorate_image_panel,
    load_image,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Render multiple RGB/segmentation/LiDAR/echo frames into one overview image."
    )
    parser.add_argument(
        "--dataset-dir",
        required=True,
        help="Capture run directory or a parent directory containing capture runs.",
    )
    parser.add_argument(
        "--output",
        default="",
        help="Optional explicit output image path.",
    )
    parser.add_argument(
        "--frame-limit",
        type=int,
        default=8,
        help="Maximum number of frames to include. 0 means all frames.",
    )
    parser.add_argument(
        "--cols",
        type=int,
        default=2,
        help="Number of columns in the overview image.",
    )
    parser.add_argument(
        "--panel-size",
        type=int,
        default=280,
        help="Square panel size in pixels for each modality.",
    )
    parser.add_argument(
        "--gap",
        type=int,
        default=18,
        help="Gap between the four modality panels in each row item.",
    )
    parser.add_argument(
        "--margin",
        type=int,
        default=18,
        help="Outer margin for each 1x4 frame strip.",
    )
    parser.add_argument(
        "--header-height",
        type=int,
        default=58,
        help="Header height for each 1x4 frame strip.",
    )
    parser.add_argument(
        "--sheet-gap",
        type=int,
        default=28,
        help="Gap between frame strips in the overview image.",
    )
    parser.add_argument(
        "--sheet-margin",
        type=int,
        default=32,
        help="Outer margin for the overview image.",
    )
    parser.add_argument(
        "--sheet-header-height",
        type=int,
        default=92,
        help="Header height for the overview image.",
    )
    parser.add_argument(
        "--range-meters",
        type=float,
        default=0.0,
        help="Shared BEV half-range for LiDAR and echo. 0 means auto.",
    )
    return parser.parse_args()


def list_frame_ids(run_dir: Path, frame_limit: int) -> List[str]:
    modality_dirs = {
        "rgb": run_dir / "rgb",
        "segmentation": run_dir / "segmentation",
        "lidar": run_dir / "lidar",
        "echo": run_dir / "echo",
    }
    frame_sets = []
    for name, folder in modality_dirs.items():
        if not folder.is_dir():
            raise FileNotFoundError(f"Missing modality folder: {folder}")
        if name in ("rgb", "segmentation"):
            frame_ids = {path.stem for path in folder.glob("*.png")}
        else:
            frame_ids = {path.stem for path in folder.glob("*.npz")}
        frame_sets.append(frame_ids)

    shared_ids = sorted(set.intersection(*frame_sets))
    if not shared_ids:
        raise RuntimeError(f"No shared frame ids found in {run_dir}")
    if frame_limit > 0:
        return shared_ids[:frame_limit]
    return shared_ids


def build_extent_frame_list(run_dir: Path, frame_ids: List[str]) -> List[Dict[str, Path]]:
    return [
        {
            "lidar": run_dir / "lidar" / f"{frame_id}.npz",
            "echo": run_dir / "echo" / f"{frame_id}.npz",
        }
        for frame_id in frame_ids
    ]


def render_frame_strip(
    run_dir: Path,
    frame_id: str,
    panel_size: int,
    gap: int,
    margin: int,
    header_height: int,
    range_meters: float,
) -> np.ndarray:
    rgb_path = run_dir / "rgb" / f"{frame_id}.png"
    seg_path = run_dir / "segmentation" / f"{frame_id}.png"

    rgb_panel = decorate_image_panel(
        image=load_image(rgb_path),
        title="RGB",
        subtitle=rgb_path.name,
        panel_size=panel_size,
    )
    seg_panel = decorate_image_panel(
        image=load_image(seg_path),
        title="Segmentation",
        subtitle=seg_path.name,
        panel_size=panel_size,
    )
    lidar_panel = build_lidar_panel(
        run_dir=run_dir,
        frame_id=frame_id,
        panel_size=panel_size,
        range_meters=range_meters,
    )
    echo_panel = build_echo_panel(
        run_dir=run_dir,
        frame_id=frame_id,
        panel_size=panel_size,
        range_meters=range_meters,
    )

    strip_width = margin * 2 + panel_size * 4 + gap * 3
    strip_height = header_height + margin * 2 + panel_size
    strip = np.full((strip_height, strip_width, 3), BG_COLOR, dtype=np.uint8)

    cv2.putText(
        strip,
        f"Frame {frame_id}",
        (margin, 28),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.9,
        TEXT_COLOR,
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        strip,
        f"range={range_meters:.0f}m",
        (margin, 52),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.56,
        SUBTEXT_COLOR,
        1,
        cv2.LINE_AA,
    )

    top_y = header_height
    for index, panel in enumerate((rgb_panel, seg_panel, lidar_panel, echo_panel)):
        left_x = margin + index * (panel_size + gap)
        strip[top_y : top_y + panel_size, left_x : left_x + panel_size] = panel

    return strip


def render_overview(
    strips: List[np.ndarray],
    run_name: str,
    cols: int,
    sheet_gap: int,
    sheet_margin: int,
    sheet_header_height: int,
) -> np.ndarray:
    if not strips:
        raise RuntimeError("No frame strips were rendered.")

    cols = max(1, cols)
    rows = math.ceil(len(strips) / cols)
    strip_height, strip_width = strips[0].shape[:2]

    canvas_width = sheet_margin * 2 + cols * strip_width + (cols - 1) * sheet_gap
    canvas_height = (
        sheet_header_height
        + sheet_margin * 2
        + rows * strip_height
        + (rows - 1) * sheet_gap
    )
    canvas = np.full((canvas_height, canvas_width, 3), BG_COLOR, dtype=np.uint8)

    cv2.putText(
        canvas,
        "RGB | Seg | LiDAR | Echo Overview",
        (sheet_margin, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.15,
        TEXT_COLOR,
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        canvas,
        f"run={run_name}  frames={len(strips)}",
        (sheet_margin, 74),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        SUBTEXT_COLOR,
        1,
        cv2.LINE_AA,
    )

    top_y = sheet_header_height
    for index, strip in enumerate(strips):
        row = index // cols
        col = index % cols
        left_x = sheet_margin + col * (strip_width + sheet_gap)
        y = top_y + row * (strip_height + sheet_gap)
        canvas[y : y + strip_height, left_x : left_x + strip_width] = strip

    return canvas


def main() -> int:
    args = parse_args()
    run_dir = resolve_run_dir(Path(args.dataset_dir).resolve())
    frame_ids = list_frame_ids(run_dir, args.frame_limit)
    range_meters = (
        args.range_meters
        if args.range_meters > 0
        else compute_shared_extent(
            build_extent_frame_list(run_dir, frame_ids),
            explicit_range_m=0.0,
        )
    )

    strips = [
        render_frame_strip(
            run_dir=run_dir,
            frame_id=frame_id,
            panel_size=args.panel_size,
            gap=args.gap,
            margin=args.margin,
            header_height=args.header_height,
            range_meters=range_meters,
        )
        for frame_id in frame_ids
    ]

    overview = render_overview(
        strips=strips,
        run_name=run_dir.name,
        cols=args.cols,
        sheet_gap=args.sheet_gap,
        sheet_margin=args.sheet_margin,
        sheet_header_height=args.sheet_header_height,
    )

    output_path = (
        Path(args.output).resolve()
        if args.output
        else run_dir / f"rgb_seg_lidar_echo_overview_{len(frame_ids):02d}frames.png"
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(output_path), overview)
    print(f"Rendered: {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
