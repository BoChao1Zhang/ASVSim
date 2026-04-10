"""
Visualize LiDAR and echo point clouds as stitched bird's-eye-view images.

Input:
    A capture run directory produced by sync_multisensor_capture.py, or a parent
    directory containing multiple timestamped capture runs.

Output:
    Per-frame stitched PNGs with LiDAR and echo panels side by side.
    An optional contact sheet is also generated for quick inspection.
"""

import argparse
import json
import math
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import cv2
import numpy as np


DEFAULT_BG_COLOR = (20, 24, 28)
DEFAULT_GRID_COLOR = (52, 58, 66)
DEFAULT_AXIS_COLOR = (180, 190, 200)
DEFAULT_TEXT_COLOR = (232, 236, 241)
DEFAULT_SUBTEXT_COLOR = (180, 188, 196)
DEFAULT_HEADER_COLOR = (14, 17, 20)
DEFAULT_HEATMAP_BASE_COLOR = (255, 120, 60)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Render LiDAR and echo point clouds into stitched images."
    )
    parser.add_argument(
        "--dataset-dir",
        required=True,
        help=(
            "Capture run directory or a parent directory containing multiple "
            "timestamped capture runs."
        ),
    )
    parser.add_argument(
        "--output-dir",
        default="",
        help="Output directory for stitched visualizations. Defaults inside the run dir.",
    )
    parser.add_argument(
        "--panel-size",
        type=int,
        default=920,
        help="Square size in pixels for each LiDAR/echo panel.",
    )
    parser.add_argument(
        "--panel-gap",
        type=int,
        default=48,
        help="Gap between LiDAR and echo panels in pixels.",
    )
    parser.add_argument(
        "--outer-margin",
        type=int,
        default=32,
        help="Outer margin around the stitched image in pixels.",
    )
    parser.add_argument(
        "--title-height",
        type=int,
        default=88,
        help="Header height for the stitched image in pixels.",
    )
    parser.add_argument(
        "--frame-limit",
        type=int,
        default=0,
        help="Maximum number of frames to render. 0 means all frames.",
    )
    parser.add_argument(
        "--range-meters",
        type=float,
        default=0.0,
        help="Shared half-range in meters for both panels. 0 means auto-compute.",
    )
    parser.add_argument(
        "--contact-sheet-cols",
        type=int,
        default=2,
        help="Number of columns in the generated contact sheet.",
    )
    parser.add_argument(
        "--contact-sheet-thumb-width",
        type=int,
        default=960,
        help="Thumbnail width inside the contact sheet.",
    )
    parser.add_argument(
        "--skip-contact-sheet",
        action="store_true",
        help="Do not generate a contact sheet.",
    )
    return parser.parse_args()


def resolve_run_dir(path: Path) -> Path:
    if (path / "lidar").is_dir() and (path / "echo").is_dir():
        return path

    candidates = [
        subdir
        for subdir in path.iterdir()
        if subdir.is_dir() and (subdir / "lidar").is_dir() and (subdir / "echo").is_dir()
    ]
    if not candidates:
        raise FileNotFoundError(
            f"No capture run directories with lidar/echo folders were found in: {path}"
        )
    return max(candidates, key=lambda item: item.stat().st_mtime)


def load_frame_paths(run_dir: Path) -> List[Dict[str, Path]]:
    lidar_paths = sorted((run_dir / "lidar").glob("*.npz"))
    echo_paths = sorted((run_dir / "echo").glob("*.npz"))
    label_dir = run_dir / "labels"

    if len(lidar_paths) != len(echo_paths):
        raise RuntimeError(
            f"LiDAR and echo frame counts differ: {len(lidar_paths)} vs {len(echo_paths)}"
        )

    frames: List[Dict[str, Path]] = []
    for lidar_path, echo_path in zip(lidar_paths, echo_paths):
        frame_id = lidar_path.stem
        label_path = label_dir / f"{frame_id}.json"
        frames.append(
            {
                "frame_id": frame_id,
                "lidar": lidar_path,
                "echo": echo_path,
                "label": label_path,
            }
        )
    return frames


def choose_grid_step(extent_m: float) -> float:
    if extent_m <= 10:
        return 1.0
    if extent_m <= 25:
        return 2.0
    if extent_m <= 50:
        return 5.0
    if extent_m <= 100:
        return 10.0
    return 20.0


def round_extent(extent_m: float) -> float:
    grid_step = choose_grid_step(extent_m)
    return max(grid_step, math.ceil(extent_m / grid_step) * grid_step)


def compute_shared_extent(
    frames: Sequence[Dict[str, Path]],
    explicit_range_m: float,
) -> float:
    if explicit_range_m > 0:
        return explicit_range_m

    max_abs_xy = 1.0
    for frame in frames:
        lidar_npz = np.load(frame["lidar"], allow_pickle=True)
        echo_npz = np.load(frame["echo"], allow_pickle=True)

        lidar_points = np.asarray(lidar_npz["point_cloud"], dtype=np.float32)
        echo_points = np.asarray(echo_npz["point_cloud"], dtype=np.float32)

        for points in (lidar_points, echo_points):
            if points.ndim == 2 and points.shape[1] >= 2 and points.size > 0:
                max_abs_xy = max(max_abs_xy, float(np.max(np.abs(points[:, :2]))))

    return round_extent(max_abs_xy * 1.15)


def filter_lidar_points(point_cloud: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    if point_cloud.ndim != 2 or point_cloud.shape[1] < 3:
        return np.empty((0, 3), dtype=np.float32), np.empty((0,), dtype=np.float32)

    mask = np.isfinite(point_cloud).all(axis=1)
    mask &= np.linalg.norm(point_cloud[:, :2], axis=1) > 1e-4
    points = point_cloud[mask]
    distances = np.linalg.norm(points[:, :2], axis=1)
    return points, distances


def filter_echo_points(point_cloud: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    if point_cloud.ndim != 2 or point_cloud.shape[1] < 6:
        return (
            np.empty((0, 6), dtype=np.float32),
            np.empty((0,), dtype=np.float32),
            np.empty((0,), dtype=np.float32),
        )

    mask = np.isfinite(point_cloud).all(axis=1)
    mask &= np.linalg.norm(point_cloud[:, :2], axis=1) > 1e-4
    points = point_cloud[mask]
    total_distance = points[:, 4]
    reflection_count = points[:, 5]
    return points, total_distance, reflection_count


def colorize_values(
    values: np.ndarray, color_map_id: int, fallback_color: Tuple[int, int, int]
) -> np.ndarray:
    if values.size == 0:
        return np.empty((0, 3), dtype=np.uint8)

    value_min = float(np.min(values))
    value_max = float(np.max(values))
    if abs(value_max - value_min) < 1e-6:
        return np.repeat(np.asarray([fallback_color], dtype=np.uint8), values.size, axis=0)

    normalized = ((values - value_min) / (value_max - value_min) * 255.0).astype(np.uint8)
    try:
        colored = cv2.applyColorMap(normalized.reshape(-1, 1), color_map_id)
        return colored.reshape(-1, 3)
    except cv2.error:
        return np.repeat(np.asarray([fallback_color], dtype=np.uint8), values.size, axis=0)


def world_to_canvas(
    x_m: float,
    y_m: float,
    extent_m: float,
    plot_left: int,
    plot_top: int,
    plot_size: int,
) -> Tuple[int, int]:
    half_size = plot_size / 2.0
    px = plot_left + int(round(half_size + (y_m / extent_m) * half_size))
    py = plot_top + int(round(half_size - (x_m / extent_m) * half_size))
    return px, py


def draw_grid(
    canvas: np.ndarray,
    extent_m: float,
    grid_step_m: float,
    plot_left: int,
    plot_top: int,
    plot_size: int,
) -> None:
    for step in np.arange(-extent_m, extent_m + 0.1, grid_step_m):
        vertical_start = world_to_canvas(-extent_m, step, extent_m, plot_left, plot_top, plot_size)
        vertical_end = world_to_canvas(extent_m, step, extent_m, plot_left, plot_top, plot_size)
        horizontal_start = world_to_canvas(step, -extent_m, extent_m, plot_left, plot_top, plot_size)
        horizontal_end = world_to_canvas(step, extent_m, extent_m, plot_left, plot_top, plot_size)

        color = DEFAULT_GRID_COLOR
        thickness = 1
        if abs(step) < 1e-6:
            color = DEFAULT_AXIS_COLOR
            thickness = 2

        cv2.line(canvas, vertical_start, vertical_end, color, thickness, cv2.LINE_AA)
        cv2.line(canvas, horizontal_start, horizontal_end, color, thickness, cv2.LINE_AA)


def draw_origin_and_axes(
    canvas: np.ndarray,
    extent_m: float,
    plot_left: int,
    plot_top: int,
    plot_size: int,
) -> None:
    origin = world_to_canvas(0.0, 0.0, extent_m, plot_left, plot_top, plot_size)
    forward = world_to_canvas(2.0, 0.0, extent_m, plot_left, plot_top, plot_size)
    right = world_to_canvas(0.0, 2.0, extent_m, plot_left, plot_top, plot_size)

    cv2.circle(canvas, origin, 5, (255, 255, 255), -1, cv2.LINE_AA)
    cv2.arrowedLine(canvas, origin, forward, (98, 203, 255), 2, cv2.LINE_AA, tipLength=0.22)
    cv2.arrowedLine(canvas, origin, right, (147, 255, 164), 2, cv2.LINE_AA, tipLength=0.22)
    cv2.putText(
        canvas,
        "+X",
        (forward[0] + 6, forward[1] - 6),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (98, 203, 255),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        canvas,
        "+Y",
        (right[0] + 6, right[1] - 6),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (147, 255, 164),
        2,
        cv2.LINE_AA,
    )


def render_panel(
    title: str,
    subtitle_lines: Sequence[str],
    points_xy: np.ndarray,
    point_colors: np.ndarray,
    point_radius: int,
    extent_m: float,
    panel_size: int,
) -> np.ndarray:
    panel = np.full((panel_size, panel_size, 3), DEFAULT_BG_COLOR, dtype=np.uint8)
    panel_margin = 28
    header_height = 90
    footer_height = 22
    plot_size = panel_size - panel_margin * 2 - header_height - footer_height
    plot_left = panel_margin
    plot_top = panel_margin + header_height

    cv2.putText(
        panel,
        title,
        (panel_margin, panel_margin + 34),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.0,
        DEFAULT_TEXT_COLOR,
        2,
        cv2.LINE_AA,
    )
    for idx, line in enumerate(subtitle_lines):
        cv2.putText(
            panel,
            line,
            (panel_margin, panel_margin + 62 + idx * 22),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            DEFAULT_SUBTEXT_COLOR,
            1,
            cv2.LINE_AA,
        )

    grid_step_m = choose_grid_step(extent_m)
    draw_grid(panel, extent_m, grid_step_m, plot_left, plot_top, plot_size)
    draw_origin_and_axes(panel, extent_m, plot_left, plot_top, plot_size)

    if points_xy.size > 0:
        for (x_m, y_m), color in zip(points_xy, point_colors):
            px, py = world_to_canvas(
                float(x_m), float(y_m), extent_m, plot_left, plot_top, plot_size
            )
            if 0 <= px < panel.shape[1] and 0 <= py < panel.shape[0]:
                cv2.circle(panel, (px, py), point_radius, color.tolist(), -1, cv2.LINE_AA)
    else:
        cv2.putText(
            panel,
            "No points",
            (plot_left + 12, plot_top + 32),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (220, 120, 120),
            2,
            cv2.LINE_AA,
        )

    cv2.rectangle(
        panel,
        (plot_left, plot_top),
        (plot_left + plot_size, plot_top + plot_size),
        (96, 103, 110),
        1,
        cv2.LINE_AA,
    )

    cv2.putText(
        panel,
        f"Range +/- {extent_m:.1f} m | Grid {grid_step_m:g} m",
        (panel_margin, panel_size - 12),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        DEFAULT_SUBTEXT_COLOR,
        1,
        cv2.LINE_AA,
    )

    return panel


def build_echo_heatmap(
    points_xy: np.ndarray,
    extent_m: float,
    plot_size: int,
    sigma: int = 17,
) -> np.ndarray:
    heatmap = np.zeros((plot_size, plot_size), dtype=np.float32)
    if points_xy.size == 0:
        return heatmap

    half_size = plot_size / 2.0
    for x_m, y_m in points_xy:
        px = int(round(half_size + (float(y_m) / extent_m) * half_size))
        py = int(round(half_size - (float(x_m) / extent_m) * half_size))
        if 0 <= px < plot_size and 0 <= py < plot_size:
            heatmap[py, px] += 1.0

    if sigma > 0:
        heatmap = cv2.GaussianBlur(heatmap, (0, 0), sigmaX=sigma, sigmaY=sigma)

    max_value = float(np.max(heatmap))
    if max_value > 0:
        heatmap /= max_value
    return heatmap


def render_echo_heatmap_panel(
    title: str,
    subtitle_lines: Sequence[str],
    points_xy: np.ndarray,
    extent_m: float,
    panel_size: int,
) -> np.ndarray:
    panel = np.full((panel_size, panel_size, 3), DEFAULT_BG_COLOR, dtype=np.uint8)
    panel_margin = 28
    header_height = 90
    footer_height = 22
    plot_size = panel_size - panel_margin * 2 - header_height - footer_height
    plot_left = panel_margin
    plot_top = panel_margin + header_height

    cv2.putText(
        panel,
        title,
        (panel_margin, panel_margin + 34),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.0,
        DEFAULT_TEXT_COLOR,
        2,
        cv2.LINE_AA,
    )
    for idx, line in enumerate(subtitle_lines):
        cv2.putText(
            panel,
            line,
            (panel_margin, panel_margin + 62 + idx * 22),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            DEFAULT_SUBTEXT_COLOR,
            1,
            cv2.LINE_AA,
        )

    heatmap = build_echo_heatmap(points_xy, extent_m, plot_size)
    if np.max(heatmap) > 0:
        heatmap_uint8 = (heatmap * 255.0).astype(np.uint8)
        colored = cv2.applyColorMap(heatmap_uint8, cv2.COLORMAP_INFERNO)
        alpha = np.clip(heatmap[..., None] * 0.95, 0.0, 0.95).astype(np.float32)
        plot_region = panel[plot_top : plot_top + plot_size, plot_left : plot_left + plot_size].astype(np.float32)
        blended = plot_region * (1.0 - alpha) + colored.astype(np.float32) * alpha
        panel[plot_top : plot_top + plot_size, plot_left : plot_left + plot_size] = blended.astype(np.uint8)
    else:
        cv2.putText(
            panel,
            "No echo hits",
            (plot_left + 12, plot_top + 32),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (220, 120, 120),
            2,
            cv2.LINE_AA,
        )

    grid_step_m = choose_grid_step(extent_m)
    draw_grid(panel, extent_m, grid_step_m, plot_left, plot_top, plot_size)
    draw_origin_and_axes(panel, extent_m, plot_left, plot_top, plot_size)

    if points_xy.size > 0:
        hotspot = points_xy[np.argmin(np.linalg.norm(points_xy, axis=1))]
        hotspot_px = world_to_canvas(
            float(hotspot[0]), float(hotspot[1]), extent_m, plot_left, plot_top, plot_size
        )
        cv2.circle(panel, hotspot_px, 7, DEFAULT_HEATMAP_BASE_COLOR, 2, cv2.LINE_AA)

    cv2.rectangle(
        panel,
        (plot_left, plot_top),
        (plot_left + plot_size, plot_top + plot_size),
        (96, 103, 110),
        1,
        cv2.LINE_AA,
    )
    cv2.putText(
        panel,
        f"Range +/- {extent_m:.1f} m | Grid {grid_step_m:g} m",
        (panel_margin, panel_size - 12),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        DEFAULT_SUBTEXT_COLOR,
        1,
        cv2.LINE_AA,
    )

    legend_x = plot_left + plot_size - 24
    legend_y0 = plot_top + 16
    legend_y1 = plot_top + 160
    for idx, t in enumerate(np.linspace(0, 1, legend_y1 - legend_y0)):
        color = cv2.applyColorMap(
            np.array([[int((1.0 - t) * 255)]], dtype=np.uint8), cv2.COLORMAP_INFERNO
        )[0, 0]
        y = legend_y0 + idx
        cv2.line(panel, (legend_x, y), (legend_x + 10, y), color.tolist(), 1, cv2.LINE_AA)
    cv2.putText(
        panel,
        "Dense",
        (legend_x - 52, legend_y0 + 8),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        DEFAULT_SUBTEXT_COLOR,
        1,
        cv2.LINE_AA,
    )
    cv2.putText(
        panel,
        "Sparse",
        (legend_x - 56, legend_y1),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        DEFAULT_SUBTEXT_COLOR,
        1,
        cv2.LINE_AA,
    )

    return panel


def stitch_panels(
    frame_title: str,
    left_panel: np.ndarray,
    right_panel: np.ndarray,
    outer_margin: int,
    panel_gap: int,
    title_height: int,
) -> np.ndarray:
    composite_height = title_height + outer_margin * 2 + left_panel.shape[0]
    composite_width = (
        outer_margin * 2
        + left_panel.shape[1]
        + panel_gap
        + right_panel.shape[1]
    )
    composite = np.full(
        (composite_height, composite_width, 3), DEFAULT_HEADER_COLOR, dtype=np.uint8
    )

    cv2.putText(
        composite,
        frame_title,
        (outer_margin, int(title_height * 0.62)),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.1,
        DEFAULT_TEXT_COLOR,
        2,
        cv2.LINE_AA,
    )

    panel_top = title_height + outer_margin
    left_x = outer_margin
    right_x = outer_margin + left_panel.shape[1] + panel_gap
    composite[panel_top : panel_top + left_panel.shape[0], left_x : left_x + left_panel.shape[1]] = left_panel
    composite[panel_top : panel_top + right_panel.shape[0], right_x : right_x + right_panel.shape[1]] = right_panel
    return composite


def make_contact_sheet(
    stitched_paths: Sequence[Path],
    output_path: Path,
    cols: int,
    thumb_width: int,
    gap: int = 28,
    margin: int = 28,
) -> None:
    if not stitched_paths:
        return

    images = []
    thumb_height = None
    for path in stitched_paths:
        image = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if image is None:
            continue
        scale = thumb_width / image.shape[1]
        resized = cv2.resize(
            image, (thumb_width, int(round(image.shape[0] * scale))), interpolation=cv2.INTER_AREA
        )
        thumb_height = resized.shape[0]
        images.append((path.stem, resized))

    if not images or thumb_height is None:
        return

    rows = math.ceil(len(images) / cols)
    sheet_height = margin * 2 + rows * thumb_height + (rows - 1) * gap
    sheet_width = margin * 2 + cols * thumb_width + (cols - 1) * gap
    sheet = np.full((sheet_height, sheet_width, 3), DEFAULT_HEADER_COLOR, dtype=np.uint8)

    for idx, (_, image) in enumerate(images):
        row = idx // cols
        col = idx % cols
        x = margin + col * (thumb_width + gap)
        y = margin + row * (thumb_height + gap)
        sheet[y : y + thumb_height, x : x + thumb_width] = image

    cv2.imwrite(str(output_path), sheet)


def read_label_summary(label_path: Path) -> Tuple[int, List[str]]:
    if not label_path.exists():
        return 0, []
    payload = json.loads(label_path.read_text(encoding="utf-8"))
    objects = payload.get("objects", [])
    names = [str(obj.get("name", "")) for obj in objects[:3]]
    return len(objects), names


def render_frame(
    frame: Dict[str, Path],
    output_dir: Path,
    panel_size: int,
    panel_gap: int,
    outer_margin: int,
    title_height: int,
    extent_m: float,
) -> Path:
    lidar_npz = np.load(frame["lidar"], allow_pickle=True)
    echo_npz = np.load(frame["echo"], allow_pickle=True)

    lidar_points, lidar_distances = filter_lidar_points(
        np.asarray(lidar_npz["point_cloud"], dtype=np.float32)
    )
    echo_points, echo_total_distance, echo_reflection_count = filter_echo_points(
        np.asarray(echo_npz["point_cloud"], dtype=np.float32)
    )

    lidar_colors = colorize_values(
        lidar_distances, cv2.COLORMAP_TURBO, (255, 200, 80)
    )
    object_count, object_names = read_label_summary(frame["label"])
    lidar_subtitle = [
        f"points={len(lidar_points)}  labels={len(lidar_npz['groundtruth'])}",
        f"timestamp={int(lidar_npz['time_stamp'])}",
    ]
    echo_subtitle = [
        f"active_points={len(echo_points)}  labels={len(echo_npz['groundtruth'])}",
        f"timestamp={int(echo_npz['time_stamp'])}",
    ]
    if echo_points.size > 0:
        echo_subtitle.append(
            f"reflection_count<= {int(np.max(echo_reflection_count))}"
        )

    lidar_panel = render_panel(
        title="LiDAR BEV",
        subtitle_lines=lidar_subtitle,
        points_xy=lidar_points[:, :2] if lidar_points.size else np.empty((0, 2), dtype=np.float32),
        point_colors=lidar_colors,
        point_radius=2,
        extent_m=extent_m,
        panel_size=panel_size,
    )
    echo_panel = render_echo_heatmap_panel(
        title="Echo Heatmap",
        subtitle_lines=echo_subtitle,
        points_xy=echo_points[:, :2] if echo_points.size else np.empty((0, 2), dtype=np.float32),
        extent_m=extent_m,
        panel_size=panel_size,
    )

    title_suffix = f"objects={object_count}"
    if object_names:
        title_suffix += "  " + ",".join(object_names)
    stitched = stitch_panels(
        frame_title=f"Frame {frame['frame_id']}  |  {title_suffix}",
        left_panel=lidar_panel,
        right_panel=echo_panel,
        outer_margin=outer_margin,
        panel_gap=panel_gap,
        title_height=title_height,
    )

    output_path = output_dir / f"{frame['frame_id']}.png"
    cv2.imwrite(str(output_path), stitched)
    return output_path


def main() -> int:
    args = parse_args()
    run_dir = resolve_run_dir(Path(args.dataset_dir).resolve())
    frames = load_frame_paths(run_dir)
    if args.frame_limit > 0:
        frames = frames[: args.frame_limit]

    output_dir = (
        Path(args.output_dir).resolve()
        if args.output_dir
        else run_dir / "visualizations_lidar_echo"
    )
    output_dir.mkdir(parents=True, exist_ok=True)

    extent_m = compute_shared_extent(frames, args.range_meters)
    stitched_paths: List[Path] = []
    for frame in frames:
        stitched_path = render_frame(
            frame=frame,
            output_dir=output_dir,
            panel_size=args.panel_size,
            panel_gap=args.panel_gap,
            outer_margin=args.outer_margin,
            title_height=args.title_height,
            extent_m=extent_m,
        )
        stitched_paths.append(stitched_path)
        print(f"Rendered {stitched_path.name}")

    if not args.skip_contact_sheet:
        make_contact_sheet(
            stitched_paths=stitched_paths,
            output_path=output_dir / "contact_sheet.png",
            cols=max(1, args.contact_sheet_cols),
            thumb_width=max(320, args.contact_sheet_thumb_width),
        )
        print(f"Rendered contact sheet: {(output_dir / 'contact_sheet.png')}")

    print(f"Visualization output: {output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
