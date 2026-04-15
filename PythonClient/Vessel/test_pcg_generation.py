"""
Lightweight PCG generation test utility for vessel environments.

It resets the vessel runtime, activates PCG, generates one terrain,
queries getGoal() sections, writes normalized waypoint/boundary data,
and renders a simple top-down debug image.
"""

import argparse
import json
import math
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Sequence, Tuple

import cv2
import numpy as np

import setup_path
import cosysairsim as airsim


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parents[1]
OUTPUT_ROOT = REPO_ROOT / "data" / "vessel" / "pcg_test_runs"
NEUTRAL_ANGLE = 0.5
GOAL_EPSILON = 1e-6


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Test one PCG terrain generation and export debug artifacts."
    )
    parser.add_argument("--ip", default="127.0.0.1", help="RPC server IP.")
    parser.add_argument("--port", type=int, default=41451, help="RPC server port.")
    parser.add_argument("--vehicle-name", default="Vessel1", help="Vehicle name.")
    parser.add_argument("--seed", type=int, default=42, help="PCG seed.")
    parser.add_argument("--terrain-type", default="port", help="Terrain type.")
    parser.add_argument("--terrain-length", type=int, default=10)
    parser.add_argument("--min-angle", type=float, default=-45.0)
    parser.add_argument("--max-angle", type=float, default=45.0)
    parser.add_argument("--min-distance-cm", type=float, default=5000.0)
    parser.add_argument("--max-distance-cm", type=float, default=10000.0)
    parser.add_argument("--max-sections", type=int, default=64)
    parser.add_argument("--goal-retry-count", type=int, default=10)
    parser.add_argument("--goal-retry-sleep", type=float, default=0.5)
    parser.add_argument("--generation-settle-seconds", type=float, default=2.0)
    parser.add_argument(
        "--output-dir",
        default=str(OUTPUT_ROOT),
        help="Directory that will contain the timestamped test run.",
    )
    parser.add_argument(
        "--plot-size",
        type=int,
        default=1400,
        help="Size in pixels for the rendered debug plot.",
    )
    parser.add_argument(
        "--plot-padding",
        type=int,
        default=64,
        help="Outer padding in pixels for the debug plot.",
    )
    parser.add_argument(
        "--keep-runtime-state",
        action="store_true",
        help="Do not reset the vessel runtime after the test finishes.",
    )
    return parser.parse_args()


def vector2_tuple(value) -> Tuple[float, float]:
    if isinstance(value, dict):
        return float(value["x_val"]), float(value["y_val"])
    if hasattr(value, "x_val") and hasattr(value, "y_val"):
        return float(value.x_val), float(value.y_val)
    if isinstance(value, Sequence) and len(value) >= 2:
        return float(value[0]), float(value[1])
    raise TypeError(f"Unsupported Vector2r payload: {value!r}")


def goal_result_to_tuples(
    result,
) -> Tuple[Tuple[float, float], Tuple[float, float], Tuple[float, float]]:
    if not isinstance(result, Sequence) or len(result) < 3:
        raise RuntimeError(f"Unexpected getGoal payload: {result!r}")
    return (
        vector2_tuple(result[0]),
        vector2_tuple(result[1]),
        vector2_tuple(result[2]),
    )


def goal_is_valid(result) -> bool:
    try:
        goal, _, _ = goal_result_to_tuples(result)
    except Exception:
        return False
    return math.isfinite(goal[0]) and math.isfinite(goal[1]) and goal != (0.0, 0.0)


def collect_waypoints(
    client: airsim.VesselClient,
    initial_location: airsim.Vector2r,
    max_sections: int,
    retry_count: int,
    retry_sleep: float,
) -> Tuple[List[Tuple[float, float]], List[Dict[str, Tuple[float, float]]]]:
    waypoints: List[Tuple[float, float]] = []
    boundaries: List[Dict[str, Tuple[float, float]]] = []

    for section in range(1, max_sections + 1):
        latest_result = None
        for _ in range(retry_count):
            latest_result = client.getGoal(initial_location, section)
            if goal_is_valid(latest_result):
                break
            time.sleep(retry_sleep)

        if not goal_is_valid(latest_result):
            if section == 1:
                raise RuntimeError(
                    "getGoal did not return a valid first waypoint after generation."
                )
            break

        goal, left, right = goal_result_to_tuples(latest_result)
        waypoints.append(goal)
        boundaries.append({"left": left, "right": right})

    if not waypoints:
        raise RuntimeError("No valid waypoints were collected from getGoal.")
    return waypoints, boundaries


def normalize_xy_points(
    points: Sequence[Tuple[float, float]],
    offset_xy: Tuple[float, float],
) -> List[Tuple[float, float]]:
    return [
        (point[0] + offset_xy[0], point[1] + offset_xy[1]) for point in points
    ]


def normalize_boundaries(
    boundaries: Sequence[Dict[str, Tuple[float, float]]],
    offset_xy: Tuple[float, float],
) -> List[Dict[str, Tuple[float, float]]]:
    normalized: List[Dict[str, Tuple[float, float]]] = []
    for item in boundaries:
        normalized.append(
            {
                "left": (
                    item["left"][0] + offset_xy[0],
                    item["left"][1] + offset_xy[1],
                ),
                "right": (
                    item["right"][0] + offset_xy[0],
                    item["right"][1] + offset_xy[1],
                ),
            }
        )
    return normalized


def build_stop_controls() -> airsim.VesselControls:
    return airsim.VesselControls([0.0, 0.0], [NEUTRAL_ANGLE, NEUTRAL_ANGLE])


def ensure_clean_motion_state(
    client: airsim.VesselClient,
    vehicle_name: str,
    stop_controls: airsim.VesselControls,
) -> None:
    client.setVesselControls(vehicle_name, stop_controls)
    client.setDisturbanceControls(vehicle_name, airsim.DisturbanceControls(0.0, 0.0))


def reset_vessel_runtime(
    client: airsim.VesselClient,
    vehicle_name: str,
    stop_controls: airsim.VesselControls,
) -> airsim.Pose:
    client.reset()
    client.enableApiControl(True, vehicle_name)
    try:
        client.armDisarm(True, vehicle_name)
    except Exception:
        pass
    ensure_clean_motion_state(client, vehicle_name, stop_controls)
    time.sleep(0.2)
    return client.simGetVehiclePose(vehicle_name)


def cleanup_runtime(
    client: airsim.VesselClient,
    vehicle_name: str,
    stop_controls: airsim.VesselControls,
) -> None:
    client.simPause(False)
    client.reset()
    time.sleep(0.2)
    client.enableApiControl(True, vehicle_name)
    ensure_clean_motion_state(client, vehicle_name, stop_controls)


def compute_plot_bounds(
    path_points: Sequence[Tuple[float, float]],
    boundaries: Sequence[Dict[str, Tuple[float, float]]],
) -> Tuple[float, float, float, float]:
    xs = [point[0] for point in path_points]
    ys = [point[1] for point in path_points]
    for item in boundaries:
        xs.extend([item["left"][0], item["right"][0]])
        ys.extend([item["left"][1], item["right"][1]])

    min_x = min(xs)
    max_x = max(xs)
    min_y = min(ys)
    max_y = max(ys)

    if math.isclose(min_x, max_x):
        min_x -= 1.0
        max_x += 1.0
    if math.isclose(min_y, max_y):
        min_y -= 1.0
        max_y += 1.0
    return min_x, max_x, min_y, max_y


def world_to_canvas(
    point_xy: Tuple[float, float],
    bounds: Tuple[float, float, float, float],
    plot_size: int,
    padding: int,
) -> Tuple[int, int]:
    min_x, max_x, min_y, max_y = bounds
    usable = plot_size - padding * 2
    scale_x = usable / (max_x - min_x)
    scale_y = usable / (max_y - min_y)
    scale = min(scale_x, scale_y)

    x_val = padding + int(round((point_xy[0] - min_x) * scale))
    y_val = plot_size - padding - int(round((point_xy[1] - min_y) * scale))
    return x_val, y_val


def render_debug_plot(
    output_path: Path,
    seed: int,
    path_points: Sequence[Tuple[float, float]],
    boundaries: Sequence[Dict[str, Tuple[float, float]]],
    plot_size: int,
    padding: int,
) -> None:
    canvas = np.full((plot_size, plot_size, 3), 255, dtype=np.uint8)
    bounds = compute_plot_bounds(path_points, boundaries)

    center_points = [
        world_to_canvas(point_xy, bounds, plot_size, padding) for point_xy in path_points
    ]
    left_points = [
        world_to_canvas(item["left"], bounds, plot_size, padding) for item in boundaries
    ]
    right_points = [
        world_to_canvas(item["right"], bounds, plot_size, padding) for item in boundaries
    ]

    if len(left_points) >= 2:
        cv2.polylines(
            canvas,
            [np.asarray(left_points, dtype=np.int32)],
            False,
            (70, 140, 255),
            2,
            cv2.LINE_AA,
        )
    if len(right_points) >= 2:
        cv2.polylines(
            canvas,
            [np.asarray(right_points, dtype=np.int32)],
            False,
            (70, 140, 255),
            2,
            cv2.LINE_AA,
        )
    if len(center_points) >= 2:
        cv2.polylines(
            canvas,
            [np.asarray(center_points, dtype=np.int32)],
            False,
            (0, 0, 0),
            3,
            cv2.LINE_AA,
        )

    for index, point_xy in enumerate(center_points):
        color = (60, 180, 75)
        radius = 5
        if index == 0:
            color = (50, 120, 240)
            radius = 8
        elif index == len(center_points) - 1:
            color = (230, 80, 60)
            radius = 8
        cv2.circle(canvas, point_xy, radius, color, -1, cv2.LINE_AA)

    cv2.putText(
        canvas,
        f"PCG Test | seed={seed}",
        (padding, 32),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.9,
        (32, 32, 32),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        canvas,
        f"path_points={len(path_points)}",
        (padding, 62),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (96, 96, 96),
        1,
        cv2.LINE_AA,
    )

    output_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(output_path), canvas)


def main() -> int:
    args = parse_args()

    output_root = Path(args.output_dir).resolve()
    run_dir = output_root / datetime.now().strftime("pcg_test_%Y_%m_%d_%H_%M_%S")
    run_dir.mkdir(parents=True, exist_ok=False)
    (run_dir / "meta").mkdir(parents=True, exist_ok=True)

    client = airsim.VesselClient(ip=args.ip, port=args.port)
    client.confirmConnection()

    stop_controls = build_stop_controls()
    try:
        start_pose = reset_vessel_runtime(client, args.vehicle_name, stop_controls)
        start_xy = (
            float(start_pose.position.x_val),
            float(start_pose.position.y_val),
        )
        initial_location = airsim.Vector2r(
            start_xy[0] + GOAL_EPSILON,
            start_xy[1] + GOAL_EPSILON,
        )

        if not client.activateGeneration(False):
            raise RuntimeError(
                "activateGeneration(False) failed. Ensure the target world is ready."
            )

        terrain_ok = client.generatePortTerrain(
            args.terrain_type,
            args.seed,
            args.terrain_length,
            args.min_angle,
            args.max_angle,
            args.min_distance_cm,
            args.max_distance_cm,
        )
        if not terrain_ok:
            raise RuntimeError("generatePortTerrain returned False.")

        time.sleep(args.generation_settle_seconds)
        raw_waypoints, raw_boundaries = collect_waypoints(
            client=client,
            initial_location=initial_location,
            max_sections=args.max_sections,
            retry_count=args.goal_retry_count,
            retry_sleep=args.goal_retry_sleep,
        )

        section0_raw = client.getGoal(initial_location, 0)
        if not goal_is_valid(section0_raw):
            raise RuntimeError("getGoal(0) did not return a valid start point.")

        start_raw, start_left_raw, start_right_raw = goal_result_to_tuples(section0_raw)
        normalized_waypoints = normalize_xy_points(raw_waypoints, start_xy)
        normalized_boundaries = normalize_boundaries(raw_boundaries, start_xy)
        normalized_start = normalize_xy_points([start_raw], start_xy)[0]
        normalized_start_boundary = normalize_boundaries(
            [{"left": start_left_raw, "right": start_right_raw}], start_xy
        )[0]

        path_points = [normalized_start, *normalized_waypoints]
        boundary_points = [normalized_start_boundary, *normalized_boundaries]

        render_debug_plot(
            output_path=run_dir / "pcg_debug_plot.png",
            seed=args.seed,
            path_points=path_points,
            boundaries=boundary_points,
            plot_size=args.plot_size,
            padding=args.plot_padding,
        )

        metadata = {
            "seed": args.seed,
            "vehicle_name": args.vehicle_name,
            "terrain": {
                "type": args.terrain_type,
                "length": args.terrain_length,
                "min_angle": args.min_angle,
                "max_angle": args.max_angle,
                "min_distance_cm": args.min_distance_cm,
                "max_distance_cm": args.max_distance_cm,
            },
            "start_xy": list(normalized_start),
            "end_xy": list(path_points[-1]),
            "path_points": [[x_val, y_val] for x_val, y_val in path_points],
            "boundaries": [
                {
                    "left": list(item["left"]),
                    "right": list(item["right"]),
                }
                for item in boundary_points
            ],
        }
        (run_dir / "meta" / "pcg_test_result.json").write_text(
            json.dumps(metadata, indent=2),
            encoding="utf-8",
        )

        print(f"PCG test run written to: {run_dir}")
        print(
            f"seed={args.seed} path_points={len(path_points)} "
            f"end=({path_points[-1][0]:.1f}, {path_points[-1][1]:.1f})"
        )
        return 0
    finally:
        if not args.keep_runtime_state:
            try:
                cleanup_runtime(client, args.vehicle_name, stop_controls)
            except Exception:
                pass


if __name__ == "__main__":
    raise SystemExit(main())
