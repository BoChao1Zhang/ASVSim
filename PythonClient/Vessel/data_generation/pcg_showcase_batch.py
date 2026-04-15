"""
Generate PCG vessel showcase runs with one orthographic overview and
start/end front-view multisensor captures.
"""

import argparse
import csv
import json
import math
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Sequence, Tuple

import cv2
import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
PYTHONCLIENT_DIR = SCRIPT_DIR.parents[1]
REPO_ROOT = SCRIPT_DIR.parents[2]
DATA_ROOT = REPO_ROOT / "data" / "vessel"
USER_AIRSIM_SETTINGS_PATH = Path.home() / "Documents" / "AirSim" / "settings.json"
FRAME_ID = "000000"
NEUTRAL_ANGLE = 0.5
GOAL_EPSILON = 1e-6

if str(PYTHONCLIENT_DIR) not in sys.path:
    sys.path.insert(0, str(PYTHONCLIENT_DIR))

import cosysairsim as airsim
from sync_multisensor_capture import (
    capture_images_while_paused,
    derive_max_age_ns,
    image_response_to_array,
    pose_to_dict,
    reshape_point_cloud,
    save_png,
    to_string_array,
    wait_for_fresh_sensor_data,
    write_point_cloud_bundle,
)
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
        description=(
            "Generate PCG port channels and capture overview plus start/end "
            "RGB/Seg/LiDAR/Echo showcase images."
        )
    )
    parser.add_argument("--ip", default="127.0.0.1", help="RPC server IP.")
    parser.add_argument("--port", type=int, default=41451, help="RPC server port.")
    parser.add_argument(
        "--settings-path",
        default=str(USER_AIRSIM_SETTINGS_PATH),
        help="Path to the active AirSim settings.json file.",
    )
    parser.add_argument("--vehicle-name", default="Vessel1", help="Vehicle name.")
    parser.add_argument(
        "--front-camera",
        default="capture_front",
        help="Front camera used for start/end captures.",
    )
    parser.add_argument(
        "--overview-camera",
        default="capture_overview",
        help="Orthographic overview camera name.",
    )
    parser.add_argument("--lidar-name", default="lidar1", help="LiDAR sensor name.")
    parser.add_argument("--echo-name", default="echo_center", help="Echo sensor name.")
    parser.add_argument(
        "--output-dir",
        default=str(DATA_ROOT / "pcg_showcase_runs"),
        help="Directory that will contain the timestamped showcase batch.",
    )
    parser.add_argument("--runs", type=int, default=20, help="Number of seeds to run.")
    parser.add_argument("--base-seed", type=int, default=42, help="Base random seed.")
    parser.add_argument("--terrain-type", default="port", help="Terrain type.")
    parser.add_argument(
        "--terrain-length",
        type=int,
        default=10,
        help="Length passed to generatePortTerrain.",
    )
    parser.add_argument("--min-angle", type=float, default=-45.0)
    parser.add_argument("--max-angle", type=float, default=45.0)
    parser.add_argument("--min-distance-cm", type=float, default=5000.0)
    parser.add_argument("--max-distance-cm", type=float, default=10000.0)
    parser.add_argument("--max-sections", type=int, default=256)
    parser.add_argument("--goal-retry-count", type=int, default=10)
    parser.add_argument("--goal-retry-sleep", type=float, default=1.0)
    parser.add_argument("--generation-settle-seconds", type=float, default=2.0)
    parser.add_argument("--position-settle-seconds", type=float, default=0.6)
    parser.add_argument("--sensor-sync-timeout", type=float, default=2.0)
    parser.add_argument("--poll-interval", type=float, default=0.05)
    parser.add_argument("--retry-limit", type=int, default=4)
    parser.add_argument("--image-timestamp-tolerance-ns", type=int, default=10_000_000)
    parser.add_argument("--max-lidar-age-ns", type=int, default=None)
    parser.add_argument("--max-echo-age-ns", type=int, default=None)
    parser.add_argument("--range-meters", type=float, default=0.0)
    parser.add_argument("--panel-size", type=int, default=520)
    parser.add_argument("--gap", type=int, default=28)
    parser.add_argument("--margin", type=int, default=28)
    parser.add_argument("--header-height", type=int, default=88)
    parser.add_argument("--overview-banner-height", type=int, default=92)
    return parser.parse_args()


def load_settings(settings_path: Path) -> Dict[str, object]:
    if not settings_path.is_file():
        raise FileNotFoundError(f"settings.json not found: {settings_path}")
    return json.loads(settings_path.read_text(encoding="utf-8"))


def resolve_targets(settings: Dict[str, object], args: argparse.Namespace) -> Dict[str, object]:
    vehicles = settings.get("Vehicles", {})
    if args.vehicle_name not in vehicles:
        raise KeyError(f"Vehicle '{args.vehicle_name}' not found in settings.json")

    vehicle_cfg = vehicles[args.vehicle_name]
    cameras = vehicle_cfg.get("Cameras", {})
    sensors = vehicle_cfg.get("Sensors", {})

    if args.front_camera not in cameras:
        raise KeyError(f"Front camera '{args.front_camera}' not found in settings.json")
    if args.overview_camera not in cameras:
        raise KeyError(
            f"Overview camera '{args.overview_camera}' not found in settings.json"
        )
    if args.lidar_name not in sensors:
        raise KeyError(f"LiDAR sensor '{args.lidar_name}' not found in settings.json")
    if args.echo_name not in sensors:
        raise KeyError(f"Echo sensor '{args.echo_name}' not found in settings.json")

    return {
        "vehicle_cfg": vehicle_cfg,
        "front_camera_cfg": cameras[args.front_camera],
        "overview_camera_cfg": cameras[args.overview_camera],
        "lidar_cfg": sensors[args.lidar_name],
        "echo_cfg": sensors[args.echo_name],
    }


def vector2_tuple(value) -> Tuple[float, float]:
    if isinstance(value, dict):
        return float(value["x_val"]), float(value["y_val"])
    if hasattr(value, "x_val") and hasattr(value, "y_val"):
        return float(value.x_val), float(value.y_val)
    if isinstance(value, Sequence) and len(value) >= 2:
        return float(value[0]), float(value[1])
    raise TypeError(f"Unsupported Vector2r payload: {value!r}")


def goal_result_to_tuples(result) -> Tuple[Tuple[float, float], Tuple[float, float], Tuple[float, float]]:
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


def yaw_between(source_xy: Tuple[float, float], target_xy: Tuple[float, float]) -> float:
    return math.atan2(target_xy[1] - source_xy[1], target_xy[0] - source_xy[0])


def make_vehicle_pose(
    x_val: float,
    y_val: float,
    z_val: float,
    yaw_radians: float,
) -> airsim.Pose:
    return airsim.Pose(
        airsim.Vector3r(x_val, y_val, z_val),
        airsim.euler_to_quaternion(0.0, 0.0, yaw_radians),
    )


def build_stop_controls() -> airsim.VesselControls:
    return airsim.VesselControls([0.0, 0.0], [NEUTRAL_ANGLE, NEUTRAL_ANGLE])


def ensure_clean_motion_state(
    client: airsim.VesselClient,
    vehicle_name: str,
    stop_controls: airsim.VesselControls,
) -> None:
    client.setVesselControls(vehicle_name, stop_controls)
    client.setDisturbanceControls(vehicle_name, airsim.DisturbanceControls(0.0, 0.0))


def set_vehicle_pose_and_settle(
    client: airsim.VesselClient,
    vehicle_name: str,
    pose: airsim.Pose,
    stop_controls: airsim.VesselControls,
    settle_seconds: float,
) -> None:
    client.simPause(True)
    try:
        ensure_clean_motion_state(client, vehicle_name, stop_controls)
        client.simSetVehiclePose(pose, True, vehicle_name)
    finally:
        client.simPause(False)

    if settle_seconds > 0:
        time.sleep(settle_seconds)
    ensure_clean_motion_state(client, vehicle_name, stop_controls)


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


def cleanup_after_run(
    client: airsim.VesselClient,
    vehicle_name: str,
    stop_controls: airsim.VesselControls,
) -> None:
    client.simPause(False)
    client.reset()
    time.sleep(0.2)
    client.enableApiControl(True, vehicle_name)
    try:
        client.armDisarm(True, vehicle_name)
    except Exception:
        pass
    ensure_clean_motion_state(client, vehicle_name, stop_controls)


def capture_overview_image(
    client: airsim.VesselClient,
    vehicle_name: str,
    camera_name: str,
) -> np.ndarray:
    responses = client.simGetImages(
        [airsim.ImageRequest(camera_name, airsim.ImageType.Scene, False, False)],
        vehicle_name=vehicle_name,
    )
    if len(responses) != 1:
        raise RuntimeError(
            f"Expected 1 overview response, received {len(responses)} responses."
        )
    return image_response_to_array(responses[0])


def annotate_overview_image(
    image: np.ndarray,
    run_index: int,
    seed: int,
    start_xy: Tuple[float, float],
    end_xy: Tuple[float, float],
    waypoint_count: int,
    banner_height: int,
) -> np.ndarray:
    canvas = image.copy()
    overlay = canvas.copy()
    cv2.rectangle(
        overlay,
        (0, 0),
        (canvas.shape[1], min(banner_height, canvas.shape[0])),
        (248, 248, 248),
        -1,
    )
    canvas = cv2.addWeighted(overlay, 0.88, canvas, 0.12, 0)

    lines = [
        f"PCG Channel Overview | run={run_index:03d} seed={seed}",
        f"start=({start_xy[0]:.1f}, {start_xy[1]:.1f})  end=({end_xy[0]:.1f}, {end_xy[1]:.1f})",
        f"waypoints={waypoint_count}",
    ]

    y_val = 34
    for index, text in enumerate(lines):
        cv2.putText(
            canvas,
            text,
            (24, y_val + index * 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.68,
            TEXT_COLOR if index == 0 else SUBTEXT_COLOR,
            2 if index == 0 else 1,
            cv2.LINE_AA,
        )
    return canvas


def compute_panel_range_meters(
    args: argparse.Namespace,
    lidar_cfg: Dict[str, object],
    echo_cfg: Dict[str, object],
) -> float:
    if args.range_meters > 0:
        return float(args.range_meters)

    lidar_range = float(lidar_cfg.get("range", 0.0))
    echo_range = float(echo_cfg.get("DistanceLimit", 0.0))
    derived = max(lidar_range, echo_range, 50.0)
    return float(derived)


def save_capture_bundle(
    capture_dir: Path,
    image_capture: Dict[str, object],
    lidar_data,
    echo_data,
    vehicle_pose: airsim.Pose,
    metadata: Dict[str, object],
) -> None:
    for folder_name in ("rgb", "segmentation", "lidar", "echo", "meta"):
        (capture_dir / folder_name).mkdir(parents=True, exist_ok=True)

    rgb_path = capture_dir / "rgb" / f"{FRAME_ID}.png"
    segmentation_path = capture_dir / "segmentation" / f"{FRAME_ID}.png"
    lidar_path = capture_dir / "lidar" / f"{FRAME_ID}.npz"
    echo_path = capture_dir / "echo" / f"{FRAME_ID}.npz"
    metadata_path = capture_dir / "meta" / "capture_meta.json"

    save_png(image_capture["scene_image"], rgb_path)
    save_png(image_capture["segmentation_image"], segmentation_path)

    write_point_cloud_bundle(
        lidar_path,
        point_cloud=reshape_point_cloud(lidar_data.point_cloud, 3),
        groundtruth=to_string_array(lidar_data.groundtruth),
        pose=pose_to_dict(lidar_data.pose),
        time_stamp=int(lidar_data.time_stamp),
    )
    write_point_cloud_bundle(
        echo_path,
        point_cloud=reshape_point_cloud(echo_data.point_cloud, 6),
        groundtruth=to_string_array(echo_data.groundtruth),
        pose=pose_to_dict(echo_data.pose),
        time_stamp=int(echo_data.time_stamp),
        passive_beacons_point_cloud=reshape_point_cloud(
            echo_data.passive_beacons_point_cloud, 9
        ),
        passive_beacons_groundtruth=to_string_array(
            echo_data.passive_beacons_groundtruth
        ),
    )

    metadata_path.write_text(
        json.dumps(
            {
                **metadata,
                "vehicle_pose": pose_to_dict(vehicle_pose),
                "timestamps": {
                    "scene": int(image_capture["scene_timestamp"]),
                    "segmentation": int(image_capture["segmentation_timestamp"]),
                    "image": int(image_capture["image_timestamp"]),
                    "lidar": int(lidar_data.time_stamp),
                    "echo": int(echo_data.time_stamp),
                },
            },
            indent=2,
        ),
        encoding="utf-8",
    )


def render_quad_image(
    capture_dir: Path,
    output_path: Path,
    capture_label: str,
    panel_size: int,
    gap: int,
    margin: int,
    header_height: int,
    range_meters: float,
    subtitle: str,
) -> None:
    rgb_path = capture_dir / "rgb" / f"{FRAME_ID}.png"
    seg_path = capture_dir / "segmentation" / f"{FRAME_ID}.png"

    rgb_panel = decorate_image_panel(
        load_image(rgb_path), "RGB", rgb_path.name, panel_size
    )
    seg_panel = decorate_image_panel(
        load_image(seg_path), "Segmentation", seg_path.name, panel_size
    )
    lidar_panel = build_lidar_panel(
        run_dir=capture_dir,
        frame_id=FRAME_ID,
        panel_size=panel_size,
        range_meters=range_meters,
    )
    echo_panel = build_echo_panel(
        run_dir=capture_dir,
        frame_id=FRAME_ID,
        panel_size=panel_size,
        range_meters=range_meters,
    )

    width = margin * 2 + panel_size * 4 + gap * 3
    height = header_height + margin * 2 + panel_size
    canvas = np.full((height, width, 3), BG_COLOR, dtype=np.uint8)

    cv2.putText(
        canvas,
        f"{capture_label.upper()} | RGB | Seg | LiDAR | Echo",
        (margin, 42),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.05,
        TEXT_COLOR,
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        canvas,
        subtitle,
        (margin, 74),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.63,
        SUBTEXT_COLOR,
        1,
        cv2.LINE_AA,
    )

    top_y = header_height
    x_positions = [
        margin,
        margin + panel_size + gap,
        margin + (panel_size + gap) * 2,
        margin + (panel_size + gap) * 3,
    ]

    for panel, left_x in zip(
        (rgb_panel, seg_panel, lidar_panel, echo_panel), x_positions
    ):
        canvas[top_y : top_y + panel_size, left_x : left_x + panel_size] = panel

    output_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(output_path), canvas)


def capture_modalities_at_pose(
    client: airsim.VesselClient,
    vehicle_name: str,
    camera_name: str,
    lidar_name: str,
    echo_name: str,
    pose: airsim.Pose,
    capture_dir: Path,
    label: str,
    stop_controls: airsim.VesselControls,
    panel_range_meters: float,
    args: argparse.Namespace,
    previous_lidar_ts: int,
    previous_echo_ts: int,
    max_lidar_age_ns: int,
    max_echo_age_ns: int,
    extra_metadata: Dict[str, object],
) -> Tuple[int, int]:
    set_vehicle_pose_and_settle(
        client=client,
        vehicle_name=vehicle_name,
        pose=pose,
        stop_controls=stop_controls,
        settle_seconds=args.position_settle_seconds,
    )

    latest_error = None
    for _ in range(args.retry_limit):
        client.simPause(False)
        lidar_data, echo_data = wait_for_fresh_sensor_data(
            client=client,
            vehicle_name=vehicle_name,
            lidar_name=lidar_name,
            echo_name=echo_name,
            previous_lidar_ts=previous_lidar_ts,
            previous_echo_ts=previous_echo_ts,
            timeout_seconds=args.sensor_sync_timeout,
            poll_interval_seconds=args.poll_interval,
        )
        candidate_lidar_ts = int(lidar_data.time_stamp)
        candidate_echo_ts = int(echo_data.time_stamp)

        client.simPause(True)
        try:
            image_capture = capture_images_while_paused(
                client=client,
                vehicle_name=vehicle_name,
                camera_name=camera_name,
                image_timestamp_tolerance_ns=args.image_timestamp_tolerance_ns,
            )
            vehicle_pose = client.simGetVehiclePose(vehicle_name)
        finally:
            client.simPause(False)

        image_ts = int(image_capture["image_timestamp"])
        lidar_age_ns = image_ts - candidate_lidar_ts
        echo_age_ns = image_ts - candidate_echo_ts
        if 0 <= lidar_age_ns <= max_lidar_age_ns and 0 <= echo_age_ns <= max_echo_age_ns:
            save_capture_bundle(
                capture_dir=capture_dir,
                image_capture=image_capture,
                lidar_data=lidar_data,
                echo_data=echo_data,
                vehicle_pose=vehicle_pose,
                metadata={
                    **extra_metadata,
                    "capture_label": label,
                    "requested_pose": pose_to_dict(pose),
                    "timestamp_deltas_ns": {
                        "image_pair_diff_ns": int(image_capture["image_pair_diff_ns"]),
                        "lidar_age_ns": int(lidar_age_ns),
                        "echo_age_ns": int(echo_age_ns),
                    },
                },
            )
            render_quad_image(
                capture_dir=capture_dir,
                output_path=capture_dir / f"{label}_quad.png",
                capture_label=label,
                panel_size=args.panel_size,
                gap=args.gap,
                margin=args.margin,
                header_height=args.header_height,
                range_meters=panel_range_meters,
                subtitle=(
                    f"frame={FRAME_ID} range={panel_range_meters:.0f}m "
                    f"lidar_age={lidar_age_ns}ns echo_age={echo_age_ns}ns"
                ),
            )
            return candidate_lidar_ts, candidate_echo_ts

        latest_error = (
            f"age validation failed: lidar_age_ns={lidar_age_ns} "
            f"echo_age_ns={echo_age_ns}"
        )
        time.sleep(args.poll_interval)

    raise RuntimeError(
        f"Failed to capture {label} frame after retries. Last error: {latest_error}"
    )


def ensure_batch_dirs(batch_dir: Path) -> None:
    batch_dir.mkdir(parents=True, exist_ok=False)
    (batch_dir / "meta").mkdir(parents=True, exist_ok=True)


def write_summary_header(summary_csv_path: Path) -> None:
    with summary_csv_path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(
            [
                "run_index",
                "seed",
                "status",
                "waypoint_count",
                "end_x",
                "end_y",
                "overview_path",
                "start_quad_path",
                "end_quad_path",
                "error",
            ]
        )


def append_summary_row(summary_csv_path: Path, row: Sequence[object]) -> None:
    with summary_csv_path.open("a", newline="", encoding="utf-8") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(list(row))


def relative_path(base: Path, target: Path) -> str:
    return target.relative_to(base).as_posix()


def main() -> int:
    args = parse_args()
    settings_path = Path(args.settings_path).expanduser().resolve()
    settings = load_settings(settings_path)
    targets = resolve_targets(settings, args)

    output_root = Path(args.output_dir).resolve()
    batch_dir = output_root / datetime.now().strftime("pcg_showcase_%Y_%m_%d_%H_%M_%S")
    ensure_batch_dirs(batch_dir)

    summary_csv_path = batch_dir / "meta" / "summary.csv"
    summary_json_path = batch_dir / "meta" / "summary.json"
    write_summary_header(summary_csv_path)

    client = airsim.VesselClient(ip=args.ip, port=args.port)
    client.confirmConnection()

    stop_controls = build_stop_controls()
    initial_pose = reset_vessel_runtime(client, args.vehicle_name, stop_controls)
    initial_xy = (
        float(initial_pose.position.x_val),
        float(initial_pose.position.y_val),
    )
    initial_z = float(initial_pose.position.z_val)
    initial_yaw = 0.0
    initial_location = airsim.Vector2r(
        initial_xy[0] + GOAL_EPSILON,
        initial_xy[1] + GOAL_EPSILON,
    )

    if not client.activateGeneration(False):
        raise RuntimeError(
            "activateGeneration(False) failed. Ensure the target UE world is ready."
        )

    lidar_rate_hz = float(targets["lidar_cfg"].get("RotationsPerSecond", 0.0))
    echo_rate_hz = float(targets["echo_cfg"].get("MeasurementFrequency", 0.0))
    max_lidar_age_ns = derive_max_age_ns(args.max_lidar_age_ns, lidar_rate_hz)
    max_echo_age_ns = derive_max_age_ns(args.max_echo_age_ns, echo_rate_hz)
    panel_range_meters = compute_panel_range_meters(
        args, targets["lidar_cfg"], targets["echo_cfg"]
    )

    summary_rows: List[Dict[str, object]] = []
    previous_lidar_ts = 0
    previous_echo_ts = 0

    try:
        for run_index in range(args.runs):
            seed = args.base_seed + run_index
            run_dir = batch_dir / f"run_{run_index:03d}_seed_{seed}"
            run_dir.mkdir(parents=True, exist_ok=False)
            (run_dir / "meta").mkdir(parents=True, exist_ok=True)
            run_status = "success"
            run_error = ""
            waypoint_count = 0
            end_xy = (0.0, 0.0)
            overview_path = run_dir / "overview" / "channel_overview.png"
            start_quad_path = run_dir / "captures" / "start" / "start_quad.png"
            end_quad_path = run_dir / "captures" / "end" / "end_quad.png"

            try:
                reset_pose = reset_vessel_runtime(
                    client=client,
                    vehicle_name=args.vehicle_name,
                    stop_controls=stop_controls,
                )
                initial_xy = (
                    float(reset_pose.position.x_val),
                    float(reset_pose.position.y_val),
                )
                initial_z = float(reset_pose.position.z_val)
                initial_location = airsim.Vector2r(
                    initial_xy[0] + GOAL_EPSILON,
                    initial_xy[1] + GOAL_EPSILON,
                )

                terrain_ok = client.generatePortTerrain(
                    args.terrain_type,
                    seed,
                    args.terrain_length,
                    args.min_angle,
                    args.max_angle,
                    args.min_distance_cm,
                    args.max_distance_cm,
                )
                if not terrain_ok:
                    raise RuntimeError("generatePortTerrain returned False.")

                time.sleep(args.generation_settle_seconds)
                waypoints, boundaries = collect_waypoints(
                    client=client,
                    initial_location=initial_location,
                    max_sections=args.max_sections,
                    retry_count=args.goal_retry_count,
                    retry_sleep=args.goal_retry_sleep,
                )
                section0_raw = client.getGoal(initial_location, 0)
                if not goal_is_valid(section0_raw):
                    raise RuntimeError("getGoal(0) did not return a valid start point.")

                start_raw, start_left_raw, start_right_raw = goal_result_to_tuples(
                    section0_raw
                )
                normalized_waypoints = normalize_xy_points(waypoints, initial_xy)
                normalized_boundaries = normalize_boundaries(boundaries, initial_xy)
                start_xy = normalize_xy_points([start_raw], initial_xy)[0]
                start_boundary = normalize_boundaries(
                    [{"left": start_left_raw, "right": start_right_raw}], initial_xy
                )[0]

                path_points = [start_xy, *normalized_waypoints]
                waypoint_count = len(path_points)
                end_xy = path_points[-1]

                start_target = path_points[1] if len(path_points) > 1 else start_xy
                start_yaw = yaw_between(start_xy, start_target)
                end_source = path_points[-2] if len(path_points) > 1 else start_xy
                end_yaw = yaw_between(end_source, end_xy)

                overview_image = capture_overview_image(
                    client=client,
                    vehicle_name=args.vehicle_name,
                    camera_name=args.overview_camera,
                )
                overview_annotated = annotate_overview_image(
                    image=overview_image,
                    run_index=run_index,
                    seed=seed,
                    start_xy=start_xy,
                    end_xy=end_xy,
                    waypoint_count=waypoint_count,
                    banner_height=args.overview_banner_height,
                )
                overview_path.parent.mkdir(parents=True, exist_ok=True)
                save_png(overview_annotated, overview_path)

                previous_lidar_ts, previous_echo_ts = capture_modalities_at_pose(
                    client=client,
                    vehicle_name=args.vehicle_name,
                    camera_name=args.front_camera,
                    lidar_name=args.lidar_name,
                    echo_name=args.echo_name,
                    pose=make_vehicle_pose(
                        start_xy[0], start_xy[1], initial_z, start_yaw
                    ),
                    capture_dir=run_dir / "captures" / "start",
                    label="start",
                    stop_controls=stop_controls,
                    panel_range_meters=panel_range_meters,
                    args=args,
                    previous_lidar_ts=previous_lidar_ts,
                    previous_echo_ts=previous_echo_ts,
                    max_lidar_age_ns=max_lidar_age_ns,
                    max_echo_age_ns=max_echo_age_ns,
                    extra_metadata={
                        "seed": seed,
                        "run_index": run_index,
                        "target_xy": [start_xy[0], start_xy[1]],
                        "yaw_radians": start_yaw,
                    },
                )

                previous_lidar_ts, previous_echo_ts = capture_modalities_at_pose(
                    client=client,
                    vehicle_name=args.vehicle_name,
                    camera_name=args.front_camera,
                    lidar_name=args.lidar_name,
                    echo_name=args.echo_name,
                    pose=make_vehicle_pose(end_xy[0], end_xy[1], initial_z, end_yaw),
                    capture_dir=run_dir / "captures" / "end",
                    label="end",
                    stop_controls=stop_controls,
                    panel_range_meters=panel_range_meters,
                    args=args,
                    previous_lidar_ts=previous_lidar_ts,
                    previous_echo_ts=previous_echo_ts,
                    max_lidar_age_ns=max_lidar_age_ns,
                    max_echo_age_ns=max_echo_age_ns,
                    extra_metadata={
                        "seed": seed,
                        "run_index": run_index,
                        "target_xy": [end_xy[0], end_xy[1]],
                        "yaw_radians": end_yaw,
                    },
                )

                run_meta = {
                    "run_index": run_index,
                    "seed": seed,
                    "settings_path": str(settings_path),
                    "vehicle_name": args.vehicle_name,
                    "front_camera": args.front_camera,
                    "overview_camera": args.overview_camera,
                    "lidar_name": args.lidar_name,
                    "echo_name": args.echo_name,
                    "terrain": {
                        "type": args.terrain_type,
                        "length": args.terrain_length,
                        "min_angle": args.min_angle,
                        "max_angle": args.max_angle,
                        "min_distance_cm": args.min_distance_cm,
                        "max_distance_cm": args.max_distance_cm,
                    },
                    "start_xy": list(start_xy),
                    "end_xy": list(end_xy),
                    "waypoints": [[x_val, y_val] for x_val, y_val in path_points],
                    "boundaries": [
                        {
                            "left": list(item["left"]),
                            "right": list(item["right"]),
                        }
                        for item in [start_boundary, *normalized_boundaries]
                    ],
                    "paths": {
                        "overview": relative_path(batch_dir, overview_path),
                        "start_quad": relative_path(batch_dir, start_quad_path),
                        "end_quad": relative_path(batch_dir, end_quad_path),
                    },
                }
                (run_dir / "meta" / "run_meta.json").write_text(
                    json.dumps(run_meta, indent=2), encoding="utf-8"
                )
                print(
                    f"[run {run_index:03d}] seed={seed} waypoints={waypoint_count} "
                    f"end=({end_xy[0]:.1f}, {end_xy[1]:.1f})"
                )

            except Exception as exc:
                run_status = "failed"
                run_error = str(exc)
                (run_dir / "meta" / "run_error.json").write_text(
                    json.dumps(
                        {
                            "run_index": run_index,
                            "seed": seed,
                            "error": run_error,
                        },
                        indent=2,
                    ),
                    encoding="utf-8",
                )
                print(f"[run {run_index:03d}] seed={seed} failed: {run_error}")

            summary_entry = {
                "run_index": run_index,
                "seed": seed,
                "status": run_status,
                "waypoint_count": waypoint_count,
                "end_x": end_xy[0],
                "end_y": end_xy[1],
                "overview_path": relative_path(batch_dir, overview_path)
                if overview_path.exists()
                else "",
                "start_quad_path": relative_path(batch_dir, start_quad_path)
                if start_quad_path.exists()
                else "",
                "end_quad_path": relative_path(batch_dir, end_quad_path)
                if end_quad_path.exists()
                else "",
                "error": run_error,
            }
            summary_rows.append(summary_entry)
            append_summary_row(
                summary_csv_path,
                [
                    summary_entry["run_index"],
                    summary_entry["seed"],
                    summary_entry["status"],
                    summary_entry["waypoint_count"],
                    summary_entry["end_x"],
                    summary_entry["end_y"],
                    summary_entry["overview_path"],
                    summary_entry["start_quad_path"],
                    summary_entry["end_quad_path"],
                    summary_entry["error"],
                ],
            )
            try:
                cleanup_after_run(
                    client=client,
                    vehicle_name=args.vehicle_name,
                    stop_controls=stop_controls,
                )
            except Exception as cleanup_exc:
                print(
                    f"[run {run_index:03d}] cleanup warning: {cleanup_exc}"
                )
    finally:
        client.simPause(False)
        ensure_clean_motion_state(client, args.vehicle_name, stop_controls)
        client.enableApiControl(False, args.vehicle_name)

    summary_json_path.write_text(
        json.dumps(summary_rows, indent=2), encoding="utf-8"
    )
    print(f"Showcase batch written to: {batch_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
