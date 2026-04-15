"""
Synchronized vessel data capture for RGB detection, LiDAR and echo sensor data.

This script follows the current runtime behavior of Cosys-AirSim:
camera images are captured on demand, while LiDAR and echo data expose the most
recent completed scan. The capture loop therefore waits for fresh LiDAR and echo
timestamps, freezes the simulation, captures images and ground-truth pose, and
validates that the image-to-sensor age stays within configurable bounds.
"""

import argparse
import csv
import json
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np
from PIL import Image


SCRIPT_DIR = Path(__file__).resolve().parent
PYTHONCLIENT_DIR = SCRIPT_DIR.parents[1]
REPO_ROOT = SCRIPT_DIR.parents[2]
DATA_ROOT = REPO_ROOT / "data" / "vessel"
USER_AIRSIM_SETTINGS_PATH = Path.home() / "Documents" / "AirSim" / "settings.json"

if str(PYTHONCLIENT_DIR) not in sys.path:
    sys.path.insert(0, str(PYTHONCLIENT_DIR))

import cosysairsim as airsim


DEFAULT_OBJECT_PREFIXES = [
    "Boat",
    "Buoy",
    "Cybership",
    "Drone",
    "MilliAmpere",
    "Qiuxin",
    "StaticMeshActor_13",
    "StaticMeshActor_14",
    "StaticMeshActor_15",
    "Vessel",
]
PREFERRED_CAMERA_NAMES = ("capture_front", "frontcamera")
PREFERRED_LIDAR_NAMES = ("lidar1", "lidar")
PREFERRED_ECHO_NAMES = ("echo_center", "echo")
NEUTRAL_ANGLE = 0.5
MAX_THRUSTER_COUNT = 10


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Capture synchronized RGB, segmentation, LiDAR and echo data."
    )
    parser.add_argument("--ip", default="127.0.0.1", help="RPC server IP.")
    parser.add_argument("--port", type=int, default=41451, help="RPC server port.")
    parser.add_argument(
        "--settings-path",
        default=str(USER_AIRSIM_SETTINGS_PATH),
        help="Path to the active AirSim settings.json file.",
    )
    parser.add_argument(
        "--vehicle-name",
        default="auto",
        help="Vehicle name configured in settings.json. Use 'auto' to detect.",
    )
    parser.add_argument(
        "--camera-name",
        default="auto",
        help="Camera name configured in settings.json. Use 'auto' to detect.",
    )
    parser.add_argument(
        "--lidar-name",
        default="auto",
        help="LiDAR sensor name configured in settings.json. Use 'auto' to detect.",
    )
    parser.add_argument(
        "--echo-name",
        default="auto",
        help="Echo sensor name configured in settings.json. Use 'auto' to detect.",
    )
    parser.add_argument(
        "--output-dir",
        default=str(DATA_ROOT / "dataset_multisensor"),
        help="Directory that will contain the timestamped dataset run folder.",
    )
    parser.add_argument(
        "--num-frames", type=int, default=10, help="Number of frames to capture."
    )
    parser.add_argument(
        "--capture-interval",
        type=float,
        default=0.2,
        help="Wall-clock delay between frames while the simulation is running.",
    )
    parser.add_argument(
        "--image-timestamp-tolerance-ns",
        type=int,
        default=10_000_000,
        help="Allowed timestamp delta between Scene and Segmentation images.",
    )
    parser.add_argument(
        "--max-lidar-age-ns",
        type=int,
        default=None,
        help="Maximum allowed age between image timestamp and LiDAR timestamp.",
    )
    parser.add_argument(
        "--max-echo-age-ns",
        type=int,
        default=None,
        help="Maximum allowed age between image timestamp and echo timestamp.",
    )
    parser.add_argument(
        "--sensor-sync-timeout",
        type=float,
        default=2.0,
        help="Seconds to wait for fresh LiDAR and echo scans.",
    )
    parser.add_argument(
        "--poll-interval",
        type=float,
        default=0.02,
        help="Polling interval in seconds while waiting for fresh sensor scans.",
    )
    parser.add_argument(
        "--retry-limit",
        type=int,
        default=5,
        help="Maximum retries for a frame when timing validation fails.",
    )
    parser.add_argument(
        "--object-prefix",
        nargs="*",
        default=DEFAULT_OBJECT_PREFIXES,
        help=(
            "Object name prefixes to convert from segmentation masks into detection "
            "labels. Matching is performed on cleaned object names."
        ),
    )
    parser.add_argument(
        "--include-all-objects",
        action="store_true",
        help="Create bounding boxes for every object in the segmentation color map.",
    )
    parser.add_argument(
        "--enable-api-control",
        action="store_true",
        help="Enable API control for the vessel before capturing.",
    )
    parser.add_argument(
        "--drive-straight-throttle",
        type=float,
        default=0.0,
        help="If > 0, drive the vessel straight during capture using this throttle.",
    )
    parser.add_argument(
        "--drive-steering-angle",
        type=float,
        default=NEUTRAL_ANGLE,
        help="Steering angle used while driving straight. 0.5 is neutral.",
    )
    parser.add_argument(
        "--drive-thruster-indices",
        default="0,1",
        help="Comma-separated thruster indices to drive while capturing.",
    )
    parser.add_argument(
        "--drive-warmup-seconds",
        type=float,
        default=0.0,
        help="Optional straight-driving warmup before the first captured frame.",
    )
    parser.add_argument(
        "--drive-resend-period",
        type=float,
        default=0.1,
        help="How often to resend straight-driving controls while waiting between captures.",
    )
    return parser.parse_args()


def normalize_text(value) -> str:
    if isinstance(value, bytes):
        return value.decode("utf-8", errors="replace")
    return str(value)


def clean_object_name(value) -> str:
    text = normalize_text(value)
    if len(text) >= 3 and text[0] == "b" and text[1] in ("'", '"') and text[-1] == text[1]:
        return text[2:-1]
    return text


def matches_prefixes(object_name: str, object_prefixes: Sequence[str]) -> bool:
    lowered_name = object_name.lower()
    return any(lowered_name.startswith(prefix.lower()) for prefix in object_prefixes)


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def parse_thruster_indices(raw_value: str) -> List[int]:
    indices: List[int] = []
    for chunk in raw_value.split(","):
        chunk = chunk.strip()
        if not chunk:
            continue
        value = int(chunk)
        if value < 0 or value >= MAX_THRUSTER_COUNT:
            raise ValueError(
                f"Thruster index {value} is out of range [0, {MAX_THRUSTER_COUNT - 1}]"
            )
        indices.append(value)
    if not indices:
        raise ValueError("At least one thruster index must be provided.")
    return sorted(set(indices))


def build_vessel_controls(
    throttle: float, steering_angle: float, thruster_indices: Sequence[int]
) -> airsim.VesselControls:
    thrust_values = [0.0] * MAX_THRUSTER_COUNT
    angle_values = [NEUTRAL_ANGLE] * MAX_THRUSTER_COUNT
    for index in thruster_indices:
        thrust_values[index] = throttle
        angle_values[index] = steering_angle
    return airsim.VesselControls(thrust_values, angle_values)


def maintain_vessel_controls(
    client: airsim.VesselClient,
    vehicle_name: str,
    controls: airsim.VesselControls,
    duration_seconds: float,
    resend_period_seconds: float,
) -> None:
    if duration_seconds <= 0:
        return

    deadline = time.time() + duration_seconds
    resend_period_seconds = max(0.02, resend_period_seconds)
    while True:
        now = time.time()
        if now >= deadline:
            break
        client.setVesselControls(vehicle_name, controls)
        time.sleep(min(resend_period_seconds, deadline - now))


def load_settings(settings_path: Path) -> Dict[str, object]:
    if not settings_path.exists():
        raise FileNotFoundError(f"settings.json not found: {settings_path}")
    return json.loads(settings_path.read_text(encoding="utf-8"))


def pick_mapping_key(
    mapping: Dict[str, object],
    explicit_name: str,
    preferred_names: Sequence[str],
    description: str,
) -> str:
    if not mapping:
        raise RuntimeError(f"No {description} entries are available.")

    if explicit_name and explicit_name.lower() != "auto":
        if explicit_name not in mapping:
            raise RuntimeError(
                f"{description} '{explicit_name}' was not found. Available: {list(mapping.keys())}"
            )
        return explicit_name

    for preferred_name in preferred_names:
        if preferred_name in mapping:
            return preferred_name

    return next(iter(mapping.keys()))


def pick_sensor_name(
    sensors: Dict[str, Dict[str, object]],
    sensor_type: int,
    explicit_name: str,
    preferred_names: Sequence[str],
    description: str,
) -> str:
    eligible = {
        name: cfg
        for name, cfg in sensors.items()
        if int(cfg.get("SensorType", -1)) == sensor_type and cfg.get("Enabled", True)
    }
    return pick_mapping_key(eligible, explicit_name, preferred_names, description)


def resolve_capture_targets(
    settings: Dict[str, object], args: argparse.Namespace
) -> Dict[str, object]:
    vehicles = settings.get("Vehicles", {})
    if not vehicles:
        raise RuntimeError("No Vehicles section found in settings.json.")

    vehicle_name = pick_mapping_key(
        vehicles, args.vehicle_name, (), "vehicle"
    )
    vehicle_cfg = vehicles[vehicle_name]

    camera_name = pick_mapping_key(
        vehicle_cfg.get("Cameras", {}),
        args.camera_name,
        PREFERRED_CAMERA_NAMES,
        "camera",
    )
    sensors = vehicle_cfg.get("Sensors", {})
    lidar_name = pick_sensor_name(
        sensors, 6, args.lidar_name, PREFERRED_LIDAR_NAMES, "LiDAR sensor"
    )
    echo_name = pick_sensor_name(
        sensors, 7, args.echo_name, PREFERRED_ECHO_NAMES, "echo sensor"
    )

    return {
        "vehicle_name": vehicle_name,
        "camera_name": camera_name,
        "lidar_name": lidar_name,
        "echo_name": echo_name,
        "vehicle_cfg": vehicle_cfg,
        "lidar_cfg": sensors[lidar_name],
        "echo_cfg": sensors[echo_name],
    }


def derive_max_age_ns(explicit_value: Optional[int], frequency_hz: float) -> int:
    if explicit_value is not None:
        return explicit_value
    if frequency_hz <= 0:
        return 500_000_000
    return int((2.0 / frequency_hz) * 1_000_000_000)


def make_run_dir(output_dir: Path) -> Path:
    run_dir = output_dir / datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
    run_dir.mkdir(parents=True, exist_ok=False)
    for subdir in ("rgb", "segmentation", "lidar", "echo", "labels", "meta"):
        (run_dir / subdir).mkdir(parents=True, exist_ok=True)
    return run_dir


def save_run_config(
    run_dir: Path, args: argparse.Namespace, resolved_targets: Dict[str, object]
) -> None:
    config_path = run_dir / "meta" / "capture_config.json"
    config_payload = {
        "args": vars(args),
        "resolved_targets": {
            key: value
            for key, value in resolved_targets.items()
            if key.endswith("_name")
        },
        "lidar_config": resolved_targets["lidar_cfg"],
        "echo_config": resolved_targets["echo_cfg"],
    }
    config_path.write_text(json.dumps(config_payload, indent=2), encoding="utf-8")


def build_object_specs(
    client: airsim.VesselClient,
    run_dir: Path,
    object_prefixes: Sequence[str],
    include_all_objects: bool,
) -> List[Dict[str, object]]:
    color_map = client.simGetSegmentationColorMap()
    object_names = client.simListInstanceSegmentationObjects()

    csv_path = run_dir / "segmentation_colormap_list.csv"
    selected_specs: List[Dict[str, object]] = []

    with csv_path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(["ObjectName", "R", "G", "B"])

        for index, raw_name in enumerate(object_names):
            raw_name_text = normalize_text(raw_name)
            clean_name = clean_object_name(raw_name_text)
            color = [int(value) for value in color_map[index, :3]]
            writer.writerow([raw_name_text, color[0], color[1], color[2]])

            if include_all_objects or matches_prefixes(clean_name, object_prefixes):
                selected_specs.append(
                    {
                        "raw_name": raw_name_text,
                        "name": clean_name,
                        "color_rgb": color,
                    }
                )

    return selected_specs


def image_response_to_array(response) -> np.ndarray:
    image = np.frombuffer(response.image_data_uint8, dtype=np.uint8)
    expected_size = response.height * response.width * 3
    if image.size != expected_size:
        raise ValueError(
            f"Unexpected image buffer size {image.size}, expected {expected_size}."
        )
    return image.reshape((response.height, response.width, 3))


def reshape_point_cloud(flat_values, column_count: int) -> np.ndarray:
    array = np.asarray(flat_values, dtype=np.float32)
    if array.size == 0:
        return np.empty((0, column_count), dtype=np.float32)
    if array.size % column_count != 0:
        return array.reshape((1, array.size))
    return array.reshape((-1, column_count))


def to_string_array(values) -> np.ndarray:
    if values is None:
        return np.asarray([], dtype=str)
    if isinstance(values, str):
        return np.asarray([values], dtype=str)
    return np.asarray([clean_object_name(value) for value in values], dtype=str)


def pose_to_dict(pose) -> Dict[str, List[float]]:
    return {
        "position": [
            float(pose.position.x_val),
            float(pose.position.y_val),
            float(pose.position.z_val),
        ],
        "orientation_xyzw": [
            float(pose.orientation.x_val),
            float(pose.orientation.y_val),
            float(pose.orientation.z_val),
            float(pose.orientation.w_val),
        ],
    }


def kinematics_to_dict(kinematics) -> Dict[str, List[float]]:
    return {
        "position": [
            float(kinematics.position.x_val),
            float(kinematics.position.y_val),
            float(kinematics.position.z_val),
        ],
        "orientation_xyzw": [
            float(kinematics.orientation.x_val),
            float(kinematics.orientation.y_val),
            float(kinematics.orientation.z_val),
            float(kinematics.orientation.w_val),
        ],
        "linear_velocity": [
            float(kinematics.linear_velocity.x_val),
            float(kinematics.linear_velocity.y_val),
            float(kinematics.linear_velocity.z_val),
        ],
        "angular_velocity": [
            float(kinematics.angular_velocity.x_val),
            float(kinematics.angular_velocity.y_val),
            float(kinematics.angular_velocity.z_val),
        ],
        "linear_acceleration": [
            float(kinematics.linear_acceleration.x_val),
            float(kinematics.linear_acceleration.y_val),
            float(kinematics.linear_acceleration.z_val),
        ],
        "angular_acceleration": [
            float(kinematics.angular_acceleration.x_val),
            float(kinematics.angular_acceleration.y_val),
            float(kinematics.angular_acceleration.z_val),
        ],
    }


def get_bounding_boxes(
    segmentation_image: np.ndarray, object_specs: Sequence[Dict[str, object]]
) -> List[Dict[str, object]]:
    labels: List[Dict[str, object]] = []

    for object_spec in object_specs:
        color = np.asarray(object_spec["color_rgb"], dtype=np.uint8)
        mask = np.all(segmentation_image == color, axis=-1)
        if not np.any(mask):
            continue

        y_indices, x_indices = np.where(mask)
        x1 = int(x_indices.min())
        x2 = int(x_indices.max())
        y1 = int(y_indices.min())
        y2 = int(y_indices.max())

        if x1 >= x2 or y1 >= y2:
            continue

        labels.append(
            {
                "name": object_spec["name"],
                "raw_name": object_spec["raw_name"],
                "bbox_xyxy": [x1, y1, x2, y2],
                "color_rgb": list(object_spec["color_rgb"]),
                "distance_m": None,
                "pixel_count": int(mask.sum()),
            }
        )

    labels.sort(key=lambda item: item["name"])
    return labels


def save_png(image_array: np.ndarray, output_path: Path) -> None:
    Image.fromarray(image_array, "RGB").save(output_path)


def wait_for_fresh_sensor_data(
    client: airsim.VesselClient,
    vehicle_name: str,
    lidar_name: str,
    echo_name: str,
    previous_lidar_ts: int,
    previous_echo_ts: int,
    timeout_seconds: float,
    poll_interval_seconds: float,
):
    start_time = time.time()
    fresh_lidar = None
    fresh_echo = None

    while time.time() - start_time < timeout_seconds:
        lidar_candidate = client.getLidarData(
            lidar_name=lidar_name, vehicle_name=vehicle_name
        )
        echo_candidate = client.getEchoData(
            echo_name=echo_name, vehicle_name=vehicle_name
        )

        if int(lidar_candidate.time_stamp) > previous_lidar_ts:
            fresh_lidar = lidar_candidate
        if int(echo_candidate.time_stamp) > previous_echo_ts:
            fresh_echo = echo_candidate

        if fresh_lidar is not None and fresh_echo is not None:
            return fresh_lidar, fresh_echo

        time.sleep(poll_interval_seconds)

    raise RuntimeError(
        "Timed out waiting for fresh LiDAR and echo scans "
        f"(prev_lidar_ts={previous_lidar_ts}, prev_echo_ts={previous_echo_ts})."
    )


def capture_images_while_paused(
    client: airsim.VesselClient,
    vehicle_name: str,
    camera_name: str,
    image_timestamp_tolerance_ns: int,
) -> Dict[str, object]:
    responses = client.simGetImages(
        [
            airsim.ImageRequest(camera_name, airsim.ImageType.Scene, False, False),
            airsim.ImageRequest(
                camera_name, airsim.ImageType.Segmentation, False, False
            ),
        ],
        vehicle_name=vehicle_name,
    )
    if len(responses) != 2:
        raise RuntimeError(
            f"Expected 2 image responses, received {len(responses)} responses."
        )

    scene_ts = int(responses[0].time_stamp)
    segmentation_ts = int(responses[1].time_stamp)
    image_pair_diff_ns = abs(segmentation_ts - scene_ts)
    if image_pair_diff_ns > image_timestamp_tolerance_ns:
        raise RuntimeError(
            "Scene/Segmentation timestamp delta exceeded tolerance: "
            f"{image_pair_diff_ns} ns > {image_timestamp_tolerance_ns} ns"
        )

    return {
        "scene_image": image_response_to_array(responses[0]),
        "segmentation_image": image_response_to_array(responses[1]),
        "scene_timestamp": scene_ts,
        "segmentation_timestamp": segmentation_ts,
        "image_timestamp": max(scene_ts, segmentation_ts),
        "image_pair_diff_ns": image_pair_diff_ns,
    }


def write_point_cloud_bundle(
    output_path: Path,
    point_cloud: np.ndarray,
    groundtruth: np.ndarray,
    pose: Dict[str, List[float]],
    time_stamp: int,
    **extra_arrays,
) -> None:
    np.savez_compressed(
        output_path,
        point_cloud=point_cloud,
        groundtruth=groundtruth,
        position=np.asarray(pose["position"], dtype=np.float32),
        orientation_xyzw=np.asarray(pose["orientation_xyzw"], dtype=np.float32),
        time_stamp=np.uint64(time_stamp),
        **extra_arrays,
    )


def write_frame_index_header(csv_path: Path) -> None:
    with csv_path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(
            [
                "frame_id",
                "scene_timestamp",
                "segmentation_timestamp",
                "image_timestamp",
                "lidar_timestamp",
                "echo_timestamp",
                "image_pair_diff_ns",
                "lidar_age_ns",
                "echo_age_ns",
                "rgb_path",
                "segmentation_path",
                "lidar_path",
                "echo_path",
                "label_path",
                "object_count",
            ]
        )


def append_frame_index(csv_path: Path, row: Iterable[object]) -> None:
    with csv_path.open("a", newline="", encoding="utf-8") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(list(row))


def relative_to_run_dir(run_dir: Path, path: Path) -> str:
    return path.relative_to(run_dir).as_posix()


def main() -> int:
    args = parse_args()
    settings = load_settings(Path(args.settings_path).expanduser())
    resolved_targets = resolve_capture_targets(settings, args)
    drive_enabled = args.drive_straight_throttle > 0.0
    drive_thruster_indices = parse_thruster_indices(args.drive_thruster_indices)
    drive_controls = build_vessel_controls(
        throttle=clamp(args.drive_straight_throttle, 0.0, 1.0),
        steering_angle=clamp(args.drive_steering_angle, 0.0, 1.0),
        thruster_indices=drive_thruster_indices,
    )
    stop_controls = build_vessel_controls(
        throttle=0.0,
        steering_angle=NEUTRAL_ANGLE,
        thruster_indices=drive_thruster_indices,
    )

    lidar_rate_hz = float(resolved_targets["lidar_cfg"].get("RotationsPerSecond", 0.0))
    echo_rate_hz = float(
        resolved_targets["echo_cfg"].get("MeasurementFrequency", 0.0)
    )
    max_lidar_age_ns = derive_max_age_ns(args.max_lidar_age_ns, lidar_rate_hz)
    max_echo_age_ns = derive_max_age_ns(args.max_echo_age_ns, echo_rate_hz)

    output_dir = Path(args.output_dir).resolve()
    run_dir = make_run_dir(output_dir)
    save_run_config(run_dir, args, resolved_targets)

    client = airsim.VesselClient(ip=args.ip, port=args.port)
    client.confirmConnection()

    vehicle_name = resolved_targets["vehicle_name"]
    camera_name = resolved_targets["camera_name"]
    lidar_name = resolved_targets["lidar_name"]
    echo_name = resolved_targets["echo_name"]

    if args.enable_api_control or drive_enabled:
        client.enableApiControl(True, vehicle_name)

    object_specs = build_object_specs(
        client=client,
        run_dir=run_dir,
        object_prefixes=args.object_prefix,
        include_all_objects=args.include_all_objects,
    )
    print(
        "Capture targets:",
        f"vehicle={vehicle_name}",
        f"camera={camera_name}",
        f"lidar={lidar_name}",
        f"echo={echo_name}",
    )
    print(
        f"Selected {len(object_specs)} segmentation objects for detection labels. "
        f"Max ages: lidar={max_lidar_age_ns} ns echo={max_echo_age_ns} ns"
    )
    if drive_enabled:
        print(
            "Straight-drive capture enabled:",
            f"throttle={args.drive_straight_throttle:.2f}",
            f"steering_angle={args.drive_steering_angle:.2f}",
            f"thrusters={drive_thruster_indices}",
        )

    frame_index_path = run_dir / "meta" / "frame_index.csv"
    write_frame_index_header(frame_index_path)

    previous_lidar_ts = 0
    previous_echo_ts = 0

    try:
        client.simPause(False)
        if drive_enabled:
            client.setVesselControls(vehicle_name, drive_controls)
            maintain_vessel_controls(
                client=client,
                vehicle_name=vehicle_name,
                controls=drive_controls,
                duration_seconds=args.drive_warmup_seconds,
                resend_period_seconds=args.drive_resend_period,
            )

        for frame_id in range(args.num_frames):
            if frame_id > 0 and args.capture_interval > 0:
                if drive_enabled:
                    maintain_vessel_controls(
                        client=client,
                        vehicle_name=vehicle_name,
                        controls=drive_controls,
                        duration_seconds=args.capture_interval,
                        resend_period_seconds=args.drive_resend_period,
                    )
                else:
                    time.sleep(args.capture_interval)

            frame_capture = None

            for attempt in range(1, args.retry_limit + 1):
                client.simPause(False)
                if drive_enabled:
                    client.setVesselControls(vehicle_name, drive_controls)
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
                previous_lidar_ts = int(lidar_data.time_stamp)
                previous_echo_ts = int(echo_data.time_stamp)

                client.simPause(True)
                try:
                    image_capture = capture_images_while_paused(
                        client=client,
                        vehicle_name=vehicle_name,
                        camera_name=camera_name,
                        image_timestamp_tolerance_ns=args.image_timestamp_tolerance_ns,
                    )
                    vehicle_pose = client.simGetVehiclePose(vehicle_name)
                    ground_truth_kinematics = client.simGetGroundTruthKinematics(
                        vehicle_name
                    )
                finally:
                    client.simPause(False)

                image_ts = int(image_capture["image_timestamp"])
                lidar_ts = int(lidar_data.time_stamp)
                echo_ts = int(echo_data.time_stamp)
                lidar_age_ns = image_ts - lidar_ts
                echo_age_ns = image_ts - echo_ts

                if 0 <= lidar_age_ns <= max_lidar_age_ns and 0 <= echo_age_ns <= max_echo_age_ns:
                    frame_capture = {
                        "image_capture": image_capture,
                        "lidar_data": lidar_data,
                        "echo_data": echo_data,
                        "vehicle_pose": vehicle_pose,
                        "ground_truth_kinematics": ground_truth_kinematics,
                        "lidar_age_ns": lidar_age_ns,
                        "echo_age_ns": echo_age_ns,
                    }
                    break

                if attempt == args.retry_limit:
                    raise RuntimeError(
                        "Sensor age validation failed for frame "
                        f"{frame_id}: lidar_age_ns={lidar_age_ns}, echo_age_ns={echo_age_ns}, "
                        f"limits=({max_lidar_age_ns}, {max_echo_age_ns})"
                    )

                time.sleep(args.poll_interval)

            if frame_capture is None:
                raise RuntimeError(f"Failed to capture frame {frame_id}.")

            scene_image = frame_capture["image_capture"]["scene_image"]
            segmentation_image = frame_capture["image_capture"]["segmentation_image"]
            lidar_data = frame_capture["lidar_data"]
            echo_data = frame_capture["echo_data"]

            labels = get_bounding_boxes(segmentation_image, object_specs)

            rgb_path = run_dir / "rgb" / f"{frame_id:06d}.png"
            segmentation_path = run_dir / "segmentation" / f"{frame_id:06d}.png"
            lidar_path = run_dir / "lidar" / f"{frame_id:06d}.npz"
            echo_path = run_dir / "echo" / f"{frame_id:06d}.npz"
            label_path = run_dir / "labels" / f"{frame_id:06d}.json"

            save_png(scene_image, rgb_path)
            save_png(segmentation_image, segmentation_path)

            lidar_pose = pose_to_dict(lidar_data.pose)
            echo_pose = pose_to_dict(echo_data.pose)

            write_point_cloud_bundle(
                lidar_path,
                point_cloud=reshape_point_cloud(lidar_data.point_cloud, 3),
                groundtruth=to_string_array(lidar_data.groundtruth),
                pose=lidar_pose,
                time_stamp=int(lidar_data.time_stamp),
            )

            write_point_cloud_bundle(
                echo_path,
                point_cloud=reshape_point_cloud(echo_data.point_cloud, 6),
                groundtruth=to_string_array(echo_data.groundtruth),
                pose=echo_pose,
                time_stamp=int(echo_data.time_stamp),
                passive_beacons_point_cloud=reshape_point_cloud(
                    echo_data.passive_beacons_point_cloud, 9
                ),
                passive_beacons_groundtruth=to_string_array(
                    echo_data.passive_beacons_groundtruth
                ),
            )

            label_payload = {
                "frame_id": frame_id,
                "capture_targets": {
                    "vehicle_name": vehicle_name,
                    "camera_name": camera_name,
                    "lidar_name": lidar_name,
                    "echo_name": echo_name,
                },
                "timestamps": {
                    "scene": int(frame_capture["image_capture"]["scene_timestamp"]),
                    "segmentation": int(
                        frame_capture["image_capture"]["segmentation_timestamp"]
                    ),
                    "image": int(frame_capture["image_capture"]["image_timestamp"]),
                    "lidar": int(lidar_data.time_stamp),
                    "echo": int(echo_data.time_stamp),
                },
                "timestamp_deltas_ns": {
                    "image_pair_diff_ns": int(
                        frame_capture["image_capture"]["image_pair_diff_ns"]
                    ),
                    "lidar_age_ns": int(frame_capture["lidar_age_ns"]),
                    "echo_age_ns": int(frame_capture["echo_age_ns"]),
                },
                "vehicle_pose": pose_to_dict(frame_capture["vehicle_pose"]),
                "vehicle_kinematics": kinematics_to_dict(
                    frame_capture["ground_truth_kinematics"]
                ),
                "rgb_path": relative_to_run_dir(run_dir, rgb_path),
                "segmentation_path": relative_to_run_dir(run_dir, segmentation_path),
                "lidar_path": relative_to_run_dir(run_dir, lidar_path),
                "echo_path": relative_to_run_dir(run_dir, echo_path),
                "objects": labels,
            }
            label_path.write_text(
                json.dumps(label_payload, indent=2), encoding="utf-8"
            )

            append_frame_index(
                frame_index_path,
                [
                    frame_id,
                    int(frame_capture["image_capture"]["scene_timestamp"]),
                    int(frame_capture["image_capture"]["segmentation_timestamp"]),
                    int(frame_capture["image_capture"]["image_timestamp"]),
                    int(lidar_data.time_stamp),
                    int(echo_data.time_stamp),
                    int(frame_capture["image_capture"]["image_pair_diff_ns"]),
                    int(frame_capture["lidar_age_ns"]),
                    int(frame_capture["echo_age_ns"]),
                    relative_to_run_dir(run_dir, rgb_path),
                    relative_to_run_dir(run_dir, segmentation_path),
                    relative_to_run_dir(run_dir, lidar_path),
                    relative_to_run_dir(run_dir, echo_path),
                    relative_to_run_dir(run_dir, label_path),
                    len(labels),
                ],
            )

            print(
                "Captured frame "
                f"{frame_id:06d} image_ts={frame_capture['image_capture']['image_timestamp']} "
                f"lidar_age_ns={frame_capture['lidar_age_ns']} "
                f"echo_age_ns={frame_capture['echo_age_ns']} objects={len(labels)}"
            )
    finally:
        client.simPause(False)
        if drive_enabled:
            client.setVesselControls(vehicle_name, stop_controls)
        if args.enable_api_control or drive_enabled:
            client.enableApiControl(False, vehicle_name)

    print(f"Dataset written to: {run_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
