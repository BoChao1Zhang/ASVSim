"""
Drive the vessel through a short deterministic maneuver for capture demos.
"""

import argparse
import sys
import time
from pathlib import Path
from typing import Iterable, List, Tuple


SCRIPT_DIR = Path(__file__).resolve().parent
PYTHONCLIENT_DIR = SCRIPT_DIR.parent

if str(PYTHONCLIENT_DIR) not in sys.path:
    sys.path.insert(0, str(PYTHONCLIENT_DIR))

import cosysairsim as airsim
from cosysairsim.types import VesselControls


NEUTRAL_ANGLE = 0.5
MAX_THRUSTER_COUNT = 10


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Drive the vessel through a short, fixed maneuver."
    )
    parser.add_argument("--ip", default="127.0.0.1", help="RPC server IP.")
    parser.add_argument("--port", type=int, default=41451, help="RPC server port.")
    parser.add_argument(
        "--vehicle-name",
        default="Vessel1",
        help="AirSim vehicle name.",
    )
    parser.add_argument(
        "--thruster-indices",
        default="0,1",
        help="Comma-separated thruster indices to drive.",
    )
    parser.add_argument(
        "--step-seconds",
        type=float,
        default=0.1,
        help="Control update interval in seconds.",
    )
    parser.add_argument(
        "--warmup-seconds",
        type=float,
        default=0.6,
        help="Initial delay before the maneuver starts.",
    )
    return parser.parse_args()


def parse_thruster_indices(raw_value: str) -> List[int]:
    result: List[int] = []
    for chunk in raw_value.split(","):
        chunk = chunk.strip()
        if not chunk:
            continue
        index = int(chunk)
        if index < 0 or index >= MAX_THRUSTER_COUNT:
            raise ValueError(
                f"Thruster index {index} is out of range [0, {MAX_THRUSTER_COUNT - 1}]"
            )
        result.append(index)
    if not result:
        raise ValueError("At least one thruster index must be provided.")
    return sorted(set(result))


def build_controls(
    throttle: float, steering_angle: float, thruster_indices: Iterable[int]
) -> VesselControls:
    thrust_values = [0.0] * MAX_THRUSTER_COUNT
    angle_values = [NEUTRAL_ANGLE] * MAX_THRUSTER_COUNT
    for index in thruster_indices:
        thrust_values[index] = throttle
        angle_values[index] = steering_angle
    return VesselControls(thrust_values, angle_values)


def hold_segment(
    client: airsim.VesselClient,
    vehicle_name: str,
    controls: VesselControls,
    duration_seconds: float,
    step_seconds: float,
) -> None:
    iterations = max(1, int(round(duration_seconds / max(step_seconds, 0.02))))
    for _ in range(iterations):
        client.setVesselControls(vehicle_name, controls)
        time.sleep(step_seconds)


def main() -> int:
    args = parse_args()
    thruster_indices = parse_thruster_indices(args.thruster_indices)

    client = airsim.VesselClient(ip=args.ip, port=args.port)
    client.confirmConnection()
    client.enableApiControl(True, args.vehicle_name)

    schedule: List[Tuple[float, float, float, str]] = [
        (2.4, 0.36, 0.50, "straight_1"),
        (2.2, 0.40, 0.38, "left_sweep"),
        (2.2, 0.40, 0.62, "right_sweep"),
        (2.0, 0.34, 0.50, "straight_2"),
    ]

    stop_controls = build_controls(0.0, NEUTRAL_ANGLE, thruster_indices)

    try:
        time.sleep(max(0.0, args.warmup_seconds))
        for duration_seconds, throttle, steering_angle, label in schedule:
            controls = build_controls(throttle, steering_angle, thruster_indices)
            print(
                f"segment={label} duration={duration_seconds:.1f}s "
                f"throttle={throttle:.2f} steering={steering_angle:.2f}"
            )
            hold_segment(
                client=client,
                vehicle_name=args.vehicle_name,
                controls=controls,
                duration_seconds=duration_seconds,
                step_seconds=args.step_seconds,
            )
    finally:
        client.setVesselControls(args.vehicle_name, stop_controls)
        client.enableApiControl(False, args.vehicle_name)
        print("drive pattern complete")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
