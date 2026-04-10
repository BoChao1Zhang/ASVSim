"""
Keyboard teleoperation for Cosys-AirSim vessels on Windows.

Controls:
    Up / W      Increase throttle
    Down / S    Decrease throttle
    Left / A    Turn left while held
    Right / D   Turn right while held
    Space       Emergency stop
    Esc         Exit
"""

import argparse
import ctypes
import sys
import time
from typing import List

import setup_path
import cosysairsim as airsim
from cosysairsim.types import VesselControls


VK_UP = 0x26
VK_DOWN = 0x28
VK_LEFT = 0x25
VK_RIGHT = 0x27
VK_ESCAPE = 0x1B
VK_SPACE = 0x20
VK_W = 0x57
VK_A = 0x41
VK_S = 0x53
VK_D = 0x44

NEUTRAL_ANGLE = 0.5
MAX_THRUSTER_COUNT = 10


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Manual vessel control using arrow keys or WASD."
    )
    parser.add_argument("--ip", default="127.0.0.1", help="RPC server IP.")
    parser.add_argument("--port", type=int, default=41451, help="RPC server port.")
    parser.add_argument(
        "--vehicle-name",
        default="",
        help="AirSim vehicle name. Empty string uses the default vessel.",
    )
    parser.add_argument(
        "--poll-hz",
        type=float,
        default=20.0,
        help="Control loop frequency in Hz.",
    )
    parser.add_argument(
        "--throttle-step",
        type=float,
        default=0.03,
        help="Throttle increment applied per control update while Up/Down is held.",
    )
    parser.add_argument(
        "--max-throttle",
        type=float,
        default=1.0,
        help="Maximum throttle value.",
    )
    parser.add_argument(
        "--turn-delta",
        type=float,
        default=0.15,
        help="Steering delta around the neutral angle 0.5.",
    )
    parser.add_argument(
        "--start-throttle",
        type=float,
        default=0.0,
        help="Initial throttle value on startup.",
    )
    parser.add_argument(
        "--thruster-indices",
        default="0,1",
        help="Comma-separated thruster indices that receive the same control value.",
    )
    parser.add_argument(
        "--status-interval",
        type=float,
        default=0.5,
        help="How often to print status updates in seconds.",
    )
    return parser.parse_args()


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


def is_pressed(user32, virtual_key: int) -> bool:
    return bool(user32.GetAsyncKeyState(virtual_key) & 0x8000)


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def build_vessel_controls(
    throttle: float, steering_angle: float, thruster_indices: List[int]
) -> VesselControls:
    thrust_values = [0.0] * MAX_THRUSTER_COUNT
    angle_values = [NEUTRAL_ANGLE] * MAX_THRUSTER_COUNT

    for index in thruster_indices:
        thrust_values[index] = throttle
        angle_values[index] = steering_angle

    return VesselControls(thrust_values, angle_values)


def main() -> int:
    if sys.platform != "win32":
        print("This teleop script currently supports Windows only.", file=sys.stderr)
        return 1

    args = parse_args()
    thruster_indices = parse_thruster_indices(args.thruster_indices)
    user32 = ctypes.windll.user32

    client = airsim.VesselClient(ip=args.ip, port=args.port)
    client.confirmConnection()
    client.enableApiControl(True, args.vehicle_name)

    throttle = clamp(args.start_throttle, 0.0, args.max_throttle)
    steering_angle = NEUTRAL_ANGLE
    loop_sleep = 1.0 / max(args.poll_hz, 1.0)
    last_status_print = 0.0

    print("Teleop active. Use arrow keys or WASD, Space to stop, Esc to exit.")

    try:
        while True:
            if is_pressed(user32, VK_ESCAPE):
                break

            increase_throttle = is_pressed(user32, VK_UP) or is_pressed(user32, VK_W)
            decrease_throttle = is_pressed(user32, VK_DOWN) or is_pressed(user32, VK_S)
            steer_left = is_pressed(user32, VK_LEFT) or is_pressed(user32, VK_A)
            steer_right = is_pressed(user32, VK_RIGHT) or is_pressed(user32, VK_D)
            emergency_stop = is_pressed(user32, VK_SPACE)

            if emergency_stop:
                throttle = 0.0

            if increase_throttle and not decrease_throttle:
                throttle = clamp(throttle + args.throttle_step, 0.0, args.max_throttle)
            elif decrease_throttle and not increase_throttle:
                throttle = clamp(throttle - args.throttle_step, 0.0, args.max_throttle)

            steering_angle = NEUTRAL_ANGLE
            if steer_left and not steer_right:
                steering_angle = clamp(
                    NEUTRAL_ANGLE - args.turn_delta, 0.0, 1.0
                )
            elif steer_right and not steer_left:
                steering_angle = clamp(
                    NEUTRAL_ANGLE + args.turn_delta, 0.0, 1.0
                )

            controls = build_vessel_controls(throttle, steering_angle, thruster_indices)
            client.setVesselControls(args.vehicle_name, controls)

            now = time.time()
            if now - last_status_print >= args.status_interval:
                print(
                    f"throttle={throttle:.2f} steering_angle={steering_angle:.2f} "
                    f"thrusters={thruster_indices}"
                )
                last_status_print = now

            time.sleep(loop_sleep)
    finally:
        stop_controls = build_vessel_controls(0.0, NEUTRAL_ANGLE, thruster_indices)
        client.setVesselControls(args.vehicle_name, stop_controls)
        client.enableApiControl(False, args.vehicle_name)
        print("Teleop stopped. Vessel controls reset.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
