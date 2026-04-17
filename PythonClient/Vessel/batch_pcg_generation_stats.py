"""
Batch PCG generation stats utility for a running PIE/editor vessel session.

Runs multiple seeds and terrain lengths against the current runtime, collects
waypoint counts plus obstacle-count deltas, and writes a JSON/CSV summary.
Designed for GenerationTopDownTest after PIE is already running.
"""

import argparse
import csv
import json
import math
import statistics
import time
from datetime import datetime
from pathlib import Path

import setup_path
import cosysairsim as airsim

from test_pcg_generation import (
    GOAL_EPSILON,
    build_stop_controls,
    cleanup_runtime,
    collect_waypoints,
    reset_vessel_runtime,
)


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parents[1]
OUTPUT_ROOT = REPO_ROOT / "data" / "vessel" / "pcg_batch_stats"
OBSTACLE_PATTERNS = [
    ".*BP_BuoySpawn.*",
    ".*Boat_Blueprint.*",
    ".*Barge_Blueprint.*",
    ".*BP_CargoPawn.*",
    ".*BP_NPCSpawn.*",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Batch-run PCG terrain generation stats over multiple seeds.")
    parser.add_argument("--ip", default="127.0.0.1", help="RPC server IP.")
    parser.add_argument("--port", type=int, default=41451, help="RPC server port.")
    parser.add_argument("--vehicle-name", default="Vessel1", help="Vehicle name.")
    parser.add_argument("--terrain-type", default="port", help="Terrain type.")
    parser.add_argument("--lengths", type=int, nargs="+", default=[1, 3, 10], help="Terrain lengths to test.")
    parser.add_argument("--seeds", type=int, nargs="+", default=[101, 202, 303, 404, 505], help="Seeds to test.")
    parser.add_argument("--min-angle", type=float, default=-45.0)
    parser.add_argument("--max-angle", type=float, default=45.0)
    parser.add_argument("--min-distance-cm", type=float, default=5000.0)
    parser.add_argument("--max-distance-cm", type=float, default=10000.0)
    parser.add_argument("--max-sections", type=int, default=64)
    parser.add_argument("--goal-retry-count", type=int, default=10)
    parser.add_argument("--goal-retry-sleep", type=float, default=0.5)
    parser.add_argument("--generation-settle-seconds", type=float, default=2.0)
    parser.add_argument("--output-dir", default=str(OUTPUT_ROOT))
    return parser.parse_args()


def snapshot_obstacles(client: airsim.VesselClient) -> set[str]:
    names: set[str] = set()
    for pattern in OBSTACLE_PATTERNS:
        try:
            names.update(client.simListSceneObjects(pattern))
        except Exception as exc:
            print(f"WARNING: simListSceneObjects({pattern}) failed: {exc}")
    return names


def run_case(client: airsim.VesselClient, args: argparse.Namespace, stop_controls, seed: int, terrain_length: int) -> dict:
    start_pose = reset_vessel_runtime(client, args.vehicle_name, stop_controls)
    start_xy = (float(start_pose.position.x_val), float(start_pose.position.y_val))
    initial_location = airsim.Vector2r(start_xy[0] + GOAL_EPSILON, start_xy[1] + GOAL_EPSILON)

    if not client.activateGeneration(False):
        raise RuntimeError("activateGeneration(False) failed.")

    before = snapshot_obstacles(client)
    terrain_ok = client.generatePortTerrain(
        args.terrain_type,
        seed,
        terrain_length,
        args.min_angle,
        args.max_angle,
        args.min_distance_cm,
        args.max_distance_cm,
    )
    if not terrain_ok:
        raise RuntimeError("generatePortTerrain returned False.")

    time.sleep(args.generation_settle_seconds)
    waypoints, _boundaries = collect_waypoints(
        client=client,
        initial_location=initial_location,
        max_sections=args.max_sections,
        retry_count=args.goal_retry_count,
        retry_sleep=args.goal_retry_sleep,
    )
    after = snapshot_obstacles(client)
    spawned = after - before

    final_goal = waypoints[-1]
    return {
        "seed": seed,
        "terrain_length": terrain_length,
        "waypoint_count": len(waypoints),
        "obstacle_count": len(spawned),
        "final_goal_x_m": final_goal[0],
        "final_goal_y_m": final_goal[1],
        "final_goal_distance_m": math.hypot(final_goal[0], final_goal[1]),
        "success": True,
        "error": "",
    }


def summarize(rows: list[dict]) -> dict:
    summary: dict[str, dict] = {}
    lengths = sorted({row["terrain_length"] for row in rows})
    for terrain_length in lengths:
        group = [row for row in rows if row["terrain_length"] == terrain_length]
        successes = [row for row in group if row["success"]]
        obstacle_counts = [row["obstacle_count"] for row in successes]
        waypoint_counts = [row["waypoint_count"] for row in successes]
        goal_distances = [row["final_goal_distance_m"] for row in successes]
        summary[str(terrain_length)] = {
            "runs": len(group),
            "successes": len(successes),
            "failures": len(group) - len(successes),
            "obstacle_count_min": min(obstacle_counts) if obstacle_counts else None,
            "obstacle_count_max": max(obstacle_counts) if obstacle_counts else None,
            "obstacle_count_mean": statistics.fmean(obstacle_counts) if obstacle_counts else None,
            "waypoint_count_min": min(waypoint_counts) if waypoint_counts else None,
            "waypoint_count_max": max(waypoint_counts) if waypoint_counts else None,
            "waypoint_count_mean": statistics.fmean(waypoint_counts) if waypoint_counts else None,
            "goal_distance_mean_m": statistics.fmean(goal_distances) if goal_distances else None,
        }
    return summary


def main() -> int:
    args = parse_args()
    output_root = Path(args.output_dir).resolve()
    run_dir = output_root / datetime.now().strftime("pcg_batch_%Y_%m_%d_%H_%M_%S")
    run_dir.mkdir(parents=True, exist_ok=False)

    client = airsim.VesselClient(ip=args.ip, port=args.port)
    client.confirmConnection()
    stop_controls = build_stop_controls()

    rows: list[dict] = []
    try:
        for terrain_length in args.lengths:
            for seed in args.seeds:
                print(f"Running terrain_length={terrain_length} seed={seed}")
                row = {"seed": seed, "terrain_length": terrain_length, "success": False, "error": ""}
                try:
                    row.update(run_case(client, args, stop_controls, seed, terrain_length))
                except Exception as exc:
                    row["error"] = str(exc)
                    print(f"  FAILED: {exc}")
                rows.append(row)
    finally:
        try:
            cleanup_runtime(client, args.vehicle_name, stop_controls)
        except Exception:
            pass

    summary = summarize(rows)

    summary_path = run_dir / "summary.json"
    summary_path.write_text(json.dumps({"rows": rows, "summary": summary}, indent=2), encoding="utf-8")

    csv_path = run_dir / "runs.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        fieldnames = [
            "terrain_length",
            "seed",
            "success",
            "waypoint_count",
            "obstacle_count",
            "final_goal_distance_m",
            "error",
        ]
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({
                "terrain_length": row.get("terrain_length"),
                "seed": row.get("seed"),
                "success": row.get("success"),
                "waypoint_count": row.get("waypoint_count"),
                "obstacle_count": row.get("obstacle_count"),
                "final_goal_distance_m": row.get("final_goal_distance_m"),
                "error": row.get("error", ""),
            })

    print(f"Batch stats written to: {run_dir}")
    for terrain_length, item in summary.items():
        print(
            f"length={terrain_length} runs={item['runs']} successes={item['successes']} "
            f"obstacles[min/mean/max]={item['obstacle_count_min']}/{item['obstacle_count_mean']}/{item['obstacle_count_max']} "
            f"waypoints[min/mean/max]={item['waypoint_count_min']}/{item['waypoint_count_mean']}/{item['waypoint_count_max']}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
