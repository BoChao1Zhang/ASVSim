import setup_path

import argparse
import math
import os
import subprocess
import time
from pathlib import Path

import numpy as np
import pandas as pd
from sb3_contrib import CrossQ
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

from airgym.envs.vessel_env import PCGVesselEnv
from config import DEFAULT_CONFIG, ensure_known_cli_tokens, load_config, resolve_simulator_path, validate_config
from diagnostics import diag_log
from plot_trajectories import plot_trajectory_file

try:
    import wandb
except ImportError:  # pragma: no cover - optional dependency
    wandb = None


def parse_args():
    parser = argparse.ArgumentParser(description="Evaluate a trained vessel policy across the curriculum suite")
    parser.add_argument("--run-dir", type=str, default=None, help="Run directory containing config.yaml and vecnorm.pkl")
    parser.add_argument("--checkpoint", type=str, default=None, help="Optional checkpoint override. Defaults to run-dir/final_model.zip")
    parser.add_argument("--config", type=str, default=str(DEFAULT_CONFIG), help="Fallback config path if run-dir cannot be inferred")
    parser.add_argument("--stage", type=str, default=None, help="Evaluate only one named stage")
    parser.add_argument("--seeds", type=int, default=None, help="Override number of seeds per stage")
    parser.add_argument("--episodes-per-seed", type=int, default=None, help="Override episodes per seed")
    parser.add_argument("--output-csv", type=str, default=None, help="CSV path. Defaults to run-dir/eval_suite_results.csv")
    parser.add_argument("--deterministic", action="store_true", default=False)

    # Legacy compatibility flags from eval_vessel.py
    parser.add_argument("--episodes", type=int, default=None)
    parser.add_argument("--ip", type=str, default=None)
    parser.add_argument("--terrain-regen", type=int, default=None)
    parser.add_argument("--num-obstacles", type=int, default=None)
    parser.add_argument("--num-dynamic-obstacles", type=int, default=None)
    parser.add_argument("--num-waypoints", type=int, default=None)
    parser.add_argument("--launch-sim", choices=("none", "exe"), default=None)
    parser.add_argument("--sim-path", type=str, default=None)
    parser.add_argument("--sim-wait", type=int, default=None)
    parser.add_argument("--action-repeat", type=int, default=None)
    parser.add_argument("--seed", type=int, default=None)

    parser.add_argument("overrides", nargs="*", help="OmegaConf dotlist overrides")
    args, unknown = parser.parse_known_args()
    ensure_known_cli_tokens(unknown)
    return args


def legacy_eval_overrides(args) -> list[str]:
    overrides = list(args.overrides)
    mapping = {
        "ip": "env.ip_address",
        "terrain_regen": "env.terrain_regen_interval",
        "num_obstacles": "env.num_obstacles",
        "num_dynamic_obstacles": "env.num_dynamic_obstacles",
        "num_waypoints": "env.num_waypoints",
        "launch_sim": "env.launch_sim",
        "sim_path": "env.sim_path",
        "sim_wait": "env.sim_wait",
        "action_repeat": "env.action_repeat",
        "seed": "train.seed",
    }
    for arg_name, key in mapping.items():
        value = getattr(args, arg_name)
        if value is not None:
            overrides.append(f"{key}={value}")
    return overrides


def resolve_run_dir(args) -> Path | None:
    if args.run_dir:
        return Path(args.run_dir).resolve()
    if not args.checkpoint:
        return None

    checkpoint_path = Path(args.checkpoint).resolve()
    if checkpoint_path.parent.name == "checkpoints":
        return checkpoint_path.parent.parent
    if checkpoint_path.parent.joinpath("config.yaml").exists():
        return checkpoint_path.parent
    return None


def resolve_checkpoint(run_dir: Path | None, checkpoint_arg: str | None) -> Path:
    if checkpoint_arg:
        return Path(checkpoint_arg).resolve()
    if run_dir is None:
        raise ValueError("--checkpoint is required when --run-dir cannot be inferred")

    candidates = [run_dir / "final_model.zip"]
    candidates.extend(sorted((run_dir / "checkpoints").glob("*_final_model.zip")))
    candidates.extend(sorted((run_dir / "checkpoints").glob("*_steps.zip")))
    for candidate in reversed(candidates):
        if candidate.exists():
            return candidate
    raise FileNotFoundError(f"No checkpoint found under {run_dir}")


def start_simulator(sim_path: str, wait: int = 10):
    sim_path = os.path.abspath(sim_path)
    if not os.path.isfile(sim_path):
        raise FileNotFoundError(f"Simulator executable not found: {sim_path}")

    print(f"Starting simulator executable: {sim_path}")
    proc = subprocess.Popen([sim_path], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    print(f"Simulator started (PID: {proc.pid}), waiting {wait}s for initialization...")
    time.sleep(wait)
    return proc


def stop_simulator(proc):
    if proc is None:
        return
    proc.terminate()
    try:
        proc.wait(timeout=15)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()


def make_env(config):
    def _init():
        max_timesteps = max(1, int(config.env.max_timesteps) // int(config.env.action_repeat))
        env = PCGVesselEnv(
            ip_address=config.env.ip_address,
            terrain_regen_interval=int(config.env.terrain_regen_interval),
            num_obstacles=int(config.env.num_obstacles),
            num_dynamic_obstacles=int(config.env.num_dynamic_obstacles),
            goal_distance=int(config.env.num_waypoints),
            max_timesteps=max_timesteps,
            step_sleep=float(config.env.step_sleep),
            action_repeat=int(config.env.action_repeat),
            seed=int(config.train.seed),
            sim_path=config.env.sim_path,
            sim_wait=int(config.env.sim_wait),
            sim_launch_mode=config.env.launch_sim,
            use_c_side_pcg_obstacles=bool(config.env.use_c_side_pcg_obstacles),
            terrain_length=int(config.env.terrain_length),
            angle_range=list(config.env.angle_range),
            terrain_min_width_cm=float(config.env.terrain_min_width_cm),
            terrain_max_width_cm=float(config.env.terrain_max_width_cm),
            reward_config=config.reward,
            n_stack=int(config.env.n_stack),
            lidar_noise_sigma=float(config.env.lidar_noise_sigma),
            heading_noise_sigma=float(config.env.heading_noise_sigma),
            waypoint_radius=float(config.env.waypoint_radius),
            yaw_angle_scale=float(config.env.yaw_angle_scale),
        )
        return Monitor(env)

    return _init


def unwrap_env(env):
    current = env
    while hasattr(current, "env"):
        current = current.env
    return current


def build_stage_specs(config, single_stage: bool = False, stage_name: str | None = None):
    if single_stage or not config.curriculum.stages:
        return [
            {
                "name": stage_name or "configured",
                "num_obstacles": int(config.env.num_obstacles),
                "num_dynamic_obstacles": int(config.env.num_dynamic_obstacles),
                "num_waypoints": int(config.env.num_waypoints),
                "length": int(config.env.terrain_length),
                "angle_range": [float(config.env.angle_range[0]), float(config.env.angle_range[1])],
            }
        ]

    stages = []
    for stage in config.curriculum.stages:
        if stage_name is not None and stage.name != stage_name:
            continue
        stages.append(
            {
                "name": stage.name,
                "num_obstacles": int(stage.num_obstacles),
                "num_dynamic_obstacles": int(stage.num_dynamic_obstacles),
                "num_waypoints": int(stage.num_waypoints),
                "length": int(stage.length),
                "angle_range": [float(stage.angle_range[0]), float(stage.angle_range[1])],
            }
        )
    if stage_name is not None and not stages:
        raise ValueError(f"Unknown stage '{stage_name}'")
    return stages


def create_eval_env(config, vecnormalize_path: Path | None):
    env = DummyVecEnv([make_env(config)])
    if vecnormalize_path is not None and vecnormalize_path.exists():
        env = VecNormalize.load(str(vecnormalize_path), env)
        env.training = False
        env.norm_reward = False
        return env
    env = VecNormalize(
        env,
        norm_obs=bool(config.algo.norm_obs),
        norm_reward=False,
        clip_obs=float(config.algo.clip_obs),
    )
    env.training = False
    env.norm_reward = False
    return env


def load_model(config, checkpoint_path: Path, env):
    if str(config.algo.name).lower() != "crossq":
        raise ValueError(f"Unsupported algorithm: {config.algo.name}")
    return CrossQ.load(str(checkpoint_path), env=env, device=config.algo.device)


def dump_trajectory(info: dict, output_dir: Path, stage_name: str, reason: str) -> Path:
    trajectories_dir = output_dir / "trajectories"
    trajectories_dir.mkdir(parents=True, exist_ok=True)

    trajectory = np.asarray(info.get("episode_trajectory", []), dtype=np.float32)
    waypoints = np.asarray(info.get("waypoints", []), dtype=np.float32)

    obstacles = info.get("obstacles", [])
    obstacle_array = np.asarray(
        [[float(item.get("x", 0.0)), float(item.get("y", 0.0)), 1.0 if item.get("dynamic", False) else 0.0] for item in obstacles],
        dtype=np.float32,
    )
    npz_path = trajectories_dir / f"{stage_name}_{reason}.npz"
    np.savez(
        npz_path,
        trajectory=trajectory,
        waypoints=waypoints,
        obstacles=obstacle_array,
        end_reason=np.asarray([reason]),
    )
    png_path = npz_path.with_suffix(".png")
    plot_trajectory_file(npz_path, png_path)
    return png_path


def summarise_results(results_df: pd.DataFrame):
    stage_summary = (
        results_df.groupby("stage")
        .agg(
            success_rate=("end_reason", lambda values: float(np.mean(values == "goal_reached"))),
            collision_rate=("end_reason", lambda values: float(np.mean(values == "collision"))),
            timeout_rate=("end_reason", lambda values: float(np.mean(values == "timeout"))),
            sim_crash_rate=("end_reason", lambda values: float(np.mean(values == "sim_crash"))),
            mean_reward=("reward", "mean"),
            mean_final_dist=("final_dist", "mean"),
            median_path_length_ratio=("path_length_ratio", "median"),
        )
        .reset_index()
    )
    overall = {
        "success_rate": float(np.mean(results_df["end_reason"] == "goal_reached")),
        "collision_rate": float(np.mean(results_df["end_reason"] == "collision")),
        "timeout_rate": float(np.mean(results_df["end_reason"] == "timeout")),
        "sim_crash_rate": float(np.mean(results_df["end_reason"] == "sim_crash")),
        "mean_reward": float(results_df["reward"].mean()),
        "mean_final_dist": float(results_df["final_dist"].mean()),
        "median_path_length_ratio": float(results_df["path_length_ratio"].median()),
    }
    return stage_summary, overall


def reset_eval_env(env, seed: int | None = None):
    if seed is None:
        diag_log("eval_env_reset", seed=None, reset_mode="plain")
        return env.reset()

    try:
        diag_log("eval_env_reset", seed=seed, reset_mode="direct_reset_seed")
        return env.reset(seed=seed)
    except TypeError:
        diag_log("eval_env_reset", seed=seed, reset_mode="seed_then_reset")
        env.seed(seed)
        return env.reset()


def log_summary_to_wandb(stage_summary: pd.DataFrame, overall: dict, output_dir: Path, prefix: str):
    if wandb is None or wandb.run is None:
        return

    payload = {f"{prefix}/overall/{key}": value for key, value in overall.items()}
    for record in stage_summary.to_dict(orient="records"):
        stage_name = record["stage"]
        for key, value in record.items():
            if key == "stage":
                continue
            payload[f"{prefix}/{stage_name}/{key}"] = float(value)

    for png_path in sorted((output_dir / "trajectories").glob("*.png")):
        payload[f"{prefix}/trajectories/{png_path.stem}"] = wandb.Image(str(png_path))
    wandb.log(payload)


def run_eval_suite(
    config,
    run_dir: Path,
    checkpoint_path: Path | None = None,
    model=None,
    output_csv: Path | None = None,
    output_dir: Path | None = None,
    num_seeds: int | None = None,
    episodes_per_seed: int | None = None,
    deterministic: bool | None = None,
    use_wandb: bool = False,
    wandb_prefix: str = "eval_suite",
    single_stage: bool = False,
    stage_name: str | None = None,
):
    output_dir = output_dir or run_dir
    output_dir.mkdir(parents=True, exist_ok=True)
    output_csv = output_csv or (run_dir / "eval_suite_results.csv")

    vecnormalize_path = run_dir / "vecnorm.pkl"
    env = create_eval_env(config, vecnormalize_path if vecnormalize_path.exists() else None)
    if model is None:
        if checkpoint_path is None:
            checkpoint_path = resolve_checkpoint(run_dir, None)
        model = load_model(config, checkpoint_path, env)

    base_env = unwrap_env(env.envs[0])
    stages = build_stage_specs(config, single_stage=single_stage, stage_name=stage_name)
    num_seeds = int(num_seeds if num_seeds is not None else config.train.full_eval_seeds)
    episodes_per_seed = int(episodes_per_seed if episodes_per_seed is not None else config.train.full_eval_episodes)
    deterministic = bool(config.train.deterministic_eval if deterministic is None else deterministic)

    results = []
    captured = set()
    diag_log(
        "eval_suite_begin",
        run_dir=str(run_dir),
        output_csv=str(output_csv),
        num_stages=len(stages),
        num_seeds=num_seeds,
        episodes_per_seed=episodes_per_seed,
        deterministic=deterministic,
    )

    try:
        for stage_index, stage in enumerate(stages):
            for seed_index in range(num_seeds):
                seed_value = int(config.train.seed) + seed_index
                seeded_group = False
                diag_log("eval_seed_group_begin", stage=stage["name"], stage_index=stage_index, seed=seed_value)
                for episode in range(episodes_per_seed):
                    terrain_seed = int(config.train.seed) + stage_index * 1_000_000 + seed_index * 1_000 + episode
                    base_env.set_next_episode_params(
                        num_obstacles=stage["num_obstacles"],
                        num_dynamic_obstacles=stage["num_dynamic_obstacles"],
                        goal_distance=stage["num_waypoints"],
                        terrain_length=stage["length"],
                        angle_range=list(stage["angle_range"]),
                        terrain_seed=terrain_seed,
                        force_terrain_regen=True,
                    )
                    diag_log(
                        "eval_episode_begin",
                        stage=stage["name"],
                        stage_index=stage_index,
                        seed=seed_value,
                        episode=episode + 1,
                        terrain_seed=terrain_seed,
                        seeded_group=seeded_group,
                        num_obstacles=stage["num_obstacles"],
                        num_waypoints=stage["num_waypoints"],
                        terrain_length=stage["length"],
                    )

                    obs = reset_eval_env(env, seed=seed_value if not seeded_group else None)
                    seeded_group = True
                    done = False
                    total_reward = 0.0
                    steps = 0

                    while not done:
                        action, _ = model.predict(obs, deterministic=deterministic)
                        obs, reward, done, info = env.step(action)
                        total_reward += float(reward[0])
                        steps += 1

                    info = info[0]
                    end_reason = info.get("end_reason", "unknown")
                    final_dist = float(info.get("distance_to_final_goal", np.nan))
                    row = {
                        "stage": stage["name"],
                        "seed": seed_value,
                        "episode": episode + 1,
                        "end_reason": end_reason,
                        "steps": steps,
                        "reward": total_reward,
                        "final_dist": final_dist,
                        "path_length_ratio": float(info.get("path_length_ratio", np.nan)),
                    }
                    results.append(row)
                    diag_log("eval_episode_end", **row)

                    capture_key = (stage["name"], end_reason)
                    if capture_key not in captured and end_reason in {"goal_reached", "collision", "timeout"}:
                        dump_trajectory(info, output_dir, stage["name"], end_reason)
                        captured.add(capture_key)

        results_df = pd.DataFrame(results)
        results_df.to_csv(output_csv, index=False)
        stage_summary, overall = summarise_results(results_df)

        print("\nStage Summary")
        print(stage_summary.to_string(index=False, float_format=lambda value: f"{value:.3f}"))
        print("\nOverall Summary")
        for key, value in overall.items():
            print(f"{key:24s}: {value:.3f}")

        if use_wandb:
            log_summary_to_wandb(stage_summary, overall, output_dir, wandb_prefix)
        diag_log("eval_suite_complete", stage_summary=stage_summary.to_dict(orient="records"), overall=overall)
        return {"results": results_df, "stage_summary": stage_summary, "overall": overall}
    finally:
        env.close()


def main():
    args = parse_args()
    overrides = legacy_eval_overrides(args)
    run_dir = resolve_run_dir(args)

    if run_dir is not None:
        config = load_config(run_dir / "config.yaml", overrides)
    else:
        config = load_config(args.config, overrides)
    validate_config(config)
    if config.env.launch_sim == "exe":
        config.env.sim_path = resolve_simulator_path(config.env.sim_path)

    checkpoint_path = resolve_checkpoint(run_dir, args.checkpoint)
    if run_dir is None:
        run_dir = checkpoint_path.parent if checkpoint_path.parent.name != "checkpoints" else checkpoint_path.parent.parent

    output_csv = Path(args.output_csv).resolve() if args.output_csv else (run_dir / "eval_suite_results.csv")
    sim_proc = None
    try:
        if config.env.launch_sim == "exe":
            sim_proc = start_simulator(config.env.sim_path, int(config.env.sim_wait))
        else:
            print(
                "Attach mode selected. Ensure UnrealEditor.exe is already running on the PortEnv default map "
                "(GenerationTopDownTest) and the world is ready."
            )

        single_stage = args.episodes is not None
        episodes_per_seed = args.episodes if args.episodes is not None else args.episodes_per_seed
        num_seeds = 1 if single_stage else args.seeds
        deterministic = True if args.deterministic or single_stage else None

        run_eval_suite(
            config=config,
            run_dir=run_dir,
            checkpoint_path=checkpoint_path,
            output_csv=output_csv,
            output_dir=output_csv.parent,
            num_seeds=num_seeds,
            episodes_per_seed=episodes_per_seed,
            deterministic=deterministic,
            use_wandb=False,
            single_stage=single_stage,
            stage_name=args.stage,
        )
    finally:
        stop_simulator(sim_proc)


if __name__ == "__main__":
    main()
