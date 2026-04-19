import setup_path

import argparse
import os
import random
import subprocess
import time
from collections import defaultdict
from pathlib import Path

import numpy as np
from sb3_contrib import CrossQ
from stable_baselines3.common.callbacks import BaseCallback, CheckpointCallback
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

from airgym.envs.vessel_env import PCGVesselEnv
from airgym.wrappers import CurriculumWrapper
from config import (
    DEFAULT_CONFIG,
    config_to_dict,
    ensure_known_cli_tokens,
    load_config,
    make_run_dir,
    resolve_simulator_path,
    save_resolved_config,
    validate_config,
    write_git_sha,
)
from diagnostics import diag_log

try:
    import torch
except ImportError:  # pragma: no cover - optional dependency
    torch = None

try:
    import wandb
    from wandb.integration.sb3 import WandbCallback
except ImportError:  # pragma: no cover - optional dependency
    wandb = None
    WandbCallback = None


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parents[1]
DATA_ROOT = REPO_ROOT / "data" / "reinforcement_learning"


class EpisodeEndCallback(BaseCallback):
    _REASONS = ("collision", "goal_reached", "timeout", "sim_crash")

    def __init__(self, use_wandb: bool = False, verbose: int = 0):
        super().__init__(verbose)
        self.use_wandb = use_wandb
        self._ep_actions = []
        self._ep_v_surge = []
        self._ep_is_moving = []
        self._reward_component_sums = defaultdict(float)

    def _reset_episode_buffers(self):
        self._ep_actions = []
        self._ep_v_surge = []
        self._ep_is_moving = []
        self._reward_component_sums = defaultdict(float)

    def _on_step(self) -> bool:
        actions = self.locals["actions"]
        infos = self.locals["infos"]
        action = np.asarray(actions[0], dtype=np.float32).copy()
        self._ep_actions.append(action)
        primary_info = infos[0] if infos else {}
        self._ep_v_surge.append(float(primary_info.get("v_surge", 0.0)))
        self._ep_is_moving.append(1.0 if float(primary_info.get("speed", 0.0)) > 0.0 else 0.0)

        for info in infos:
            for key, value in info.get("reward_components", {}).items():
                self._reward_component_sums[key] += float(value)

            reason = info.get("end_reason")
            if reason is None:
                continue

            ep_actions = np.asarray(self._ep_actions, dtype=np.float32) if self._ep_actions else np.zeros((1, 2), dtype=np.float32)
            ep_v_surge = np.asarray(self._ep_v_surge, dtype=np.float32) if self._ep_v_surge else np.zeros(1, dtype=np.float32)
            ep_is_moving = np.asarray(self._ep_is_moving, dtype=np.float32) if self._ep_is_moving else np.zeros(1, dtype=np.float32)
            metrics = {
                **{f"episode/{label}": int(label == reason) for label in self._REASONS},
                "episode/final_distance_to_goal": float(info.get("distance_to_final_goal", 0.0)),
                "episode/final_distance_to_current_wp": float(info.get("distance_to_current_wp", 0.0)),
                "episode/waypoints_reached": int(info.get("waypoints_reached", 0)),
                "episode/mean_thrust": float(ep_actions[:, 0].mean()),
                "episode/mean_yaw_cmd": float(ep_actions[:, 1].mean()),
                "episode/mean_v_surge": float(ep_v_surge.mean()),
                "episode/time_moving_frac": float(ep_is_moving.mean()),
                "episode/count": int(info.get("episode_num", 0)),
                "episode/path_length_ratio": float(info.get("path_length_ratio", 0.0)),
                "curriculum/stage": float(info.get("episode_stage_before_promotion", info.get("curriculum_stage", 0))),
            }
            diag_log(
                "train_episode_end",
                reason=reason,
                timestep=self.num_timesteps,
                episode_num=int(info.get("episode_num", 0)),
                waypoints_reached=int(info.get("waypoints_reached", 0)),
                final_distance=float(metrics["episode/final_distance_to_goal"]),
                path_length_ratio=float(metrics["episode/path_length_ratio"]),
                curriculum_stage=float(metrics["curriculum/stage"]),
            )
            for key, value in self._reward_component_sums.items():
                metrics[f"reward_components/{key}"] = float(value)

            for key, value in metrics.items():
                self.logger.record(key, value)
            if self.use_wandb:
                wandb.log(metrics, step=self.num_timesteps)
            self._reset_episode_buffers()
        return True


class EvalSuiteCallback(BaseCallback):
    def __init__(self, config, run_dir: Path, vecnormalize_path: Path, use_wandb: bool = False, verbose: int = 0):
        super().__init__(verbose)
        self.config = config
        self.run_dir = run_dir
        self.vecnormalize_path = vecnormalize_path
        self.use_wandb = use_wandb
        self.next_eval = int(config.train.eval_freq)

    def _on_step(self) -> bool:
        if self.next_eval <= 0 or self.num_timesteps < self.next_eval:
            return True

        infos = self.locals.get("infos", [])
        if not any(info.get("end_reason") is not None for info in infos):
            return True

        from eval_suite import run_eval_suite

        eval_dir = self.run_dir / "eval_subsets" / f"step_{self.num_timesteps:08d}"
        eval_dir.mkdir(parents=True, exist_ok=True)

        training_env = self.model.get_env()
        training_env.save(str(self.vecnormalize_path))
        diag_log(
            "subset_eval_begin",
            timestep=self.num_timesteps,
            output_dir=str(eval_dir),
            num_seeds=int(self.config.train.subset_eval_seeds),
            episodes_per_seed=int(self.config.train.subset_eval_episodes),
            deterministic=bool(self.config.train.deterministic_eval),
            terminal_reasons=[info.get("end_reason") for info in infos if info.get("end_reason") is not None],
        )
        run_eval_suite(
            config=self.config,
            run_dir=self.run_dir,
            model=self.model,
            output_csv=eval_dir / "results.csv",
            output_dir=eval_dir,
            num_seeds=int(self.config.train.subset_eval_seeds),
            episodes_per_seed=int(self.config.train.subset_eval_episodes),
            deterministic=bool(self.config.train.deterministic_eval),
            use_wandb=self.use_wandb,
            wandb_prefix="eval_subset",
        )
        diag_log("subset_eval_complete", timestep=self.num_timesteps, next_eval=self.next_eval + int(self.config.train.eval_freq))

        self.next_eval += int(self.config.train.eval_freq)
        return True


def parse_args():
    parser = argparse.ArgumentParser(description="Train vessel RL agent with OmegaConf configuration")
    parser.add_argument("--config", type=str, default=str(DEFAULT_CONFIG))
    parser.add_argument("--run-id", type=str, default=None, help="Optional fixed run-id. Defaults to timestamp_algo.")

    # Legacy compatibility flags from crossq_vessel.py
    parser.add_argument("--ip", type=str, default=None)
    parser.add_argument("--timesteps", type=int, default=None)
    parser.add_argument("--terrain-regen", type=int, default=None)
    parser.add_argument("--num-obstacles", type=int, default=None)
    parser.add_argument("--num-dynamic-obstacles", type=int, default=None)
    parser.add_argument("--num-waypoints", type=int, default=None)
    parser.add_argument("--launch-sim", choices=("none", "exe"), default=None)
    parser.add_argument("--sim-path", type=str, default=None)
    parser.add_argument("--sim-wait", type=int, default=None)
    parser.add_argument("--sim-log", action="store_true", default=False)
    parser.add_argument("--action-repeat", type=int, default=None)
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--wandb-key", type=str, default=None)

    parser.add_argument("overrides", nargs="*", help="OmegaConf dotlist overrides, e.g. train.total_timesteps=5000")
    args, unknown = parser.parse_known_args()
    ensure_known_cli_tokens(unknown)
    return args


def legacy_train_overrides(args) -> list[str]:
    overrides = list(args.overrides)
    mapping = {
        "ip": "env.ip_address",
        "timesteps": "train.total_timesteps",
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
    if args.sim_log:
        overrides.append("train.sim_log=true")
    if args.wandb_key:
        overrides.append(f"train.wandb_key={args.wandb_key}")
        overrides.append("train.wandb_enabled=true")
    return overrides


def set_global_seed(seed: int, torch_deterministic: bool = True) -> None:
    os.environ["PYTHONHASHSEED"] = str(seed)
    random.seed(seed)
    np.random.seed(seed)

    if torch is not None:
        torch.manual_seed(seed)
        if torch.cuda.is_available():
            torch.cuda.manual_seed_all(seed)
        if torch_deterministic:
            torch.use_deterministic_algorithms(True, warn_only=True)
            if hasattr(torch.backends, "cudnn"):
                torch.backends.cudnn.deterministic = True
                torch.backends.cudnn.benchmark = False


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
        if bool(config.curriculum.enabled):
            env = CurriculumWrapper(env, config.curriculum, base_seed=int(config.train.seed))
        return Monitor(env)

    return _init


def start_simulator(sim_path: str, wait: int = 10, log_path: str | None = None):
    sim_path = os.path.abspath(sim_path)
    if not os.path.isfile(sim_path):
        raise FileNotFoundError(f"Simulator executable not found: {sim_path}")

    print(f"Starting simulator executable: {sim_path}")
    log_handle = None
    if log_path:
        log_handle = open(log_path, "w", encoding="utf-8")
        proc = subprocess.Popen([sim_path], stdout=log_handle, stderr=subprocess.STDOUT)
    else:
        proc = subprocess.Popen([sim_path], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    print(f"Simulator started (PID: {proc.pid}), waiting {wait}s for initialization...")
    time.sleep(wait)
    return proc, log_handle


def stop_simulator(proc, log_handle=None):
    if proc is not None:
        proc.terminate()
        try:
            proc.wait(timeout=15)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()
    if log_handle is not None:
        log_handle.close()


def create_model(config, env, tensorboard_dir: Path):
    algo_name = str(config.algo.name).lower()
    if algo_name != "crossq":
        raise ValueError(f"Unsupported algorithm: {config.algo.name}")

    return CrossQ(
        config.algo.policy,
        env,
        learning_rate=float(config.algo.learning_rate),
        gamma=float(config.algo.gamma),
        verbose=1,
        batch_size=int(config.algo.batch_size),
        buffer_size=int(config.algo.buffer_size),
        learning_starts=int(config.algo.learning_starts),
        train_freq=int(config.algo.train_freq),
        stats_window_size=int(config.algo.stats_window_size),
        seed=int(config.train.seed),
        device=config.algo.device,
        tensorboard_log=str(tensorboard_dir),
        policy_kwargs=dict(net_arch=list(config.algo.net_arch)),
    )


def init_wandb(config, run_dir: Path):
    use_wandb = bool(config.train.wandb_enabled or config.train.wandb_key)
    if not use_wandb:
        return None, False

    if wandb is None or WandbCallback is None:
        raise RuntimeError("Weights & Biases support was requested, but the 'wandb' package is not installed.")

    if config.train.wandb_key:
        wandb.login(key=config.train.wandb_key)
    run = wandb.init(
        project=str(config.train.wandb_project),
        config=config_to_dict(config),
        sync_tensorboard=True,
        dir=str(run_dir),
        name=run_dir.name,
    )
    return run, True


def main():
    args = parse_args()
    overrides = legacy_train_overrides(args)
    config = load_config(args.config, overrides)
    validate_config(config)
    if config.env.launch_sim == "exe":
        config.env.sim_path = resolve_simulator_path(config.env.sim_path)
    set_global_seed(int(config.train.seed), torch_deterministic=bool(config.train.torch_deterministic))
    diag_log(
        "train_start",
        seed=int(config.train.seed),
        total_timesteps=int(config.train.total_timesteps),
        eval_freq=int(config.train.eval_freq),
        checkpoint_freq=int(config.train.checkpoint_freq),
        launch_sim=str(config.env.launch_sim),
        curriculum_enabled=bool(config.curriculum.enabled),
        action_repeat=int(config.env.action_repeat),
        max_timesteps=int(config.env.max_timesteps),
        step_sleep=float(config.env.step_sleep),
    )

    run_dir = make_run_dir(config, run_id=args.run_id)
    checkpoint_dir = run_dir / "checkpoints"
    tensorboard_dir = run_dir / "tb"
    vecnormalize_path = run_dir / "vecnorm.pkl"
    checkpoint_dir.mkdir(parents=True, exist_ok=True)
    tensorboard_dir.mkdir(parents=True, exist_ok=True)

    save_resolved_config(config, run_dir / "config.yaml")
    write_git_sha(run_dir / "git_sha.txt")

    if bool(config.train.sim_log) and config.env.launch_sim != "exe":
        print("Ignoring train.sim_log because env.launch_sim=none attaches to an existing simulator/editor.")

    sim_proc = None
    sim_log_handle = None
    wandb_run = None

    try:
        if config.env.launch_sim == "exe":
            sim_log_path = str(run_dir / "sim.log") if bool(config.train.sim_log) else None
            sim_proc, sim_log_handle = start_simulator(config.env.sim_path, int(config.env.sim_wait), sim_log_path)
        else:
            print(
                "Attach mode selected. Start UnrealEditor.exe with the target project only. "
                "PortEnv defaults to GenerationTopDownTest, so do not pass an explicit map override unless needed."
            )

        wandb_run, use_wandb = init_wandb(config, run_dir)

        env = DummyVecEnv([make_env(config)])
        env = VecNormalize(
            env,
            norm_obs=bool(config.algo.norm_obs),
            norm_reward=bool(config.algo.norm_reward),
            clip_obs=float(config.algo.clip_obs),
        )

        model = create_model(config, env, tensorboard_dir)

        callbacks = [
            EpisodeEndCallback(use_wandb=use_wandb),
        ]
        if int(config.train.checkpoint_freq) > 0:
            callbacks.append(
                CheckpointCallback(
                    save_freq=int(config.train.checkpoint_freq),
                    save_path=str(checkpoint_dir),
                    name_prefix=str(config.algo.name),
                    save_vecnormalize=True,
                )
            )
        if use_wandb:
            callbacks.append(WandbCallback(verbose=2))
        if int(config.train.eval_freq) > 0:
            callbacks.append(EvalSuiteCallback(config, run_dir, vecnormalize_path, use_wandb=use_wandb))

        model.learn(
            total_timesteps=int(config.train.total_timesteps),
            tb_log_name="train",
            callback=callbacks,
        )

        final_model_path = run_dir / "final_model"
        model.save(str(final_model_path))
        env.save(str(vecnormalize_path))
        diag_log("train_learn_complete", run_dir=str(run_dir), final_model=str(final_model_path), vecnormalize_path=str(vecnormalize_path))

        from eval_suite import run_eval_suite

        run_eval_suite(
            config=config,
            run_dir=run_dir,
            model=model,
            output_csv=run_dir / "eval_suite_results.csv",
            output_dir=run_dir,
            num_seeds=int(config.train.full_eval_seeds),
            episodes_per_seed=int(config.train.full_eval_episodes),
            deterministic=bool(config.train.deterministic_eval),
            use_wandb=use_wandb,
            wandb_prefix="eval_full",
        )
        print(f"Training complete. Artifacts saved to {run_dir}")
    finally:
        if wandb_run is not None:
            wandb_run.finish()
        stop_simulator(sim_proc, sim_log_handle)


if __name__ == "__main__":
    main()
