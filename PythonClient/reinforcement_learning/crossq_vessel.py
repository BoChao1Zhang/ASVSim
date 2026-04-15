import setup_path
import argparse
import os
import subprocess
import time
from pathlib import Path

import numpy as np

from sb3_contrib import CrossQ
from stable_baselines3.common.callbacks import BaseCallback, CheckpointCallback
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

from airgym.envs.vessel_env import PCGVesselEnv


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parents[1]
DATA_ROOT = REPO_ROOT / "data" / "reinforcement_learning"
TRAINING_ROOT = DATA_ROOT / "training"
MODEL_DIR = TRAINING_ROOT / "models"
TB_DIR = TRAINING_ROOT / "tb"
SIM_LOG_PATH = DATA_ROOT / "sim.log"

try:
    import wandb
    from wandb.integration.sb3 import WandbCallback
except ImportError:
    wandb = None
    WandbCallback = None


class EpisodeEndCallback(BaseCallback):
    """Logs episode end reason, mean actions, and final goal distance to wandb."""

    _REASONS = ("collision", "goal_reached", "timeout")

    def __init__(self, use_wandb=False, verbose=0):
        super().__init__(verbose)
        self.use_wandb = use_wandb
        self._ep_actions = []

    def _on_step(self) -> bool:
        actions = self.locals["actions"]  # shape (n_envs, 2)
        self._ep_actions.append(actions[0].copy())

        for info in self.locals["infos"]:
            reason = info.get("end_reason")
            if reason is not None:
                if self.use_wandb:
                    dx = info.get("distance_to_goal_x", 0.0)
                    dy = info.get("distance_to_goal_y", 0.0)
                    ep_actions = np.array(self._ep_actions)
                    wandb.log(
                        {
                            **{f"episode/{r}": int(r == reason) for r in self._REASONS},
                            "episode/final_distance_to_goal": float(np.sqrt(dx**2 + dy**2)),
                            "episode/waypoints_reached": info.get("waypoints_reached", 0),
                            "episode/mean_thrust": float(ep_actions[:, 0].mean()),
                            "episode/mean_rudder_angle": float(ep_actions[:, 1].mean()),
                            "episode/count": info.get("episode_num", 0),
                        },
                        step=self.num_timesteps,
                    )
                self._ep_actions = []
        return True


def validate_args(args):
    if args.action_repeat < 1:
        raise ValueError("--action-repeat must be >= 1")
    if args.num_waypoints < 1:
        raise ValueError("--num-waypoints must be >= 1")
    if args.launch_sim == "exe" and not args.sim_path:
        raise ValueError("--sim-path is required when --launch-sim=exe")


def make_env(args):
    def _init():
        max_timesteps = 800 // args.action_repeat
        env = PCGVesselEnv(
            ip_address=args.ip,
            terrain_regen_interval=args.terrain_regen,
            num_obstacles=args.num_obstacles,
            num_dynamic_obstacles=args.num_dynamic_obstacles,
            goal_distance=args.num_waypoints,
            max_timesteps=max_timesteps,
            seed=args.seed,
            sim_path=args.sim_path,
            sim_wait=args.sim_wait,
            action_repeat=args.action_repeat,
            sim_launch_mode=args.launch_sim,
        )
        return Monitor(env)

    return _init


def start_simulator(sim_path, wait=10, log_path=None):
    sim_path = os.path.abspath(sim_path)
    if not os.path.isfile(sim_path):
        raise FileNotFoundError(f"Simulator executable not found: {sim_path}")

    print(f"Starting simulator executable: {sim_path}")
    log_handle = None
    if log_path:
        log_handle = open(log_path, "w", encoding="utf-8")
        proc = subprocess.Popen(
            [sim_path],
            stdout=log_handle,
            stderr=subprocess.STDOUT,
        )
    else:
        proc = subprocess.Popen(
            [sim_path],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
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


def main():
    parser = argparse.ArgumentParser(description="Train vessel RL agent with PCG environments")
    parser.add_argument("--ip", type=str, default="127.0.0.1", help="Simulator RPC IP address")
    parser.add_argument("--timesteps", type=int, default=2500000, help="Total training timesteps")
    parser.add_argument("--terrain-regen", type=int, default=10, help="Regenerate terrain every N episodes")
    parser.add_argument("--num-obstacles", type=int, default=4, help="Number of static obstacles per episode")
    parser.add_argument("--num-dynamic-obstacles", type=int, default=0, help="Number of dynamic (moving) obstacles per episode")
    parser.add_argument("--num-waypoints", type=int, default=1, help="Number of waypoints to navigate")
    parser.add_argument(
        "--launch-sim",
        choices=("none", "exe"),
        default="none",
        help="Use 'none' to attach to an already running UE Editor or simulator RPC server, or 'exe' to launch --sim-path first.",
    )
    parser.add_argument("--sim-path", type=str, default="Blocks/Blocks.exe", help="Path to simulator executable when --launch-sim=exe")
    parser.add_argument("--sim-wait", type=int, default=10, help="Seconds to wait for simulator startup when --launch-sim=exe")
    parser.add_argument(
        "--sim-log",
        action="store_true",
        default=False,
        help=f"Log launched simulator output to {SIM_LOG_PATH.as_posix()}",
    )
    parser.add_argument("--action-repeat", type=int, default=1, help="Number of times to repeat each action")
    parser.add_argument("--seed", type=int, default=43, help="Random seed for reproducibility")
    parser.add_argument("--wandb-key", type=str, default=None, help="Weights & Biases API key (optional, disables wandb if not set)")
    args = parser.parse_args()

    validate_args(args)

    MODEL_DIR.mkdir(parents=True, exist_ok=True)
    TB_DIR.mkdir(parents=True, exist_ok=True)

    if args.sim_log and args.launch_sim != "exe":
        print("Ignoring --sim-log because --launch-sim=none attaches to an existing simulator/editor.")

    sim_proc = None
    sim_log_handle = None
    run = None

    try:
        if args.launch_sim == "exe":
            sim_log_path = str(SIM_LOG_PATH) if args.sim_log else None
            sim_proc, sim_log_handle = start_simulator(args.sim_path, args.sim_wait, sim_log_path)
        else:
            print("Attach mode selected. Start UnrealEditor.exe with the target project and map before running training.")

        use_wandb = args.wandb_key is not None
        if use_wandb:
            if wandb is None or WandbCallback is None:
                raise RuntimeError(
                    "Weights & Biases support was requested, but the 'wandb' package is not installed."
                )
            wandb.login(key=args.wandb_key)
            run = wandb.init(
                project="vessel-rl",
                config={
                    "algorithm": "CrossQ",
                    "total_timesteps": args.timesteps,
                    "terrain_regen_interval": args.terrain_regen,
                    "num_obstacles": args.num_obstacles,
                    "num_waypoints": args.num_waypoints,
                    "num_dynamic_obstacles": args.num_dynamic_obstacles,
                    "learning_rate": 0.0003,
                    "batch_size": 256,
                    "buffer_size": 500000,
                    "gamma": 0.99,
                    "seed": args.seed,
                    "launch_sim": args.launch_sim,
                },
                sync_tensorboard=True,
            )

        env = DummyVecEnv([make_env(args)])
        env = VecNormalize(env, norm_obs=True, norm_reward=True, clip_obs=10.0)
        model = CrossQ(
            "MlpPolicy",
            env,
            learning_rate=0.0003,
            gamma=0.99,
            verbose=1,
            batch_size=256,
            buffer_size=500000,
            learning_starts=5000,
            train_freq=1,
            stats_window_size=10,
            seed=args.seed,
            device="auto",
            tensorboard_log=str(TB_DIR),
            policy_kwargs=dict(net_arch=[512, 512]),
        )

        callbacks = [EpisodeEndCallback(use_wandb=use_wandb)]
        if use_wandb:
            callbacks.append(WandbCallback(verbose=2))
        callbacks.append(
            CheckpointCallback(
                save_freq=25000,
                save_path=str(MODEL_DIR),
                name_prefix="crossq_pcg_vessel",
            )
        )

        model.learn(
            total_timesteps=args.timesteps,
            tb_log_name="crossq_pcg_vessel_" + str(int(time.time())),
            callback=callbacks,
        )

        model.save(str(MODEL_DIR / "crossq_pcg_vessel_policy"))
        print(f"Training complete. Model saved to {MODEL_DIR.as_posix()}/")

    finally:
        if run is not None:
            run.finish()
        stop_simulator(sim_proc, sim_log_handle)


if __name__ == "__main__":
    main()
