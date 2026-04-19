from __future__ import annotations

import os
import subprocess
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Iterable, Sequence

from omegaconf import DictConfig, OmegaConf


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parents[1]
DATA_ROOT = REPO_ROOT / "data" / "reinforcement_learning"
RUNS_ROOT = DATA_ROOT / "runs"
DEFAULT_CONFIG = SCRIPT_DIR / "configs" / "crossq.yaml"


@dataclass
class RewardConfig:
    progress: float = 1.0
    heading_align: float = 0.3
    obstacle_proximity: float = 0.5
    action_rate: float = 0.05
    cross_track: float = 0.2
    step_penalty: float = 0.1
    forward_velocity: float = 0.5
    stall_penalty: float = 0.1
    stall_speed_threshold: float = 0.3
    stall_warmup_seconds: float = 10.0
    terminal: float = 1.0
    terminal_collision: float = -100.0
    terminal_goal: float = 500.0
    obstacle_decay: float = 5.0


@dataclass
class EnvConfig:
    ip_address: str = "127.0.0.1"
    terrain_regen_interval: int = 10
    num_obstacles: int = 4
    num_dynamic_obstacles: int = 0
    num_waypoints: int = 1
    terrain_length: int = 10
    angle_range: list[float] = field(default_factory=lambda: [-45.0, 45.0])
    terrain_min_width_cm: float = 5000.0
    terrain_max_width_cm: float = 10000.0
    max_timesteps: int = 800
    step_sleep: float = 0.25
    action_repeat: int = 1
    sim_path: str = "Blocks/Blocks.exe"
    sim_wait: int = 10
    launch_sim: str = "none"
    use_c_side_pcg_obstacles: bool = False
    n_stack: int = 1
    lidar_noise_sigma: float = 0.0
    heading_noise_sigma: float = 0.0
    waypoint_radius: float = 10.0
    yaw_angle_scale: float = 0.25


@dataclass
class AlgoConfig:
    name: str = "crossq"
    policy: str = "MlpPolicy"
    learning_rate: float = 3e-4
    gamma: float = 0.99
    batch_size: int = 256
    buffer_size: int = 500_000
    learning_starts: int = 5_000
    train_freq: int = 1
    stats_window_size: int = 10
    net_arch: list[int] = field(default_factory=lambda: [512, 512])
    device: str = "cpu"
    norm_obs: bool = True
    norm_reward: bool = True
    clip_obs: float = 10.0


@dataclass
class TrainConfig:
    total_timesteps: int = 2_500_000
    seed: int = 43
    checkpoint_freq: int = 25_000
    eval_freq: int = 50_000
    subset_eval_seeds: int = 5
    subset_eval_episodes: int = 5
    full_eval_seeds: int = 20
    full_eval_episodes: int = 20
    deterministic_eval: bool = True
    wandb_enabled: bool = False
    wandb_project: str = "vessel-rl"
    wandb_key: str | None = None
    run_root: str = "data/reinforcement_learning/runs"
    sim_log: bool = False
    torch_deterministic: bool = True


@dataclass
class CurriculumStageConfig:
    name: str = "base"
    num_obstacles: int = 4
    num_dynamic_obstacles: int = 0
    num_waypoints: int = 1
    length: int = 10
    angle_range: list[float] = field(default_factory=lambda: [-45.0, 45.0])


@dataclass
class CurriculumConfig:
    enabled: bool = False
    file: str | None = "curriculum.yaml"
    promote_threshold: float = 0.8
    promote_window: int = 50
    stages: list[CurriculumStageConfig] = field(default_factory=list)


@dataclass
class RootConfig:
    env: EnvConfig = field(default_factory=EnvConfig)
    reward: RewardConfig = field(default_factory=RewardConfig)
    algo: AlgoConfig = field(default_factory=AlgoConfig)
    train: TrainConfig = field(default_factory=TrainConfig)
    curriculum: CurriculumConfig = field(default_factory=CurriculumConfig)


def _resolve_config_path(config_path: str | Path) -> Path:
    path = Path(config_path)
    if not path.is_absolute():
        path = (SCRIPT_DIR / path).resolve()
    return path


def _to_plain_object(config: DictConfig):
    return OmegaConf.to_container(config, resolve=False)


def _load_config_tree(config_path: Path) -> DictConfig:
    loaded = OmegaConf.load(config_path)
    if loaded is None:
        loaded = OmegaConf.create({})
    if not isinstance(loaded, DictConfig):
        loaded = OmegaConf.create(loaded)

    plain = _to_plain_object(loaded)
    extends = plain.pop("extends", None)
    current = OmegaConf.create(plain)
    if extends is None:
        return current

    base_path = (config_path.parent / extends).resolve()
    base_cfg = _load_config_tree(base_path)
    return OmegaConf.merge(base_cfg, current)


def _normalize_override(override: str) -> str:
    if override.startswith("env.reward."):
        return "reward." + override[len("env.reward.") :]
    if override.startswith("timesteps="):
        return "train.total_timesteps=" + override.split("=", 1)[1]
    return override


def _merge_curriculum_file(config: DictConfig, config_path: Path) -> DictConfig:
    curriculum_file = config.curriculum.file
    if not curriculum_file:
        return config

    curriculum_path = (config_path.parent / curriculum_file).resolve()
    if not curriculum_path.exists():
        return config
    curriculum_cfg = OmegaConf.load(curriculum_path)
    if curriculum_cfg is None:
        return config

    if isinstance(curriculum_cfg, DictConfig) and "curriculum" in curriculum_cfg:
        curriculum_body = curriculum_cfg.curriculum
    else:
        curriculum_body = curriculum_cfg

    return OmegaConf.merge(config, OmegaConf.create({"curriculum": curriculum_body}))


def load_config(config_path: str | Path, overrides: Sequence[str] | None = None) -> DictConfig:
    config_file = _resolve_config_path(config_path)
    base = OmegaConf.structured(RootConfig)
    OmegaConf.set_struct(base, True)

    yaml_cfg = _load_config_tree(config_file)
    merged = OmegaConf.merge(base, yaml_cfg)
    merged = _merge_curriculum_file(merged, config_file)

    if overrides:
        override_cfg = OmegaConf.from_dotlist([_normalize_override(item) for item in overrides])
        merged = OmegaConf.merge(merged, override_cfg)

    OmegaConf.resolve(merged)
    return merged


def validate_config(config) -> None:
    if int(config.env.action_repeat) < 1:
        raise ValueError("env.action_repeat must be >= 1")
    if int(config.env.num_waypoints) < 1:
        raise ValueError("env.num_waypoints must be >= 1")
    if int(config.env.n_stack) < 1:
        raise ValueError("env.n_stack must be >= 1")
    if len(config.env.angle_range) != 2:
        raise ValueError("env.angle_range must contain exactly two values")
    if not 0.0 <= float(config.env.yaw_angle_scale) <= 1.0:
        raise ValueError("env.yaw_angle_scale must be within [0.0, 1.0]")
    if config.env.launch_sim == "exe" and not config.env.sim_path:
        raise ValueError("env.sim_path is required when env.launch_sim=exe")
    if int(config.env.num_dynamic_obstacles) != 0:
        raise ValueError(
            "env.num_dynamic_obstacles is not supported because dynamic obstacle spawning is disabled at runtime; use 0."
        )
    if float(config.reward.forward_velocity) < 0.0:
        raise ValueError("reward.forward_velocity must be >= 0.0")
    if float(config.reward.stall_penalty) < 0.0:
        raise ValueError("reward.stall_penalty must be >= 0.0")
    if float(config.reward.stall_speed_threshold) < 0.0:
        raise ValueError("reward.stall_speed_threshold must be >= 0.0")
    if float(config.reward.stall_warmup_seconds) < 0.0:
        raise ValueError("reward.stall_warmup_seconds must be >= 0.0")
    for index, stage in enumerate(config.curriculum.stages):
        if int(stage.num_dynamic_obstacles) != 0:
            raise ValueError(
                f"curriculum.stages[{index}].num_dynamic_obstacles is not supported because "
                "dynamic obstacle spawning is disabled at runtime; use 0."
            )


def resolve_simulator_path(sim_path: str | Path) -> str:
    """Resolve a packaged launcher path to the real game binary when present."""
    resolved = Path(os.path.abspath(str(sim_path)))
    packaged_binary = resolved.parent / resolved.stem / "Binaries" / "Win64" / resolved.name
    if packaged_binary.is_file():
        return str(packaged_binary)
    return str(resolved)


def save_resolved_config(config: DictConfig, output_path: str | Path) -> None:
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(OmegaConf.to_yaml(config, resolve=True, sort_keys=False), encoding="utf-8")


def resolve_run_root(config: DictConfig) -> Path:
    run_root = Path(config.train.run_root)
    if not run_root.is_absolute():
        run_root = (REPO_ROOT / run_root).resolve()
    return run_root


def make_run_dir(config: DictConfig, run_id: str | None = None) -> Path:
    if run_id is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        run_id = f"{timestamp}_{config.algo.name}"
    run_dir = resolve_run_root(config) / run_id
    run_dir.mkdir(parents=True, exist_ok=False)
    return run_dir


def get_git_sha() -> str:
    try:
        result = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            check=True,
            capture_output=True,
            text=True,
            cwd=REPO_ROOT,
        )
    except (FileNotFoundError, subprocess.CalledProcessError):
        return "UNKNOWN"
    return result.stdout.strip()


def write_git_sha(output_path: str | Path) -> None:
    output_path = Path(output_path)
    output_path.write_text(get_git_sha() + "\n", encoding="utf-8")


def config_to_dict(config: DictConfig) -> dict:
    return OmegaConf.to_container(config, resolve=True)  # type: ignore[return-value]


def stage_to_episode_params(stage_config) -> dict:
    return {
        "num_obstacles": int(stage_config.num_obstacles),
        "num_dynamic_obstacles": int(stage_config.num_dynamic_obstacles),
        "goal_distance": int(stage_config.num_waypoints),
        "terrain_length": int(stage_config.length),
        "angle_range": [float(stage_config.angle_range[0]), float(stage_config.angle_range[1])],
    }


def ensure_known_cli_tokens(tokens: Iterable[str]) -> None:
    unknown_flags = [token for token in tokens if token.startswith("--")]
    if unknown_flags:
        joined = ", ".join(unknown_flags)
        raise ValueError(f"Unknown CLI arguments: {joined}")
