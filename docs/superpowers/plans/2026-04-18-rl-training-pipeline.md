# RL Training Pipeline Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement a stable `Blocks.exe`-driven vessel RL pipeline with smoke test gating, full checkpoint resume, offline evaluation, and offline best-model promotion for the fixed `CrossQ` minimum closed-loop task.

**Architecture:** Split the current monolithic runtime into three explicit phases: training, offline evaluation, and best-checkpoint selection. Keep the environment/task surface mostly intact, add a small support layer for checkpointing and smoke testing, and remove online evaluation from the training loop so simulator state progression and evaluation progression stay isolated.

**Tech Stack:** Python 3.11, Gymnasium, Stable-Baselines3, `sb3-contrib` CrossQ, OmegaConf, `unittest`, `numpy`, `pandas`

## Milestone Gate

Every integrated feature milestone in this plan must satisfy all of the following before it is marked complete:

1. the relevant unit-test slice passes
2. a smoke test passes for the integrated runtime path
3. the current milestone result is captured in a dedicated Git commit
4. Git LFS coverage and status are checked for changed binary assets, especially under `Unreal/Environments/PortEnv`

Use this gate in addition to task-specific checks, not instead of them.

Current repository LFS coverage already includes:

- `*.uasset`
- `*.umap`
- `*.fbx`
- `*.zip`
- `*.rar`
- `Unreal/Environments/PortEnv.zip`

If a new PortEnv binary artifact type appears outside those rules, update the appropriate `.gitattributes` file before creating the milestone commit.

---

## File Structure

### Existing files to modify

- `PythonClient/reinforcement_learning/config.py`
  Add explicit config surface for fixed train/dev seed pools, smoke test settings, crash budget, and best-checkpoint selection.
- `PythonClient/reinforcement_learning/configs/base.yaml`
  Define the default minimum-closed-loop config values approved in the design spec.
- `PythonClient/reinforcement_learning/train.py`
  Remove online eval from the main training loop, add smoke test execution, add `--resume`, and integrate full checkpoint save/load plus crash budget enforcement.
- `PythonClient/reinforcement_learning/eval_suite.py`
  Evaluate checkpoints against fixed `dev eval` seeds from config and write per-checkpoint metrics for offline selection.
- `PythonClient/reinforcement_learning/airgym/envs/reward.py`
  Add `terminal_timeout` support without redesigning the dense reward terms.
- `PythonClient/reinforcement_learning/airgym/envs/vessel_env.py`
  Mark timeout as explicit failure input to reward computation while preserving current task and observation structure.
- `PythonClient/reinforcement_learning/tests/test_train_callbacks.py`
  Replace the obsolete online-eval callback assertions with full-checkpoint and crash-budget callback coverage.
- `PythonClient/reinforcement_learning/tests/test_eval_suite_semantics.py`
  Update expectations to use fixed `dev eval` seed pools instead of derived seed ranges.
- `docs/vessel/reinforcement_learning.md`
  Document the new train/smoke/resume/eval/select-best flow.

### New files to create

- `PythonClient/reinforcement_learning/checkpointing.py`
  Path helpers, trainer state persistence, full snapshot save/load, and `latest/final/best` promotion helpers.
- `PythonClient/reinforcement_learning/training_callbacks.py`
  Focused callbacks for episode metrics, crash-budget accounting, and full checkpoint persistence.
- `PythonClient/reinforcement_learning/smoke_test.py`
  Pre-training simulator and environment sanity checks plus `smoke_test.json`/`smoke_test.log` output.
- `PythonClient/reinforcement_learning/checkpoint_selection.py`
  Enumerate the last `33%` of checkpoints, rank candidates, and promote the winning checkpoint to `best/`.
- `PythonClient/reinforcement_learning/select_best_checkpoint.py`
  CLI wrapper for offline best-checkpoint selection.
- `PythonClient/reinforcement_learning/tests/test_config_runtime_contract.py`
  Verify config loading/validation for seed pools, smoke test settings, and selection policy.
- `PythonClient/reinforcement_learning/tests/test_reward_timeout_semantics.py`
  Verify `terminal_timeout` semantics without changing collision or goal reward behavior.
- `PythonClient/reinforcement_learning/tests/test_checkpointing.py`
  Verify trainer state persistence and full checkpoint save/load/promotion.
- `PythonClient/reinforcement_learning/tests/test_smoke_test.py`
  Verify smoke test pass/fail behavior with fake envs and fake simulator checks.
- `PythonClient/reinforcement_learning/tests/test_checkpoint_selection.py`
  Verify last-`33%` candidate selection, ranking, and `best/` promotion behavior.

## Task 1: Extend Config Surface For Fixed Seeds, Smoke Test, And Selection

**Files:**
- Modify: `PythonClient/reinforcement_learning/config.py`
- Modify: `PythonClient/reinforcement_learning/configs/base.yaml`
- Test: `PythonClient/reinforcement_learning/tests/test_config_runtime_contract.py`

- [ ] **Step 1: Write the failing config contract test**

```python
from tempfile import TemporaryDirectory
import unittest

from config import DEFAULT_CONFIG, load_config, make_run_dir, validate_config


class ConfigRuntimeContractTests(unittest.TestCase):
    def test_default_config_exposes_seed_pools_and_runtime_policy(self):
        config = load_config(DEFAULT_CONFIG)

        self.assertEqual(list(config.train.train_seeds), [43, 44, 45, 46, 47, 48, 49, 50])
        self.assertEqual(list(config.train.dev_eval_seeds), [1043, 1044, 1045, 1046])
        self.assertTrue(config.smoke_test.enabled)
        self.assertEqual(config.smoke_test.seed_count, 2)
        self.assertEqual(config.smoke_test.rollout_steps, 8)
        self.assertAlmostEqual(float(config.train.crash_budget_percent), 1.0)
        self.assertAlmostEqual(float(config.selection.best_checkpoint_tail_fraction), 0.33)

    def test_validate_config_rejects_overlap_and_invalid_selection_fraction(self):
        config = load_config(DEFAULT_CONFIG)
        config.train.dev_eval_seeds = [50]
        with self.assertRaisesRegex(ValueError, "seed pools must be disjoint"):
            validate_config(config)

        config = load_config(DEFAULT_CONFIG)
        config.selection.best_checkpoint_tail_fraction = 1.2
        with self.assertRaisesRegex(ValueError, "best_checkpoint_tail_fraction"):
            validate_config(config)

    def test_make_run_dir_reuses_existing_path_when_resume_mode_is_enabled(self):
        with TemporaryDirectory() as temp_dir:
            config = load_config(DEFAULT_CONFIG, [f"train.run_root={temp_dir}"])

            created = make_run_dir(config, run_id="resume-me")
            reused = make_run_dir(config, run_id="resume-me", exist_ok=True)

            self.assertEqual(created, reused)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run the config contract test to verify it fails**

Run:

```powershell
python -m unittest tests.test_config_runtime_contract -q
```

Expected:

```text
FAIL: test_default_config_exposes_seed_pools_and_runtime_policy
AttributeError: 'TrainConfig' object has no attribute 'train_seeds'
```

- [ ] **Step 3: Add the new config dataclasses, defaults, and validation**

Add these dataclasses in `PythonClient/reinforcement_learning/config.py` near the existing config models:

```python
@dataclass
class SmokeTestConfig:
    enabled: bool = True
    seed_count: int = 2
    rollout_steps: int = 8


@dataclass
class SelectionConfig:
    enabled: bool = True
    best_checkpoint_tail_fraction: float = 0.33
```

Extend `TrainConfig`:

```python
@dataclass
class TrainConfig:
    total_timesteps: int = 2_500_000
    seed: int = 43
    train_seeds: list[int] = field(default_factory=lambda: [43, 44, 45, 46, 47, 48, 49, 50])
    dev_eval_seeds: list[int] = field(default_factory=lambda: [1043, 1044, 1045, 1046])
    checkpoint_freq: int = 25_000
    eval_freq: int = 0
    subset_eval_seeds: int = 0
    subset_eval_episodes: int = 0
    full_eval_seeds: int = 4
    full_eval_episodes: int = 20
    deterministic_eval: bool = True
    wandb_enabled: bool = False
    wandb_project: str = "vessel-rl"
    wandb_key: str | None = None
    run_root: str = "data/reinforcement_learning/runs"
    sim_log: bool = False
    torch_deterministic: bool = True
    crash_budget_percent: float = 1.0
```

Extend `RootConfig`:

```python
@dataclass
class RootConfig:
    env: EnvConfig = field(default_factory=EnvConfig)
    reward: RewardConfig = field(default_factory=RewardConfig)
    algo: AlgoConfig = field(default_factory=AlgoConfig)
    train: TrainConfig = field(default_factory=TrainConfig)
    curriculum: CurriculumConfig = field(default_factory=CurriculumConfig)
    smoke_test: SmokeTestConfig = field(default_factory=SmokeTestConfig)
    selection: SelectionConfig = field(default_factory=SelectionConfig)
```

Add validation in `validate_config()`:

```python
    train_seed_set = set(int(seed) for seed in config.train.train_seeds)
    dev_seed_set = set(int(seed) for seed in config.train.dev_eval_seeds)
    if not train_seed_set:
        raise ValueError("train.train_seeds must not be empty")
    if not dev_seed_set:
        raise ValueError("train.dev_eval_seeds must not be empty")
    if train_seed_set & dev_seed_set:
        raise ValueError("train/dev seed pools must be disjoint")
    if not 0.0 < float(config.train.crash_budget_percent) <= 100.0:
        raise ValueError("train.crash_budget_percent must be within (0, 100]")
    if int(config.smoke_test.seed_count) < 1:
        raise ValueError("smoke_test.seed_count must be >= 1")
    if int(config.smoke_test.rollout_steps) < 1:
        raise ValueError("smoke_test.rollout_steps must be >= 1")
    if not 0.0 < float(config.selection.best_checkpoint_tail_fraction) <= 1.0:
        raise ValueError("selection.best_checkpoint_tail_fraction must be within (0, 1]")
```

Extend `make_run_dir()` so resume mode can reuse an existing run directory:

```python
def make_run_dir(config: DictConfig, run_id: str | None = None, exist_ok: bool = False) -> Path:
    if run_id is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        run_id = f"{timestamp}_{config.algo.name}"
    run_dir = resolve_run_root(config) / run_id
    run_dir.mkdir(parents=True, exist_ok=exist_ok)
    return run_dir
```

Update `PythonClient/reinforcement_learning/configs/base.yaml`:

```yaml
train:
  total_timesteps: 2500000
  seed: 43
  train_seeds: [43, 44, 45, 46, 47, 48, 49, 50]
  dev_eval_seeds: [1043, 1044, 1045, 1046]
  checkpoint_freq: 25000
  eval_freq: 0
  subset_eval_seeds: 0
  subset_eval_episodes: 0
  full_eval_seeds: 4
  full_eval_episodes: 20
  deterministic_eval: true
  wandb_enabled: false
  wandb_project: vessel-rl
  wandb_key: null
  run_root: data/reinforcement_learning/runs
  sim_log: false
  torch_deterministic: true
  crash_budget_percent: 1.0

smoke_test:
  enabled: true
  seed_count: 2
  rollout_steps: 8

selection:
  enabled: true
  best_checkpoint_tail_fraction: 0.33
```

- [ ] **Step 4: Run the config contract test to verify it passes**

Run:

```powershell
python -m unittest tests.test_config_runtime_contract -q
```

Expected:

```text
OK
```

- [ ] **Step 5: Commit the config contract work**

Apply the Milestone Gate first. For this task, the smoke check may be a lightweight CLI/config smoke rather than a live simulator run because no simulator-backed integration path changes yet.

```bash
git add PythonClient/reinforcement_learning/config.py PythonClient/reinforcement_learning/configs/base.yaml PythonClient/reinforcement_learning/tests/test_config_runtime_contract.py
git commit -m "feat: add RL pipeline runtime config surface"
```

## Task 2: Add Explicit Timeout Reward Semantics Without Redesigning The Task

**Files:**
- Modify: `PythonClient/reinforcement_learning/airgym/envs/reward.py`
- Modify: `PythonClient/reinforcement_learning/airgym/envs/vessel_env.py`
- Modify: `PythonClient/reinforcement_learning/config.py`
- Modify: `PythonClient/reinforcement_learning/configs/base.yaml`
- Test: `PythonClient/reinforcement_learning/tests/test_reward_timeout_semantics.py`

- [ ] **Step 1: Write the failing timeout reward test**

```python
import unittest

from airgym.envs.reward import RewardComputer


class RewardTimeoutSemanticsTests(unittest.TestCase):
    def test_timeout_adds_terminal_timeout_penalty_only(self):
        reward = RewardComputer(
            {
                "progress": 0.0,
                "heading_align": 0.0,
                "obstacle_proximity": 0.0,
                "action_rate": 0.0,
                "cross_track": 0.0,
                "step_penalty": 0.0,
                "terminal": 1.0,
                "terminal_collision": -100.0,
                "terminal_goal": 500.0,
                "terminal_timeout": -25.0,
            }
        )
        state = {
            "distance_to_goal": 10.0,
            "heading_error": 0.0,
            "min_obstacle_distance": 999.0,
            "cross_track_error": 0.0,
            "collision": False,
            "goal_reached": False,
            "timeout": True,
        }
        previous = dict(state)

        total, components = reward.compute(state, previous, [0.0, 0.0], [0.0, 0.0])

        self.assertEqual(components["terminal"], 0.0)
        self.assertEqual(components["terminal_timeout"], -25.0)
        self.assertEqual(total, -25.0)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run the timeout reward test to verify it fails**

Run:

```powershell
python -m unittest tests.test_reward_timeout_semantics -q
```

Expected:

```text
ERROR: KeyError: 'terminal_timeout'
```

- [ ] **Step 3: Implement `terminal_timeout` in config, reward, and env state**

Add the new config field in `PythonClient/reinforcement_learning/config.py`:

```python
@dataclass
class RewardConfig:
    progress: float = 1.0
    heading_align: float = 0.3
    obstacle_proximity: float = 0.5
    action_rate: float = 0.05
    cross_track: float = 0.2
    step_penalty: float = 0.1
    terminal: float = 1.0
    terminal_collision: float = -100.0
    terminal_goal: float = 500.0
    terminal_timeout: float = -25.0
    obstacle_decay: float = 5.0
```

Add the default in `PythonClient/reinforcement_learning/configs/base.yaml`:

```yaml
reward:
  progress: 1.0
  heading_align: 0.3
  obstacle_proximity: 0.5
  action_rate: 0.05
  cross_track: 0.2
  step_penalty: 0.1
  terminal: 1.0
  terminal_collision: -100.0
  terminal_goal: 500.0
  terminal_timeout: -25.0
  obstacle_decay: 5.0
```

Update `PythonClient/reinforcement_learning/airgym/envs/reward.py`:

```python
        terminal_raw = 0.0
        terminal_timeout = 0.0
        if state.get("collision", False):
            terminal_raw = float(_cfg_get(self.reward_config, "terminal_collision", -100.0))
        elif state.get("goal_reached", False):
            terminal_raw = float(_cfg_get(self.reward_config, "terminal_goal", 500.0))
        elif state.get("timeout", False):
            terminal_timeout = float(_cfg_get(self.reward_config, "terminal_timeout", -25.0))

        components = {
            "progress": float(_cfg_get(self.reward_config, "progress", 1.0)) * progress,
            "heading_align": float(_cfg_get(self.reward_config, "heading_align", 0.0)) * heading_align,
            "obstacle_proximity": float(_cfg_get(self.reward_config, "obstacle_proximity", 0.0)) * obstacle_proximity,
            "action_rate": float(_cfg_get(self.reward_config, "action_rate", 0.0)) * action_rate,
            "cross_track": float(_cfg_get(self.reward_config, "cross_track", 0.0)) * cross_track,
            "step_penalty": float(_cfg_get(self.reward_config, "step_penalty", 0.0)) * step_penalty,
            "terminal": float(_cfg_get(self.reward_config, "terminal", 1.0)) * terminal_raw,
            "terminal_timeout": terminal_timeout,
        }
```

Update `PythonClient/reinforcement_learning/airgym/envs/vessel_env.py`:

```python
    def _build_reward_state(self, goal_reached: bool = False, timeout: bool = False):
        dx = float(self.state["distance_to_goal_x"])
        dy = float(self.state["distance_to_goal_y"])
        return {
            "distance_to_goal": float(math.sqrt(dx**2 + dy**2)),
            "heading_error": float(self.state.get("heading_error", 0.0)),
            "min_obstacle_distance": float(self.min_obstacle_distance),
            "cross_track_error": float(self.cross_track_error),
            "collision": bool(self.state["collision"]),
            "goal_reached": bool(goal_reached),
            "timeout": bool(timeout),
        }
```

and in `_compute_reward()`:

```python
        timeout = False
        if self.state["collision"]:
            terminated = True
            self.state["success"] = False
            current_state["collision"] = True
        elif distance < self.waypoint_radius:
            is_final = self.current_waypoint_idx == len(self.waypoints) - 1
            if is_final:
                terminated = True
                self.state["success"] = True
                current_state["goal_reached"] = True
            else:
                reached = self.waypoints[self.current_waypoint_idx]
                self.prev_waypoint_x = reached[0]
                self.prev_waypoint_y = reached[1]
                self.current_waypoint_idx += 1
                self._retarget_current_waypoint()
        elif self.timestep >= self.max_timesteps:
            truncated = True
            timeout = True
            self.state["success"] = False

        current_state["timeout"] = timeout
        reward, components = self.reward_computer.compute(current_state, self.prev_reward_state or current_state, action, prev_action)
```

- [ ] **Step 4: Run the timeout reward test to verify it passes**

Run:

```powershell
python -m unittest tests.test_reward_timeout_semantics -q
```

Expected:

```text
OK
```

- [ ] **Step 5: Commit the timeout reward work**

Apply the Milestone Gate first. Because timeout semantics affect runtime episode results, the smoke check for this milestone must exercise the integrated reward path, not just the isolated unit test.

```bash
git add PythonClient/reinforcement_learning/airgym/envs/reward.py PythonClient/reinforcement_learning/airgym/envs/vessel_env.py PythonClient/reinforcement_learning/config.py PythonClient/reinforcement_learning/configs/base.yaml PythonClient/reinforcement_learning/tests/test_reward_timeout_semantics.py
git commit -m "feat: add explicit timeout reward semantics"
```

## Task 3: Add Full Checkpoint Persistence And Promotion Helpers

**Files:**
- Create: `PythonClient/reinforcement_learning/checkpointing.py`
- Test: `PythonClient/reinforcement_learning/tests/test_checkpointing.py`

- [ ] **Step 1: Write the failing checkpoint persistence test**

```python
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from checkpointing import TrainerState, read_trainer_state, save_training_snapshot


class _FakeModel:
    def save(self, path):
        Path(path).write_text("model", encoding="utf-8")

    def save_replay_buffer(self, path):
        Path(path).write_text("buffer", encoding="utf-8")


class _FakeVecEnv:
    def save(self, path):
        Path(path).write_text("vecnorm", encoding="utf-8")


class CheckpointingTests(unittest.TestCase):
    def test_save_training_snapshot_writes_model_vecnorm_buffer_and_state(self):
        with TemporaryDirectory() as temp_dir:
            run_dir = Path(temp_dir)
            state = TrainerState(
                global_timesteps=25000,
                completed_checkpoints=1,
                total_episodes=12,
                sim_crash_episodes=0,
                crash_rate=0.0,
                train_seed_pool_id="train8_v1",
                dev_eval_seed_pool_id="dev4_v1",
                algo="crossq",
                resume_parent=None,
            )

            checkpoint_dir = save_training_snapshot(run_dir, _FakeModel(), _FakeVecEnv(), state)

            self.assertTrue((checkpoint_dir / "model.zip").exists())
            self.assertTrue((checkpoint_dir / "vecnormalize.pkl").exists())
            self.assertTrue((checkpoint_dir / "replay_buffer.pkl").exists())
            self.assertEqual(read_trainer_state(checkpoint_dir / "trainer_state.json").global_timesteps, 25000)
            self.assertTrue((run_dir / "latest" / "model.zip").exists())

    def test_resolve_resume_snapshot_supports_latest_and_step_dir(self):
        with TemporaryDirectory() as temp_dir:
            run_dir = Path(temp_dir)
            (run_dir / "latest").mkdir(parents=True, exist_ok=True)
            (run_dir / "checkpoints" / "step_00050000").mkdir(parents=True, exist_ok=True)

            from checkpointing import resolve_resume_snapshot

            self.assertEqual(resolve_resume_snapshot(run_dir, "latest"), run_dir / "latest")
            self.assertEqual(
                resolve_resume_snapshot(run_dir, "step_00050000"),
                run_dir / "checkpoints" / "step_00050000",
            )


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run the checkpoint persistence test to verify it fails**

Run:

```powershell
python -m unittest tests.test_checkpointing -q
```

Expected:

```text
ERROR: ModuleNotFoundError: No module named 'checkpointing'
```

- [ ] **Step 3: Create the checkpointing module**

Create `PythonClient/reinforcement_learning/checkpointing.py` with these core pieces:

```python
from __future__ import annotations

import json
import shutil
from dataclasses import asdict, dataclass
from pathlib import Path


@dataclass
class TrainerState:
    global_timesteps: int
    completed_checkpoints: int
    total_episodes: int
    sim_crash_episodes: int
    crash_rate: float
    train_seed_pool_id: str
    dev_eval_seed_pool_id: str
    algo: str
    resume_parent: str | None = None


def checkpoint_dir_for_step(run_dir: Path, global_timesteps: int) -> Path:
    return run_dir / "checkpoints" / f"step_{global_timesteps:08d}"


def write_trainer_state(path: Path, state: TrainerState) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(asdict(state), indent=2, sort_keys=True) + "\n", encoding="utf-8")


def read_trainer_state(path: Path) -> TrainerState:
    return TrainerState(**json.loads(path.read_text(encoding="utf-8")))


def _copy_snapshot(src_dir: Path, dst_dir: Path) -> None:
    if dst_dir.exists():
        shutil.rmtree(dst_dir)
    shutil.copytree(src_dir, dst_dir)


def save_training_snapshot(run_dir: Path, model, vec_env, state: TrainerState) -> Path:
    snapshot_dir = checkpoint_dir_for_step(run_dir, state.global_timesteps)
    snapshot_dir.mkdir(parents=True, exist_ok=True)

    model.save(str(snapshot_dir / "model.zip"))
    vec_env.save(str(snapshot_dir / "vecnormalize.pkl"))
    model.save_replay_buffer(str(snapshot_dir / "replay_buffer.pkl"))
    write_trainer_state(snapshot_dir / "trainer_state.json", state)

    _copy_snapshot(snapshot_dir, run_dir / "latest")
    return snapshot_dir


def save_final_snapshot(run_dir: Path, model, vec_env) -> Path:
    final_dir = run_dir / "final"
    final_dir.mkdir(parents=True, exist_ok=True)
    model.save(str(final_dir / "model.zip"))
    vec_env.save(str(final_dir / "vecnormalize.pkl"))
    return final_dir


def resolve_resume_snapshot(run_dir: Path, resume_value: str) -> Path:
    if resume_value == "latest":
        return run_dir / "latest"
    return run_dir / "checkpoints" / resume_value


def promote_best_snapshot(snapshot_dir: Path, metrics: dict) -> Path:
    best_dir = snapshot_dir.parents[1] / "best"
    _copy_snapshot(snapshot_dir, best_dir)
    (best_dir / "metrics.json").write_text(json.dumps(metrics, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return best_dir
```

- [ ] **Step 4: Run the checkpoint persistence test to verify it passes**

Run:

```powershell
python -m unittest tests.test_checkpointing -q
```

Expected:

```text
OK
```

- [ ] **Step 5: Commit the checkpoint helper work**

```bash
git add PythonClient/reinforcement_learning/checkpointing.py PythonClient/reinforcement_learning/tests/test_checkpointing.py
git commit -m "feat: add full RL checkpoint persistence helpers"
```

## Task 4: Add A Pre-Training Smoke Test Module

**Files:**
- Create: `PythonClient/reinforcement_learning/smoke_test.py`
- Test: `PythonClient/reinforcement_learning/tests/test_smoke_test.py`

- [ ] **Step 1: Write the failing smoke test coverage**

```python
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from smoke_test import run_smoke_test


class _FakeEnv:
    def __init__(self, fail_on_step=False):
        self.fail_on_step = fail_on_step
        self.observation_space = type("Space", (), {"shape": (4,)})()
        self.reset_calls = []

    def reset(self, seed=None):
        self.reset_calls.append(seed)
        return [0.0, 0.0, 0.0, 0.0], {}

    def step(self, action):
        if self.fail_on_step:
            raise RuntimeError("sim crash")
        return [0.0, 0.0, 0.0, 0.0], 0.0, False, False, {
            "distance_to_goal_x": 1.0,
            "distance_to_goal_y": 1.0,
            "path_length_ratio": 1.0,
            "end_reason": None,
        }

    def close(self):
        return None


class SmokeTestTests(unittest.TestCase):
    def test_smoke_test_writes_artifacts_and_passes_when_env_is_stable(self):
        with TemporaryDirectory() as temp_dir:
            output_dir = Path(temp_dir)
            result = run_smoke_test(lambda: _FakeEnv(), [43, 44], 4, output_dir)

            self.assertTrue(result["passed"])
            self.assertTrue((output_dir / "smoke_test.json").exists())
            self.assertTrue((output_dir / "smoke_test.log").exists())

    def test_smoke_test_fails_on_runtime_exception(self):
        with TemporaryDirectory() as temp_dir:
            output_dir = Path(temp_dir)
            result = run_smoke_test(lambda: _FakeEnv(fail_on_step=True), [43], 2, output_dir)

            self.assertFalse(result["passed"])
            self.assertEqual(result["results"][0]["failure_reason"], "step_exception")


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run the smoke test coverage to verify it fails**

Run:

```powershell
python -m unittest tests.test_smoke_test -q
```

Expected:

```text
ERROR: ModuleNotFoundError: No module named 'smoke_test'
```

- [ ] **Step 3: Implement the smoke test runner**

Create `PythonClient/reinforcement_learning/smoke_test.py`:

```python
from __future__ import annotations

import json
import traceback
from pathlib import Path

import numpy as np


GENTLE_ACTIONS = [
    np.array([0.2, 0.0], dtype=np.float32),
    np.array([0.2, 0.1], dtype=np.float32),
    np.array([0.2, -0.1], dtype=np.float32),
]


def _is_finite_vector(values) -> bool:
    array = np.asarray(values, dtype=np.float32)
    return bool(np.all(np.isfinite(array)))


def run_smoke_test(env_factory, seeds: list[int], rollout_steps: int, output_dir: Path) -> dict:
    output_dir.mkdir(parents=True, exist_ok=True)
    log_lines: list[str] = []
    results: list[dict] = []

    for seed in seeds:
        env = env_factory()
        try:
            obs, info = env.reset(seed=seed)
            if tuple(np.asarray(obs).shape) != tuple(env.observation_space.shape):
                results.append({"seed": seed, "passed": False, "failure_reason": "observation_shape"})
                continue
            if not _is_finite_vector(obs):
                results.append({"seed": seed, "passed": False, "failure_reason": "non_finite_reset_obs"})
                continue

            for step_index in range(rollout_steps):
                action = GENTLE_ACTIONS[step_index % len(GENTLE_ACTIONS)]
                obs, reward, terminated, truncated, info = env.step(action)
                if not _is_finite_vector(obs):
                    results.append({"seed": seed, "passed": False, "failure_reason": "non_finite_step_obs"})
                    break
                if not np.isfinite(float(reward)):
                    results.append({"seed": seed, "passed": False, "failure_reason": "non_finite_reward"})
                    break
                required_keys = {"distance_to_goal_x", "distance_to_goal_y", "path_length_ratio", "end_reason"}
                if not required_keys.issubset(info):
                    results.append({"seed": seed, "passed": False, "failure_reason": "missing_info_keys"})
                    break
            else:
                results.append({"seed": seed, "passed": True, "failure_reason": None})
        except Exception:
            log_lines.append(traceback.format_exc())
            results.append({"seed": seed, "passed": False, "failure_reason": "step_exception"})
        finally:
            env.close()

    payload = {"passed": all(item["passed"] for item in results), "results": results}
    (output_dir / "smoke_test.json").write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (output_dir / "smoke_test.log").write_text("\n".join(log_lines) + ("\n" if log_lines else ""), encoding="utf-8")
    return payload
```

- [ ] **Step 4: Run the smoke test coverage to verify it passes**

Run:

```powershell
python -m unittest tests.test_smoke_test -q
```

Expected:

```text
OK
```

- [ ] **Step 5: Commit the smoke test work**

```bash
git add PythonClient/reinforcement_learning/smoke_test.py PythonClient/reinforcement_learning/tests/test_smoke_test.py
git commit -m "feat: add RL pre-training smoke test"
```

## Task 5: Refactor Training Around Full Checkpoints, Crash Budget, And Resume

**Files:**
- Create: `PythonClient/reinforcement_learning/training_callbacks.py`
- Modify: `PythonClient/reinforcement_learning/train.py`
- Modify: `PythonClient/reinforcement_learning/tests/test_train_callbacks.py`
- Test: `PythonClient/reinforcement_learning/tests/test_train_callbacks.py`

- [ ] **Step 1: Write the failing training runtime contract test**

Replace `PythonClient/reinforcement_learning/tests/test_train_callbacks.py` with:

```python
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest import mock

from checkpointing import TrainerState
from training_callbacks import CrashBudgetCallback, FullCheckpointCallback


class _FakeEnv:
    def __init__(self):
        self.saved_paths = []

    def save(self, path):
        self.saved_paths.append(path)


class _FakeModel:
    def __init__(self):
        self.saved_replay_buffers = []

    def save(self, path):
        Path(path).write_text("model", encoding="utf-8")

    def save_replay_buffer(self, path):
        self.saved_replay_buffers.append(path)
        Path(path).write_text("buffer", encoding="utf-8")

    def get_env(self):
        return _FakeEnv()


class TrainingCallbacksTests(unittest.TestCase):
    def test_full_checkpoint_callback_saves_snapshot_when_frequency_hits(self):
        with TemporaryDirectory() as temp_dir:
            run_dir = Path(temp_dir)
            state = TrainerState(25000, 1, 12, 0, 0.0, "train8_v1", "dev4_v1", "crossq")
            callback = FullCheckpointCallback(run_dir=run_dir, trainer_state=state, save_freq=25000)
            callback.model = _FakeModel()
            callback.locals = {}
            callback.num_timesteps = 25000

            self.assertTrue(callback._on_step())
            self.assertTrue((run_dir / "checkpoints" / "step_00025000" / "model.zip").exists())

    def test_crash_budget_callback_stops_training_when_budget_is_exceeded(self):
        state = TrainerState(0, 0, 99, 1, 1.01, "train8_v1", "dev4_v1", "crossq")
        callback = CrashBudgetCallback(trainer_state=state, crash_budget_percent=1.0)
        callback.locals = {"infos": [{"end_reason": "sim_crash"}]}
        callback.num_timesteps = 1000

        self.assertFalse(callback._on_step())


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run the training runtime contract test to verify it fails**

Run:

```powershell
python -m unittest tests.test_train_callbacks -q
```

Expected:

```text
ERROR: ModuleNotFoundError: No module named 'training_callbacks'
```

- [ ] **Step 3: Implement callbacks and integrate them into `train.py`**

Create `PythonClient/reinforcement_learning/training_callbacks.py`:

```python
from __future__ import annotations

from pathlib import Path

from stable_baselines3.common.callbacks import BaseCallback

from checkpointing import save_training_snapshot


class FullCheckpointCallback(BaseCallback):
    def __init__(self, run_dir: Path, trainer_state, save_freq: int, verbose: int = 0):
        super().__init__(verbose)
        self.run_dir = Path(run_dir)
        self.trainer_state = trainer_state
        self.save_freq = int(save_freq)

    def _on_step(self) -> bool:
        if self.save_freq <= 0 or self.num_timesteps == 0 or self.num_timesteps % self.save_freq != 0:
            return True
        self.trainer_state.global_timesteps = int(self.num_timesteps)
        self.trainer_state.completed_checkpoints += 1
        save_training_snapshot(self.run_dir, self.model, self.model.get_env(), self.trainer_state)
        return True


class CrashBudgetCallback(BaseCallback):
    def __init__(self, trainer_state, crash_budget_percent: float, verbose: int = 0):
        super().__init__(verbose)
        self.trainer_state = trainer_state
        self.crash_budget_percent = float(crash_budget_percent)

    def _on_step(self) -> bool:
        infos = self.locals.get("infos", [])
        for info in infos:
            reason = info.get("end_reason")
            if reason is None:
                continue
            self.trainer_state.total_episodes += 1
            if reason == "sim_crash":
                self.trainer_state.sim_crash_episodes += 1
            self.trainer_state.crash_rate = (
                100.0 * self.trainer_state.sim_crash_episodes / max(self.trainer_state.total_episodes, 1)
            )
            if self.trainer_state.crash_rate > self.crash_budget_percent:
                return False
        return True
```

Refactor `PythonClient/reinforcement_learning/train.py`:

```python
from checkpointing import TrainerState, read_trainer_state, resolve_resume_snapshot, save_final_snapshot
from smoke_test import run_smoke_test
from training_callbacks import CrashBudgetCallback, FullCheckpointCallback
```

Add CLI resume support:

```python
    parser.add_argument("--resume", type=str, default=None, help="Resume from latest or a checkpoint subdir name")
```

Add a helper to build the initial trainer state:

```python
def make_initial_trainer_state(config) -> TrainerState:
    return TrainerState(
        global_timesteps=0,
        completed_checkpoints=0,
        total_episodes=0,
        sim_crash_episodes=0,
        crash_rate=0.0,
        train_seed_pool_id="train8_v1",
        dev_eval_seed_pool_id="dev4_v1",
        algo=str(config.algo.name).lower(),
        resume_parent=None,
    )
```

When `--resume` is present, reuse the existing run directory instead of trying to create a new one:

```python
    run_dir = make_run_dir(config, run_id=args.run_id, exist_ok=bool(args.resume))
```

Add explicit resume loading in `main()`:

```python
        if args.resume:
            snapshot_dir = resolve_resume_snapshot(run_dir, args.resume)
            trainer_state = read_trainer_state(snapshot_dir / "trainer_state.json")
            trainer_state.resume_parent = snapshot_dir.name

            env = DummyVecEnv([make_env(config)])
            env = VecNormalize.load(str(snapshot_dir / "vecnormalize.pkl"), env)
            model = CrossQ.load(str(snapshot_dir / "model.zip"), env=env, device=config.algo.device)
            model.load_replay_buffer(str(snapshot_dir / "replay_buffer.pkl"))
        else:
            env = DummyVecEnv([make_env(config)])
            env = VecNormalize(
                env,
                norm_obs=bool(config.algo.norm_obs),
                norm_reward=bool(config.algo.norm_reward),
                clip_obs=float(config.algo.clip_obs),
            )
            model = create_model(config, env, tensorboard_dir)
            trainer_state = make_initial_trainer_state(config)
```

Replace the old callback list in `main()`:

```python
        callbacks = [
            EpisodeEndCallback(use_wandb=use_wandb),
            CrashBudgetCallback(trainer_state=trainer_state, crash_budget_percent=float(config.train.crash_budget_percent)),
            FullCheckpointCallback(run_dir=run_dir, trainer_state=trainer_state, save_freq=int(config.train.checkpoint_freq)),
        ]
```

Delete the `EvalSuiteCallback` registration entirely.

Before model creation, add smoke test gating:

```python
        if bool(config.smoke_test.enabled):
            smoke_result = run_smoke_test(
                env_factory=make_env(config),
                seeds=list(config.train.train_seeds)[: int(config.smoke_test.seed_count)],
                rollout_steps=int(config.smoke_test.rollout_steps),
                output_dir=run_dir,
            )
            if not smoke_result["passed"]:
                raise RuntimeError("Smoke test failed; refusing to start long training run.")
```

At the end of training, save final snapshot only:

```python
        save_final_snapshot(run_dir, model, env)
```

Do not call `run_eval_suite()` from `train.py`.

- [ ] **Step 4: Run the training runtime contract test to verify it passes**

Run:

```powershell
python -m unittest tests.test_train_callbacks -q
```

Expected:

```text
OK
```

- [ ] **Step 5: Commit the training runtime refactor**

Apply the Milestone Gate first. This milestone must include a live smoke-gated `train.py` check before the commit is created.

```bash
git add PythonClient/reinforcement_learning/training_callbacks.py PythonClient/reinforcement_learning/train.py PythonClient/reinforcement_learning/tests/test_train_callbacks.py
git commit -m "feat: refactor RL training around smoke test and full checkpoints"
```

## Task 6: Update Evaluation To Use Fixed Dev Seeds And Add Best-Checkpoint Selection

**Files:**
- Create: `PythonClient/reinforcement_learning/checkpoint_selection.py`
- Create: `PythonClient/reinforcement_learning/select_best_checkpoint.py`
- Modify: `PythonClient/reinforcement_learning/eval_suite.py`
- Modify: `PythonClient/reinforcement_learning/tests/test_eval_suite_semantics.py`
- Test: `PythonClient/reinforcement_learning/tests/test_checkpoint_selection.py`
- Test: `PythonClient/reinforcement_learning/tests/test_eval_suite_semantics.py`

- [ ] **Step 1: Write the failing candidate selection and dev-seed tests**

Create `PythonClient/reinforcement_learning/tests/test_checkpoint_selection.py`:

```python
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from checkpoint_selection import select_candidate_checkpoints


class CheckpointSelectionTests(unittest.TestCase):
    def test_select_candidate_checkpoints_uses_last_third_of_sorted_steps(self):
        with TemporaryDirectory() as temp_dir:
            checkpoints_dir = Path(temp_dir) / "checkpoints"
            for step in (25000, 50000, 75000, 100000, 125000, 150000):
                (checkpoints_dir / f"step_{step:08d}").mkdir(parents=True, exist_ok=True)

            candidates = select_candidate_checkpoints(checkpoints_dir, 0.33)

            self.assertEqual([path.name for path in candidates], ["step_00125000", "step_00150000"])

    def test_rank_candidate_metrics_prefers_non_crashy_eligible_record(self):
        from checkpoint_selection import rank_candidate_metrics

        winner = rank_candidate_metrics(
            [
                {
                    "snapshot_dir": Path("step_00125000"),
                    "global_timesteps": 125000,
                    "success_rate": 0.90,
                    "collision_rate": 0.00,
                    "sim_crash_rate": 0.02,
                    "mean_final_dist": 1.0,
                },
                {
                    "snapshot_dir": Path("step_00150000"),
                    "global_timesteps": 150000,
                    "success_rate": 0.85,
                    "collision_rate": 0.00,
                    "sim_crash_rate": 0.00,
                    "mean_final_dist": 1.5,
                },
            ],
            crash_budget_percent=1.0,
        )

        self.assertEqual(winner["global_timesteps"], 150000)


if __name__ == "__main__":
    unittest.main()
```

Update `PythonClient/reinforcement_learning/tests/test_eval_suite_semantics.py` to assert fixed dev seeds:

```python
        config = SimpleNamespace(
            train=SimpleNamespace(
                seed=43,
                dev_eval_seeds=[1043, 1044, 1045, 1046],
                full_eval_episodes=2,
                deterministic_eval=True,
            )
        )
```

and change the assertion:

```python
        self.assertEqual(fake_env.seed_calls, [1043, 1044])
```

- [ ] **Step 2: Run the selection and eval tests to verify they fail**

Run:

```powershell
python -m unittest tests.test_checkpoint_selection tests.test_eval_suite_semantics -q
```

Expected:

```text
ERROR: ModuleNotFoundError: No module named 'checkpoint_selection'
FAIL: test_run_eval_suite_reseeds_vec_env_for_each_seed_group
```

- [ ] **Step 3: Implement fixed dev-seed evaluation and checkpoint selection**

Create `PythonClient/reinforcement_learning/checkpoint_selection.py`:

```python
from __future__ import annotations

from math import ceil
from pathlib import Path

from checkpointing import promote_best_snapshot


def select_candidate_checkpoints(checkpoints_dir: Path, tail_fraction: float) -> list[Path]:
    checkpoints = sorted(path for path in checkpoints_dir.glob("step_*") if path.is_dir())
    if not checkpoints:
        return []
    tail_count = max(1, ceil(len(checkpoints) * float(tail_fraction)))
    return checkpoints[-tail_count:]


def rank_candidate_metrics(records: list[dict], crash_budget_percent: float) -> dict:
    eligible = [
        item
        for item in records
        if float(item["sim_crash_rate"]) <= float(crash_budget_percent) / 100.0
    ]
    ranking_pool = eligible or records
    return sorted(
        ranking_pool,
        key=lambda item: (
            -float(item["success_rate"]),
            float(item["collision_rate"]),
            float(item["mean_final_dist"]),
            -int(item["global_timesteps"]),
        ),
    )[0]


def promote_best_candidate(winning_snapshot: Path, metrics: dict) -> Path:
    return promote_best_snapshot(winning_snapshot, metrics)
```

Create `PythonClient/reinforcement_learning/select_best_checkpoint.py`:

```python
import argparse
from pathlib import Path

from checkpoint_selection import promote_best_candidate, rank_candidate_metrics, select_candidate_checkpoints
from eval_suite import load_config, run_eval_suite


def main():
    parser = argparse.ArgumentParser(description="Evaluate the final checkpoint tail and promote the best snapshot")
    parser.add_argument("--run-dir", required=True, type=str)
    args = parser.parse_args()

    run_dir = Path(args.run_dir).resolve()
    config = load_config(run_dir / "config.yaml")
    candidates = select_candidate_checkpoints(run_dir / "checkpoints", float(config.selection.best_checkpoint_tail_fraction))

    records = []
    for candidate in candidates:
        result = run_eval_suite(
            config=config,
            run_dir=run_dir,
            checkpoint_path=candidate / "model.zip",
            output_csv=run_dir / "best_selection" / f"{candidate.name}.csv",
            output_dir=run_dir / "best_selection" / candidate.name,
            num_seeds=len(config.train.dev_eval_seeds),
            episodes_per_seed=int(config.train.full_eval_episodes),
            deterministic=bool(config.train.deterministic_eval),
            use_wandb=False,
        )
        overall = result["overall"]
        records.append(
            {
                "snapshot_dir": candidate,
                "global_timesteps": int(candidate.name.split("_")[1]),
                "success_rate": overall["success_rate"],
                "collision_rate": overall["collision_rate"],
                "sim_crash_rate": overall["sim_crash_rate"],
                "mean_final_dist": overall["mean_final_dist"],
            }
        )

    winner = rank_candidate_metrics(records, crash_budget_percent=float(config.train.crash_budget_percent))
    promote_best_candidate(winner["snapshot_dir"], winner)


if __name__ == "__main__":
    main()
```

Update `PythonClient/reinforcement_learning/eval_suite.py` so `run_eval_suite()` uses fixed dev seeds:

```python
    seed_values = list(config.train.dev_eval_seeds)
    if num_seeds is not None:
        seed_values = seed_values[: int(num_seeds)]
```

and replace the nested seed loop:

```python
            for seed_index, seed_value in enumerate(seed_values):
                seeded_group = False
                diag_log("eval_seed_group_begin", stage=stage["name"], stage_index=stage_index, seed=seed_value)
```

- [ ] **Step 4: Run the selection and eval tests to verify they pass**

Run:

```powershell
python -m unittest tests.test_checkpoint_selection tests.test_eval_suite_semantics -q
```

Expected:

```text
OK
```

- [ ] **Step 5: Commit the evaluation and selection work**

Apply the Milestone Gate first. Because this milestone changes integrated evaluation behavior, run the offline evaluation smoke path before committing and verify any changed artifacts remain LFS-managed if PortEnv content was touched during validation.

```bash
git add PythonClient/reinforcement_learning/checkpoint_selection.py PythonClient/reinforcement_learning/select_best_checkpoint.py PythonClient/reinforcement_learning/eval_suite.py PythonClient/reinforcement_learning/tests/test_checkpoint_selection.py PythonClient/reinforcement_learning/tests/test_eval_suite_semantics.py
git commit -m "feat: add offline best checkpoint selection"
```

## Task 7: Update Operator Docs And Run End-To-End Verification

**Files:**
- Modify: `docs/vessel/reinforcement_learning.md`
- Test: `PythonClient/reinforcement_learning/tests/test_config_runtime_contract.py`
- Test: `PythonClient/reinforcement_learning/tests/test_reward_timeout_semantics.py`
- Test: `PythonClient/reinforcement_learning/tests/test_checkpointing.py`
- Test: `PythonClient/reinforcement_learning/tests/test_smoke_test.py`
- Test: `PythonClient/reinforcement_learning/tests/test_train_callbacks.py`
- Test: `PythonClient/reinforcement_learning/tests/test_checkpoint_selection.py`
- Test: `PythonClient/reinforcement_learning/tests/test_eval_suite_semantics.py`

- [ ] **Step 1: Write the doc update with the new train/eval/select-best workflow**

Add this section to `docs/vessel/reinforcement_learning.md`:

```markdown
## Minimum Closed-Loop Training Workflow

Version 1 of the vessel RL pipeline uses `Blocks.exe` as the primary simulator target and separates training from evaluation.

### Train

```powershell
Set-Location E:\code\ASVSim\PythonClient\reinforcement_learning
python .\train.py --launch-sim exe --sim-path E:\ASVSimBuilds\Blocks\Blocks.exe
```

The training entrypoint:

- starts `Blocks.exe`
- runs a short smoke test before `learn()`
- writes full checkpoints under `run_dir/checkpoints/`
- maintains a resumable `run_dir/latest/`
- stops if cumulative `sim_crash_rate > 1%`

### Resume

```powershell
python .\train.py --launch-sim exe --sim-path E:\ASVSimBuilds\Blocks\Blocks.exe --resume latest
```

### Evaluate

```powershell
python .\eval_suite.py --run-dir E:\code\ASVSim\data\reinforcement_learning\runs\20260418_120000_crossq
```

### Promote best checkpoint

```powershell
python .\select_best_checkpoint.py --run-dir E:\code\ASVSim\data\reinforcement_learning\runs\20260418_120000_crossq
```

Only the last `33%` of checkpoints are considered for best-model promotion.
```

- [ ] **Step 2: Run the full RL unit-test suite**

Run:

```powershell
Set-Location E:\code\ASVSim\PythonClient\reinforcement_learning
python -m unittest discover -s tests -q
```

Expected:

```text
OK
```

- [ ] **Step 3: Run CLI smoke checks for the new entrypoints**

Run:

```powershell
Set-Location E:\code\ASVSim\PythonClient\reinforcement_learning
python .\train.py --help
python .\eval_suite.py --help
python .\select_best_checkpoint.py --help
```

Expected:

```text
usage: train.py [-h]
usage: eval_suite.py [-h]
usage: select_best_checkpoint.py [-h]
```

- [ ] **Step 4: Run one short live smoke-gated training check**

Run:

```powershell
Set-Location E:\code\ASVSim\PythonClient\reinforcement_learning
python .\train.py --launch-sim exe --sim-path E:\ASVSimBuilds\Blocks\Blocks.exe train.total_timesteps=5000
```

Expected:

```text
Smoke test passed
Starting simulator executable: E:\ASVSimBuilds\Blocks\Blocks.exe
Training complete
```

Also verify these artifacts exist in the run directory:

```text
smoke_test.json
checkpoints/step_00005000/
latest/model.zip
final/model.zip
```

- [ ] **Step 5: Commit the docs and verification finish**

Apply the Milestone Gate first. The final milestone must confirm both unit-test completion and the live smoke run, then perform an explicit LFS status check before the closing commit.

```bash
git add docs/vessel/reinforcement_learning.md
git commit -m "docs: describe the offline RL training pipeline"
```

## Self-Review

### Spec Coverage

- fixed train/dev seed pools:
  - Task 1
- explicit timeout failure semantics:
  - Task 2
- full resumable checkpoints:
  - Tasks 3 and 5
- smoke test before long training:
  - Tasks 4 and 5
- no online eval in training:
  - Task 5
- offline evaluation on fixed dev seeds:
  - Task 6
- best model from last `33%` of checkpoints:
  - Task 6
- operator docs and end-to-end verification:
  - Task 7
- milestone-level smoke test + commit + LFS discipline:
  - Milestone Gate

No uncovered spec requirement remains.

### Placeholder Scan

Checked for forbidden placeholders:

- no `TODO`
- no `TBD`
- no `implement later`
- no vague “add validation” steps without code
- every code-changing step contains concrete code blocks

### Type Consistency

Planned names are consistent across tasks:

- `train.train_seeds`
- `train.dev_eval_seeds`
- `train.crash_budget_percent`
- `smoke_test.enabled`
- `smoke_test.seed_count`
- `smoke_test.rollout_steps`
- `selection.best_checkpoint_tail_fraction`
- `TrainerState`
- `save_training_snapshot()`
- `run_smoke_test()`
- `select_candidate_checkpoints()`
