# RL Timeout Remediation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement the approved timeout-remediation design for the vessel RL stack by fixing metric semantics, adding time-consistent reward shaping, switching velocity observations to body-frame semantics, and enabling a warmup curriculum stage.

**Architecture:** Keep the current `CrossQ + PCGVesselEnv` pipeline intact and make the smallest code changes that correct diagnosis and improve movement incentives. The work is split into focused tasks so reward semantics, logging semantics, observation semantics, and curriculum defaults can each be implemented and reviewed independently.

**Tech Stack:** Python 3.11, Gymnasium, SB3-Contrib CrossQ, OmegaConf, `unittest`, `numpy`, YAML config files

---

## File Structure

### Existing Files To Modify

- `PythonClient/reinforcement_learning/airgym/envs/vessel_env.py`
  Export explicit terminal-distance fields, motion diagnostics, and body-frame velocity observations.
- `PythonClient/reinforcement_learning/airgym/envs/reward.py`
  Add `dt`-scaled step penalty, forward-motion shaping, and time-based stall penalty.
- `PythonClient/reinforcement_learning/config.py`
  Add reward config fields for forward-motion/stall shaping and validate them.
- `PythonClient/reinforcement_learning/configs/base.yaml`
  Update approved defaults for yaw scaling, reward weights, and curriculum enablement.
- `PythonClient/reinforcement_learning/configs/curriculum.yaml`
  Insert the warmup stage ahead of the existing stages.
- `PythonClient/reinforcement_learning/train.py`
  Consume the new explicit info keys and log new episode-level motion diagnostics.
- `PythonClient/reinforcement_learning/eval_suite.py`
  Read final-distance metrics from the new explicit terminal key.
- `PythonClient/reinforcement_learning/tests/test_train_callbacks.py`
  Verify the training callback logs the new episode metrics.
- `PythonClient/reinforcement_learning/tests/test_eval_suite_semantics.py`
  Verify evaluation uses the explicit final-goal distance key.
- `PythonClient/reinforcement_learning/tests/test_curriculum_wrapper.py`
  Verify warmup-stage naming and promotion behavior still work.

### New Files To Create

- `PythonClient/reinforcement_learning/tests/test_reward_semantics.py`
  Unit-test time-scaled step penalty, forward-motion reward, and stall penalty semantics.
- `PythonClient/reinforcement_learning/tests/test_vessel_env_observation_semantics.py`
  Unit-test pure helpers for body-frame velocity conversion and explicit distance extraction.

## Task 1: Make Terminal Metrics Explicit In Env, Train, And Eval

**Files:**
- Modify: `PythonClient/reinforcement_learning/airgym/envs/vessel_env.py`
- Modify: `PythonClient/reinforcement_learning/train.py`
- Modify: `PythonClient/reinforcement_learning/eval_suite.py`
- Test: `PythonClient/reinforcement_learning/tests/test_train_callbacks.py`
- Test: `PythonClient/reinforcement_learning/tests/test_eval_suite_semantics.py`

- [ ] **Step 1: Write the failing callback and eval tests**

Add these expectations to `PythonClient/reinforcement_learning/tests/test_train_callbacks.py`:

```python
from pathlib import Path
from types import SimpleNamespace
import unittest

import numpy as np

import train


class EpisodeEndCallbackMetricsTests(unittest.TestCase):
    def test_episode_end_callback_uses_explicit_distance_and_motion_keys(self):
        callback = train.EpisodeEndCallback(use_wandb=False)
        callback._logger = train.configure_logger(verbose=0)
        callback.num_timesteps = 128
        callback.locals = {
            "actions": np.asarray([[0.6, 0.2]], dtype=np.float32),
            "infos": [
                {
                    "end_reason": "timeout",
                    "distance_to_final_goal": 42.0,
                    "distance_to_current_wp": 18.0,
                    "waypoints_reached": 1,
                    "episode_num": 7,
                    "path_length_ratio": 1.3,
                    "v_surge": 0.55,
                    "speed": 0.62,
                    "time_moving_frac": 0.75,
                    "reward_components": {"progress": 3.0},
                }
            ],
        }

        self.assertTrue(callback._on_step())
```

Add these expectations to `PythonClient/reinforcement_learning/tests/test_eval_suite_semantics.py`:

```python
info = {
    "end_reason": "goal_reached",
    "distance_to_final_goal": 0.0,
    "distance_to_current_wp": 0.0,
    "path_length_ratio": 1.0,
}
```

and assert the produced `final_dist` is derived from `distance_to_final_goal`.

- [ ] **Step 2: Run the focused tests to verify they fail**

Run:

```powershell
python -m unittest PythonClient.reinforcement_learning.tests.test_train_callbacks PythonClient.reinforcement_learning.tests.test_eval_suite_semantics -q
```

Expected:

```text
FAIL or ERROR because EpisodeEndCallback still reads distance_to_goal_x/y
FAIL because eval_suite still computes final_dist from distance_to_goal_x/y
```

- [ ] **Step 3: Implement explicit distance keys and motion diagnostics**

In `PythonClient/reinforcement_learning/airgym/envs/vessel_env.py`, replace the overloaded terminal fields with explicit keys and export motion diagnostics:

```python
speed = float(np.hypot(linear_velocity_x, linear_velocity_y))
v_surge, v_sway = body_frame_velocity(linear_velocity_x, linear_velocity_y, heading)
v_los = los_velocity(linear_velocity_x, linear_velocity_y, dx_curr, dy_curr)
```

Return explicit info fields from `_step_inner()`:

```python
info = {
    "reward": reward,
    "reward_components": components,
    "thrust": action[0],
    "yaw_cmd": action[1],
    "distance_to_final_goal": float(np.sqrt(final_dist_x**2 + final_dist_y**2)),
    "distance_to_current_wp": float(distance),
    "success": int(self.state["success"]),
    "collision": int(self.state["collision"]),
    "episode_num": self.episode_count,
    "end_reason": end_reason,
    "waypoints_reached": waypoints_reached,
    "path_length_ratio": float(path_length_ratio),
    "actual_path_length": float(self.episode_path_length),
    "straight_line_distance": float(self.initial_final_goal_distance),
    "v_surge": float(v_surge),
    "v_los": float(v_los),
    "speed": float(speed),
}
```

Update `EpisodeEndCallback` in `PythonClient/reinforcement_learning/train.py` to consume the explicit keys:

```python
final_distance = float(info.get("distance_to_final_goal", 0.0))
current_wp_distance = float(info.get("distance_to_current_wp", 0.0))
mean_v_surge = float(info.get("v_surge", 0.0))
time_moving_frac = float(info.get("time_moving_frac", 0.0))
```

Record:

```python
"episode/final_distance_to_goal": final_distance,
"episode/final_distance_to_current_wp": current_wp_distance,
"episode/mean_v_surge": mean_v_surge,
"episode/time_moving_frac": time_moving_frac,
```

Update `PythonClient/reinforcement_learning/eval_suite.py` to compute:

```python
final_dist = float(info.get("distance_to_final_goal", 0.0))
```

- [ ] **Step 4: Run the focused tests to verify they pass**

Run:

```powershell
python -m unittest PythonClient.reinforcement_learning.tests.test_train_callbacks PythonClient.reinforcement_learning.tests.test_eval_suite_semantics -q
```

Expected:

```text
Ran ... tests in ...s
OK
```

- [ ] **Step 5: Commit**

```bash
git add PythonClient/reinforcement_learning/airgym/envs/vessel_env.py PythonClient/reinforcement_learning/train.py PythonClient/reinforcement_learning/eval_suite.py PythonClient/reinforcement_learning/tests/test_train_callbacks.py PythonClient/reinforcement_learning/tests/test_eval_suite_semantics.py
git commit -m "feat: make RL terminal metrics explicit"
```

## Task 2: Add Time-Consistent Reward Shaping And Config Surface

**Files:**
- Modify: `PythonClient/reinforcement_learning/airgym/envs/reward.py`
- Modify: `PythonClient/reinforcement_learning/config.py`
- Modify: `PythonClient/reinforcement_learning/configs/base.yaml`
- Test: `PythonClient/reinforcement_learning/tests/test_reward_semantics.py`

- [ ] **Step 1: Write failing reward semantics tests**

Create `PythonClient/reinforcement_learning/tests/test_reward_semantics.py`:

```python
import unittest

from airgym.envs.reward import RewardComputer


class RewardSemanticsTests(unittest.TestCase):
    def test_step_penalty_scales_with_dt(self):
        reward = RewardComputer(
            {
                "step_penalty": 0.1,
                "forward_velocity": 0.0,
                "stall_penalty": 0.0,
                "stall_speed_threshold": 0.3,
                "stall_warmup_seconds": 10.0,
            }
        )
        state = {"distance_to_goal": 100.0, "dt": 0.5, "speed": 1.0, "v_los": 0.0}
        total, components = reward.compute(state, state, [0.0, 0.0], [0.0, 0.0])
        self.assertAlmostEqual(components["step_penalty"], -0.2)

    def test_forward_velocity_reward_is_positive_when_moving_toward_waypoint(self):
        reward = RewardComputer(
            {
                "step_penalty": 0.0,
                "forward_velocity": 0.5,
                "stall_penalty": 0.0,
                "stall_speed_threshold": 0.3,
                "stall_warmup_seconds": 10.0,
            }
        )
        state = {"distance_to_goal": 100.0, "dt": 0.25, "speed": 0.8, "v_los": 0.8}
        total, components = reward.compute(state, state, [0.0, 0.0], [0.0, 0.0])
        self.assertGreater(components["forward_velocity"], 0.0)

    def test_stall_penalty_activates_only_after_time_warmup(self):
        reward = RewardComputer(
            {
                "step_penalty": 0.0,
                "forward_velocity": 0.0,
                "stall_penalty": 0.1,
                "stall_speed_threshold": 0.3,
                "stall_warmup_seconds": 10.0,
            }
        )
        warm_state = {"distance_to_goal": 100.0, "dt": 0.25, "speed": 0.1, "v_los": 0.0, "elapsed_time": 5.0}
        stalled_state = {"distance_to_goal": 100.0, "dt": 0.25, "speed": 0.1, "v_los": 0.0, "elapsed_time": 12.0}

        _, warm_components = reward.compute(warm_state, warm_state, [0.0, 0.0], [0.0, 0.0])
        _, stalled_components = reward.compute(stalled_state, stalled_state, [0.0, 0.0], [0.0, 0.0])

        self.assertEqual(warm_components["stall_penalty"], 0.0)
        self.assertLess(stalled_components["stall_penalty"], 0.0)
```

- [ ] **Step 2: Run the reward tests to verify they fail**

Run:

```powershell
python -m unittest PythonClient.reinforcement_learning.tests.test_reward_semantics -q
```

Expected:

```text
ERROR because RewardComputer does not expose forward_velocity or stall_penalty components yet
```

- [ ] **Step 3: Implement config fields and reward logic**

Extend `RewardConfig` in `PythonClient/reinforcement_learning/config.py`:

```python
forward_velocity: float = 0.5
stall_penalty: float = 0.1
stall_speed_threshold: float = 0.3
stall_warmup_seconds: float = 10.0
```

Validate:

```python
if float(config.reward.stall_speed_threshold) < 0.0:
    raise ValueError("reward.stall_speed_threshold must be >= 0.0")
if float(config.reward.stall_warmup_seconds) < 0.0:
    raise ValueError("reward.stall_warmup_seconds must be >= 0.0")
```

In `PythonClient/reinforcement_learning/airgym/envs/reward.py`, compute time-scaled terms:

```python
dt = float(state.get("dt", 0.25))
step_scale = dt / 0.25
step_penalty = -1.0 * step_scale
forward_velocity = max(float(state.get("v_los", 0.0)), 0.0) * step_scale
stall_penalty = 0.0
if float(state.get("speed", 0.0)) < float(_cfg_get(self.reward_config, "stall_speed_threshold", 0.3)):
    if float(state.get("elapsed_time", 0.0)) > float(_cfg_get(self.reward_config, "stall_warmup_seconds", 10.0)):
        stall_penalty = -1.0 * step_scale
```

Add the new weighted components:

```python
"forward_velocity": float(_cfg_get(self.reward_config, "forward_velocity", 0.0)) * forward_velocity,
"stall_penalty": float(_cfg_get(self.reward_config, "stall_penalty", 0.0)) * stall_penalty,
```

Update `PythonClient/reinforcement_learning/configs/base.yaml`:

```yaml
reward:
  forward_velocity: 0.5
  stall_penalty: 0.1
  stall_speed_threshold: 0.3
  stall_warmup_seconds: 10.0
```

- [ ] **Step 4: Run the reward tests to verify they pass**

Run:

```powershell
python -m unittest PythonClient.reinforcement_learning.tests.test_reward_semantics -q
```

Expected:

```text
Ran 3 tests in ...s
OK
```

- [ ] **Step 5: Commit**

```bash
git add PythonClient/reinforcement_learning/airgym/envs/reward.py PythonClient/reinforcement_learning/config.py PythonClient/reinforcement_learning/configs/base.yaml PythonClient/reinforcement_learning/tests/test_reward_semantics.py
git commit -m "feat: add dt-consistent vessel reward shaping"
```

## Task 3: Export Body-Frame Velocity Semantics And Yaw Scaling Defaults

**Files:**
- Modify: `PythonClient/reinforcement_learning/airgym/envs/vessel_env.py`
- Modify: `PythonClient/reinforcement_learning/configs/base.yaml`
- Test: `PythonClient/reinforcement_learning/tests/test_vessel_env_observation_semantics.py`

- [ ] **Step 1: Write failing body-frame observation tests**

Create `PythonClient/reinforcement_learning/tests/test_vessel_env_observation_semantics.py`:

```python
import math
import unittest

from airgym.envs.vessel_env import body_frame_velocity, los_velocity


class VesselObservationSemanticsTests(unittest.TestCase):
    def test_body_frame_velocity_matches_heading_rotation(self):
        surge, sway = body_frame_velocity(vx=1.0, vy=0.0, heading=math.pi / 2.0)
        self.assertAlmostEqual(surge, 0.0, places=5)
        self.assertAlmostEqual(sway, -1.0, places=5)

    def test_los_velocity_projects_velocity_onto_current_waypoint_direction(self):
        projected = los_velocity(vx=1.0, vy=1.0, dx=10.0, dy=0.0)
        self.assertAlmostEqual(projected, 1.0, places=5)
```

- [ ] **Step 2: Run the new tests to verify they fail**

Run:

```powershell
python -m unittest PythonClient.reinforcement_learning.tests.test_vessel_env_observation_semantics -q
```

Expected:

```text
ImportError or AttributeError because body_frame_velocity / los_velocity do not exist yet
```

- [ ] **Step 3: Implement pure helpers and switch the observation vector**

Add pure helpers near the top of `PythonClient/reinforcement_learning/airgym/envs/vessel_env.py`:

```python
def body_frame_velocity(vx: float, vy: float, heading: float) -> tuple[float, float]:
    return (
        float(vx * math.cos(heading) + vy * math.sin(heading)),
        float(-vx * math.sin(heading) + vy * math.cos(heading)),
    )


def los_velocity(vx: float, vy: float, dx: float, dy: float) -> float:
    norm = math.hypot(dx, dy)
    if norm <= 1e-6:
        return 0.0
    return float((vx * dx + vy * dy) / norm)
```

Replace the world-frame velocity entries in `_get_obs()`:

```python
v_surge, v_sway = body_frame_velocity(linear_velocity_x, linear_velocity_y, heading)
```

and then build `obs` with:

```python
v_surge, v_sway,
linear_acceleration_x, linear_acceleration_y,
angular_acceleration_z,
```

Update defaults in `PythonClient/reinforcement_learning/configs/base.yaml`:

```yaml
env:
  yaw_angle_scale: 0.6
```

- [ ] **Step 4: Run the new body-frame tests and the existing focused slice**

Run:

```powershell
python -m unittest PythonClient.reinforcement_learning.tests.test_vessel_env_observation_semantics PythonClient.reinforcement_learning.tests.test_reward_semantics -q
```

Expected:

```text
Ran ... tests in ...s
OK
```

- [ ] **Step 5: Commit**

```bash
git add PythonClient/reinforcement_learning/airgym/envs/vessel_env.py PythonClient/reinforcement_learning/configs/base.yaml PythonClient/reinforcement_learning/tests/test_vessel_env_observation_semantics.py
git commit -m "feat: switch vessel velocity observations to body frame"
```

## Task 4: Enable Warmup Curriculum And Updated Training Defaults

**Files:**
- Modify: `PythonClient/reinforcement_learning/configs/base.yaml`
- Modify: `PythonClient/reinforcement_learning/configs/curriculum.yaml`
- Test: `PythonClient/reinforcement_learning/tests/test_curriculum_wrapper.py`

- [ ] **Step 1: Write the failing curriculum config test**

Update `PythonClient/reinforcement_learning/tests/test_curriculum_wrapper.py` with a config-level expectation:

```python
from config import DEFAULT_CONFIG, load_config


class CurriculumConfigTests(unittest.TestCase):
    def test_default_curriculum_starts_with_warmup_stage(self):
        config = load_config(DEFAULT_CONFIG)
        self.assertTrue(config.curriculum.enabled)
        self.assertEqual(config.curriculum.stages[0].name, "stage_0_warmup")
        self.assertEqual(int(config.curriculum.stages[0].num_obstacles), 0)
        self.assertEqual(int(config.curriculum.stages[0].num_waypoints), 1)
```

- [ ] **Step 2: Run the curriculum tests to verify they fail**

Run:

```powershell
python -m unittest PythonClient.reinforcement_learning.tests.test_curriculum_wrapper -q
```

Expected:

```text
FAIL because curriculum is disabled by default and the first stage is still easy
```

- [ ] **Step 3: Update base and curriculum defaults**

In `PythonClient/reinforcement_learning/configs/base.yaml`:

```yaml
curriculum:
  enabled: true
```

In `PythonClient/reinforcement_learning/configs/curriculum.yaml`, insert:

```yaml
stages:
  - name: stage_0_warmup
    num_obstacles: 0
    num_dynamic_obstacles: 0
    num_waypoints: 1
    length: 4
    angle_range: [-10.0, 10.0]
  - name: easy
    num_obstacles: 2
    num_dynamic_obstacles: 0
    num_waypoints: 1
    length: 6
    angle_range: [-20.0, 20.0]
```

Keep the remaining stages unchanged.

- [ ] **Step 4: Run the curriculum tests to verify they pass**

Run:

```powershell
python -m unittest PythonClient.reinforcement_learning.tests.test_curriculum_wrapper -q
```

Expected:

```text
Ran ... tests in ...s
OK
```

- [ ] **Step 5: Commit**

```bash
git add PythonClient/reinforcement_learning/configs/base.yaml PythonClient/reinforcement_learning/configs/curriculum.yaml PythonClient/reinforcement_learning/tests/test_curriculum_wrapper.py
git commit -m "feat: enable warmup curriculum defaults"
```

## Task 5: Full Focused Verification Slice

**Files:**
- Modify: none
- Test: `PythonClient/reinforcement_learning/tests/test_train_callbacks.py`
- Test: `PythonClient/reinforcement_learning/tests/test_eval_suite_semantics.py`
- Test: `PythonClient/reinforcement_learning/tests/test_reward_semantics.py`
- Test: `PythonClient/reinforcement_learning/tests/test_vessel_env_observation_semantics.py`
- Test: `PythonClient/reinforcement_learning/tests/test_curriculum_wrapper.py`

- [ ] **Step 1: Run the full focused verification slice**

Run:

```powershell
python -m unittest PythonClient.reinforcement_learning.tests.test_train_callbacks PythonClient.reinforcement_learning.tests.test_eval_suite_semantics PythonClient.reinforcement_learning.tests.test_reward_semantics PythonClient.reinforcement_learning.tests.test_vessel_env_observation_semantics PythonClient.reinforcement_learning.tests.test_curriculum_wrapper -q
```

Expected:

```text
Ran ... tests in ...s
OK
```

- [ ] **Step 2: Capture the implementation state**

```bash
git status --short
git log --oneline -5
```

Expected:

```text
working tree clean
latest commits correspond to Tasks 1-4
```

- [ ] **Step 3: Commit if needed**

```bash
git add -A
git commit -m "chore: finalize RL timeout remediation implementation"
```

Only do this step if Task 1-4 left uncommitted changes.
