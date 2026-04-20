# ASVSim RL Timeout Remediation Design

Date: `2026-04-19`
Scope: `PythonClient/reinforcement_learning`
Status: `Approved for implementation`

## 1. Goal

Raise evaluation success from `0` to a measurable non-zero level for the current sequential waypoint-following vessel task without changing the task definition itself:

- waypoint task remains sequential
- `waypoint_radius = 10 m`
- only the final waypoint ends the episode with `goal_reached`
- the fix must target control efficiency, reward incentives, observability, and experiment trustworthiness

The first implementation optimizes for trustworthy diagnosis and minimum viable policy improvement, not for architectural refactors such as hierarchical control or residual guidance priors.

## 2. Fixed Constraints

The following contracts are fixed for this remediation:

- simulator/runtime stack remains the current `CrossQ + vessel_env.py` training loop
- the current environment/action API shape remains two continuous controls
- waypoint semantics remain unchanged
- reward redesign must preserve the current dense-reward structure where possible instead of replacing it wholesale
- implementation must proceed in small phases with probe checkpoints between phases

## 3. Non-Goals

This remediation does not include:

- changing the task from sequential waypoint following to terminal-per-waypoint episodes
- replacing the action interface with `[u_desired, r_desired] + PID`
- adding LOS residual priors or model-based inner loops
- broad reward re-architecture
- recurrent policies
- large-scale benchmark sweeps before the core timeout issue is fixed

## 4. Verified Current Context

### 4.1 Local Evidence

The local repository and run artifacts support the following facts:

- `stability_probe_20260419_1630` proves the train/eval loop can complete without simulator failure, but its physical episode horizon is only `150 * 0.05 * 1 = 7.5 s`, so it is not sufficient by itself to diagnose control inefficiency.
- `waypoint_budget_probe_20260419_1905` is the primary behavioral baseline for this remediation. It uses waypoint-derived time budgets and still shows only `timeout` or `collision`, which is stronger evidence that the problem is not only a short-horizon configuration issue.
- the current action scaling is strongly biased toward turning before moving:
  - `yaw_angle_scale = 0.25` and `angle_delta = 0.5 * yaw_cmd * yaw_angle_scale`
  - this yields a maximum effective rudder command of `7.5 deg`
  - `ThrusterParams` allows `30 deg`
  - thrust and rudder filters are asymmetric: `2.0 s` vs `0.5 s`
- the current reward contains a stationary local optimum region because heading alignment can outweigh the per-step penalty when obstacle cost is small and progress is near zero.
- the current logging is semantically ambiguous:
  - step-state `distance_to_goal_x/y` refers to the current waypoint
  - terminal `info["distance_to_goal_x/y"]` refers to the final goal
  - both `train.py` and `eval_suite.py` consume the terminal fields as if they were a single unambiguous metric

### 4.2 Observation Context

The current observation already contains:

- world-frame linear velocity `vx`, `vy`
- linear acceleration `ax`, `ay`
- angular acceleration `yaw_accel`

Therefore `n_stack = 4` is not justified as a way to add missing acceleration information. If enabled later, its purpose is to expose short-horizon temporal context, actuator lag, and disturbance persistence.

### 4.3 External Guidance

The following external guidance is accepted as relevant for this design:

- potential-based shaping preserves policy invariance; fixed subgoal bonuses do not automatically inherit that guarantee
- curriculum learning is well-supported for navigation tasks with sparse success signals
- underactuated marine systems are naturally expressed in body-fixed surge/sway/yaw variables and are slower to change velocity and heading than ground robots
- action repetition can reduce effective horizon, but it is a trade-off rather than a universal improvement

References:

- [Ng et al., 1999, Sec. 3, https://people.eecs.berkeley.edu/~russell/papers/icml99-shaping.pdf]
- [Devlin and Kudenko, 2012, Sec. 2, https://eprints.whiterose.ac.uk/id/eprint/75121/2/p433_devlin.pdf]
- [Morad et al., 2020, Abstract and Sec. 1, http://www.rudrapoudel.com/docs/2020-RA-L-NavACL.pdf]
- [Narvekar et al., 2020, Sec. 1-2, https://jmlr.org/papers/v21/20-212.html]
- [Li et al., 2023, Sec. 1 and Sec. 2.3, https://pmc.ncbi.nlm.nih.gov/articles/PMC10099039/]
- [Luo et al., 2025, Abstract and Sec. 1, https://www.mdpi.com/2077-1312/13/12/2392]
- [Kalyanakrishnan et al., 2021, Abstract and Sec. 3, https://arxiv.org/pdf/2102.03718]

## 5. Approved Design Decisions

### 5.1 Baseline Evidence Contract

Use the two existing runs with different roles:

- `stability_probe_20260419_1630`
  - role: closed-loop stability baseline only
- `waypoint_budget_probe_20260419_1905`
  - role: primary timeout/collision behavioral baseline

All future spec and implementation notes must preserve this distinction.

### 5.2 Time-Scale Consistency Contract

All per-step reward terms that encode elapsed time or per-step incentives must be normalized by physical step duration, not just the legacy `step_penalty`.

Implementation rule:

```markdown
step_scale = dt / 0.25
```

Where:

```markdown
dt = step_sleep * action_repeat
```

The following terms must obey this rule:

- legacy step penalty
- forward-motion reward
- stall penalty
- any later per-step motion gate that would otherwise vary with `action_repeat`

This prevents `action_repeat` changes from silently redefining task difficulty.

### 5.3 Reward Directionality Contract

The dense reward must explicitly favor moving toward the active waypoint, not only facing it.

Approved forward-motion shaping:

```markdown
forward_velocity_term = max(v_los, 0.0) * step_scale
```

Where:

```markdown
v_los = dot([vx, vy], unit([dx_curr, dy_curr]))
```

Approved stall logic:

```markdown
if speed < stall_speed_threshold and elapsed_time_s > stall_warmup_seconds:
    stall_term = -1.0 * step_scale
else:
    stall_term = 0.0
```

Config weights multiply these raw terms in the same style as the existing reward components.

### 5.4 Minimal Action Fix Contract

The only action-space change in this remediation is:

```markdown
env.yaw_angle_scale: 0.25 -> 0.6
```

This raises the effective maximum rudder command from `7.5 deg` to `18 deg` while remaining below the actuator limit.

No decoupled speed/yaw target interface is part of this implementation.

### 5.5 Observation Contract

The first observation improvement is semantic, not dimensional:

- replace world-frame `vx`, `vy` in the observation with body-frame `surge`, `sway`
- keep `single_obs_size = 54`
- keep observation shape unchanged during this phase

Definitions:

```markdown
v_surge = vx * cos(heading) + vy * sin(heading)
v_sway = -vx * sin(heading) + vy * cos(heading)
```

This change is approved ahead of frame stacking.

### 5.6 Logging Contract

Distance fields must become explicit and non-overloaded.

Approved terminal info keys:

- `distance_to_final_goal`
- `distance_to_current_wp`

Approved motion diagnostics exported from the environment:

- `v_los`
- `v_surge`
- `speed`

Approved episode-level diagnostics aggregated by training callbacks:

- `episode/mean_v_surge`
- `episode/time_moving_frac`
- `episode/final_distance_to_current_wp`
- `episode/final_distance_to_goal`

### 5.7 Curriculum Contract

Curriculum is part of the mainline fix.

Approved change:

- enable curriculum wrapper
- insert a new warmup stage before the existing stages
- preserve the current success criterion: `info["end_reason"] == "goal_reached"`

Warmup stage target:

```yaml
name: stage_0_warmup
num_obstacles: 0
num_dynamic_obstacles: 0
num_waypoints: 1
length: 4
angle_range: [-10.0, 10.0]
```

### 5.8 Conditional And Deferred Items

The following decisions are closed by default:

- `C1: n_stack = 4`
  - status: conditional
  - default: keep `n_stack = 1`
  - only enable if P1 still shows clear actuator-lag or short-memory failure modes
- `A4: heading-align weakening + movement gate`
  - status: conditional
  - default: keep current `heading_align = 0.3`
  - only enable if stationary alignment behavior persists after forward-motion shaping
- `F1: norm_reward = false`
  - status: ablation only
  - default: keep `norm_reward = true`
- `A3: intermediate waypoint bonus`
  - status: deferred
  - default: do not implement in the mainline
  - if reintroduced later, document it as a heuristic subgoal reward, not as PBRS

## 6. Naming And Compatibility Rules

### 6.1 New Config Field Names

Use time-based names rather than step-based names:

- `reward.forward_velocity`
- `reward.stall_penalty`
- `reward.stall_speed_threshold`
- `reward.stall_warmup_seconds`

Do not introduce a step-count-based `stall_warmup` in the final config surface.

### 6.2 Resume And Artifact Compatibility

Compatibility rules by phase:

- after `C2`, observation shape stays the same, but observation semantics change
- after `A2`, reward statistics change
- after `F2`, physical cadence per policy step changes
- after `C1`, observation shape changes from `54` to `216`

Operational rule:

```markdown
Every phase must use a fresh run directory.
Do not reuse replay buffers or VecNormalize artifacts across phases.
```

## 7. Implementation Scope By File

### 7.1 `PythonClient/reinforcement_learning/airgym/envs/reward.py`

Add:

- time-normalized step penalty
- forward-motion reward term
- stall penalty term

Preserve:

- progress term
- heading alignment term
- obstacle proximity term
- action-rate term
- cross-track term

Do not add intermediate waypoint bonus in the mainline.

### 7.2 `PythonClient/reinforcement_learning/airgym/envs/vessel_env.py`

Add or change:

- compute `dt = step_sleep * action_repeat`
- compute `v_los`, `speed`, `v_surge`, `v_sway`
- inject those values into reward state / info
- replace world-frame velocity observation entries with body-frame entries
- split terminal distance reporting into explicit final-goal and current-waypoint fields
- include per-episode diagnostic values needed by callbacks

### 7.3 `PythonClient/reinforcement_learning/config.py`

Add:

- new reward config fields
- any validation needed for time-based stall configuration
- preserve current `action_repeat` step-budget normalization behavior

### 7.4 `PythonClient/reinforcement_learning/configs/base.yaml`

Mainline target values:

```yaml
env:
  yaw_angle_scale: 0.6
  action_repeat: 1
  n_stack: 1

reward:
  heading_align: 0.3
  forward_velocity: 0.5
  stall_penalty: 0.1
  stall_speed_threshold: 0.3
  stall_warmup_seconds: 10.0

curriculum:
  enabled: true

algo:
  norm_reward: true

train:
  total_timesteps: 2_500_000
```

Conditional P2 values:

```yaml
env:
  action_repeat: 2

train:
  total_timesteps: 1_250_000
```

Conditional `C1` values:

```yaml
env:
  n_stack: 4
```

Conditional `A4` values:

```yaml
reward:
  heading_align: 0.1
```

### 7.5 `PythonClient/reinforcement_learning/configs/curriculum.yaml`

Insert `stage_0_warmup` before the existing stages and keep the rest unchanged unless later probes demonstrate a clear need for retuning.

### 7.6 `PythonClient/reinforcement_learning/train.py`

Add or change:

- consume the new explicit terminal distance fields
- log `episode/mean_v_surge`
- log `episode/time_moving_frac`
- keep reward component aggregation generic so new `forward_velocity` and `stall_penalty` panels appear automatically

### 7.7 `PythonClient/reinforcement_learning/eval_suite.py`

Change:

- consume `distance_to_final_goal` rather than overloaded `distance_to_goal_x/y`
- preserve stage-level success/collision/timeout reporting

### 7.8 Tests

Update or add tests in:

- `PythonClient/reinforcement_learning/tests/test_eval_suite_semantics.py`
  - verify `final_dist` comes from `distance_to_final_goal`
- `PythonClient/reinforcement_learning/tests/test_train_callbacks.py`
  - verify new episode metrics are logged from explicit info keys
- `PythonClient/reinforcement_learning/tests/test_curriculum_wrapper.py`
  - verify warmup stage naming and promotion semantics still hold
- new focused reward/info tests if the existing suite does not cleanly cover:
  - time-normalized step penalty
  - forward-motion reward positivity
  - stall penalty activation after elapsed-time warmup
  - body-frame velocity conversion

## 8. Phased Implementation Plan

### Phase P0: Diagnostic Correctness And Minimum Control Fix

Scope:

- `E1` explicit distance fields
- `A1` time-normalized per-step penalty
- `B1` `yaw_angle_scale = 0.6`
- `E2` motion diagnostics
- `C2` body-frame velocity observation

Exit criteria:

- `episode/mean_thrust > 0.3`
- `reward_components/progress > 0`
- `episode/final_distance_to_current_wp` decreases relative to baseline probes

### Phase P1: Reward Directionality And Learnable Curriculum

Scope:

- `A2` forward-motion shaping + stall penalty
- `D1` warmup curriculum stage

Exit criteria:

- `stage_0_warmup` produces at least one `goal_reached` in probe-scale training
- `reward_components/forward_velocity` is positive on successful or partially successful episodes
- `reward_components/stall_penalty` trends toward zero as movement improves

### Phase P2: Cadence Compression

Scope:

- `F2` `action_repeat = 2`
- `train.total_timesteps = 1_250_000`

Exit criteria:

- timeout rate decreases without losing diagnostic clarity
- physical episode duration remains comparable to the `action_repeat = 1` baseline

### Conditional Phase P3

Only if needed:

- `C1` `n_stack = 4`
- `A4` weaker heading alignment plus motion gate
- `F1` `norm_reward = false` as a late ablation

## 9. Validation Protocol

### 9.1 Baselines

Keep both prior run artifacts for comparison:

- stability baseline: `stability_probe_20260419_1630`
- behavioral baseline: `waypoint_budget_probe_20260419_1905`

### 9.2 Probe Sequence

Probe 1:

```markdown
P0 only
short training/probe run
fresh run_dir
same seed policy as the baseline comparison
```

Probe 2:

```markdown
P0 + P1
short training/probe run
fresh run_dir
expect first warmup-stage goal
```

Probe 3:

```markdown
P0 + P1 + P2
small training run before long training
fresh run_dir
```

Main run:

```markdown
full training with approved settings
full eval after training
fresh run_dir
```

### 9.3 Primary Metrics

Ranked primary metrics:

1. `episode/goal_reached_rate`
2. `episode/timeout_rate`
3. `episode/mean_v_surge`
4. `episode/time_moving_frac`
5. `episode/final_distance_to_current_wp`
6. `reward_components/progress`
7. `reward_components/forward_velocity`
8. `reward_components/stall_penalty`
9. `curriculum/stage`

## 10. Risks And Fallbacks

### 10.1 If P0 Fails

Symptoms:

- `mean_thrust <= 0.2`
- progress remains non-positive

Fallback checks:

- confirm `dt` is computed and passed correctly
- confirm `yaw_angle_scale` is resolved from config and reaches the environment instance
- confirm terminal-distance field consumers were updated in both training and evaluation code paths

### 10.2 If P1 Fails

Symptoms:

- no warmup-stage goals
- forward-motion reward remains near zero

Fallback order:

1. increase `reward.forward_velocity` from `0.5` to `0.8`
2. increase `reward.stall_warmup_seconds` from `10.0` to `20.0`
3. only then consider enabling `C1`

### 10.3 If P2 Regresses

Symptoms:

- lower timeout rate but worse movement quality
- unstable reward statistics

Fallback:

- revert to `action_repeat = 1`
- keep the reward and curriculum changes
- delay cadence compression until after a successful long run

## 11. Acceptance Criteria

The remediation is accepted when all of the following are true:

- full evaluation success is measurably non-zero
- timeout rate decreases from the current behavioral baseline
- motion diagnostics show real forward movement rather than only heading alignment
- final-distance logging is semantically correct and consistent across training and evaluation
- the mainline result does not depend on deferred heuristic subgoal bonuses

