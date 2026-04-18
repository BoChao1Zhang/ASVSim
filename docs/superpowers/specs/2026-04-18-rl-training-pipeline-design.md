# ASVSim RL Training Pipeline Design

Date: `2026-04-18`
Scope: `PythonClient/reinforcement_learning`
Status: `Approved in brainstorming, pending implementation`

## 1. Goal

Build a stable minimum-closed-loop RL training pipeline for the vessel task under the following fixed constraints:

- simulator mode: `Blocks.exe` launched and managed by the training pipeline
- baseline task: small seed pool, `1 waypoint + 2 static obstacles`
- algorithm: `CrossQ` only
- success definition: only `end_reason == "goal_reached"`
- train/eval split: fixed and strictly separated seed pools
- online eval: disabled during training
- checkpoint resume: full recovery of `model + VecNormalize + replay buffer + timestep`
- acceptance gate: `dev eval success_rate >= 80%`
- crash budget: training must stop if `sim_crash_rate > 1%`

The first version optimizes for training stability and experiment trustworthiness, not for maximum sample efficiency or curriculum complexity.

## 2. Non-Goals

The first version does not attempt to solve the following:

- online curriculum promotion during training
- multi-algorithm benchmarking
- generalized PCG training across large seed spaces
- reward redesign
- editor-attach-first workflow
- distributed or multi-instance simulator rollouts

## 3. Current Context

The local repository already contains a modernized RL stack:

- training entrypoint: `PythonClient/reinforcement_learning/train.py`
- environment: `PythonClient/reinforcement_learning/airgym/envs/vessel_env.py`
- evaluation suite: `PythonClient/reinforcement_learning/eval_suite.py`
- structured config: `PythonClient/reinforcement_learning/config.py`

Recent local fixes already addressed several semantic issues:

- terminal/reset semantics under `DummyVecEnv`
- evaluation reseeding semantics
- curriculum stage attribution
- PCG cleanup ownership and runtime edge cases

However, the current pipeline still mixes concerns that should be isolated for stable simulator-backed training:

- training and evaluation are both triggered from the same training process
- checkpoints are not yet defined as full recovery snapshots
- `best model` selection is not yet a first-class offline stage
- simulator health checks are not gated by a pre-training smoke test

## 4. External Design Guidance

This design follows the same engineering direction as current Gymnasium and SB3 guidance:

- custom envs must implement strict `reset(seed=...)` and `step(...) -> obs, reward, terminated, truncated, info`
- evaluation should use separate env instances and frozen normalization stats
- simulator-backed RL experiments should separate training progression from evaluation progression

Comparable simulator RL projects also reinforce the same pattern:

- MetaDrive uses explicit train/test scenario separation and held-out evaluation
- SB3 and RL Zoo treat evaluation and checkpointing as explicit experiment stages, not hidden side effects

References:

- [Farama Foundation, n.d., Gymnasium Environment Creation, https://gymnasium.farama.org/tutorials/gymnasium_basics/environment_creation/]
- [Farama Foundation, n.d., Gymnasium Env API, https://gymnasium.farama.org/api/env/]
- [Stable-Baselines3 Team, n.d., Custom Environments, https://stable-baselines3.readthedocs.io/en/master/guide/custom_env.html]
- [Stable-Baselines3 Team, n.d., VecEnv and VecNormalize, https://stable-baselines3.readthedocs.io/en/master/guide/vec_envs.html]
- [Stable-Baselines3 Team, n.d., Callbacks, https://stable-baselines3.readthedocs.io/en/master/guide/callbacks.html]
- [Stable-Baselines3 Team, n.d., RL Zoo, https://stable-baselines3.readthedocs.io/en/master/guide/rl_zoo.html]
- [SB3-Contrib Team, n.d., CrossQ, https://sb3-contrib.readthedocs.io/en/master/modules/crossq.html]
- [MetaDrive Project, n.d., RL Environments, https://metadrive-simulator.readthedocs.io/en/latest/rl_environments.html]

## 5. High-Level Architecture

The first version adopts a minimal split architecture:

1. `train.py`
   Produces recoverable checkpoints only. Does not perform online evaluation.

2. `eval_suite.py`
   Consumes a checkpoint in a separate process and evaluates it on the fixed `dev eval` seed pool.

3. `best-model selection`
   Runs after training and evaluates only the last `33%` of checkpoints, then promotes the highest-scoring eligible checkpoint to `best/`.

Core rule:

```markdown
Training state progression and evaluation state progression must be isolated.
```

In practical terms:

- the training process advances timesteps and replay buffer state
- the evaluation process never mutates training state
- `best model` is derived from offline evaluation artifacts, not from in-training callbacks

## 6. Fixed Experiment Contract

### 6.1 Task Contract

- seed pool:
  - `train`: 8 fixed seeds
  - `dev eval`: 4 fixed seeds
  - no overlap between the two pools
- map/task shape:
  - small seed pool
  - `1 waypoint`
  - `2 static obstacles`
- control:
  - keep current control cadence
  - do not change `action_repeat`, `step_sleep`, or general control feel in version 1

### 6.2 Success Contract

- success:
  - only `goal_reached`
- failure:
  - `collision`
  - `timeout`
  - `sim_crash`
- pass condition:
  - `dev eval success_rate >= 80%`

### 6.3 Crash Contract

- `sim_crash` is treated as an infrastructure event, not a valid learning outcome
- the training process may restart `Blocks.exe` and continue
- once cumulative `sim_crash_rate > 1%`, training must stop and mark the run as failed

## 7. Training Process Design

### 7.1 Startup Flow

The training process should follow this order:

1. load resolved config
2. create run directory
3. launch `Blocks.exe`
4. run smoke test
5. if smoke test passes, create train env and model
6. train while checkpointing and monitoring crash budget
7. save final snapshot
8. terminate simulator cleanly

If smoke test fails, the run must stop before `learn()`.

### 7.2 No Online Eval

The first version intentionally disables online evaluation during training.

Reason:

- this repository uses a simulator-backed environment
- online evaluation would create a second environment lifecycle inside the training process
- that increases the chance of state contamination between training and evaluation
- the minimum stable pipeline should favor isolation over convenience

This means:

- no periodic subset eval callback during `learn()`
- training only writes checkpoints and trainer state
- evaluation is a separate explicit stage after or between training sessions

### 7.3 Resume Behavior

Resume must be a real continuation, not a partial warm start.

When resuming:

- load model weights
- load `VecNormalize` stats
- load replay buffer
- restore global timestep count
- restore crash counters
- continue writing into the same run directory lineage

`best/` is not resumable state.

## 8. Checkpoint Design

Each checkpoint is a complete recovery snapshot.

Recommended layout:

```markdown
run_dir/
- config.yaml
- git_sha.txt
- smoke_test.json
- checkpoints/
  - step_00025000/
    - model.zip
    - vecnormalize.pkl
    - replay_buffer.pkl
    - trainer_state.json
- latest/
  - model.zip
  - vecnormalize.pkl
  - replay_buffer.pkl
  - trainer_state.json
- best/
  - model.zip
  - vecnormalize.pkl
  - metrics.json
- final/
  - model.zip
  - vecnormalize.pkl
```

`trainer_state.json` should include at least:

- `global_timesteps`
- `completed_checkpoints`
- `total_episodes`
- `sim_crash_episodes`
- `crash_rate`
- `train_seed_pool_id`
- `dev_eval_seed_pool_id`
- `algo`
- `resume_parent` if applicable

Checkpoint policy:

- keep full step-indexed checkpoints
- update `latest/` every checkpoint
- keep `final/` only once at training end
- create/update `best/` only after offline checkpoint selection completes

## 9. Smoke Test Design

### 9.1 Purpose

The smoke test verifies that the current simulator instance and environment wiring are healthy enough to justify long training.

It is not a policy quality test.

### 9.2 Proposed Flow

Use the first two train seeds:

1. start simulator and wait for world readiness
2. create env
3. for each selected seed:
   - `reset(seed=...)`
   - take a short fixed sequence of gentle actions
   - collect environment and simulator sanity signals

### 9.3 Required Checks

Process and connectivity:

- `Blocks.exe` is alive
- RPC connection succeeds
- API control is enabled
- vehicle setup succeeds

Scenario generation:

- `activateGeneration(False)` succeeds
- terrain generation succeeds
- waypoint result is valid
- no invalid `(0, 0)` final target

Env API:

- `reset()` succeeds
- observation shape matches `observation_space`
- rewards and observations are finite
- `info` fields are present and well-formed

Runtime safety:

- no `sim_crash`
- no reset/step exception
- vessel positions remain finite and non-explosive
- lidar pooled dimensions are valid

### 9.4 Failure Policy

If any of the following occurs, the run must fail before training:

- any `sim_crash`
- any reset/step exception
- any `NaN` or `Inf`
- invalid waypoint/goal state
- observation dimension mismatch

Artifact outputs:

- `smoke_test.json`
- `smoke_test.log`

## 10. Evaluation Design

### 10.1 Separate Evaluation Process

`eval_suite.py` should evaluate one checkpoint at a time in a separate process.

Evaluation requirements:

- load checkpoint model
- load the paired `vecnormalize.pkl`
- set `VecNormalize.training = False`
- set `VecNormalize.norm_reward = False`
- use only `dev eval` seeds
- do not mutate train seed scheduling

### 10.2 Metrics

Primary metric:

- `success_rate`

Secondary metrics:

- `collision_rate`
- `timeout_rate`
- `sim_crash_rate`
- `mean_final_dist`
- `median_path_length_ratio`

The acceptance gate is based on primary metric only:

- pass if `success_rate >= 0.80`

### 10.3 Checkpoint Candidate Set

Offline `best model` selection should evaluate only the last `33%` of produced checkpoints.

Reason:

- early checkpoints are unlikely to be competitive
- evaluating all checkpoints is expensive under simulator-backed rollouts
- the first version should minimize evaluation overhead while preserving a real model selection stage

## 11. Best Model Selection

Eligibility rules:

1. candidate must belong to the last `33%` of checkpoints
2. candidate must satisfy crash eligibility if evaluation exposes simulator instability
3. candidate ranking is by `dev eval success_rate`

Tie-break order:

1. higher `success_rate`
2. lower `collision_rate`
3. lower `mean_final_dist`
4. later checkpoint

The selected candidate is copied or promoted to:

```markdown
run_dir/best/
```

and must include:

- `model.zip`
- `vecnormalize.pkl`
- `metrics.json`

## 12. Reward and Termination Policy

The first version must preserve the current task definition and most of the current reward structure.

### 12.1 Preserve

- observation structure
- action space
- waypoint task
- dense reward components:
  - progress
  - heading alignment
  - obstacle proximity
  - action rate
  - cross-track
  - step penalty

### 12.2 Adjust

Add one new reward configuration term:

```yaml
reward:
  terminal_timeout: -25.0
```

Policy:

- `goal_reached`
  - terminal success
- `collision`
  - terminal failure
- `timeout`
  - explicit failure
  - receives mild terminal penalty
- `sim_crash`
  - explicit infrastructure failure
  - never counted as success
  - tracked separately from task reward analysis

Rationale:

- timeout should be visibly worse than ordinary step accumulation
- timeout should remain less severe than collision
- reward changes in version 1 must stay minimal to preserve debuggability

## 13. Configuration Changes

The implementation should introduce explicit config for:

- fixed train seed pool
- fixed dev eval seed pool
- smoke test enable/disable and step count
- crash budget threshold
- checkpoint resume mode
- offline best-model selection policy
- `terminal_timeout`

Version 1 should not keep these rules implicit inside callbacks or hard-coded loops.

## 14. Validation Plan

Implementation is considered complete only if all of the following are verified:

1. smoke test fails correctly on invalid simulator state
2. a run can train from scratch and produce checkpoints
3. a run can resume from `latest/` with replay buffer and timestep continuity
4. offline evaluation can score a chosen checkpoint on the dev seed pool
5. `best/` can be selected from the final `33%` checkpoint subset
6. crash budget enforcement stops the run once cumulative `sim_crash_rate > 1%`

### 14.1 Milestone Gate

Beyond unit tests, every integrated feature milestone must satisfy this release gate before it is considered complete:

1. the relevant unit-test slice passes
2. a smoke test passes against the integrated runtime path
3. the milestone result is captured in a dedicated commit
4. Git LFS coverage and status are verified for changed binary assets, especially under `Unreal/Environments/PortEnv`

Operational notes:

- a milestone is not considered verified by unit tests alone
- if a new PortEnv binary artifact type appears and is not already covered by existing LFS rules, `.gitattributes` must be updated before the commit is created
- existing repository coverage already includes `*.uasset`, `*.umap`, `*.zip`, and `*.rar`, so PortEnv map and packaged artifacts should remain LFS-managed unless a new binary extension is introduced

Recommended verification stages:

- unit tests for checkpoint metadata and resume logic
- unit tests for best-model candidate selection
- unit tests for crash-budget aggregation
- short live smoke run against `Blocks.exe`
- longer manual training run

## 15. Risks and Limitations

- keeping current control cadence will constrain throughput
- `CrossQ` remains a newer algorithm than core SB3 baselines, so hyperparameter stability is less battle-tested
- a single simulator instance still limits parallelism
- fixed small seed pools are suitable for minimum closed-loop validation, not for generalization claims
- crash recovery can preserve throughput but may still hide deeper simulator faults if logs are not inspected

## 16. Implementation Boundary

Version 1 should implement only the following product surface:

- stable training entrypoint for `Blocks.exe`
- pre-training smoke test
- full resumable checkpoints
- separate offline evaluation
- offline best-model promotion
- minimal reward/timeout clarification

Anything beyond that should be deferred until the minimum closed loop is proven by `dev eval success_rate >= 80%`.
