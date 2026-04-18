# Issue 02: RL Training And Evaluation Semantics Are Not Trustworthy Yet

## Summary

The current Python RL stack has several control-flow and measurement bugs that can distort training curves, evaluation metrics, and curriculum statistics even if the simulator world state is completely clean.

This issue is independent from runtime PCG cleanup.

## Scope

- training callback behavior
- evaluation reproducibility
- curriculum metric labeling
- configuration surface vs actual behavior
- summary metric completeness

## Verified Facts

### 1. SB3 `DummyVecEnv` already resets environments automatically after `done`

Local installed file:

```text
.venv/Lib/site-packages/stable_baselines3/common/vec_env/dummy_vec_env.py
```

Relevant lines:

```text
dummy_vec_env.py:68-72
```

Behavior:

- when `terminated or truncated` is true
- `DummyVecEnv` stores `terminal_observation`
- then immediately calls `env.reset()`

Consequence:

- user code should not blindly issue another reset unless it deliberately wants to skip ahead to a brand-new episode

### 2. `EvalSuiteCallback` performs an extra training-environment reset after evaluation

File:

```text
PythonClient/reinforcement_learning/train.py
```

Relevant lines:

```text
train.py:120-138
```

Behavior:

1. callback waits until `num_timesteps >= eval_freq`
2. callback only runs after an episode terminates
3. it launches `run_eval_suite(...)`
4. it then calls `training_env.reset()` manually

Consequence:

- because `DummyVecEnv` already reset on episode termination, this callback burns one extra episode boundary
- `episode_count` in the underlying env advances again
- terrain regeneration cadence, curriculum scheduling, and RNG progression all shift

### 3. The vessel environment increments episode state on every reset

File:

```text
PythonClient/reinforcement_learning/airgym/envs/vessel_env.py
```

Relevant lines:

```text
vessel_env.py:836-842
```

Behavior:

- `episode_count += 1`
- next-episode parameters are applied
- terrain regeneration policy is evaluated

Consequence:

- any extra reset is not harmless bookkeeping
- it mutates actual experiment state

### 4. `eval_suite.py` advertises a seed dimension but does not reseed the environment RNG per seed

File:

```text
PythonClient/reinforcement_learning/eval_suite.py
```

Relevant lines:

```text
eval_suite.py:327-368
```

Current behavior:

- loops over `seed_index`
- computes `seed_value`
- writes `seed_value` into the result row
- never calls `env.reset(seed=seed_value)`

Consequence:

- the CSV has a `seed` column
- but that column does not correspond to a full environment reseed

### 5. The environment only resets its internal RNG when `reset(seed=...)` is used

File:

```text
PythonClient/reinforcement_learning/airgym/envs/vessel_env.py
```

Relevant lines:

```text
vessel_env.py:83
vessel_env.py:837-839
```

Current behavior:

- `self.rng = np.random.RandomState(seed)` at construction
- `self.rng` is reassigned only when `reset(seed=...)` receives a non-`None` seed

Consequence:

- without explicit reseeding, the same RNG stream continues across all evaluation seeds

### 6. The supposedly per-seed evaluation randomness only controls terrain seed

File:

```text
PythonClient/reinforcement_learning/eval_suite.py
```

Relevant lines:

```text
eval_suite.py:331-339
```

Current behavior:

- `terrain_seed` is injected through `set_next_episode_params(...)`
- obstacle placement RNG, LiDAR noise RNG, and heading noise RNG still come from the same continuing `self.rng` stream unless `reset(seed=...)` is also used

Related environment call sites:

```text
vessel_env.py:381-383   obstacle placement RNG
vessel_env.py:588       lidar noise RNG
vessel_env.py:634       heading noise RNG
```

Consequence:

- evaluation is not fully stratified by seed
- reproducibility claims based on the `seed` column are overstated

### 7. Curriculum stage labels are overwritten after promotion

File:

```text
PythonClient/reinforcement_learning/airgym/wrappers/curriculum.py
```

Relevant lines:

```text
curriculum.py:50-69
```

Current behavior:

1. terminal episode result is observed
2. success history is updated
3. wrapper may promote to the next stage immediately
4. `info["curriculum_stage"]` and `info["curriculum_stage_name"]` are rewritten after promotion

Consequence:

- the terminal episode that caused the promotion can be logged as if it belonged to the next stage

### 8. Training metrics consume the possibly rewritten curriculum stage directly

File:

```text
PythonClient/reinforcement_learning/train.py
```

Relevant lines:

```text
train.py:74-94
```

Consequence:

- stage-level reward or success curves can be misattributed around promotion boundaries

### 9. `num_dynamic_obstacles` is exposed as a formal experiment dimension, but runtime behavior is hard-disabled

Files:

```text
PythonClient/reinforcement_learning/config.py
PythonClient/reinforcement_learning/airgym/wrappers/curriculum.py
PythonClient/reinforcement_learning/eval_suite.py
PythonClient/reinforcement_learning/airgym/envs/vessel_env.py
```

Evidence:

- config exposes `num_dynamic_obstacles`
- curriculum stages carry `num_dynamic_obstacles`
- eval suite forwards `num_dynamic_obstacles`
- env `_spawn_dynamic_obstacles()` prints a warning and returns immediately

Relevant lines:

```text
config.py:38
config.py:99
curriculum.py:28-35
eval_suite.py:333-339
vessel_env.py:484-495
```

Consequence:

- experiment configs can claim a difficulty axis that is currently inactive
- results are easy to misread

### 10. `sim_crash` is tracked during training but omitted from evaluation summaries

Files:

```text
PythonClient/reinforcement_learning/airgym/envs/vessel_env.py
PythonClient/reinforcement_learning/train.py
PythonClient/reinforcement_learning/eval_suite.py
```

Evidence:

- the env emits `end_reason = "sim_crash"` on recovery
- training callback explicitly includes `sim_crash`
- evaluation summary only aggregates `goal_reached`, `collision`, and `timeout`

Relevant lines:

```text
vessel_env.py:735-753
train.py:48
eval_suite.py:250-270
```

Consequence:

- evaluation headline metrics can hide simulator instability

## Inference

The current RL pipeline can produce misleading outputs in at least four ways:

1. extra resets alter the training episode stream
2. the evaluation `seed` column overstates reproducibility
3. curriculum stage metrics can be mislabeled
4. some configured difficulty dimensions are not actually active

Confidence:

- High

These are direct control-flow and data-semantics issues, not speculative reward-quality complaints.

## Relationship To Issue 01

- Issue 01 can corrupt the simulator world between episodes.
- Issue 02 can corrupt the meaning of training and evaluation statistics even when the world is correct.
- Both issues can exist at the same time.

## Open Questions

### 1. Does the extra reset materially affect learned policy quality, or mostly experiment reproducibility?

Current answer:

- not yet quantified
- but it definitely changes environment state progression

### 2. How much variance in current evaluation reports is caused by terrain seed alone versus continuing RNG state?

Current answer:

- not yet measured

### 3. Are there any downstream dashboards or analysis notebooks that already assume `num_dynamic_obstacles` is active?

Current answer:

- not checked in this review

## Recommended Next Validation

1. Remove the extra training-environment reset after subset evaluation and compare episode cadence.
2. Rerun `eval_suite` with explicit `reset(seed=seed_value)` and compare per-seed reproducibility.
3. Split curriculum logging into:

```text
episode_stage_before_promotion
episode_stage_after_promotion
```

4. Either re-enable dynamic obstacles or remove the parameter from experiment surfaces until it is real.
5. Add `sim_crash_rate` to evaluation summaries.

## Candidate Acceptance Criteria

This issue should be considered fixed only when all of the following are true:

1. Training callbacks do not mutate experiment state with unintended extra resets.
2. Evaluation seeds fully reseed all stochastic environment components.
3. Terminal curriculum metrics are attributed to the stage actually played.
4. Exposed config dimensions correspond to real runtime behavior.
5. Evaluation summaries surface simulator crash rates explicitly.
