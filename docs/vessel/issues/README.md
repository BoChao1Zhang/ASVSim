# Vessel RL Issues

This directory tracks three independent issue threads found during the RL pipeline review.

## Issue List

1. `issue_01_pcg_runtime_cleanup.md`
   Runtime `generatePortTerrain(...)` may leave native-generated actors alive across terrain regenerations, which can break episode isolation.

2. `issue_02_rl_pipeline_semantics.md`
   The Python RL training and evaluation pipeline contains control-flow and measurement bugs that can distort statistics even when the simulator world state is correct.

3. `issue_03_environment_pass_rate_evidence.md`
   Evidence-only report for the current waypoint-following Environment task: confirmed code facts, probe observations, derived dynamics implications, and external literature conclusions, with no implementation advice.

## Relationship Between The Issues

- Issue 01 is about Unreal runtime world-state ownership and cleanup.
- Issue 02 is about Python-side RL control flow, seeding, curriculum labeling, and metric semantics.
- Issue 03 is about the current Environment task's pass-rate evidence: task semantics, timeout-heavy behavior, control/reward structure, and external literature context.
- They are related because all three affect how RL performance should be interpreted.
- They are not the same bug.
- Fixing only one thread is not sufficient to make RL results trustworthy.

## Evidence Policy

- Each issue document separates `Verified Facts`, `Inferences`, and `Open Questions`, or explicitly labels evidence classes.
- `Verified Facts` are tied to current source files and local logs.
- `Inferences` are conclusions that follow from the verified facts but still need runtime instrumentation or editor-side inspection.
- `Open Questions` identify what still needs to be validated before claiming a full root cause.
