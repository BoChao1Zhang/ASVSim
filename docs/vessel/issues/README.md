# Vessel RL Issues

This directory tracks two independent issue threads found during the RL pipeline review.

## Issue List

1. `issue_01_pcg_runtime_cleanup.md`
   Runtime `generatePortTerrain(...)` may leave native-generated actors alive across terrain regenerations, which can break episode isolation.

2. `issue_02_rl_pipeline_semantics.md`
   The Python RL training and evaluation pipeline contains control-flow and measurement bugs that can distort statistics even when the simulator world state is correct.

## Relationship Between The Two Issues

- Issue 01 is about Unreal runtime world-state ownership and cleanup.
- Issue 02 is about Python-side RL control flow, seeding, curriculum labeling, and metric semantics.
- They are related because both affect RL stability and evaluation credibility.
- They are not the same bug.
- Fixing only one of them is not sufficient to make the RL results trustworthy.

## Evidence Policy

- Each issue document separates `Verified Facts`, `Inferences`, and `Open Questions`.
- `Verified Facts` are tied to current source files and local logs.
- `Inferences` are conclusions that follow from the verified facts but still need runtime instrumentation or editor-side inspection.
- `Open Questions` identify what still needs to be validated before claiming a full root cause.
