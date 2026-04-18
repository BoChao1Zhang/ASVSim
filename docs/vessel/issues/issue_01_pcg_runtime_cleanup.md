# Issue 01: Runtime PCG Cleanup And Obstacle Ownership

## Status Update 2026-04-18

Current state after source changes in this repo:

- `generationManager` Blueprint `generateTerrain(...)` was verified in the running editor and **does call `Clear()` first**
- Blueprint `Clear()` was verified in the running editor and **does destroy `Generated` actors and clear `road/left/right/temp`**
- the original "no cleanup exists anywhere" hypothesis is therefore **too strong**
- the remaining real runtime defect was the **ownership mismatch**: Unreal always spawned native PCG obstacles even when Python config selected Python-owned obstacle spawning
- an additional validation-time root cause was found: Unreal runtime RPC still used the old `7`-argument `generatePortTerrain(...)` binding until `build.cmd --Release` rebuilt and mirrored `AirLib.lib`
- this happened because `AirSim.Build.cs` enables `AIRLIB_HEADER_ONLY=1`, so changing `AirLib/src/api/RpcLibServerBase.cpp` alone is not enough for PIE; the linked static library under `Unreal/Plugins/AirSim/Source/AirLib/lib/x64/Release/AirLib.lib` must also be rebuilt

Implemented in code:

1. Added native reflection cleanup in `AGenerationManager::CleanupGeneratedActorsNative(...)` plus `GetGeneratedActorCountNative(...)`.
2. Added runtime logging in `ASimModeBase::generatePortTerrain(...)` for:
   - `generated_before`
   - `destroyed`
   - `generated_after`
   - `generated_after_spawn`
3. Added an explicit RPC/API obstacle-ownership flag `spawn_native_obstacles`.
4. Forwarded RL ownership from Python via `use_c_side_pcg_obstacles`.
5. Added a native automation regression test for cleanup behavior.

Verification status:

- Verified from running editor:
  - Blueprint `generateTerrain(...)` begins with `Clear()`
  - Blueprint `Clear()` compiles cleanly and performs array/actor cleanup
- Verified locally:
  - modified Python files pass `py_compile`
  - `build.cmd --Release` succeeded and rebuilt `AirLib.lib`
  - rebuilt `AirLib.lib` was mirrored into `Unreal/Plugins/AirSim/Source/AirLib/lib/x64/Release/AirLib.lib`
  - clean `BlocksEditor` rebuild succeeded
  - in a fresh PIE session, the new runtime RPC ABI is active:
    - `generatePortTerrain(..., False)` with `8` explicit arguments returns through the server normally
    - a raw direct `7`-argument RPC call now fails with an arity error, confirming the stale runtime binding is gone
    - Python wrapper call sites that omit the optional ownership argument still work because `client.py` now supplies the default eighth argument internally
  - after `activateGeneration(False)`, both ownership modes returned success in PIE:
    - `generatePortTerrain(..., False) == True`
    - `generatePortTerrain(..., True) == True`
  - PIE log evidence confirms the intended cleanup / ownership behavior:
    - `generatePortTerrain cleanup: native_owner=python generated_before=0 destroyed=0 generated_after=0`
    - `generatePortTerrain skipped native obstacle spawning because caller selected Python obstacle ownership. generated_after_spawn=100`
    - `generatePortTerrain cleanup: native_owner=unreal generated_before=100 destroyed=100 generated_after=0`
    - `ApplyAdvancedPCGNative returned spawned=6 generated_after_spawn=106`

Interpretation:

- stale native obstacle accumulation is now defended in **both** places:
  - Blueprint `Clear()`
  - new native reflection cleanup before regeneration
- obstacle ownership is now explicit per call instead of implicit and conflicting
- successful runtime verification now depends on **both** build steps:
  - `build.cmd --Release` for the AirLib static library used by the Unreal plugin
  - `Build.bat BlocksEditor ...` for the editor module / DLLs

## Historical Investigation Record

The sections below preserve the pre-fix investigation notes that motivated the code changes above.

They are retained as historical context and should **not** be read as the current post-fix state unless a subsection explicitly says so.

## Summary

At the start of this investigation, the runtime terrain regeneration path had no verified native cleanup step before calling `generateTerrain(...)` and then `ApplyAdvancedPCGNative(...)`.

This creates a high-risk failure mode:

- old native-generated obstacle actors may survive across repeated `generatePortTerrain(...)` calls
- world state may no longer be episode-independent
- Python and Unreal may both believe they own obstacle spawning

This issue is about Unreal runtime scene ownership, not Python reward shaping.

## Scope

- Unreal runtime terrain regeneration
- native obstacle spawning
- ownership split between Unreal native PCG and Python `simAddObstacle(...)`
- RL episode isolation

## Pre-Fix Findings

### 1. `AGenerationManager::Clear()` is empty

File:

```text
Unreal/Plugins/AirSim/Source/PCG/GenerationManager.cpp
```

Relevant lines:

```cpp
void AGenerationManager::Clear()
{
}
```

Current location:

```text
GenerationManager.cpp:496-498
```

Consequence:

- there is no native cleanup implementation behind the public `Clear()` API today

### 2. Runtime `generatePortTerrain(...)` does not perform a native cleanup step

File:

```text
Unreal/Plugins/AirSim/Source/SimMode/SimModeBase.cpp
```

Current runtime sequence:

1. find the existing `generationManager` actor
2. call `SetSeed`
3. call Blueprint `generateTerrain`
4. validate `road/left/right`
5. call `AGenerationManager::ApplyAdvancedPCGNative(...)`

Relevant lines:

```text
SimModeBase.cpp:2306-2390
```

There is no C++ cleanup call between steps 1 and 5.

### 3. The runtime path reuses a persistent `generationManager` actor from the world

File:

```text
Unreal/Plugins/AirSim/Source/SimMode/SimModeBase.cpp
```

`FindGenerationManagerActor(UWorld* World)` searches the current world with `TActorIterator`, then falls back to the BP class path.

Relevant lines:

```text
SimModeBase.cpp:73-100
```

Consequence:

- runtime terrain generation is not operating on a fresh transient manager each episode
- state can carry over unless explicitly cleared

### 4. Native obstacle spawning appends to `Generated`

File:

```text
Unreal/Plugins/AirSim/Source/PCG/GenerationManager.cpp
```

`SpawnAdvancedObstaclesNative(...)` resolves:

- `road` with `FindTransformArrayProperty(...)`
- `left` with `FindTransformArrayProperty(...)`
- `right` with `FindTransformArrayProperty(...)`
- `Generated` with `FindActorArrayProperty(...)`

and then appends newly spawned actors:

```cpp
if (GeneratedActors) GeneratedActors->Add(Spawned);
```

Relevant lines:

```text
GenerationManager.cpp:1504-1510
GenerationManager.cpp:1600-1602
```

Consequence:

- native obstacle creation has a tracked actor container
- current code shown above does not remove prior entries before adding new ones

### 5. `road/left/right` are path transforms, not actor arrays

This matters because any cleanup design must not treat them like spawned actor containers.

Evidence:

- `FindTransformArrayProperty(...)` is used for `road/left/right`
- `FindActorArrayProperty(...)` is used for `Generated`
- the automation tests populate `road/left/right` with `FTransform`

Relevant lines:

```text
GenerationManager.cpp:20-35
GenerationManager.cpp:1504-1509
PCGNativeAutomationTests.cpp:65-81
```

Consequence:

- cleanup logic for spawned runtime actors should focus on `Generated` or other actual actor arrays
- `road/left/right` need reset semantics only if the BP graph fails to overwrite them, but they are not destroyable actor lists

### 6. The automation tests contain manual cleanup that does not exist in runtime C++

File:

```text
Unreal/Plugins/AirSim/Source/Tests/PCGNativeAutomationTests.cpp
```

The helper `CleanupSpawnedActors(...)` does:

1. iterate `Actor->Generated`
2. `Destroy()` each valid spawned actor
3. `Actor->Generated.Reset()`

Relevant lines:

```text
PCGNativeAutomationTests.cpp:140-156
```

Consequence:

- the repository already contains one explicit cleanup pattern for native PCG actors
- that cleanup currently lives in tests, not in the runtime path

### 7. Python initially assumed native cleanup was handled elsewhere

File:

```text
PythonClient/reinforcement_learning/airgym/envs/vessel_env.py
```

Current comments say:

- when `use_c_side_pcg_obstacles=True`, native spawn has already populated `Generated`
- Python cleanup stays off because BP `Clear()` is assumed to own destruction

Relevant lines:

```text
vessel_env.py:63-69
vessel_env.py:354-358
vessel_env.py:430-432
```

Consequence at investigation start:

- the Python side encoded a cleanup assumption that was not backed by any visible native implementation

### 8. At investigation start, default config still set `use_c_side_pcg_obstacles: false`

File:

```text
PythonClient/reinforcement_learning/configs/base.yaml
```

Relevant line:

```text
base.yaml:17
```

Consequence at investigation start:

- default RL config asked Python to own obstacle spawning
- Unreal runtime still unconditionally called `ApplyAdvancedPCGNative(...)`
- this was a genuine dual-ownership design mismatch before the ownership-flag patch

### 9. Before this fix, the RPC path forwarded `generatePortTerrain(...)` without an obstacle-ownership flag

Files:

```text
AirLib/include/api/WorldSimApiBase.hpp
AirLib/include/api/RpcLibClientBase.hpp
AirLib/src/api/RpcLibServerBase.cpp
AirLib/src/api/RpcLibClientBase.cpp
Unreal/Plugins/AirSim/Source/WorldSimApi.h
Unreal/Plugins/AirSim/Source/WorldSimApi.cpp
Unreal/Plugins/AirSim/Source/SimMode/SimModeBase.h
```

Relevant lines:

```text
WorldSimApiBase.hpp:73
RpcLibClientBase.hpp:167
RpcLibServerBase.cpp:254-255
RpcLibClientBase.cpp:477-479
WorldSimApi.h:72
WorldSimApi.cpp:324-331
SimModeBase.h:180
```

Consequence at investigation start:

- there was no runtime API-level switch telling Unreal whether native obstacle spawning should run for a given terrain generation call

## Historical Supporting Log Evidence

Local logs show repeated terrain generation and repeated native spawn calls for different seeds, for example:

```text
Saved/Logs/Blocks.log:4938155  GenerationManager seed updated to 43
Saved/Logs/Blocks.log:4938236  ApplyAdvancedPCGNative returned spawned=5
Saved/Logs/Blocks.log:4961451  GenerationManager seed updated to 1000043
Saved/Logs/Blocks.log:4961588  ApplyAdvancedPCGNative returned spawned=6
Saved/Logs/Blocks.log:4987608  GenerationManager seed updated to 2000043
Saved/Logs/Blocks.log:4987802  ApplyAdvancedPCGNative returned spawned=7
```

What this proves:

- runtime terrain regeneration is happening repeatedly
- native obstacle spawning is also happening repeatedly

What this does not prove yet:

- it does not by itself prove actor accumulation across calls
- for that we still need `Generated` count instrumentation or editor-side inspection

## Historical Inference

The strongest pre-fix inference was:

- because runtime uses a persistent `generationManager`
- because native spawn appends to `Generated`
- because no runtime C++ cleanup step is visible before regeneration
- because Python cannot own cleanup when Unreal still spawns natively

the system had a high probability of leaving stale native obstacle actors alive across terrain regenerations.

Confidence:

- Medium-High for the design flaw
- Medium for the exact leak mechanism until `Generated` pre/post counts are instrumented

## Historical Open Questions

### 1. Does the Blueprint `generationManager.uasset` already destroy old generated actors inside `generateTerrain(...)`?

Current answer:

- not verified from source code in this review

Why it matters:

- if BP already destroys `Generated` before native spawn, the cleanup gap is narrower than the C++ source alone suggests

### 2. Are there additional actor arrays besides `Generated/generated` that hold spawned runtime obstacles?

Current answer:

- not established from current code review

### 3. Are path transforms (`road/left/right`) fully overwritten every regeneration?

Current answer:

- likely yes for normal generation
- not yet formally instrumented

## Original Next Validation Plan

1. Add runtime logging before and after native spawn:

```text
Generated.Num before cleanup
destroyed actor count
Generated.Num after spawn
```

2. Inspect or diff `generationManager.uasset` to verify whether BP already clears old actors.

3. Gate native spawn with an explicit runtime ownership flag so Unreal and Python cannot both own obstacle spawning in the same episode.

## Candidate Acceptance Criteria

The 2026-04-18 status update above records the implementation and runtime evidence that closed this issue against these criteria.

This issue should be considered fixed only when all of the following are true:

1. Repeated `generatePortTerrain(...)` calls are idempotent with respect to spawned obstacle ownership.
2. There is exactly one obstacle owner per episode.
3. Runtime logs expose actor counts before cleanup and after spawn.
4. `Generated` count stays bounded across repeated regenerations.
5. RL resets no longer inherit stale native obstacle actors from previous episodes.
