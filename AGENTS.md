# AGENTS.md

## Purpose

This file records the local development environment and the standard build / debug workflow for `E:\code\ASVSim`, so future work does not need to rediscover:

- Visual Studio location
- MSVC toolset version
- Unreal Engine location
- correct build entrypoints
- Editor startup and validation sequence

## Language And Output

- Tool usage and internal reasoning must stay in English.
- User-facing responses must be in Chinese.
- Use standard Markdown.

## Subagent And UE Skill Policy

- When spawning implementation subagents for this repository, use model `gpt-5.4` with reasoning effort `high` unless the user explicitly requests otherwise.
- When spawning review subagents for this repository, use model `gpt-5.4` with reasoning effort `xhigh` unless the user explicitly requests otherwise.
- For Unreal Engine development work, always load the `ue-project-dev` skill before code changes, Blueprint edits, editor-side validation, or Unreal API research.
- For Unreal Editor work, prefer `UE Editor MCP` for editor connectivity, asset inspection, Blueprint or widget changes, compile checks, and editor log validation.

## Evidence And Search

- Distinguish local code facts from external facts.
- When external verification is needed, prefer `grok_search`.
- For Unreal Engine and AirSim behavior, prefer official documentation.
- Key factual claims should be supported by at least two independent sources when possible.
- If only one official source exists, state that limitation explicitly.

## Repository Root

- Repository root: `E:\code\ASVSim`
- Default shell on this machine: `powershell`

## Fixed Local Toolchain Paths

- Unreal Engine root: `E:\ProgramFile\UE_5.7`
- Unreal Editor executable: `E:\ProgramFile\UE_5.7\Engine\Binaries\Win64\UnrealEditor.exe`
- Unreal Build script: `E:\ProgramFile\UE_5.7\Engine\Build\BatchFiles\Build.bat`
- Visual Studio root: `C:\Program Files\Microsoft Visual Studio\18\Community`
- VS developer shell bootstrap: `C:\Program Files\Microsoft Visual Studio\18\Community\Common7\Tools\VsDevCmd.bat`
- MSBuild path: `C:\Program Files\Microsoft Visual Studio\18\Community\MSBuild\Current\Bin\amd64\MSBuild.exe`

## Required MSVC Setup

- This repository is currently validated against UE 5.7 + MSVC `14.44`.
- Always prefer the local VS 2026 developer shell with `-vcvars_ver=14.44`.
- Recommended bootstrap command:

```bat
call "C:\Program Files\Microsoft Visual Studio\18\Community\Common7\Tools\VsDevCmd.bat" -arch=x64 -host_arch=x64 -vcvars_ver=14.44
```

- Quick verification after bootstrap:

```bat
echo %VisualStudioVersion%
echo %VCToolsVersion%
```

- Expected values on this machine:
  - `VisualStudioVersion=18.0`
  - `VCToolsVersion=14.44.35207` or another `14.44.xxxxx`

## Primary Unreal Project

- Active Unreal project: `E:\code\ASVSim\Unreal\Environments\PortEnv\Blocks.uproject`
- Default runtime / RL map for PortEnv vessel work: `GenerationTopDownTest`

## Standard Build Entry Points

### 1. AirLib / repo-level build

- Preferred entrypoint:

```bat
build.cmd --Release
```

- Other accepted modes:

```bat
build.cmd --Debug
build.cmd --RelWithDebInfo
```

- Notes:
  - `build.cmd` must be run from a proper MSVC developer shell.
  - `build.cmd` will bootstrap third-party dependencies such as `rpclib`.
  - Do not assume plain `cmd.exe` without `VsDevCmd.bat` is enough.

### 2. Unreal plugin / Editor target build

- Preferred validation build:

```bat
"E:\ProgramFile\UE_5.7\Engine\Build\BatchFiles\Build.bat" BlocksEditor Win64 Development "E:\code\ASVSim\Unreal\Environments\PortEnv\Blocks.uproject" -waitmutex
```

- Clean rebuild when headers or plugin integration look stale:

```bat
"E:\ProgramFile\UE_5.7\Engine\Build\BatchFiles\Build.bat" BlocksEditor Win64 Development "E:\code\ASVSim\Unreal\Environments\PortEnv\Blocks.uproject" -clean -waitmutex
"E:\ProgramFile\UE_5.7\Engine\Build\BatchFiles\Build.bat" BlocksEditor Win64 Development "E:\code\ASVSim\Unreal\Environments\PortEnv\Blocks.uproject" -waitmutex
```

## Build Rules

- Do **not** treat `UnrealPluginFiles.vcxproj` as a reliable standalone Unreal validation entrypoint.
- Prefer `build.cmd` for repo-level native code validation.
- Prefer `Build.bat BlocksEditor ...` for Unreal plugin / runtime validation.
- If `Build.bat -clean` fails with access denied, check whether `UnrealEditor.exe` is still running and holding DLLs.
- If Unreal says target is `up to date` after header-only changes and the result looks suspicious, do a clean rebuild.

## Editor Startup

- Preferred startup command:

```bat
"E:\ProgramFile\UE_5.7\Engine\Binaries\Win64\UnrealEditor.exe" "E:\code\ASVSim\Unreal\Environments\PortEnv\Blocks.uproject"
```

- For routine RL runs, do not pass an explicit map override on the command line.
- PortEnv already defaults to `GenerationTopDownTest` via `DefaultEngine.ini` and `DefaultEditor.ini`.
- If using MCP-based UE tools, wait until:
  - editor process is alive
  - editor world is ready
  - `ue_ping` returns success

## Validation Workflow

### For normal code changes

1. Bootstrap MSVC 14.44 developer shell.
2. Run `build.cmd --Release`.
3. Run Unreal `Build.bat BlocksEditor Win64 Development ...`.
4. Start Editor on `Blocks.uproject`.
5. Validate in the project default PortEnv map `GenerationTopDownTest` if the change affects runtime.

### For vessel / runtime crash fixes

1. Check local crash stack first.
2. Verify whether the crash reproduces in PIE without unrelated systems such as `GenerationManager`.
3. Apply the smallest defensible patch first.
4. Rebuild repo-level code.
5. Rebuild `BlocksEditor`.
6. Re-run PIE and inspect logs.

## Current Vessel Investigation Context

- The current primary issue is **not** `GenerationManager`.
- The main crash chain investigated recently was:

```markdown
GenerationTopDownTest vessel enters runtime
-> vessel physics / sensor update path runs
-> debug draw was executed off the game thread
-> Unreal asserted with `!bPostTickComponentUpdate`
```

- The confirmed upstream vessel risks that were already fixed in this repo include:
  - `Kinematics::initialize()` not syncing `current_`
  - `VesselEngine` using quaternion `z()` as yaw
  - vessel API control state reset mismatch
  - hydrodynamics wrench state shadowing / uninitialized state
  - Unreal-side debug draw from async physics/sensor threads

## Critical Vessel Startup Finding

- The bad PIE startup behavior matches the old runtime behavior of `HydroDynamicsWrench` and `FossenCurrent`: on the first tick, stale output could be written into the body `wrench` before the current-frame hydrodynamics force was computed, and there was no abnormal velocity / abnormal force clamping on that path.
- The current source tree already contains the protections for that issue, but Unreal runtime behavior depends on the mirrored AirLib headers under `Unreal\Plugins\AirSim\Source\AirLib\include\...`, not only the top-level `AirLib\include\...` copy. If those mirrored headers are stale, the editor can keep running the old first-tick behavior even though the top-level AirLib headers look fixed.
- For this reason, any vessel startup / hydrodynamics header fix must be followed by a **clean** `BlocksEditor` rebuild before trusting PIE validation. A normal incremental build is not sufficient when the result looks suspicious.
- Additional hardening now required for vessel startup: after `VesselPawnSimApi::initialize()`, call `reset()` immediately during vehicle creation so the vessel enters the physics world from a deterministic zero state instead of relying only on later world-level reset timing.

## Typical Commands

### Full native + Unreal validation

```bat
call "C:\Program Files\Microsoft Visual Studio\18\Community\Common7\Tools\VsDevCmd.bat" -arch=x64 -host_arch=x64 -vcvars_ver=14.44
cd /d E:\code\ASVSim
build.cmd --Release
"E:\ProgramFile\UE_5.7\Engine\Build\BatchFiles\Build.bat" BlocksEditor Win64 Development "E:\code\ASVSim\Unreal\Environments\PortEnv\Blocks.uproject" -waitmutex
```

### Start Editor

```bat
"E:\ProgramFile\UE_5.7\Engine\Binaries\Win64\UnrealEditor.exe" "E:\code\ASVSim\Unreal\Environments\PortEnv\Blocks.uproject"
```

### Clean Unreal rebuild

```bat
call "C:\Program Files\Microsoft Visual Studio\18\Community\Common7\Tools\VsDevCmd.bat" -arch=x64 -host_arch=x64 -vcvars_ver=14.44
cd /d E:\code\ASVSim
"E:\ProgramFile\UE_5.7\Engine\Build\BatchFiles\Build.bat" BlocksEditor Win64 Development "E:\code\ASVSim\Unreal\Environments\PortEnv\Blocks.uproject" -clean -waitmutex
"E:\ProgramFile\UE_5.7\Engine\Build\BatchFiles\Build.bat" BlocksEditor Win64 Development "E:\code\ASVSim\Unreal\Environments\PortEnv\Blocks.uproject" -waitmutex
```

## Operational Notes

- `build.cmd` may copy / mirror dependency outputs and project files; this is expected.
- Header-only changes in `AirLib/include` can require a full Unreal clean rebuild before the Editor actually picks them up.
- When working on runtime crashes, keep the Editor closed during clean rebuilds.
- Do not revert unrelated user changes in the worktree.

## Suggested Development Discipline

- Inspect first, patch second.
- Prefer minimal root-cause fixes over broad refactors during crash work.
- Validate with both compile-time and runtime checks.
- If a fix depends on engine-thread semantics, verify it against official UE docs with `grok_search`.
