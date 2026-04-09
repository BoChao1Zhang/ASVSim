# AGENTS.md

## Language And Output

- Tool usage and internal reasoning must stay in English.
- User-facing responses must be in Chinese.
- Use standard Markdown.

## Evidence And Search

- Distinguish local code facts from external facts.
- When external verification is needed, prefer `grok_search`.
- For Unreal Engine and AirSim behavior, prefer official documentation.
- Key factual claims should be supported by at least two independent sources when possible.
- If only one official source exists, state that limitation explicitly.

## Current Main Problem

- The current primary issue is **not** `GenerationManager`.
- The active crash chain is: `FlyingExampleMap` vessel enters runtime -> vessel physics path explodes -> Unreal asserts with `!bPostTickComponentUpdate`.
- Local crash evidence already shows samples where `B-WRN`, `N-VEL`, and `N-POS` blow up before the Unreal assertion.
- `GenerationManager` may still have separate issues, but it is not the main root-cause target for this crash investigation.

## Windows Build Environment

- Preferred Unreal Engine root on this machine: `E:\ProgramFile\UE_5.7`
- Preferred Visual Studio installation on this machine: `C:\Program Files\Microsoft Visual Studio\18\Community`
- Preferred developer environment entrypoint: `C:\Program Files\Microsoft Visual Studio\18\Community\Common7\Tools\VsDevCmd.bat`
- Preferred MSBuild path: `C:\Program Files\Microsoft Visual Studio\18\Community\MSBuild\Current\Bin\amd64\MSBuild.exe`

## Build Rules

- Do **not** treat `UnrealPluginFiles.vcxproj` as a reliable standalone Unreal build entrypoint for validation.
- Prefer the repository `build.cmd` script for Windows builds.
- Run `build.cmd` from a local MSVC developer shell.
- For UE 5.7 in this repository, prefer MSVC toolset `14.44` when using VS 2026/18.x.
- Recommended shell bootstrap:

```bat
call "C:\Program Files\Microsoft Visual Studio\18\Community\Common7\Tools\VsDevCmd.bat" -arch=x64 -host_arch=x64 -vcvars_ver=14.44
```

- Recommended project build entrypoint:

```bat
build.cmd --Release
```

- Alternative build modes accepted by `build.cmd`:

```bat
build.cmd --Debug
build.cmd --RelWithDebInfo
```

## Unreal Project Paths

- Main Windows Unreal project currently in use: `E:\code\ASVSim\Unreal\Environments\PortEnv\Blocks.uproject`
- `FlyingExampleMap` is the relevant runtime map for the current vessel crash chain.

## Repair Workflow

- Before changing root-cause direction, check local crash logs and current code paths.
- When fixing runtime crashes, prefer the smallest defensible patch first.
- After patching, validate with the local MSVC toolchain and the repo build entrypoint before expanding scope.
- Do not revert unrelated user changes in the worktree.
