# Building the mod

This directory contains the mod implementation and its local include dependencies for the Public Beta 1 baseline, build `12683251`.

## Requirements

- Windows x64.
- Visual Studio 2022 C++ Build Tools with the Windows SDK.
- [OpenXR SDK headers](https://github.com/KhronosGroup/OpenXR-SDK); the baseline uses version 1.1.59. Dependencies are obtained separately.

Open an **x64 Native Tools Command Prompt for VS 2022**, set `OPENXR_INCLUDE_DIR` to the directory containing `openxr/openxr.h`, then run `src\build.bat` from the repository root.

The output is `src/dg_hook/dg_hook.asi`. Building does not install or start anything. For playing the tested beta, use the packaged release and follow the main README.

## Layout

- `dg_hook/`: OpenXR integration, controller input, game integration, stereo, tracking, UI and diagnostic helpers. Some files contain conditional development checks.
- `shared/`: shared executable identification and integration helpers.

This is the buildable mod baseline, not the full development workspace or a complete standalone test-tool distribution. Game files and SDK dependencies are not included.

## Relationship to Public Beta 1

Documentation comments have been curated for this repository. Three optional diagnostic locations use relative paths under `logs/`: `pcvr_hud_watch.txt`, `pcvr_near/` and `pcvr_phase/`. The directories for optional captures must exist before using those diagnostics.

The gameplay code tokens are unchanged from the selected baseline; the three diagnostic path literals are the only executable-token edits. A build from this directory is a separate artifact and is not claimed to be byte-identical to, or headset-tested as, release binary `12683251`.
