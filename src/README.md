# Mod source and build instructions

Corresponding release mod source for **v0.4.0**, build **AEF2EC93**. This directory contains the mod's shipping dependency closure and build script.

## Build

Use Windows x64, Visual Studio 2022 C++ Build Tools with the Windows SDK, and [OpenXR SDK headers](https://github.com/KhronosGroup/OpenXR-SDK), baseline 1.1.59.

Open an x64 Native Tools Command Prompt. Set `OPENXR_INCLUDE_DIR` to your SDK directory containing `openxr/openxr.h`, then run `src\build.bat`. Output: `src/dg_hook/dg_hook.asi`.

The build selects `DG_ENABLE_DIAGNOSTICS=0`. Diagnostic branches, development measurements and unused measurement helpers have been removed. This is a player-only source snapshot; diagnostic builds are not supported. Ordinary error handling, bounded support logging, frame synchronization and OpenXR frame transfer remain. Some inert compatibility entry points retain their original names.

The mod builds without additional private include directories. SDK headers are external dependencies with their own licenses. Compiler/linker versions and timestamps may change a local rebuild's hash; byte-identical reproduction is not claimed.

The MIT license applies to the mod contributors' own code; third-party dependencies retain their respective licenses.
