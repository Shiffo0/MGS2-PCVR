# Building the mod

Version v0.3.3 contains the corresponding mod source for build 62E9EEAF. It includes DXGI 1.1 factory initialization for SteamVR, synchronized swapchain resizing, unlimited sessions and bounded support logging. Run test_release.bat, test_xr_resize.bat and test_factory.bat with the compiler/SDK setup below.

## Requirements

- Windows x64 and Visual Studio 2022 C++ Build Tools with the Windows SDK.
- [OpenXR SDK headers](https://github.com/KhronosGroup/OpenXR-SDK), baseline version 1.1.59.

Open an **x64 Native Tools Command Prompt for VS 2022** and set `OPENXR_INCLUDE_DIR` to the directory containing `openxr/openxr.h`.

Run `src\build.bat` for the player release. Output: `src/dg_hook/dg_hook.asi`.

Run `src\build.bat diagnostic` only for development measurements. Output: `src/dg_hook/dg_hook-diagnostic.asi`. This is not the playable release asset. Build profiles serially; they share intermediate object files.

`DG_ENABLE_DIAGNOSTICS` defaults to **0** in `dg_build_profile.h`. Set it to **1** consistently for every translation unit only when building development tools. Old marker files and environment flags cannot enable the excluded measurements in the player release.

Run `src\test_release.bat` from the same developer prompt to test legacy-setting rejection and inert measurement entry points, plus real D3D11 WARP hook behavior. No game, headset or OpenXR runtime is needed. These tests do not establish physical-headset acceptance.

## Contents and behavior

`dg_hook/` contains the mod and focused release tests; `shared/` contains integration helpers. SDKs and game files are not included. Normal frame transfer to OpenXR, input, AER, HUD and wrist radar remain part of the release profile. Diagnostic capture is distinct from that essential frame transfer.

Flight recording, GPU trials/readback probes, phase/pair traces, HUD watches, optional captures and debug visuals require the diagnostic profile. Player logs keep a bounded subset of startup/error messages (maximum 256 lines per process). No FPS improvement is claimed without measurement.

This directory contains the mod source and relative development-only diagnostic paths. A local rebuild is a separate artifact; byte identity with the packaged build is not guaranteed.
