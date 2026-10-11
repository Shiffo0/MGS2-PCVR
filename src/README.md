# MGS2 PCVR v0.5.0 — mod source

This folder contains the source of the mod itself. It contains no Metal Gear Solid 2 source code and no game assets.

## Building

Requirements: Windows x64, Visual Studio 2022 C++ Build Tools with the Windows SDK, and the OpenXR SDK headers (1.1.59 was used for this release).

1. Open an **x64 Native Tools Command Prompt for VS 2022**.
2. Set `OPENXR_INCLUDE_DIR` to the directory that contains `openxr\openxr.h`.
3. Run `src\build.bat`.

The output is `src\dg_hook\dg_hook.asi`. The build sets `DG_ENABLE_DIAGNOSTICS=0`, so development diagnostics are compiled out. Two Eye rendering does not depend on them.

`dg_hook\dg_cb_retarget_cs.h` is a precompiled compute shader; its source is `dg_hook\dg_cb_retarget.hlsl`. The colour correction in `dg_hook\dg_hdfix_colour.inl` adapts MGSHDFix's colour shader; see `THIRD_PARTY_NOTICES.md`.

Compiler and linker timestamps differ between builds, so a rebuilt binary is not byte-identical to the released one.
