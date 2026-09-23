@echo off
setlocal
rem Run from an x64 Native Tools Command Prompt for Visual Studio 2022.
rem OPENXR_INCLUDE_DIR must contain openxr\openxr.h.
if not defined OPENXR_INCLUDE_DIR (
    echo ERROR: Set OPENXR_INCLUDE_DIR to the OpenXR SDK include directory.
    exit /b 1
)
if not exist "%OPENXR_INCLUDE_DIR%\openxr\openxr.h" (
    echo ERROR: OpenXR headers not found under OPENXR_INCLUDE_DIR.
    exit /b 1
)
where cl >nul 2>nul
if errorlevel 1 (
    echo ERROR: Run this in an x64 Native Tools Command Prompt for VS 2022.
    exit /b 1
)
pushd "%~dp0dg_hook"
if errorlevel 1 exit /b 1
cl /nologo /W3 /O2 /Gy /EHsc /DDG_ENABLE_DIAGNOSTICS=0 /I "%OPENXR_INCLUDE_DIR%" /LD dg_hook.c dg_radial.c dg_radial_owner.c dg_radial_view.c dg_radial_overlay.c dg_aim_target.c dg_aim_probe.c dg_aim_capture.c dg_script_gate.c dg_xr.c dg_xr_script.c dg_present.c dg_draw_trial.cpp dg_proj.c dg_bridge.c dg_radial_inventory.c dg_radial_native_read.c dg_radial_phase.c dg_radial_commit.c dg_radial_ready.c dg_pose.c dg_ik.c dg_arm_map.c dg_fire.c dg_recoil.c dg_move.c dg_menu.c dg_rec.c dg_policy.c dg_policy_table.c ^
    /link /OPT:REF /OUT:dg_hook.asi d3d11.lib dxgi.lib dxguid.lib user32.lib
set "pcvr_build_rc=%ERRORLEVEL%"
popd
exit /b %pcvr_build_rc%
