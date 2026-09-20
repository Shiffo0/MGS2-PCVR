@echo off
setlocal
pushd "%~dp0dg_hook"
cl /nologo /W3 /O2 /Gy /I "%OPENXR_INCLUDE_DIR%" xr_resize_test.c dg_radial_view.c dg_radial_overlay.c /Fe:xr_resize_test.exe /link /OPT:REF d3d11.lib dxgi.lib dxguid.lib user32.lib
if errorlevel 1 goto failed
xr_resize_test.exe
set "pcvr_rc=%ERRORLEVEL%"
popd
exit /b %pcvr_rc%
:failed
popd
exit /b 1
