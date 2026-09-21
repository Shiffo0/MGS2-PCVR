@echo off
setlocal
where cl >nul 2>nul
if errorlevel 1 exit /b 1
pushd "%~dp0dg_hook"
cl /nologo /W3 /O2 /Gy /EHsc /DDG_ENABLE_DIAGNOSTICS=0 factory_test.c dg_draw_trial.cpp /link /OPT:REF d3d11.lib dxgi.lib dxguid.lib user32.lib
if errorlevel 1 goto :failed
factory_test.exe
set "factory_rc=%errorlevel%"
popd
exit /b %factory_rc%
:failed
popd
exit /b 1
