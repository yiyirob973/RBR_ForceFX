@echo off
rem Builds RBR_ForceFX.dll into out\RBR_ForceFX.dll. Run it from an "x86 Native Tools Command Prompt for VS" (32-bit).
rem The camera uses MinHook, which is built for the dynamic C runtime, so the DLL is built with /MD and needs the
rem Microsoft Visual C++ 2015-2022 x86 Redistributable.
rem The version shown in the game is taken from the FORCEFX_VERSION environment variable (the release workflow sets it to the
rem git tag, for example v0.7.0). Without it the build calls itself "Dev build" and shows the date it was compiled.
setlocal
cd /d "%~dp0"
set "DEFS="
if defined FORCEFX_VERSION set "DEFS=/DFORCEFX_VERSION=\"%FORCEFX_VERSION%\""
if not exist out mkdir out
cl /std:c++17 /O2 /MD /EHsc /LD %DEFS% /Fo:out\ RBR_ForceFX.cpp thirdparty\lib\libminhook.x86.lib /link /DEF:RBR_ForceFX.def dinput8.lib dxguid.lib user32.lib ws2_32.lib winmm.lib /OUT:out\RBR_ForceFX.dll
if errorlevel 1 (
    echo Build failed.
    exit /b 1
)
echo.
echo Built out\RBR_ForceFX.dll
