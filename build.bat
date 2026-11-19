@echo off
rem Builds the four ForceFX variants. Run it from an "x86 Native Tools Command Prompt for VS" (32-bit).
rem   out\full\RBR_ForceFX.dll     dynamic audio + camera
rem   out\audio\RBR_ForceFX.dll    dynamic audio only
rem   out\camera\RBR_ForceFX.dll   camera only
rem   out\basic\RBR_ForceFX.dll    force feedback only
rem The camera uses MinHook, which is built for the dynamic C runtime, so the variants with the camera are built with /MD and
rem need the Microsoft Visual C++ 2015-2022 x86 Redistributable. The others are built with /MT and need nothing.
setlocal
cd /d "%~dp0"
call :build full 1 1 || goto :fail
call :build audio 1 0 || goto :fail
call :build camera 0 1 || goto :fail
call :build basic 0 0 || goto :fail
echo.
echo Built out\full, out\audio, out\camera and out\basic
exit /b 0

:build
set "NAME=%~1"
set "DEFS="
set "CRT=/MT"
set "LIBS=dinput8.lib dxguid.lib user32.lib ws2_32.lib"
set "EXTRA="
if "%~2"=="1" (
    set "DEFS=%DEFS% /DFORCEFX_AUDIO"
    set "LIBS=%LIBS% winmm.lib"
)
if "%~3"=="1" (
    set "DEFS=%DEFS% /DFORCEFX_CAMERA"
    set "CRT=/MD"
    set "EXTRA=thirdparty\lib\libminhook.x86.lib"
)
if not exist out\%NAME% mkdir out\%NAME%
cl /std:c++17 /O2 %CRT% /EHsc /LD %DEFS% /Fo:out\%NAME%\ RBR_ForceFX.cpp %EXTRA% /link /DEF:RBR_ForceFX.def %LIBS% /OUT:out\%NAME%\RBR_ForceFX.dll
exit /b %errorlevel%

:fail
echo Build failed.
exit /b 1
