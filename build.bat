@echo off
rem Builds both ForceFX variants. Run it from an "x86 Native Tools Command Prompt for VS" (32-bit).
rem   out\audio\RBR_ForceFX.dll     with the dynamic audio module
rem   out\noaudio\RBR_ForceFX.dll   without it
setlocal
cd /d "%~dp0"
if not exist out\audio mkdir out\audio
if not exist out\noaudio mkdir out\noaudio

cl /std:c++17 /O2 /MT /EHsc /LD /DFORCEFX_AUDIO /Fo:out\audio\ RBR_ForceFX.cpp /link /DEF:RBR_ForceFX.def dinput8.lib dxguid.lib user32.lib ws2_32.lib winmm.lib /OUT:out\audio\RBR_ForceFX.dll || goto :fail
cl /std:c++17 /O2 /MT /EHsc /LD /Fo:out\noaudio\ RBR_ForceFX.cpp /link /DEF:RBR_ForceFX.def dinput8.lib dxguid.lib user32.lib ws2_32.lib /OUT:out\noaudio\RBR_ForceFX.dll || goto :fail
echo.
echo Built out\audio\RBR_ForceFX.dll and out\noaudio\RBR_ForceFX.dll
exit /b 0
:fail
echo Build failed.
exit /b 1
