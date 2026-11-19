@echo off
rem Runs Copy-RbrSounds.ps1 without having to open PowerShell yourself.
rem Double-click it and type your RBR folder, or drag the RBR folder onto this file.
rem Extra options can follow the folder when run from a command prompt, e.g.:  Copy-RbrSounds.bat "C:\Games\RBR" -OnlyMissing
setlocal
set "RBR=%~1"
if "%RBR%"=="" set /p "RBR=Enter your RBR folder (the one that contains the Audio folder), e.g. C:\Games\RichardBurnsRally: "
set "RBR=%RBR:"=%"
if "%RBR:~-1%"=="\" set "RBR=%RBR:~0,-1%"
shift
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0Copy-RbrSounds.ps1" -RbrDir "%RBR%" %1 %2 %3
echo.
pause
