@echo off
setlocal

rem Copy the pictures saved by the camera node from the robot to this computer.
rem
rem RUN THIS ON YOUR OWN COMPUTER, not on the robot.
rem Double-click it, or from a terminal:
rem
rem   get_images.cmd duckie03                 folder name taken from this repo
rem   get_images.cmd duckie03 other-folder    if the folder on the robot differs
rem
rem Password is quackquack, unless you set up an SSH key.

where scp >nul 2>&1
if errorlevel 1 goto :noscp

set "ROBOT=%~1"
if "%ROBOT%"=="" set /p "ROBOT=Robot name (for example duckie03): "
if "%ROBOT%"=="" goto :norobot

rem The folder this script sits in, for example team1.
for %%I in ("%~dp0.") do set "REPO=%%~nxI"
if not "%~2"=="" set "REPO=%~2"
set "REPO=team-2"
set "DEST=%~dp0images_from_%ROBOT%"
if not exist "%DEST%" mkdir "%DEST%"

echo Copying from duckie@%ROBOT%.local:~/%REPO%/images/ ...
echo.
scp "duckie@%ROBOT%.local:~/%REPO%/images/*.jpg" "%DEST%"
if errorlevel 1 goto :failed

echo.
echo Done. Pictures are in %DEST%
goto :end

:noscp
echo scp was not found.
echo Install it: Settings - Apps - Optional features - OpenSSH Client.
goto :end

:norobot
echo No robot name given, nothing to do.
goto :end

:failed
echo.
echo Copy failed. Check that:
echo   - you are on the same wifi as the robot
echo   - the robot name is right, try: ssh duckie@%ROBOT%.local
echo   - the folder on the robot is really called %REPO%
echo   - there are pictures there, run the camera node first
goto :end

:end
echo.
pause
