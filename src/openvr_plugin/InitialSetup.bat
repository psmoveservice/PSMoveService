@echo off
setlocal

::Select the path to steam folder
set "psCommand="(new-object -COM 'Shell.Application')^
.BrowseForFolder(0,'Please select the root folder for steam (ex: c:\Program Files (x86)\steam).',0,0).self.path""
for /f "usebackq delims=" %%I in (`powershell %psCommand%`) do set "STEAM_ROOT_PATH=%%I"
IF NOT DEFINED STEAM_ROOT_PATH (goto failure)

IF EXIST "%STEAM_ROOT_PATH%\steamapps\common\OpenVR" GOTO use_openvr
IF EXIST "%STEAM_ROOT_PATH%\steamapps\common\SteamVR" GOTO use_steamvr
goto no_steamvr_installed

:no_steamvr_installed
echo "No steamvr folder found at %STEAM_ROOT_PATH%! Please install steamvr."
goto failure

:use_openvr
set STEAMVR_RUNTIME_DIR=%STEAM_ROOT_PATH%\steamapps\common\OpenVR
goto write_set_drivers_script

:use_steamvr
set STEAMVR_RUNTIME_DIR=%STEAM_ROOT_PATH%\steamapps\common\SteamVR
goto write_set_drivers_script

:write_set_drivers_script
echo "Found SteamVR Runtime Dir: %STEAMVR_RUNTIME_DIR%"

:: Write out the paths to a config batch file
del SetDriverVars.bat
echo @echo off >> SetDriverVars.bat
echo set PLATFORM=win32>> SetDriverVars.bat
echo set INSTALL_DIR=%STEAMVR_RUNTIME_DIR%\drivers\psmove>> SetDriverVars.bat
echo set BUILD_DIR=..\..\win32\bin>> SetDriverVars.bat
echo set STEAMVR_RUNTIME_DIR=%STEAMVR_RUNTIME_DIR%>> SetDriverVars.bat

:: Copy over the openvr drivers
::call ReinstallDriver.bat
pause
goto exit

:failure
pause
goto exit

:exit
endlocal
