@echo off
setlocal

::Select the path to steam folder
set "psCommand="(new-object -COM 'Shell.Application')^
.BrowseForFolder(0,'Please select the root folder for steam (ex: c:\Program Files (x86)\steam).',0,0).self.path""
for /f "usebackq delims=" %%I in (`powershell %psCommand%`) do set "STEAM_ROOT_PATH=%%I"
if NOT DEFINED STEAM_ROOT_PATH (goto failure)

:: Write out the paths to a config batch file
del SetDriverVars.bat
echo @echo off >> SetDriverVars.bat
echo set PLATFORM=win32>> SetDriverVars.bat
echo set INSTALL_DIR=%STEAM_ROOT_PATH%\steamapps\common\OpenVR\drivers\psmove>> SetDriverVars.bat
echo set BUILD_DIR=..\..\win32\bin>> SetDriverVars.bat
echo set STEAMVR_RUNTIME_DIR=%STEAM_ROOT_PATH%\steamapps\common\OpenVR>> SetDriverVars.bat

:: Copy over the openvr drivers
::call ReinstallDriver.bat
pause
goto exit

:failure
goto exit

:exit
endlocal
