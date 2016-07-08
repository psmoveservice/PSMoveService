@echo off
call SteamVR_SetDriverVarsWin32.bat

echo "Unstalling PSMoveService SteamVR Win32 driver..."
IF NOT EXIST "%INSTALL_DIR%" goto done
"%STEAMVR_RUNTIME_DIR%\bin\win32\vrpathreg" removedriver "%INSTALL_DIR%"
del /F /S /Q "%INSTALL_DIR%*.*"
rmdir /S /Q "%INSTALL_DIR%"

:done
echo "Done"
pause