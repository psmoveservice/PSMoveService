@echo off
call SteamVR_SetDriverVars.bat

echo "Unstalling PSMoveService SteamVR driver..."
IF NOT EXIST "%INSTALL_DIR%" goto done
"%STEAMVR_RUNTIME_DIR%\bin\win32\vrpathreg" removedriver "%INSTALL_DIR%"
del /F /S /Q "%INSTALL_DIR%*.*"
rmdir /S /Q "%INSTALL_DIR%"

:done
echo "Done"
pause