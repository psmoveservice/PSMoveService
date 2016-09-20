@echo off
call SteamVR_SetDriverVarsWin64.bat

echo "(Re)Installing PSMoveService SteamVR Win64 driver..."
IF NOT EXIST "%INSTALL_DIR%\bin\win64" mkdir "%INSTALL_DIR%\bin\win64"
copy driver_psmove.dll "%INSTALL_DIR%\bin\win64\driver_psmove.dll"
copy PSMoveClient.dll "%INSTALL_DIR%\bin\win64"
copy monitor_psmove.exe "%INSTALL_DIR%\bin\win64"
copy openvr_api.dll "%INSTALL_DIR%\bin\win64"
"%STEAMVR_RUNTIME_DIR%\bin\win64\vrpathreg" adddriver "%INSTALL_DIR%"
xcopy /s /i /y "resources" "%STEAMVR_RUNTIME_DIR%\drivers\psmove\resources"

echo "Done"
pause