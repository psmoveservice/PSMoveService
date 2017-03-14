@echo off
call SteamVR_SetDriverVarsWin64.bat

echo "(Re)Installing PSMoveService SteamVR Win64 driver..."
IF NOT EXIST "%INSTALL_DIR%\bin\win64" mkdir "%INSTALL_DIR%\bin\win64"
copy driver_psmove.dll "%INSTALL_DIR%\bin\win64\driver_psmove.dll"
IF EXIST driver_psmove.pdb copy driver_psmove.pdb "%INSTALL_DIR%\bin\win64\driver_psmove.pdb"
copy PSMoveClient.dll "%INSTALL_DIR%\bin\win64"
copy PSMoveClient_CAPI.dll "%INSTALL_DIR%\bin\win64"
copy monitor_psmove.exe "%INSTALL_DIR%\bin\win64"
IF EXIST monitor_psmove.pdb copy monitor_psmove.pdb "%INSTALL_DIR%\bin\win32\monitor_psmove.pdb"
copy openvr_api.dll "%INSTALL_DIR%\bin\win64"
xcopy /s /i /y "resources" "%STEAMVR_RUNTIME_DIR%\drivers\psmove\resources"
xcopy /s /i /y configuration "%STEAMVR_RUNTIME_DIR%\drivers\psmove\configuration"
copy driver.vrdrivermanifest "%INSTALL_DIR%"
"%STEAMVR_RUNTIME_DIR%\bin\win64\vrpathreg" adddriver "%INSTALL_DIR%"

echo "Done"
pause