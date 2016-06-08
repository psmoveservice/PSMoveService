@echo off
call SteamVR_SetDriverVars.bat

echo "(Re)Installing PSMoveService SteamVR driver..."
IF NOT EXIST "%INSTALL_DIR%\bin\%PLATFORM%" mkdir "%INSTALL_DIR%\bin\%PLATFORM%"
copy driver_psmove.dll "%INSTALL_DIR%\bin\%PLATFORM%\driver_psmove.dll"
copy PSMoveClient.dll "%INSTALL_DIR%\bin\%PLATFORM%"
"%STEAMVR_RUNTIME_DIR%\bin\win32\vrpathreg" adddriver "%INSTALL_DIR%"
xcopy /s /i /y "resources" "%STEAMVR_RUNTIME_DIR%\drivers\psmove\resources"

echo "Done"
pause