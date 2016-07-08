@echo off
call SteamVR_SetDriverVarsWin32.bat

echo "(Re)Installing PSMoveService SteamVR %PLATFORM% driver..."
IF NOT EXIST "%INSTALL_DIR%\bin\win32" mkdir "%INSTALL_DIR%\bin\win32"
copy driver_psmove.dll "%INSTALL_DIR%\bin\win32\driver_psmove.dll"
copy PSMoveClient.dll "%INSTALL_DIR%\bin\win32"
"%STEAMVR_RUNTIME_DIR%\bin\win32\vrpathreg" adddriver "%INSTALL_DIR%"
xcopy /s /i /y "resources" "%STEAMVR_RUNTIME_DIR%\drivers\psmove\resources"

echo "Done"
pause