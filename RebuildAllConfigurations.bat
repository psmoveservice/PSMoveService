@echo off
setlocal

::Select the path to the root Boost folder
set "psCommand="(new-object -COM 'Shell.Application')^
.BrowseForFolder(0,'Please select the root folder for 64-bit Boost (ex: c:\local\boost_1_61_0).',0,0).self.path""
for /f "usebackq delims=" %%I in (`powershell %psCommand%`) do set "BOOST_ROOT_PATH=%%I"
if NOT DEFINED BOOST_ROOT_PATH (goto failure)

:: Cleanup any prior distribution folders
IF EXIST dist (
  del /f /s /q dist > nul
  rmdir /s /q dist
)

:: Generate the 32-bit project files and dependencies for PSMoveService
call InitialSetup_Win32.bat || goto failure
IF %ERRORLEVEL% NEQ 0 (
  echo "Error setting up 32-bit PSMoveService project"
  goto failure
)

:: Add MSVC build tools to the path
call "C:\Program Files (x86)\Microsoft Visual Studio 14.0\VC\vcvarsall.bat" x86
IF %ERRORLEVEL% NEQ 0 (
  echo "Unable to initialize 32-bit visual studio build environment"
  goto failure
)

:: Compile and install the DEBUG|Win32 build of PSMoveService to the distribution folder
pushd build\
echo "Building PSMoveService DEBUG|Win32..."
MSBuild.exe opencv.vcxproj /p:configuration=DEBUG /p:Platform="Win32" /t:Build
IF %ERRORLEVEL% NEQ 0 (
  echo "Error building OpenCV DEBUG|Win32!"
  goto failure
) 
MSBuild.exe PSMoveService.sln /p:configuration=DEBUG /p:Platform="Win32" /t:Build
IF %ERRORLEVEL% NEQ 0 (
  echo "Error building PSMoveService DEBUG|Win32!"
  goto failure
) 
MSBuild.exe INSTALL.vcxproj /p:configuration=DEBUG /p:Platform="Win32" /t:Build
IF %ERRORLEVEL% NEQ 0 (
  echo "Error deploying PSMoveService DEBUG|Win32 to distribution folder!"
  goto failure
) 

:: Compile and install the RELEASE|Win32 build of PSMoveService to the distribution folder
echo "Building PSMoveService RELEASE|Win32..."
MSBuild.exe opencv.vcxproj /p:configuration=RELEASE /p:Platform="Win32" /t:Build
IF %ERRORLEVEL% NEQ 0 (
  echo "Error building OpenCV RELEASE|Win32!"
  goto failure
) 
MSBuild.exe PSMoveService.sln /p:configuration=RELEASE /p:Platform="Win32" /t:Build
IF %ERRORLEVEL% NEQ 0 (
  echo "Error building PSMoveService RELEASE|Win32!"
  goto failure
) 
MSBuild.exe INSTALL.vcxproj /p:configuration=RELEASE /p:Platform="Win32" /t:Build
IF %ERRORLEVEL% NEQ 0 (
  echo "Error deploying PSMoveService RELEASE|Win32 to distribution folder!"
  goto failure
) 
popd

:: Generate the 64-bit project files and dependencies for PSMoveService
call InitialSetup_X64.bat || goto failure
IF %ERRORLEVEL% NEQ 0 (
  echo "Error setting up 64-bit PSMoveService project"
  goto failure
)

:: Add MSVC build tools to the path
call "C:\Program Files (x86)\Microsoft Visual Studio 14.0\VC\vcvarsall.bat" amd64
IF %ERRORLEVEL% NEQ 0 (
  echo "Unable to initialize 64-bit visual studio build environment"
  goto failure
)

:: Compile and install the DEBUG|x64 build of PSMoveService to the distribution folder
pushd build\
echo "Building PSMoveService DEBUG|x64..."
MSBuild.exe opencv.vcxproj /p:configuration=DEBUG /p:Platform="x64" /t:Build
IF %ERRORLEVEL% NEQ 0 (
  echo "Error building OpenCV DEBUG|x64!"
  goto failure
) 
MSBuild.exe PSMoveService.sln /p:configuration=DEBUG /p:Platform="x64" /t:Build
IF %ERRORLEVEL% NEQ 0 (
  echo "Error building PSMoveService DEBUG|x64!"
  goto failure
) 
MSBuild.exe INSTALL.vcxproj /p:configuration=DEBUG /p:Platform="x64" /t:Build
IF %ERRORLEVEL% NEQ 0 (
  echo "Error deploying PSMoveService DEBUG|x64 to distribution folder!"
  goto failure
) 

:: Compile and install the RELEASE|x64 build of PSMoveService to the distribution folder
echo "Building PSMoveService RELEASE|x64..."
MSBuild.exe opencv.vcxproj /p:configuration=RELEASE /p:Platform="x64" /t:Build
IF %ERRORLEVEL% NEQ 0 (
  echo "Error building OpenCV RELEASE|x64!"
  goto failure
) 
MSBuild.exe PSMoveService.sln /p:configuration=RELEASE /p:Platform="x64" /t:Build
IF %ERRORLEVEL% NEQ 0 (
  echo "Error building PSMoveService RELEASE|x64!"
  goto failure
) 
MSBuild.exe INSTALL.vcxproj /p:configuration=RELEASE /p:Platform="x64" /t:Build
IF %ERRORLEVEL% NEQ 0 (
  echo "Error deploying PSMoveService RELEASE|x64 to distribution folder!"
  goto failure
) 
popd

echo "Successfully Built All Configurations!"
pause
EXIT /B 0

:failure
pause
EXIT /B 1