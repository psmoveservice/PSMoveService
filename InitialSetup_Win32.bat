@echo off
setlocal

::Clean up the old PSMoveService build folder
IF EXIST build (
del /f /s /q build > nul
rmdir /s /q build
)

::Clean up the old PSMoveService deps folder
IF EXIST deps (
del /f /s /q deps > nul
rmdir /s /q deps
)


::Select the path to the root Boost folder
if DEFINED BOOST_ROOT_PATH (goto build_thirdparty)
set "psCommand="(new-object -COM 'Shell.Application')^
.BrowseForFolder(0,'Please select the root folder for 32-bit Boost (ex: c:\local\boost_1_61_0).',0,0).self.path""
for /f "usebackq delims=" %%I in (`powershell %psCommand%`) do set "BOOST_ROOT_PATH=%%I"
if NOT DEFINED BOOST_ROOT_PATH (goto failure)

:build_thirdparty

:: Write out the paths to a config batch file
del SetBuildVars_Win32.bat
echo @echo off >> SetBuildVars_Win32.bat
echo set BOOST_ROOT_PATH=%BOOST_ROOT_PATH% >> SetBuildVars_Win32.bat
echo set BOOST_LIB_PATH=%BOOST_ROOT_PATH%/lib32-msvc-14.0 >> SetBuildVars_Win32.bat

:: Add MSVC build tools to the path
call "C:\Program Files (x86)\Microsoft Visual Studio 14.0\VC\vcvarsall.bat" x86
IF %ERRORLEVEL% NEQ 0 (
  echo "Unable to initialize 32-bit visual studio build environment"
  goto failure
)

:: Compile the DEBUG|Win32 and RELEASE|Win32 builds of protobuf
echo "Creating protobuf project files..."
pushd thirdparty\protobuf
del /f /s /q vsprojects > nul
rmdir /s /q vsprojects
mkdir vsprojects
pushd vsprojects
cmake -G "Visual Studio 14 2015" -Dprotobuf_DEBUG_POSTFIX="" -Dprotobuf_BUILD_TESTS=OFF ../cmake
echo "Building protobuf DEBUG|Win32"
IF %ERRORLEVEL% NEQ 0 (
  echo "Error generating protobuf project files!"
  goto failure
)
MSBuild.exe protobuf.sln /p:configuration=DEBUG /t:Clean;Build 
IF %ERRORLEVEL% NEQ 0 (
  echo "Error building protobuf DEBUG|Win32!"
  goto failure
)
echo "Building protobuf RELEASE|Win32"
MSBuild.exe protobuf.sln /p:configuration=RELEASE /t:Clean;Build
IF %ERRORLEVEL% NEQ 0 (
  echo "Error building protobuf Release|Win64!"
  goto failure
)
popd
popd

:: Compile the DEBUG|Win32 and RELEASE|Win32 builds of libusb
pushd thirdparty\libusb\msvc\
echo "Building libusb DEBUG|Win32..."
MSBuild.exe libusb_2015.sln /p:configuration=DEBUG /p:Platform="Win32" /t:Clean;Build
IF %ERRORLEVEL% NEQ 0 (
  echo "Error building libusb DEBUG|Win32!"
  goto failure
) 
echo "Building libusb RELEASE|Win32..."
MSBuild.exe libusb_2015.sln /p:configuration=RELEASE /p:Platform="Win32" /t:Clean;Build
IF %ERRORLEVEL% NEQ 0 (
  echo "Error building libusb RELEASE|Win32!"
  goto failure
)
popd

:: Compile the RELEASE|Win32 build of SDL2
echo "Creating SDL2 project files..."
pushd thirdparty\SDL2
del /f /s /q build > nul
rmdir /s /q build
mkdir build
pushd build
cmake .. -G "Visual Studio 14 2015" -DDIRECTX=OFF -DDIRECTX=OFF -DSDL_STATIC=ON -DFORCE_STATIC_VCRT=ON -DEXTRA_CFLAGS="-MT -Z7 -DSDL_MAIN_HANDLED -DWIN32 -DNDEBUG -D_CRT_SECURE_NO_WARNINGS -DHAVE_LIBC -D_USE_MATH_DEFINES
IF %ERRORLEVEL% NEQ 0 (
  echo "Error generating SDL2 project files!"
  goto failure
)

echo "Building SDL2 Release|Win32..."
MSBuild.exe SDL2.sln /p:configuration=RELEASE /p:Platform="Win32" /t:Clean
IF %ERRORLEVEL% NEQ 0 (
  echo "Error building SDL2 Release|Win32!"
  goto failure
)
MSBuild.exe SDL2-static.vcxproj /p:configuration=RELEASE /p:Platform="Win32" /t:Build
IF %ERRORLEVEL% NEQ 0 (
  echo "Error building SDL2-static Release|Win32!"
  goto failure
)
MSBuild.exe SDL2main.vcxproj /p:configuration=RELEASE /p:Platform="Win32" /t:Build
IF %ERRORLEVEL% NEQ 0 (
  echo "Error building SDL2main Release|Win32!"
  goto failure
)
popd
popd

:: Generate the project files for PSMoveService
call GenerateProjectFiles_Win32.bat || goto failure
EXIT /B 0

:failure
pause
EXIT /B 1