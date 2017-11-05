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
.BrowseForFolder(0,'Please select the root folder for 64-bit Boost (ex: c:\local\boost_1_61_0).',0,0).self.path""
for /f "usebackq delims=" %%I in (`powershell %psCommand%`) do set "BOOST_ROOT_PATH=%%I"
if NOT DEFINED BOOST_ROOT_PATH (goto failure)

:build_thirdparty

:: Write out the paths to a config batch file
del SetBuildVars_x64.bat
echo @echo off >> SetBuildVars_x64.bat
echo set BOOST_ROOT_PATH=%BOOST_ROOT_PATH% >> SetBuildVars_x64.bat
echo set BOOST_LIB_PATH=%BOOST_ROOT_PATH%/lib64-msvc-14.0 >> SetBuildVars_x64.bat

:: Add MSVC build tools to the path
call "C:\Program Files (x86)\Microsoft Visual Studio 14.0\VC\vcvarsall.bat" amd64
IF %ERRORLEVEL% NEQ 0 (
  echo "Unable to initialize 64-bit visual studio build environment"
  goto failure
)

:: Compile the DEBUG|x64 and RELEASE|x64 builds of protobuf
echo "Creating protobuf project files..."
pushd thirdparty\protobuf
del /f /s /q vsprojects > nul
rmdir /s /q vsprojects
mkdir vsprojects
pushd vsprojects
cmake -G "Visual Studio 14 2015 Win64" -Dprotobuf_DEBUG_POSTFIX="" -Dprotobuf_BUILD_TESTS=OFF ../cmake
IF %ERRORLEVEL% NEQ 0 (
  echo "Error generating protobuf project files!"
  goto failure
)

echo "Building protobuf DEBUG|x64"
MSBuild.exe protobuf.sln /p:configuration=DEBUG /t:Clean;Build 
IF %ERRORLEVEL% NEQ 0 (
  echo "Error building protobuf DEBUG|x64!"
  goto failure
)
:: Work around for issue in 64-bit builds, FindProtobuf modules expects files in x64\Debug
copy Debug\*.* x64\Debug || goto failure

echo "Building protobuf RELEASE|x64"
MSBuild.exe protobuf.sln /p:configuration=RELEASE /t:Clean;Build
IF %ERRORLEVEL% NEQ 0 (
  echo "Error building protobuf RELEASE|x64!"
  goto failure
)
:: Work around for issue in 64-bit builds, FindProtobuf modules expects files in x64\Release
copy Release\*.* x64\Release || goto failure
popd
popd

:: Compile the DEBUG|x64 and RELEASE|x64 builds of libusb
pushd thirdparty\libusb\msvc\
echo "Building libusb DEBUG|x64..."
MSBuild.exe libusb_2015.sln /tv:14.0 /p:configuration=DEBUG /p:Platform="x64" /t:Clean;Build 
IF %ERRORLEVEL% NEQ 0 (
  echo "Error building libusb DEBUG|x64!"
  goto failure
)
echo "Building libusb RELEASE|x64..."
MSBuild.exe libusb_2015.sln /tv:14.0 /p:configuration=RELEASE /p:Platform="x64" /t:Clean;Build
IF %ERRORLEVEL% NEQ 0 (
  echo "Error building libusb RELEASE|x64!"
  goto failure
)
popd

:: Compile the RELEASE|x64 build of SDL2
echo "Creating SDL2 project files..."
pushd thirdparty\SDL2
del /f /s /q build > nul
rmdir /s /q build
mkdir build
pushd build
cmake .. -G "Visual Studio 14 2015 Win64" -DDIRECTX=OFF -DDIRECTX=OFF -DSDL_STATIC=ON -DFORCE_STATIC_VCRT=ON -DEXTRA_CFLAGS="-MT -Z7 -DSDL_MAIN_HANDLED -DWIN32 -DNDEBUG -D_CRT_SECURE_NO_WARNINGS -DHAVE_LIBC -D_USE_MATH_DEFINES
IF %ERRORLEVEL% NEQ 0 (
  echo "Error generating SDL2 project files!"
  goto failure
)

echo "Building SDL2 Release|x64..."
MSBuild.exe SDL2.sln /p:configuration=RELEASE /p:Platform="x64" /t:Clean
IF %ERRORLEVEL% NEQ 0 (
  echo "Error building SDL2 Release|x64!"
  goto failure
)
MSBuild.exe SDL2-static.vcxproj /p:configuration=RELEASE /p:Platform="x64" /t:Build 
IF %ERRORLEVEL% NEQ 0 (
  echo "Error building SDL2-static Release|x64!"
  goto failure
)
MSBuild.exe SDL2main.vcxproj /p:configuration=RELEASE /p:Platform="x64" /t:Build
IF %ERRORLEVEL% NEQ 0 (
  echo "Error building SDL2main Release|x64!"
  goto failure
)
popd
popd

:: Generate the project files for PSMoveService
call GenerateProjectFiles_X64.bat || goto failure
EXIT /B 0

:failure
pause
EXIT /B 1
