@echo off
setlocal

::Select the path to the root opencv folder
set "psCommand="(new-object -COM 'Shell.Application')^
.BrowseForFolder(0,'Please select the root folder for Opencv (ex: c:\OpenCV-3.0.0).',0,0).self.path""
for /f "usebackq delims=" %%I in (`powershell %psCommand%`) do set "OPENCV_ROOT_PATH=%%I"
if NOT DEFINED OPENCV_ROOT_PATH (goto failure)

::Select the path to the root Boost folder
set "psCommand="(new-object -COM 'Shell.Application')^
.BrowseForFolder(0,'Please select the root folder for Boost (ex: c:\boost_1_59_0).',0,0).self.path""
for /f "usebackq delims=" %%I in (`powershell %psCommand%`) do set "BOOST_ROOT_PATH=%%I"
if NOT DEFINED BOOST_ROOT_PATH (goto failure)

:: Write out the paths to a config batch file
del SetBuildVars.bat
echo @echo off >> SetBuildVars.bat
echo set OPENCV_BUILD_PATH=%OPENCV_ROOT_PATH%\build >> SetBuildVars.bat
echo set BOOST_ROOT_PATH=%BOOST_ROOT_PATH% >> SetBuildVars.bat
echo set BOOST_LIB_PATH=%BOOST_ROOT_PATH%\lib32-msvc-12.0 >> SetBuildVars.bat

:: Compile the DEBUG|Win32 and RELEASE|Win32 builds of protobuf
echo "Creating protobuf project files..."
call "C:\Program Files (x86)\Microsoft Visual Studio 12.0\VC\vcvarsall.bat" x86
pushd thirdparty\protobuf
mkdir vsprojects
pushd vsprojects
cmake -G "Visual Studio 12 2013" -Dprotobuf_DEBUG_POSTFIX="" -Dprotobuf_BUILD_TESTS=OFF ../cmake
echo "Building protobuf DEBUG|Win32"
MSBuild.exe protobuf.sln /p:configuration=DEBUG /t:Clean;Build 
echo "Building protobuf RELEASE|Win32"
MSBuild.exe protobuf.sln /p:configuration=RELEASE /t:Clean;Build
popd
popd

:: Compile the DEBUG|Win32 and RELEASE|Win32 builds of libusb
pushd thirdparty\libusb\msvc\
echo "Building libusb DEBUG|Win32..."
MSBuild.exe libusb_2013.sln /p:configuration=DEBUG /p:Platform="Win32" /t:Clean;Build 
echo "Building libusb RELEASE|Win32..."
MSBuild.exe libusb_2013.sln /p:configuration=RELEASE /p:Platform="Win32" /t:Clean;Build
popd

:: Generate the project files for PSMoveService
call RunCMake.bat
pause
goto exit

:failure
goto exit

:exit
endlocal
