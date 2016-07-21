@echo off
setlocal

::Select the path to the root opencv folder
set "psCommand="(new-object -COM 'Shell.Application')^
.BrowseForFolder(0,'Please select the root folder for 64-bit OpenCV (ex: c:\OpenCV-3.0.0).',0,0).self.path""
for /f "usebackq delims=" %%I in (`powershell %psCommand%`) do set "OPENCV_ROOT_PATH=%%I"
if NOT DEFINED OPENCV_ROOT_PATH (goto failure)

::Select the path to the root Boost folder
set "psCommand="(new-object -COM 'Shell.Application')^
.BrowseForFolder(0,'Please select the root folder for 64-bit Boost (ex: c:\boost_1_59_0).',0,0).self.path""
for /f "usebackq delims=" %%I in (`powershell %psCommand%`) do set "BOOST_ROOT_PATH=%%I"
if NOT DEFINED BOOST_ROOT_PATH (goto failure)

:: Write out the paths to a config batch file
del SetBuildVars_x64.bat
echo @echo off >> SetBuildVars_x64.bat
echo set OPENCV_BUILD_PATH=%OPENCV_ROOT_PATH%\build >> SetBuildVars_x64.bat
echo set BOOST_ROOT_PATH=%BOOST_ROOT_PATH% >> SetBuildVars_x64.bat
echo set BOOST_LIB_PATH=%BOOST_ROOT_PATH%\lib64-msvc-12.0 >> SetBuildVars_x64.bat

:: Add MSVC build tools to the path
call "C:\Program Files (x86)\Microsoft Visual Studio 12.0\VC\vcvarsall.bat" x86

:: Compile the DEBUG|x64 and RELEASE|x64 builds of protobuf
echo "Creating protobuf project files..."
pushd thirdparty\protobuf
del /f /s /q vsprojects > nul
rmdir /s /q vsprojects
mkdir vsprojects
pushd vsprojects
cmake -G "Visual Studio 12 2013 Win64" -Dprotobuf_DEBUG_POSTFIX="" -Dprotobuf_BUILD_TESTS=OFF ../cmake
echo "Building protobuf DEBUG|x64"
MSBuild.exe protobuf.sln /p:configuration=DEBUG /t:Clean;Build 
:: Work around for issue in 64-bit builds, FindProtobuf modules expects files in x64\Debug
move Debug\*.* x64\Debug
echo "Building protobuf RELEASE|x64"
MSBuild.exe protobuf.sln /p:configuration=RELEASE /t:Clean;Build
:: Work around for issue in 64-bit builds, FindProtobuf modules expects files in x64\Release
move Release\*.* x64\Release
popd
popd

:: Compile the DEBUG|x64 and RELEASE|x64 builds of libusb
pushd thirdparty\libusb\msvc\
echo "Building libusb DEBUG|x64..."
MSBuild.exe libusb_2013.sln /tv:12.0 /p:configuration=DEBUG /p:Platform="x64" /t:Clean;Build 
echo "Building libusb RELEASE|x64..."
MSBuild.exe libusb_2013.sln /tv:12.0 /p:configuration=RELEASE /p:Platform="x64" /t:Clean;Build
popd

:: Compile the RELEASE|x64 build of SDL2
echo "Creating SDL2 project files..."
pushd thirdparty\SDL2
del /f /s /q build > nul
rmdir /s /q build
mkdir build
pushd build
cmake .. -G "Visual Studio 12 2013 Win64" -DDIRECTX=OFF -DDIRECTX=OFF -DSDL_STATIC=ON -DFORCE_STATIC_VCRT=ON -DEXTRA_CFLAGS="-MT -Z7 -DSDL_MAIN_HANDLED -DWIN32 -DNDEBUG -D_CRT_SECURE_NO_WARNINGS -DHAVE_LIBC -D_USE_MATH_DEFINES
echo "Building SDL2 Release|x64..."
MSBuild.exe SDL2.sln /p:configuration=RELEASE /p:Platform="x64" /t:Clean
MSBuild.exe SDL2-static.vcxproj /p:configuration=RELEASE /p:Platform="x64" /t:Build 
MSBuild.exe SDL2main.vcxproj /p:configuration=RELEASE /p:Platform="x64" /t:Build
popd
popd

:: Generate the project files for PSMoveService
call GenerateProjectFiles_X64.bat
pause
goto exit

:failure
goto exit

:exit
endlocal
