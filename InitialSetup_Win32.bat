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
set "psCommand="(new-object -COM 'Shell.Application')^
.BrowseForFolder(0,'Please select the root folder for Boost (ex: c:\local\boost_1_61_0).',0,0).self.path""
for /f "usebackq delims=" %%I in (`powershell %psCommand%`) do set "BOOST_ROOT_PATH=%%I"
if NOT DEFINED BOOST_ROOT_PATH (goto failure)

:: Write out the paths to a config batch file
del SetBuildVars_Win32.bat
echo @echo off >> SetBuildVars_Win32.bat
echo set BOOST_ROOT_PATH=%BOOST_ROOT_PATH% >> SetBuildVars_Win32.bat
echo set BOOST_LIB_PATH=%BOOST_ROOT_PATH%/lib32-msvc-14.0 >> SetBuildVars_Win32.bat

:: Add MSVC build tools to the path
call "C:\Program Files (x86)\Microsoft Visual Studio 14.0\VC\vcvarsall.bat" x86

:: Generate the project files for OpenCV 32-bit
echo "Creating OpenCV 32-bit project files..."
pushd %OPENCV_ROOT_PATH%\sources
del /f /s /q build > nul
rmdir /s /q build
mkdir build
pushd build
cmake -G "Visual Studio 14 2015" -DBUILD_SHARED_LIBS=OFF -DBUILD_WITH_STATIC_CRT=OFF -DBUILD_PERF_TESTS=OFF -DBUILD_TESTS=OFF -DBUILD_DOCS=OFF  -DBUILD_opencv_apps=OFF -DBUILD_opencv_calib3d=ON -DBUILD_opencv_flann=ON -DBUILD_opencv_features2d=ON -DBUILD_opencv_objdetect=OFF -DBUILD_opencv_photo=OFF -DBUILD_opencv_ts=OFF -DBUILD_opencv_ml=OFF -DBUILD_opencv_video=OFF -DBUILD_opencv_java=OFF -DWITH_OPENEXR=OFF -DWITH_FFMPEG=OFF -DWITH_JASPER=OFF -DWITH_TIFF=OFF -DBUILD_opencv_python2=OFF -DBUILD_opencv_python3=OFF ..
popd
popd

pushd %OPENCV_ROOT_PATH%\sources\build
:: Compile the DEBUG|Win32 and RELEASE|Win32 builds of OpenCV
echo "Building OpenCV DEBUG|Win32..."
MSBuild.exe OpenCV.sln /p:configuration=DEBUG /p:Platform="Win32" /t:Clean;Build 
echo "Building OpenCV RELEASE|Win32..."
MSBuild.exe OpenCV.sln /p:configuration=RELEASE /p:Platform="Win32" /t:Clean;Build

:: "Install" the 32-bit debug and release builds of OpenCV into the install/x86 folder
echo "Installing OpenCV DEBUG|Win32..."
MSBuild.exe INSTALL.vcxproj /p:configuration=DEBUG /p:Platform="Win32" /t:Build
echo "Installing OpenCV RELEASE|Win32..."
MSBuild.exe INSTALL.vcxproj /p:configuration=RELEASE /p:Platform="Win32" /t:Build

:: Clean out any existing %OPENCV_ROOT_PATH%/build/x86 folder
echo "Cleanout out any existing x86 OpenCV build"
del /f /s /q %OPENCV_ROOT_PATH%/build/x86 > nul
rmdir /s /q %OPENCV_ROOT_PATH%/build/x86

:: Move the x86 folder into the top level build folder alongside the x64 build
echo "Moving OpenCV x86 build adjacent x64 build"
move install/x86 %OPENCV_ROOT_PATH%/build

:: Clean out build intermediates
echo "Cleaning out OpenCV x86 build intermediates"
MSBuild.exe OpenCV.sln /p:configuration=DEBUG /p:Platform="Win32" /t:Clean
MSBuild.exe OpenCV.sln /p:configuration=RELEASE /p:Platform="Win32" /t:Clean
popd

:: Compile the DEBUG|Win32 and RELEASE|Win32 builds of protobuf
echo "Creating protobuf project files..."
pushd thirdparty\protobuf
del /f /s /q vsprojects > nul
rmdir /s /q vsprojects
mkdir vsprojects
pushd vsprojects
cmake -G "Visual Studio 14 2015" -Dprotobuf_DEBUG_POSTFIX="" -Dprotobuf_BUILD_TESTS=OFF ../cmake
echo "Building protobuf DEBUG|Win32"
MSBuild.exe protobuf.sln /p:configuration=DEBUG /t:Clean;Build 
echo "Building protobuf RELEASE|Win32"
MSBuild.exe protobuf.sln /p:configuration=RELEASE /t:Clean;Build
popd
popd

:: Compile the DEBUG|Win32 and RELEASE|Win32 builds of libusb
pushd thirdparty\libusb\msvc\
echo "Building libusb DEBUG|Win32..."
MSBuild.exe libusb_2015.sln /p:configuration=DEBUG /p:Platform="Win32" /t:Clean;Build 
echo "Building libusb RELEASE|Win32..."
MSBuild.exe libusb_2015.sln /p:configuration=RELEASE /p:Platform="Win32" /t:Clean;Build
popd

:: Compile the RELEASE|Win32 build of SDL2
echo "Creating SDL2 project files..."
pushd thirdparty\SDL2
del /f /s /q build > nul
rmdir /s /q build
mkdir build
pushd build
cmake .. -G "Visual Studio 14 2015" -DDIRECTX=OFF -DDIRECTX=OFF -DSDL_STATIC=ON -DFORCE_STATIC_VCRT=ON -DEXTRA_CFLAGS="-MT -Z7 -DSDL_MAIN_HANDLED -DWIN32 -DNDEBUG -D_CRT_SECURE_NO_WARNINGS -DHAVE_LIBC -D_USE_MATH_DEFINES
echo "Building SDL2 Release|Win32..."
MSBuild.exe SDL2.sln /p:configuration=RELEASE /p:Platform="Win32" /t:Clean
MSBuild.exe SDL2-static.vcxproj /p:configuration=RELEASE /p:Platform="Win32" /t:Build 
MSBuild.exe SDL2main.vcxproj /p:configuration=RELEASE /p:Platform="Win32" /t:Build
popd
popd

:: Generate the project files for PSMoveService
call GenerateProjectFiles_Win32.bat
pause
goto exit

:failure
goto exit

:exit
endlocal
