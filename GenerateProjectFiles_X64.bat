@echo off
call SetBuildVars_x64.bat

IF NOT EXIST build mkdir build
cd build

echo "Rebuilding PSMoveService x64 Project files..."
cmake .. -G "Visual Studio 12 2013 Win64" -DOpenCV_DIR=%OPENCV_BUILD_PATH% -DBOOST_ROOT=%BOOST_ROOT_PATH% -DBOOST_LIBRARYDIR=%BOOST_LIB_PATH% -DPROTOBUF_SRC_ROOT_FOLDER=..\thirdparty\protobuf
pause