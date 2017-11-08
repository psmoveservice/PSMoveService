@echo off
call SetBuildVars_x64.bat

IF NOT EXIST build mkdir build
pushd build

echo "Rebuilding PSMoveService x64 Project files..."
cmake .. -G "Visual Studio 14 2015 Win64" -DOpenCV_DIR=%OPENCV_BUILD_PATH% -DBOOST_ROOT=%BOOST_ROOT_PATH% -DBOOST_LIBRARYDIR=%BOOST_LIB_PATH% -DPROTOBUF_SRC_ROOT_FOLDER=..\thirdparty\protobuf
IF %ERRORLEVEL% NEQ 0 (
  echo "Error generating PSMoveService 64-bit project files"
  goto failure
)

popd
EXIT /B 0

:failure
pause
EXIT /B 1