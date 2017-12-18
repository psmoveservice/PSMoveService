@echo off
call SetBuildVars_Win32.bat

IF NOT EXIST build mkdir build
pushd build

echo "Rebuilding PSMoveService Project files..."
cmake .. -G "Visual Studio 14 2015" -DBOOST_ROOT=%BOOST_ROOT_PATH% -DBOOST_LIBRARYDIR=%BOOST_LIB_PATH% -DPROTOBUF_SRC_ROOT_FOLDER=..\thirdparty\protobuf
IF %ERRORLEVEL% NEQ 0 (
  echo "Error generating PSMoveService 32-bit project files"
  goto failure
)

popd
EXIT /B 0

:failure
pause
EXIT /B 1