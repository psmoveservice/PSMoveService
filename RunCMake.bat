mkdir build
cd build
cmake .. -G "Visual Studio 12" -DOpenCV_DIR=E:\OpenCV-3.0.0\build -DBOOST_ROOT=E:\boost_1_59_0 -DBOOST_LIBRARYDIR=E:\boost_1_59_0\lib32-msvc-12.0 -DPROTOBUF_SRC_ROOT_FOLDER=E:\git-cboulay\PSMoveService__boost_asio_test\thirdparty\protobuf
pause