cmake_minimum_required(VERSION 3.0)

#DEPENDENCIES

#None so far

set(frame_srcs_hdrs PSMoveDataFrame.h PSMoveDataFrame.cpp)
add_library(PSMoveDataFrame STATIC ${frame_srcs_hdrs})