cmake_minimum_required(VERSION 3.0)
project(PSMoveService VERSION 0.0.1)

# Step into the subdirectories
add_subdirectory(thirdparty)
add_subdirectory(psmoveservice ${CMAKE_BINARY_DIR})