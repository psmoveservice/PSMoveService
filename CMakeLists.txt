cmake_minimum_required(VERSION 3.2)
project(PSMoveService VERSION 0.0.1)
include(ExternalProject)

set(ROOT_DIR ${CMAKE_CURRENT_LIST_DIR})

LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
include(cmake/Environment.cmake)
include(cmake/Version.cmake)

set(PSM_PROJECT_NAME "PSMoveService_${PSM_VERSION_STRING}")

include(cmake/ThirdParty.cmake)

# Step into the subdirectories
add_subdirectory(src)

# TODO: Move tests to root-level
# TODO: CMakeLists for misc (matlab, python)
