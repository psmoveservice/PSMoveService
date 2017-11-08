cmake_minimum_required(VERSION 3.2)
project(PSMoveService VERSION 0.0.1)
include(ExternalProject)

set(ROOT_DIR ${CMAKE_CURRENT_LIST_DIR})

LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
include(cmake/Environment.cmake)
include(cmake/Version.cmake)

set(PSM_PROJECT_NAME "PSMoveService_${PSM_VERSION_STRING}")
set(PSM_ROOT_INSTALL_PATH "${ROOT_DIR}/dist")
set(PSM_DEBUG_INSTALL_PATH "${PSM_ROOT_INSTALL_PATH}/${ARCH_LABEL}/Debug")
set(PSM_RELEASE_INSTALL_PATH "${PSM_ROOT_INSTALL_PATH}/${ARCH_LABEL}/Release")
file(MAKE_DIRECTORY "${PSM_ROOT_INSTALL_PATH}")
file(WRITE "${PSM_ROOT_INSTALL_PATH}/version.txt" "${PSM_PROJECT_NAME}")

include(cmake/ThirdParty.cmake)
include(cmake/Installer.cmake)

# Step into the subdirectories
add_subdirectory(src)

# TODO: Move tests to root-level
# TODO: CMakeLists for misc (matlab, python)
