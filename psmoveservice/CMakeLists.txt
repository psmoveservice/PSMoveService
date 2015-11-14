cmake_minimum_required(VERSION 3.0)

set(ROOT_DIR ${CMAKE_CURRENT_LIST_DIR}/..)

# Dependencies
set(PSMOVE_SERVICE_INCL_DIRS)
set(PSMOVE_SERVICE_REQ_LIBS)

# Boost.Application and type_index are header only (?)
list(APPEND PSMOVE_SERVICE_INCL_DIRS
    ${ROOT_DIR}/thirdparty/Boost.Application/include/
    ${ROOT_DIR}/thirdparty/type_index/include/)

# Protobuf (already found in top-level CMakeLists)
list(APPEND PSMOVE_SERVICE_INCL_DIRS ${PROTOBUF_INCLUDE_DIRS})
list(APPEND PSMOVE_SERVICE_REQ_LIBS ${PROTOBUF_LIBRARIES})

# Boost. TODO: Trim this list.
find_package(Boost 1.59.0 REQUIRED QUIET COMPONENTS atomic chrono date_time filesystem log log_setup program_options regex system thread)
list(APPEND PSMOVE_SERVICE_INCL_DIRS ${Boost_INCLUDE_DIRS})
list(APPEND PSMOVE_SERVICE_REQ_LIBS ${Boost_LIBRARIES})

#include_directories(${ROOT_DIR}/thirdparty/eigen)
#Boost/asio

# PSMoveDataFrame
list(APPEND PSMOVE_SERVICE_INCL_DIRS ${ROOT_DIR}/psmovedataframe/)
list(APPEND PSMOVE_SERVICE_REQ_LIBS PSMoveDataFrame)

# TODO: Camera. See tests

set(srv_srcs_hdrs
    ${CMAKE_CURRENT_LIST_DIR}/PSEye/PSEyeVideoCapture.h
    ${CMAKE_CURRENT_LIST_DIR}/PSEye/PSEyeVideoCapture.cpp
    ${CMAKE_CURRENT_LIST_DIR}/PSMoveController/PSMoveController.h
    ${CMAKE_CURRENT_LIST_DIR}/PSMoveController/PSMoveController.cpp
    PSMoveService.cpp
)

# Source files for PSMoveService
file(GLOB PSMOVESERVICE_SRC
    "${CMAKE_CURRENT_LIST_DIR}/*.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/*.h"
)
add_executable(PSMoveService ${PSMOVESERVICE_SRC})
target_include_directories(PSMoveService PUBLIC ${PSMOVE_SERVICE_INCL_DIRS})
target_link_libraries(PSMoveService ${PSMOVE_SERVICE_REQ_LIBS})
