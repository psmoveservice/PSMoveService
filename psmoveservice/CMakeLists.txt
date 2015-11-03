cmake_minimum_required(VERSION 3.0)

# Dependencies
set(ROOT_DIR ${CMAKE_CURRENT_LIST_DIR}/..)
set(PSMOVE_SERVICE_REQ_LIBS)

include_directories(${ROOT_DIR}/psmovedataframe/)
include_directories(${ROOT_DIR}/thirdparty/Boost.Application/include/)
include_directories(${ROOT_DIR}/thirdparty/type_index/include/)
include_directories(${ROOT_DIR}/thirdparty/protobuf/install/include)

link_directories(${ROOT_DIR}/thirdparty/protobuf/install/lib)

set(Boost_USE_STATIC_LIBS        ON) # only find static libs
set(Boost_USE_MULTITHREADED      ON)
set(Boost_USE_STATIC_RUNTIME     ON)
find_package(Boost 1.59.0 REQUIRED COMPONENTS atomic chrono date_time filesystem program_options regex system thread)
include_directories(${Boost_INCLUDE_DIRS})

list(APPEND PSMOVE_SERVICE_REQ_LIBS ${Boost_LIBRARIES})
list(APPEND PSMOVE_SERVICE_REQ_LIBS PSMoveDataFrame)

# TODO: Camera. See tests

#include_directories(${ROOT_DIR}/thirdparty/eigen)
#Boost
#Boost/asio
#Boost/application
#Boost/type_index
#protobuf

set(srv_srcs_hdrs
    ${CMAKE_CURRENT_LIST_DIR}/PSEye/PSEyeVideoCapture.h
    ${CMAKE_CURRENT_LIST_DIR}/PSEye/PSEyeVideoCapture.cpp
    ${CMAKE_CURRENT_LIST_DIR}/PSMoveController/PSMoveController.h
    ${CMAKE_CURRENT_LIST_DIR}/PSMoveController/PSMoveController.cpp
    PSMoveService.cpp
)

#add_executable(PSMoveService ${srv_srcs_hdrs})
#target_link_libraries(PSMoveService PSMoveDataFrame)

IF(MSVC)
	SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT")
	SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MTd")
ENDIF(MSVC)

# Source files that are needed for both the shared and static library
file(GLOB PSMOVESERVICE_SRC
    "${CMAKE_CURRENT_LIST_DIR}/*.cpp"
)
add_executable(PSMoveService ${PSMOVESERVICE_SRC})

target_link_libraries(PSMoveService ${PSMOVE_SERVICE_REQ_LIBS})
target_link_libraries(PSMoveService debug libprotobufd)
target_link_libraries(PSMoveService optimized libprotobuf)
