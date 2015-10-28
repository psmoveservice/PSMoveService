cmake_minimum_required(VERSION 3.0)

set(ROOT_DIR ${CMAKE_CURRENT_LIST_DIR}/..)
set(PSMOVE_CLIENT_REQ_LIBS)

include_directories(${ROOT_DIR}/psmovedataframe/)
include_directories(${ROOT_DIR}/thirdparty/Boost.Application/include/)
include_directories(${ROOT_DIR}/thirdparty/type_index/include/)

set(Boost_USE_STATIC_LIBS        ON) # only find static libs
set(Boost_USE_MULTITHREADED      ON)
set(Boost_USE_STATIC_RUNTIME     OFF)
find_package(Boost 1.59.0 REQUIRED COMPONENTS atomic chrono date_time filesystem program_options regex system thread)
include_directories(${Boost_INCLUDE_DIRS})
list(APPEND PSMOVE_CLIENT_REQ_LIBS ${Boost_LIBRARIES})

#IF(MSVC)
#	SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_DEBUG} /MT")
#	SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MTd")
#ENDIF(MSVC)

add_executable(PSMoveConsoleClient PSMoveConsoleClient.cpp)
target_link_libraries(PSMoveConsoleClient ${PSMOVE_CLIENT_REQ_LIBS})