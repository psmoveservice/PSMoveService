cmake_minimum_required(VERSION 3.0)

set(ROOT_DIR ${CMAKE_CURRENT_LIST_DIR}/..)
set(PSMOVE_CLIENT_INCL_DIRS)
set(PSMOVE_CLIENT_REQ_LIBS)

list(APPEND PSMOVE_CLIENT_INCL_DIRS
    ${ROOT_DIR}/thirdparty/Boost.Application/include/
    ${ROOT_DIR}/thirdparty/type_index/include/)

# Protobuf
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    list(APPEND PSMOVE_CLIENT_INCL_DIRS ${ROOT_DIR}/thirdparty/protobuf/install/include)
    link_directories(${ROOT_DIR}/thirdparty/protobuf/install/lib)
    list(APPEND PSMOVE_CLIENT_REQ_LIBS debug libprotobufd optimized libprotobuf)
ELSE()
    find_package(Protobuf REQUIRED)
    list(APPEND PSMOVE_CLIENT_INCL_DIRS ${PROTOBUF_INCLUDE_DIRS})
    list(APPEND PSMOVE_CLIENT_REQ_LIBS ${PROTOBUF_LIBRARIES})
ENDIF()

set(Boost_USE_STATIC_LIBS        ON) # only find static libs
set(Boost_USE_MULTITHREADED      ON)
set(Boost_USE_STATIC_RUNTIME     ON)
find_package(Boost 1.59.0 REQUIRED COMPONENTS atomic chrono date_time filesystem program_options regex system thread)
list(APPEND PSMOVE_CLIENT_INCL_DIRS ${Boost_INCLUDE_DIRS})
list(APPEND PSMOVE_CLIENT_REQ_LIBS ${Boost_LIBRARIES})

# PSMoveDataFrame
include_directories(${ROOT_DIR}/psmovedataframe/)
list(APPEND PSMOVE_CLIENT_REQ_LIBS PSMoveDataFrame)

IF(MSVC)
    SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT")
    SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MTd")
ENDIF(MSVC)

# Source files that are needed for both the shared and static library
file(GLOB PSMOVECLIENT_LIBRARY_SRC
    "${CMAKE_CURRENT_LIST_DIR}/*.cpp"
)

# Static library
add_library(PSMoveClient STATIC ${PSMOVECLIENT_LIBRARY_SRC})
target_include_directories(PSMoveClient PUBLIC ${PSMOVE_CLIENT_INCL_DIRS})
target_link_libraries(PSMoveClient ${PSMOVE_CLIENT_REQ_LIBS})
set_target_properties(PSMoveClient PROPERTIES
    COMPILE_FLAGS -DBUILDING_STATIC_LIBRARY)    