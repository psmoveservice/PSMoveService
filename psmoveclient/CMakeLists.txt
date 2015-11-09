cmake_minimum_required(VERSION 3.0)

set(ROOT_DIR ${CMAKE_CURRENT_LIST_DIR}/..)
set(PSMOVE_CLIENT_INCL_DIRS)
set(PSMOVE_CLIENT_REQ_LIBS)

list(APPEND PSMOVE_CLIENT_INCL_DIRS
    ${ROOT_DIR}/thirdparty/Boost.Application/include/
    ${ROOT_DIR}/thirdparty/type_index/include/)

# Protobuf
list(APPEND PSMOVE_CLIENT_INCL_DIRS ${PROTOBUF_INCLUDE_DIRS})
list(APPEND PSMOVE_CLIENT_REQ_LIBS ${PROTOBUF_LIBRARIES})

# Boost - TODO: Trim this list
find_package(Boost 1.59.0 REQUIRED QUIET COMPONENTS atomic chrono date_time filesystem program_options regex system thread)
list(APPEND PSMOVE_CLIENT_INCL_DIRS ${Boost_INCLUDE_DIRS})
list(APPEND PSMOVE_CLIENT_REQ_LIBS ${Boost_LIBRARIES})

# PSMoveDataFrame
include_directories(${ROOT_DIR}/psmovedataframe/)
list(APPEND PSMOVE_CLIENT_REQ_LIBS PSMoveDataFrame)

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