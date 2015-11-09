cmake_minimum_required(VERSION 3.0)
set(ROOT_DIR ${CMAKE_CURRENT_LIST_DIR}/..)

set(PSMOVEDATAFRAME_LIBRARY_SRC)    # Container for source files
set(PSMOVEDATAFRAME_INCLUDE_DIRS)   # Container for include directories
set(PSMOVEDATAFRAME_REQUIRED_LIBS)  # Container for library dependencies

# Dependencies

# Protobuf
list(APPEND PSMOVEDATAFRAME_INCLUDE_DIRS ${PROTOBUF_INCLUDE_DIRS})
list(APPEND PSMOVEDATAFRAME_REQUIRED_LIBS ${PROTOBUF_LIBRARIES})
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${CMAKE_CURRENT_LIST_DIR}/PSMoveDataFrame.proto)
#See http://stackoverflow.com/questions/20824194/cmake-with-google-protocol-buffers

# Boost
find_package(Boost 1.59.0 REQUIRED QUIET COMPONENTS atomic chrono date_time filesystem program_options regex system thread)
list(APPEND PSMOVEDATAFRAME_INCLUDE_DIRS ${Boost_INCLUDE_DIRS})
#TODO: Why no "list(APPEND PSMOVEDATAFRAME_REQUIRED_LIBS ${Boost_LIBRARIES})" ??
#If they are header-only modules, then the list of required components should be trimmed.

# Source files and headers
file(GLOB PSMOVEDATAFRAME_LIBRARY_SRC
    "${CMAKE_CURRENT_LIST_DIR}/*.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/*.h"
)

list(APPEND PSMOVEDATAFRAME_LIBRARY_SRC ${PROTO_SRCS} ${PROTO_HDRS})

# Static library
add_library(PSMoveDataFrame STATIC ${PSMOVEDATAFRAME_LIBRARY_SRC})
target_include_directories(PSMoveDataFrame PUBLIC ${PSMOVEDATAFRAME_INCLUDE_DIRS})
target_link_libraries(PSMoveDataFrame ${PSMOVEDATAFRAME_REQUIRED_LIBS})
set_target_properties(PSMoveDataFrame PROPERTIES
    COMPILE_FLAGS -DBUILDING_STATIC_LIBRARY)
