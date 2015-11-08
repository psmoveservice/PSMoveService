cmake_minimum_required(VERSION 3.0)
set(ROOT_DIR ${CMAKE_CURRENT_LIST_DIR}/..)

set(PSMOVEDATAFRAME_LIBRARY_SRC)    # Container for source files
set(PSMOVEDATAFRAME_INCLUDE_DIRS)   # Container for include directories
set(PSMOVEDATAFRAME_REQUIRED_LIBS)  # Container for library dependencies

# Dependencies

# Protobuf
#TODO: Try settings PROTOBUF_SRC_ROOT_FOLDER to something and use find_package in Windows too.
#According to FindProtobuf.cmake, it looks in ${PROTOBUF_SRC_ROOT_FOLDER}/vsprojects/Release and Debug
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    list(APPEND PSMOVEDATAFRAME_INCLUDE_DIRS ${ROOT_DIR}/thirdparty/protobuf/install/include)
    link_directories(${ROOT_DIR}/thirdparty/protobuf/install/lib)
    list(APPEND PSMOVEDATAFRAME_REQUIRED_LIBS debug libprotobufd optimized libprotobuf)
ELSE()
    set(PROTOBUF_ORIG_FIND_LIBRARY_SUFFIXES "${CMAKE_FIND_LIBRARY_SUFFIXES}")
    set(CMAKE_FIND_LIBRARY_SUFFIXES .a .lib .so .dylib .dll)
    find_package(Protobuf REQUIRED)
    list(APPEND PSMOVEDATAFRAME_INCLUDE_DIRS ${PROTOBUF_INCLUDE_DIRS})
    list(APPEND PSMOVEDATAFRAME_REQUIRED_LIBS ${PROTOBUF_LIBRARIES})
    set(CMAKE_FIND_LIBRARY_SUFFIXES "${PROTOBUF_ORIG_FIND_LIBRARY_SUFFIXES}")
ENDIF()
#TODO: Do we need to make use of PROTOBUF_IMPORT_DIRS ?
#TODO: Do we need to make use of
#protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS foo.proto)
#then
#add_library(PSMoveDataFrame STATIC ${PSMOVEDATAFRAME_LIBRARY_SRC} ${PROTO_SRCS} ${PROTO_HDRS})
#See http://stackoverflow.com/questions/20824194/cmake-with-google-protocol-buffers

# Boost
set(Boost_USE_STATIC_LIBS        ON) # only find static libs
set(Boost_USE_MULTITHREADED      ON)
set(Boost_USE_STATIC_RUNTIME     ON)
find_package(Boost 1.59.0 REQUIRED COMPONENTS atomic chrono date_time filesystem program_options regex system thread)
list(APPEND PSMOVEDATAFRAME_INCLUDE_DIRS ${Boost_INCLUDE_DIRS})
#TODO: Why no "list(APPEND PSMOVEDATAFRAME_REQUIRED_LIBS ${Boost_LIBRARIES})" ??
#If they are header-only modules, then the list of required components should be trimmed.

# Source files that are needed for both the shared and static library
file(GLOB PSMOVEDATAFRAME_LIBRARY_SRC
    "${CMAKE_CURRENT_LIST_DIR}/*.cc"
    "${CMAKE_CURRENT_LIST_DIR}/*.cpp"
)

IF(MSVC)
	SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT")
	SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MTd")
ENDIF(MSVC)

# Static library
add_library(PSMoveDataFrame STATIC ${PSMOVEDATAFRAME_LIBRARY_SRC})
target_include_directories(PSMoveDataFrame PUBLIC ${PSMOVEDATAFRAME_INCLUDE_DIRS})
target_link_libraries(PSMoveDataFrame ${PSMOVEDATAFRAME_REQUIRED_LIBS})
set_target_properties(PSMoveDataFrame PROPERTIES
    COMPILE_FLAGS -DBUILDING_STATIC_LIBRARY)
