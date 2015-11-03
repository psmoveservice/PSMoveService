cmake_minimum_required(VERSION 3.0)
set(ROOT_DIR ${CMAKE_CURRENT_LIST_DIR}/..)

set(PSMOVEDATAFRAME_LIBRARY_SRC)    # Container for source files
set(PSMOVEDATAFRAME_REQUIRED_DEBUG_LIBS)   # Container for library dependencies
set(PSMOVEDATAFRAME_REQUIRED_OPTIMIZED_LIBS)   # Container for library dependencies

include_directories(${ROOT_DIR}/thirdparty/protobuf/install/include)

link_directories(${ROOT_DIR}/thirdparty/protobuf/install/lib)
list(APPEND PSMOVEDATAFRAME_REQUIRED_DEBUG_LIBS libprotobufd)
list(APPEND PSMOVEDATAFRAME_REQUIRED_OPTIMIZED_LIBS libprotobuf)

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
target_link_libraries(PSMoveDataFrame debug ${PSMOVEDATAFRAME_REQUIRED_DEBUG_LIBS})
target_link_libraries(PSMoveDataFrame optimized ${PSMOVEDATAFRAME_REQUIRED_OPTIMIZED_LIBS})
set_target_properties(PSMoveDataFrame PROPERTIES
    COMPILE_FLAGS -DBUILDING_STATIC_LIBRARY)
