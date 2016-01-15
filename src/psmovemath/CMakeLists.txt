cmake_minimum_required(VERSION 3.0)

set(ROOT_DIR ${CMAKE_CURRENT_LIST_DIR}/../..)
set(PSMOVE_MATH_INCL_DIRS)

list(APPEND PSMOVE_MATH_INCL_DIRS
    ${ROOT_DIR}/thirdparty/eigen/
    ${ROOT_DIR}/thirdparty/glm/)

# Source files that are needed for the static library
file(GLOB PSMOVE_MATH_LIBRARY_SRC
    "${CMAKE_CURRENT_LIST_DIR}/*.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/*.h"
)

# Static library
add_library(PSMoveMath STATIC ${PSMOVE_MATH_LIBRARY_SRC})
target_include_directories(PSMoveMath PUBLIC ${PSMOVE_MATH_INCL_DIRS} ${CMAKE_CURRENT_BINARY_DIR})
set_target_properties(PSMoveMath PROPERTIES
    COMPILE_FLAGS -DBUILDING_STATIC_LIBRARY)