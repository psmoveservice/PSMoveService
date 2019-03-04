cmake_minimum_required(VERSION 3.0)

find_package(glm CONFIG REQUIRED)

# Source files that are needed for the object library
file(GLOB PSMOVE_MATH_LIBRARY_SRC
    "${CMAKE_CURRENT_LIST_DIR}/*.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/*.h"
)

# Object library
add_library(PSMoveMath OBJECT ${PSMOVE_MATH_LIBRARY_SRC})
# target_include_directories(PSMoveMath PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(PSMoveMath PRIVATE Eigen3::Eigen glm)

#MacOS OpenCV must be self-built, this links against older std, which is hidden
#Therefore the PSMoveConfigTool must be hidden
#Therefore this must be hidden.
IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    SET_TARGET_PROPERTIES(PSMoveMath
        PROPERTIES COMPILE_FLAGS "-fvisibility=hidden")
ENDIF()
