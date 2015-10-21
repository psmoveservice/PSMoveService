cmake_minimum_required(VERSION 3.0)

# Dependencies

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