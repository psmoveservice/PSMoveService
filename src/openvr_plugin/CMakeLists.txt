cmake_minimum_required(VERSION 3.0)

set(ROOT_DIR ${CMAKE_CURRENT_LIST_DIR}/../..)
set(OPENVR_PLUGIN_INCL_DIRS)
set(OPENVR_PLUGIN_REQ_LIBS)
set(OPENVR_MONITOR_INCL_DIRS)
set(OPENVR_MONITOR_REQ_LIBS)

# platform independent include paths
list(APPEND OPENVR_PLUGIN_INCL_DIRS
    ${ROOT_DIR}/src/psmoveclient/
    ${PROTOBUF_INCLUDE_DIRS})

#OpenVR
FIND_PACKAGE(OpenVR REQUIRED)
list(APPEND OPENVR_PLUGIN_INCL_DIRS ${OPENVR_INCLUDE_DIR})
list(APPEND OPENVR_PLUGIN_REQ_LIBS ${OPENVR_LIBRARIES})    
list(APPEND OPENVR_MONITOR_INCL_DIRS ${OPENVR_INCLUDE_DIR})
list(APPEND OPENVR_MONITOR_REQ_LIBS ${OPENVR_LIBRARIES})   
    
# platform independent libraries
list(APPEND OPENVR_PLUGIN_REQ_LIBS 
    PSMoveClient)

# Shared library
add_library(driver_psmove SHARED driver_psmoveservice.cpp)
target_include_directories(driver_psmove PUBLIC ${OPENVR_PLUGIN_INCL_DIRS})
target_link_libraries(driver_psmove ${OPENVR_PLUGIN_REQ_LIBS})

# Monitor app
add_executable(monitor_psmove monitor_psmoveservice.cpp)
target_include_directories(monitor_psmove PUBLIC ${OPENVR_MONITOR_INCL_DIRS})
target_link_libraries(monitor_psmove ${OPENVR_LIBRARIES})

# Install    
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
install(TARGETS driver_psmove
	RUNTIME DESTINATION ${ROOT_DIR}/${ARCH_LABEL}/bin
	LIBRARY DESTINATION ${ROOT_DIR}/${ARCH_LABEL}/lib
	ARCHIVE DESTINATION ${ROOT_DIR}/${ARCH_LABEL}/lib)
install(TARGETS monitor_psmove
	RUNTIME DESTINATION ${ROOT_DIR}/${ARCH_LABEL}/bin
	LIBRARY DESTINATION ${ROOT_DIR}/${ARCH_LABEL}/lib
	ARCHIVE DESTINATION ${ROOT_DIR}/${ARCH_LABEL}/lib)    
install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/
    DESTINATION ${ROOT_DIR}/${ARCH_LABEL}/bin
    FILES_MATCHING PATTERN "*${ARCH_LABEL}.bat")
install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/resources
    DESTINATION ${ROOT_DIR}/${ARCH_LABEL}/bin
    FILES_MATCHING PATTERN "*.png" PATTERN "*.tga" PATTERN "*.json" PATTERN "*.obj" PATTERN "*.mtl")
ELSE() #Linux/Darwin
ENDIF()
