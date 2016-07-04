cmake_minimum_required(VERSION 3.0)

set(ROOT_DIR ${CMAKE_CURRENT_LIST_DIR}/../..)
set(OPENVR_PLUGIN_INCL_DIRS)
set(OPENVR_PLUGIN_REQ_LIBS)

# platform independent include paths
list(APPEND OPENVR_PLUGIN_INCL_DIRS
    ${ROOT_DIR}/src/psmoveclient/
    ${PROTOBUF_INCLUDE_DIRS})

#OpenVR
FIND_PACKAGE(OpenVR REQUIRED)
list(APPEND OPENVR_PLUGIN_INCL_DIRS ${OPENVR_INCLUDE_DIR})
list(APPEND OPENVR_PLUGIN_REQ_LIBS ${OPENVR_LIBRARIES})    
    
# platform independent libraries
list(APPEND OPENVR_PLUGIN_REQ_LIBS 
    PSMoveClient)

# Source files that are needed for the shared library
file(GLOB OPENVR_PLUGIN_LIBRARY_SRC
    "${CMAKE_CURRENT_LIST_DIR}/*.h"
    "${CMAKE_CURRENT_LIST_DIR}/*.cpp"
)

# Shared library
add_library(driver_psmove SHARED ${OPENVR_PLUGIN_LIBRARY_SRC})
target_include_directories(driver_psmove PUBLIC ${OPENVR_PLUGIN_INCL_DIRS})
target_link_libraries(driver_psmove ${OPENVR_PLUGIN_REQ_LIBS})
target_compile_definitions(driver_psmove
	PRIVATE PSMOVECLIENT_CPP_API)

# Install    
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
install(TARGETS driver_psmove
	RUNTIME DESTINATION ${ROOT_DIR}/${ARCH_LABEL}/bin
	LIBRARY DESTINATION ${ROOT_DIR}/${ARCH_LABEL}/lib
	ARCHIVE DESTINATION ${ROOT_DIR}/${ARCH_LABEL}/lib)
install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/
    DESTINATION ${ROOT_DIR}/${ARCH_LABEL}/bin
    FILES_MATCHING PATTERN "*${ARCH_LABEL}.bat")
install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/resources
    DESTINATION ${ROOT_DIR}/${ARCH_LABEL}/bin
    FILES_MATCHING PATTERN "*.png" PATTERN "*.json" PATTERN "*.obj" PATTERN "*.mtl")
ELSE() #Linux/Darwin
ENDIF()
