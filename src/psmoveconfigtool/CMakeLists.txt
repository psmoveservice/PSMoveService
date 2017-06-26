set(ROOT_DIR ${CMAKE_CURRENT_LIST_DIR}/../..)
set(PSMOVECONFIGTOOL_INCL_DIRS)
set(PSMOVECONFIGTOOL_REQ_LIBS)

# Boost
find_package(Boost REQUIRED QUIET COMPONENTS date_time)
list(APPEND PSMOVECONFIGTOOL_INCL_DIRS ${Boost_INCLUDE_DIRS})
list(APPEND PSMOVECONFIGTOOL_REQ_LIBS ${Boost_LIBRARIES})

# platform independent include paths
list(APPEND PSMOVECONFIGTOOL_INCL_DIRS
    ${EIGEN3_INCLUDE_DIR}
    ${ROOT_DIR}/thirdparty/glm
    ${ROOT_DIR}/thirdparty/sparseicp
    ${ROOT_DIR}/thirdparty/stb
    ${ROOT_DIR}/thirdparty/imgui
    ${ROOT_DIR}/src/psmoveclient/
    ${ROOT_DIR}/src/psmovemath/
    ${ROOT_DIR}/src/psmoveprotocol/
    ${PROTOBUF_INCLUDE_DIRS})

# platform independent libraries
list(APPEND PSMOVECONFIGTOOL_REQ_LIBS 
    PSMoveClient_CAPI
    PSMoveMath
    PSMoveProtocol
    ${PROTOBUF_LIBRARIES})
    
# SDL/GL
list(APPEND PSMOVECONFIGTOOL_INCL_DIRS ${SDL_GL_INCLUDE_DIRS})
list(APPEND PSMOVECONFIGTOOL_REQ_LIBS ${SDL_GL_LIBS})

# OpenCV
list(APPEND PSMOVECONFIGTOOL_INCL_DIRS ${OpenCV_INCLUDE_DIRS})
list(APPEND PSMOVECONFIGTOOL_REQ_LIBS ${OpenCV_LIBS})

# Source files that are needed for the psmove config tool
file(GLOB PSMOVECONFIGTOOL_SRC
    "${CMAKE_CURRENT_LIST_DIR}/*.h"
    "${CMAKE_CURRENT_LIST_DIR}/*.cpp"
    "${ROOT_DIR}/thirdparty/imgui/*.cpp"
)

# PSMoveConfigTool
add_executable(PSMoveConfigTool ${PSMOVECONFIGTOOL_SRC})
target_include_directories(PSMoveConfigTool PUBLIC ${PSMOVECONFIGTOOL_INCL_DIRS})
target_link_libraries(PSMoveConfigTool ${PSMOVECONFIGTOOL_REQ_LIBS})

IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
add_dependencies(PSMoveService opencv)
ENDIF()

# Allow access to the protocol buffers (extracting response parameters)
target_compile_definitions(PSMoveConfigTool
    PRIVATE HAS_PROTOCOL_ACCESS)

#MacOS OpenCV must be self-built, this links against older std, which is hidden
#Therefore the PSMoveConfigTool must be hidden
IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    SET_TARGET_PROPERTIES(PSMoveConfigTool
        PROPERTIES COMPILE_FLAGS "-fvisibility=hidden")
ENDIF()

# Post build dependencies (assets and external DLLs)
add_custom_command(TARGET PSMoveConfigTool POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_CURRENT_LIST_DIR}/assets"
        $<TARGET_FILE_DIR:PSMoveConfigTool>/assets)        

# Install    
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")   
    install(TARGETS PSMoveConfigTool
        RUNTIME DESTINATION ${ROOT_DIR}/${PSM_PROJECT_NAME}/${ARCH_LABEL}/bin
        LIBRARY DESTINATION ${ROOT_DIR}/${PSM_PROJECT_NAME}/${ARCH_LABEL}/lib
        ARCHIVE DESTINATION ${ROOT_DIR}/${PSM_PROJECT_NAME}/${ARCH_LABEL}/lib)
    install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/assets/ 
            DESTINATION ${ROOT_DIR}/${PSM_PROJECT_NAME}/${ARCH_LABEL}/bin/assets
            FILES_MATCHING PATTERN "*.ttf"  PATTERN "*.jpg")
    install(DIRECTORY ${OPENVR_BINARIES_DIR}/ 
            DESTINATION ${ROOT_DIR}/${PSM_PROJECT_NAME}/${ARCH_LABEL}/bin
            FILES_MATCHING PATTERN "*.dll" PATTERN "*.pdb")        
ELSE() #Linux/Darwin
ENDIF()
