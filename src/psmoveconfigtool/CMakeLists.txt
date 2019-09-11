set(ROOT_DIR ${CMAKE_CURRENT_LIST_DIR}/../..)
set(PSMOVECONFIGTOOL_INCL_DIRS)
set(PSMOVECONFIGTOOL_REQ_LIBS)
# Source files that are needed for the psmove config tool
file(GLOB PSMOVECONFIGTOOL_SRC
        "${CMAKE_CURRENT_LIST_DIR}/*.h"
        "${CMAKE_CURRENT_LIST_DIR}/*.cpp"
)

# Boost
find_package(Boost REQUIRED QUIET COMPONENTS date_time)
list(APPEND PSMOVECONFIGTOOL_INCL_DIRS ${Boost_INCLUDE_DIRS})
list(APPEND PSMOVECONFIGTOOL_REQ_LIBS ${Boost_LIBRARIES})

# platform independent include paths
list(APPEND PSMOVECONFIGTOOL_INCL_DIRS
    ${ROOT_DIR}/thirdparty/sparseicp
    ${ROOT_DIR}/thirdparty/stb
    ${ROOT_DIR}/src/psmoveclient/
    ${ROOT_DIR}/src/psmovemath/
    ${ROOT_DIR}/src/psmoveprotocol/
)

# platform independent libraries
list(APPEND PSMOVECONFIGTOOL_REQ_LIBS 
    PSMoveClient_CAPI
    PSMoveMath
    PSMoveProtocol
    Eigen3::Eigen
    ${Protobuf_LIBRARIES})
list(APPEND PSMOVECONFIGTOOL_INCL_DIRS ${Protobuf_INCLUDE_DIRS})
    
# SDL/GL
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
	list(APPEND PSMOVECONFIGTOOL_REQ_LIBS SDL2::SDL2 SDL2::SDL2main)
ELSE()
	list(APPEND PSMOVECONFIGTOOL_REQ_LIBS ${SDL2_LIBRARIES})
ENDIF()
list(APPEND PSMOVECONFIGTOOL_INCL_DIRS ${SDL2_INCLUDE_DIRS})

# OpenCV
list(APPEND PSMOVECONFIGTOOL_INCL_DIRS ${OpenCV_INCLUDE_DIRS})
list(APPEND PSMOVECONFIGTOOL_REQ_LIBS ${OpenCV_LIBS})

#ImGui
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
	find_package(imgui CONFIG REQUIRED)
    list(APPEND PSMOVECONFIGTOOL_REQ_LIBS imgui::imgui)
ELSE()  #((${CMAKE_SYSTEM_NAME} MATCHES "Linux") OR (${CMAKE_SYSTEM_NAME} MATCHES "Darwin"))
    # No installable ImGui package on Linux/MacOS. Add headers and source manually. See build docs.
    list(APPEND PSMOVECONFIGTOOL_INCL_DIRS
        ${ROOT_DIR}/build_deps/imgui)
    file(GLOB IMGUI_SRC "${ROOT_DIR}/build_deps/imgui/*.cpp")
    list(APPEND PSMOVECONFIGTOOL_SRC ${IMGUI_SRC})
ENDIF()

# PSMoveConfigTool
add_executable(PSMoveConfigTool ${PSMOVECONFIGTOOL_SRC})
target_include_directories(PSMoveConfigTool PRIVATE ${PSMOVECONFIGTOOL_INCL_DIRS})
target_link_libraries(PSMoveConfigTool PRIVATE ${PSMOVECONFIGTOOL_REQ_LIBS})

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
        CONFIGURATIONS Debug
        RUNTIME DESTINATION ${PSM_DEBUG_INSTALL_PATH}/bin
        LIBRARY DESTINATION ${PSM_DEBUG_INSTALL_PATH}/lib
        ARCHIVE DESTINATION ${PSM_DEBUG_INSTALL_PATH}/lib)
    install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/assets/ 
            CONFIGURATIONS Debug
            DESTINATION ${PSM_DEBUG_INSTALL_PATH}/bin/assets
            FILES_MATCHING PATTERN "*.ttf"  PATTERN "*.jpg")
    install(DIRECTORY ${OPENVR_BINARIES_DIR}/ 
            CONFIGURATIONS Debug
            DESTINATION ${PSM_DEBUG_INSTALL_PATH}/bin
            FILES_MATCHING PATTERN "*.dll" PATTERN "*.pdb")   
    install(TARGETS PSMoveConfigTool
        CONFIGURATIONS Release
        RUNTIME DESTINATION ${PSM_RELEASE_INSTALL_PATH}/bin
        LIBRARY DESTINATION ${PSM_RELEASE_INSTALL_PATH}/lib
        ARCHIVE DESTINATION ${PSM_RELEASE_INSTALL_PATH}/lib)
    install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/assets/ 
            CONFIGURATIONS Release
            DESTINATION ${PSM_RELEASE_INSTALL_PATH}/bin/assets
            FILES_MATCHING PATTERN "*.ttf"  PATTERN "*.jpg")
    install(DIRECTORY ${OPENVR_BINARIES_DIR}/ 
            CONFIGURATIONS Release
            DESTINATION ${PSM_RELEASE_INSTALL_PATH}/bin
            FILES_MATCHING PATTERN "*.dll" PATTERN "*.pdb")   
ELSE() #Linux/Darwin
ENDIF()
