set(ROOT_DIR ${CMAKE_CURRENT_LIST_DIR}/../..)
set(PSMOVECONFIGTOOL_INCL_DIRS)
set(PSMOVECONFIGTOOL_REQ_LIBS)

# platform independent include paths
list(APPEND PSMOVECONFIGTOOL_INCL_DIRS
    ${ROOT_DIR}/thirdparty/glm
    ${ROOT_DIR}/thirdparty/stb
    ${ROOT_DIR}/thirdparty/imgui
    ${ROOT_DIR}/src/psmoveclient/
    ${ROOT_DIR}/src/psmoveprotocol/
    ${PROTOBUF_INCLUDE_DIRS})

# platform independent libraries
list(APPEND PSMOVECONFIGTOOL_REQ_LIBS 
    PSMoveClient
    PSMoveProtocol
    ${PROTOBUF_LIBRARIES})
   
# platform specific libraries/include paths (OpenGL/SDL)    
IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    find_library(OPENGL_FRAMEWORK OpenGL)
    FIND_PACKAGE(SDL2)
    list(APPEND PSMOVECONFIGTOOL_INCL_DIRS ${SDL2_INCLUDE_DIR})
    list(APPEND PSMOVECONFIGTOOL_REQ_LIBS
        ${SDL2_LIBRARY} ${OPENGL_FRAMEWORK} ${GLUT_FRAMEWORK})
ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    list(APPEND PSMOVECONFIGTOOL_REQ_LIBS SDL GL)
ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    SET(ENV{SDL2DIR} ${ROOT_DIR}/thirdparty/SDL2/)
    FIND_PACKAGE(SDL2)
    list(APPEND PSMOVECONFIGTOOL_INCL_DIRS ${SDL2_INCLUDE_DIR})
    list(APPEND PSMOVECONFIGTOOL_REQ_LIBS 
        ${SDL2_LIBRARY}
        imm32.lib
        version.lib)
ENDIF()

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

# Allow access to the protocol buffers (extracting response parameters)
target_compile_definitions(PSMoveConfigTool PRIVATE HAS_PROTOCOL_ACCESS)

# Post build dependencies (assets and external DLLs)
add_custom_command(TARGET PSMoveConfigTool POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_CURRENT_LIST_DIR}/assets"
        $<TARGET_FILE_DIR:PSMoveConfigTool>/assets)

# Install    
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
install(TARGETS PSMoveConfigTool
    RUNTIME DESTINATION ${ROOT_DIR}/win32/bin
    LIBRARY DESTINATION ${ROOT_DIR}/win32/lib
    ARCHIVE DESTINATION ${ROOT_DIR}/win32/lib)
install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/assets/ 
        DESTINATION ${ROOT_DIR}/win32/bin/assets
        FILES_MATCHING PATTERN "*.ttf"  PATTERN "*.jpg")
ELSE() #Linux/Darwin
ENDIF()
