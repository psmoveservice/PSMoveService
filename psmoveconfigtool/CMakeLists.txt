cmake_minimum_required(VERSION 3.0)

set(ROOT_DIR ${CMAKE_CURRENT_LIST_DIR}/..)
set(PSMOVECONFIGTOOL_INCL_DIRS)
set(PSMOVECONFIGTOOL_REQ_LIBS)

# platform independent include paths
list(APPEND PSMOVECONFIGTOOL_INCL_DIRS
    ${ROOT_DIR}/thirdparty/glm
    ${ROOT_DIR}/thirdparty/stb
    ${ROOT_DIR}/thirdparty/imgui
    ${ROOT_DIR}/psmoveclient/)

# platform independent libraries
list(APPEND PSMOVECONFIGTOOL_REQ_LIBS PSMoveClient)
   
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
    list(APPEND PSMOVECONFIGTOOL_REQ_LIBS ${SDL2_LIBRARY})
    list(GET SDL2_LIBRARY 0 FIRST_SDL2_LIB)
    get_filename_component(SDL_BIN_PATH ${FIRST_SDL2_LIB} DIRECTORY)
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

# Install    
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
install(TARGETS PSMoveConfigTool
	RUNTIME DESTINATION ${ROOT_DIR}/win32/bin
	LIBRARY DESTINATION ${ROOT_DIR}/win32/lib
	ARCHIVE DESTINATION ${ROOT_DIR}/win32/lib)
ELSE() #Linux/Darwin
ENDIF()

add_custom_command(TARGET PSMoveConfigTool POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_CURRENT_LIST_DIR}/visual_setup/assets"
        $<TARGET_FILE_DIR:PSMoveConfigTool>/assets)
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    add_custom_command(TARGET PSMoveConfigTool POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${SDL_BIN_PATH}/SDL2.dll"
            $<TARGET_FILE_DIR:PSMoveConfigTool>)
ENDIF()
