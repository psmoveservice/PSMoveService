cmake_minimum_required(VERSION 3.0)

set(ROOT_DIR ${CMAKE_CURRENT_LIST_DIR}/..)
set(PSMOVE_CLIENT_INCL_DIRS)
set(PSMOVE_CLIENT_REQ_LIBS)

list(APPEND PSMOVE_CLIENT_INCL_DIRS
    ${ROOT_DIR}/thirdparty/Boost.Application/include/
    ${ROOT_DIR}/thirdparty/type_index/include/)

# Protobuf
list(APPEND PSMOVE_CLIENT_INCL_DIRS ${PROTOBUF_INCLUDE_DIRS})
list(APPEND PSMOVE_CLIENT_REQ_LIBS ${PROTOBUF_LIBRARIES})

# Boost - TODO: Trim this list
find_package(Boost 1.59.0 REQUIRED QUIET COMPONENTS date_time regex system)
list(APPEND PSMOVE_CLIENT_INCL_DIRS ${Boost_INCLUDE_DIRS})
list(APPEND PSMOVE_CLIENT_REQ_LIBS ${Boost_LIBRARIES})

# PSMoveProtocol
include_directories(${ROOT_DIR}/psmoveprotocol/)
list(APPEND PSMOVE_CLIENT_REQ_LIBS PSMoveProtocol)

# Source files that are needed for the shared library
file(GLOB PSMOVECLIENT_LIBRARY_SRC
    "${CMAKE_CURRENT_LIST_DIR}/*.cpp"
)

# Shared library
add_library(PSMoveClient SHARED ${PSMOVECLIENT_LIBRARY_SRC})
target_include_directories(PSMoveClient PUBLIC ${PSMOVE_CLIENT_INCL_DIRS})
target_link_libraries(PSMoveClient ${PSMOVE_CLIENT_REQ_LIBS})
set_target_properties(PSMoveClient PROPERTIES
    COMPILE_FLAGS -DBUILDING_SHARED_LIBRARY)

# Copy the build to the bin folder    
add_custom_command(TARGET PSMoveClient POST_BUILD
COMMAND ${CMAKE_COMMAND} -E copy_directory  
    "$<TARGET_FILE_DIR:PSMoveClient>" 
    "${ROOT_DIR}/bin")
