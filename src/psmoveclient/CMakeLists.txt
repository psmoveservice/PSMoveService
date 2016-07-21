cmake_minimum_required(VERSION 3.0)

set(ROOT_DIR ${CMAKE_CURRENT_LIST_DIR}/../..)
set(PSMOVE_CLIENT_INCL_DIRS)
set(PSMOVE_CLIENT_REQ_LIBS)

list(APPEND PSMOVE_CLIENT_INCL_DIRS
    ${ROOT_DIR}/thirdparty/Boost.Application/include/
    ${ROOT_DIR}/thirdparty/type_index/include/)

# Protobuf
list(APPEND PSMOVE_CLIENT_INCL_DIRS ${PROTOBUF_INCLUDE_DIRS})
list(APPEND PSMOVE_CLIENT_REQ_LIBS ${PROTOBUF_LIBRARIES})

# Boost - TODO: Trim this list
find_package(Boost 1.59.0 REQUIRED QUIET COMPONENTS system)
list(APPEND PSMOVE_CLIENT_INCL_DIRS ${Boost_INCLUDE_DIRS})
list(APPEND PSMOVE_CLIENT_REQ_LIBS ${Boost_LIBRARIES})
IF(MSVC) # Disable asio auto linking in date-time and regex
add_definitions(-DBOOST_DATE_TIME_NO_LIB)
add_definitions(-DBOOST_REGEX_NO_LIB)
ENDIF()

# PSMoveProtocol
include_directories(${ROOT_DIR}/src/psmoveprotocol/)
list(APPEND PSMOVE_CLIENT_REQ_LIBS PSMoveProtocol)

# PSMoveProtocol
include_directories(${ROOT_DIR}/src/psmovemath/)
list(APPEND PSMOVE_CLIENT_REQ_LIBS PSMoveMath)

# Source files that are needed for the shared library
file(GLOB PSMOVECLIENT_LIBRARY_SRC
    "${CMAKE_CURRENT_LIST_DIR}/*.h"
    "${CMAKE_CURRENT_LIST_DIR}/*.cpp"
)

# Shared library
add_library(PSMoveClient SHARED ${PSMOVECLIENT_LIBRARY_SRC})
target_include_directories(PSMoveClient PUBLIC ${PSMOVE_CLIENT_INCL_DIRS})
target_link_libraries(PSMoveClient ${PSMOVE_CLIENT_REQ_LIBS})
set_target_properties(PSMoveClient PROPERTIES
    COMPILE_FLAGS -DBUILDING_SHARED_PSMOVECLIENT_LIBRARY)

# Install    
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    install(TARGETS PSMoveClient
        RUNTIME DESTINATION ${ROOT_DIR}/${ARCH_LABEL}/bin
        LIBRARY DESTINATION ${ROOT_DIR}/${ARCH_LABEL}/lib
        ARCHIVE DESTINATION ${ROOT_DIR}/${ARCH_LABEL}/lib)
ELSE() #Linux/Darwin
ENDIF()
