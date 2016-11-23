set(ROOT_DIR ${CMAKE_CURRENT_LIST_DIR}/../..)

set(PSMOVEPROTOCOL_LIBRARY_SRC)    # Container for source files
set(PSMOVEPROTOCOL_INCLUDE_DIRS)   # Container for include directories
set(PSMOVEPROTOCOL_REQUIRED_LIBS)  # Container for library dependencies

# Dependencies

# Protobuf
list(APPEND PSMOVEPROTOCOL_INCLUDE_DIRS ${PROTOBUF_INCLUDE_DIRS})
list(APPEND PSMOVEPROTOCOL_REQUIRED_LIBS ${PROTOBUF_LIBRARIES})
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${CMAKE_CURRENT_LIST_DIR}/PSMoveProtocol.proto)
#See http://stackoverflow.com/questions/20824194/cmake-with-google-protocol-buffers

# Source files and headers
file(GLOB PSMOVEPROTOCOL_LIBRARY_SRC
    "${CMAKE_CURRENT_LIST_DIR}/*.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/*.h"
)

list(APPEND PSMOVEPROTOCOL_LIBRARY_SRC ${PROTO_SRCS} ${PROTO_HDRS})

# Static library
add_library(PSMoveProtocol STATIC ${PSMOVEPROTOCOL_LIBRARY_SRC})
target_include_directories(PSMoveProtocol PUBLIC ${PSMOVEPROTOCOL_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(PSMoveProtocol ${PSMOVEPROTOCOL_REQUIRED_LIBS})
set_target_properties(PSMoveProtocol PROPERTIES
    COMPILE_FLAGS "-DBUILDING_STATIC_LIBRARY -fPIC")
