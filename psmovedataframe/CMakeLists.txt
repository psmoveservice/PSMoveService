cmake_minimum_required(VERSION 3.0)
set(ROOT_DIR ${CMAKE_CURRENT_LIST_DIR}/..)

set(PSMOVEDATAFRAME_LIBRARY_SRC)    # Container for source files
set(PSMOVEDATAFRAME_REQUIRED_LIBS)   # Container for library dependencies
set(PSMOVEDATAFRAME_INSTALL_TARGETS) # Container for build targets to be installed

include_directories(${ROOT_DIR}/thirdparty/protobuf/install/include)

link_directories(${ROOT_DIR}/thirdparty/protobuf/install/lib)
list(APPEND PSMOVEDATAFRAME_REQUIRED_LIBS libprotobuf)

# Source files that are needed for both the shared and static library
file(GLOB PSMOVEDATAFRAME_LIBRARY_SRC
    "${CMAKE_CURRENT_LIST_DIR}/*.cc"
    "${CMAKE_CURRENT_LIST_DIR}/*.cpp"
)

# Static library
add_library(PSMoveDataFrame STATIC ${PSMOVEDATAFRAME_LIBRARY_SRC})
target_link_libraries(PSMoveDataFrame ${PSMOVEDATAFRAME_REQUIRED_LIBS})
set_target_properties(PSMoveDataFrame PROPERTIES
    COMPILE_FLAGS -DBUILDING_STATIC_LIBRARY)
    
# For system-wide installation
if(NOT PSMOVEDATAFRAME_LIB_DEST)
    if(CMAKE_LIBRARY_ARCHITECTURE)
        set(PSMOVEDATAFRAME_LIB_DEST "lib/${CMAKE_LIBRARY_ARCHITECTURE}")
    else()
        if (${CMAKE_C_SIZEOF_DATA_PTR} EQUAL 8)
            set(PSMOVEDATAFRAME_LIB_DEST "lib64")
        else()
            set(PSMOVEDATAFRAME_LIB_DEST "lib")
        endif()
    endif()
endif()    
    
install(TARGETS ${PSMOVEDATAFRAME_INSTALL_TARGETS}
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION ${PSMOVEDATAFRAME_LIB_DEST}
    ARCHIVE DESTINATION ${PSMOVEDATAFRAME_LIB_DEST}
)
