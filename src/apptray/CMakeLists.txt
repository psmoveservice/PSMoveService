cmake_minimum_required(VERSION 3.0)

set(ROOT_DIR ${CMAKE_CURRENT_LIST_DIR}/../..)
set(APPTRAY_INCL_DIRS)
set(APPTRAY_REQ_LIBS)
 
file(GLOB APPTRAY_SRC
    "${CMAKE_CURRENT_LIST_DIR}/psmove_apptray.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/resource.h"
    "${CMAKE_CURRENT_LIST_DIR}/psmove_apptray.rc"
)

# PSMove apptray
add_executable(psmove_apptray WIN32 ${APPTRAY_SRC})
target_include_directories(psmove_apptray PUBLIC ${APPTRAY_INCL_DIRS})

# Install    
install(TARGETS psmove_apptray
	RUNTIME DESTINATION ${ROOT_DIR}/${ARCH_LABEL}/bin
	LIBRARY DESTINATION ${ROOT_DIR}/${ARCH_LABEL}/lib
	ARCHIVE DESTINATION ${ROOT_DIR}/${ARCH_LABEL}/lib)