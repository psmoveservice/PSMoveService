cmake_minimum_required(VERSION 3.0)
project(PSMoveService VERSION 0.0.1)

LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

# Step into the subdirectories
add_subdirectory(psmovedataframe)
add_subdirectory(psmoveservice)
add_subdirectory(psmoveclient)
add_subdirectory(tests)