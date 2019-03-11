#
# TEST_CAMERA and TEST_CAMERA_PARALLEL
#

SET(TEST_CAMERA_SRC)
SET(TEST_CAMERA_INCL_DIRS)
SET(TEST_CAMERA_REQ_LIBS)

# Boost
FIND_PACKAGE(Boost REQUIRED QUIET COMPONENTS atomic)
list(APPEND TEST_CAMERA_INCL_DIRS ${Boost_INCLUDE_DIRS})
list(APPEND TEST_CAMERA_REQ_LIBS ${Boost_LIBRARIES})

# OpenCV
IF(MSVC) # not necessary for OpenCV > 2.8 on other build systems
    list(APPEND TEST_CAMERA_INCL_DIRS ${OpenCV_INCLUDE_DIRS}) 
ENDIF()
list(APPEND TEST_CAMERA_REQ_LIBS ${OpenCV_LIBS})

# PS3EYE
list(APPEND TEST_CAMERA_SRC ${PSEYE_SRC})
list(APPEND TEST_CAMERA_INCL_DIRS ${PSEYE_INCLUDE_DIRS})
list(APPEND TEST_CAMERA_REQ_LIBS ${PSEYE_LIBRARIES})
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows"
    AND NOT(${CMAKE_C_SIZEOF_DATA_PTR} EQUAL 8))
    # Windows utilities for querying driver infomation (provider name)
    list(APPEND TEST_CAMERA_INCL_DIRS ${ROOT_DIR}/src/psmoveservice/Device/Interface)
    list(APPEND TEST_CAMERA_INCL_DIRS ${ROOT_DIR}/src/psmoveservice/Server)
    list(APPEND TEST_CAMERA_INCL_DIRS ${ROOT_DIR}/src/psmoveservice/Platform)
    list(APPEND TEST_CAMERA_SRC ${ROOT_DIR}/src/psmoveservice/Device/Interface/DevicePlatformInterface.h)
    list(APPEND TEST_CAMERA_SRC ${ROOT_DIR}/src/psmoveservice/Server/ServerLog.h)
    list(APPEND TEST_CAMERA_SRC ${ROOT_DIR}/src/psmoveservice/Server/ServerLog.cpp)
    list(APPEND TEST_CAMERA_SRC ${ROOT_DIR}/src/psmoveservice/Platform/PlatformDeviceAPIWin32.h)
    list(APPEND TEST_CAMERA_SRC ${ROOT_DIR}/src/psmoveservice/Platform/PlatformDeviceAPIWin32.cpp)   
ENDIF()

# Our custom OpenCV VideoCapture classes
# We could include the PSMoveService project but we want our test as isolated as possible.
list(APPEND TEST_CAMERA_INCL_DIRS 
    ${ROOT_DIR}/src/psmoveclient/
    ${ROOT_DIR}/src/psmoveprotocol/
    ${ROOT_DIR}/src/psmoveservice/PSMoveTracker/PSEye)
list(APPEND TEST_CAMERA_SRC
    ${ROOT_DIR}/src/psmoveclient/ClientConstants.h
    ${ROOT_DIR}/src/psmoveprotocol/SharedConstants.h
    ${ROOT_DIR}/src/psmoveservice/PSMoveTracker/PSEye/PSEyeVideoCapture.h
    ${ROOT_DIR}/src/psmoveservice/PSMoveTracker/PSEye/PSEyeVideoCapture.cpp)

# The test_camera app
add_executable(test_camera ${CMAKE_CURRENT_LIST_DIR}/test_camera.cpp ${TEST_CAMERA_SRC})
target_include_directories(test_camera PUBLIC ${TEST_CAMERA_INCL_DIRS})
target_link_libraries(test_camera ${PLATFORM_LIBS} ${TEST_CAMERA_REQ_LIBS})
SET_TARGET_PROPERTIES(test_camera PROPERTIES FOLDER Test)
    
# The test_camera_parallel app
IF((${CMAKE_SYSTEM_NAME} MATCHES "Windows") OR (${CMAKE_SYSTEM_NAME} MATCHES "Darwin"))
    add_executable(test_camera_parallel ${CMAKE_CURRENT_LIST_DIR}/test_camera_parallel.cpp ${TEST_CAMERA_SRC})
    target_include_directories(test_camera_parallel PUBLIC ${TEST_CAMERA_INCL_DIRS})
    target_link_libraries(test_camera_parallel ${PLATFORM_LIBS} ${TEST_CAMERA_REQ_LIBS})
    SET_TARGET_PROPERTIES(test_camera_parallel PROPERTIES FOLDER Test)
ENDIF()

# Copy CLEyeMulticam if necessary to prevent crashes.
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    IF(NOT(${CMAKE_C_SIZEOF_DATA_PTR} EQUAL 8))
        IF(${CL_EYE_SDK_PATH} STREQUAL "CL_EYE_SDK_PATH-NOTFOUND")
            add_custom_command(TARGET test_camera POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    "${ROOT_DIR}/thirdparty/CLEYE/x86/bin/CLEyeMulticam.dll"
                    $<TARGET_FILE_DIR:test_camera>)                
            add_custom_command(TARGET test_camera_parallel POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    "${ROOT_DIR}/thirdparty/CLEYE/x86/bin/CLEyeMulticam.dll"
                    $<TARGET_FILE_DIR:test_camera_parallel>)
        ENDIF()
    ENDIF()
ENDIF()

# Install
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    install(TARGETS test_camera
        CONFIGURATIONS Debug
        RUNTIME DESTINATION ${PSM_DEBUG_INSTALL_PATH}/bin
        LIBRARY DESTINATION ${PSM_DEBUG_INSTALL_PATH}/lib
        ARCHIVE DESTINATION ${PSM_DEBUG_INSTALL_PATH}/lib)
    install(TARGETS test_camera_parallel
        CONFIGURATIONS Debug
        RUNTIME DESTINATION ${PSM_DEBUG_INSTALL_PATH}/bin
        LIBRARY DESTINATION ${PSM_DEBUG_INSTALL_PATH}/lib
        ARCHIVE DESTINATION ${PSM_DEBUG_INSTALL_PATH}/lib)        
    install(TARGETS test_camera
        CONFIGURATIONS Release
        RUNTIME DESTINATION ${PSM_RELEASE_INSTALL_PATH}/bin
        LIBRARY DESTINATION ${PSM_RELEASE_INSTALL_PATH}/lib
        ARCHIVE DESTINATION ${PSM_RELEASE_INSTALL_PATH}/lib)
    install(TARGETS test_camera_parallel
        CONFIGURATIONS Release
        RUNTIME DESTINATION ${PSM_RELEASE_INSTALL_PATH}/bin
        LIBRARY DESTINATION ${PSM_RELEASE_INSTALL_PATH}/lib
        ARCHIVE DESTINATION ${PSM_RELEASE_INSTALL_PATH}/lib)        
ELSE() #Linux/Darwin
ENDIF()


#
# Test PSMove Controller
#

SET(TEST_PSMOVE_SRC)
SET(TEST_PSMOVE_INCL_DIRS)
SET(TEST_PSMOVE_REQ_LIBS)

# Dependencies

# hidapi
list(APPEND TEST_PSMOVE_INCL_DIRS ${HIDAPI_INCLUDE_DIRS})
list(APPEND TEST_PSMOVE_SRC ${HIDAPI_SRC})
list(APPEND TEST_PSMOVE_REQ_LIBS ${HIDAPI_LIBRARIES})

# libusb
find_package(USB1 REQUIRED)
list(APPEND TEST_PSMOVE_INCL_DIRS ${LIBUSB_INCLUDE_DIR})
list(APPEND TEST_PSMOVE_REQ_LIBS ${LIBUSB_LIBRARIES})

#Bluetooth
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    # Why not Windows?
	# list(APPEND TEST_PSMOVE_SRC ${ROOT_DIR}/src/psmoveservice/Platform/BluetoothQueriesWin32.cpp)
ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    list(APPEND TEST_PSMOVE_SRC ${ROOT_DIR}/src/psmoveservice/Platform/BluetoothQueriesOSX.mm)
ELSE()
    list(APPEND TEST_PSMOVE_SRC ${ROOT_DIR}/src/psmoveservice/Platform/BluetoothQueriesLinux.cpp)
ENDIF()

# libstem_gamepad
list(APPEND TEST_PSMOVE_INCL_DIRS ${LIBSTEM_GAMEPAD_INCLUDE_DIRS})
list(APPEND TEST_PSMOVE_SRC ${LIBSTEM_GAMEPAD_SRC})

# Boost
# TODO: Eliminate boost::filesystem with C++14
FIND_PACKAGE(Boost REQUIRED QUIET COMPONENTS atomic chrono filesystem program_options system thread)
list(APPEND TEST_PSMOVE_INCL_DIRS ${Boost_INCLUDE_DIRS})
list(APPEND TEST_PSMOVE_REQ_LIBS ${Boost_LIBRARIES})

# Eigen math library
list(APPEND TEST_PSMOVE_INCL_DIRS ${EIGEN3_INCLUDE_DIR})

# PSMoveController
# We are not including the PSMoveService target on purpose, because this only tests
# a small part of the service and should not depend on the whole thing building.
list(APPEND TEST_PSMOVE_INCL_DIRS
    ${ROOT_DIR}/src/psmovemath/
    ${ROOT_DIR}/src/psmoveservice/
    ${ROOT_DIR}/src/psmoveservice/Server
    ${ROOT_DIR}/src/psmoveservice/Device/Enumerator
    ${ROOT_DIR}/src/psmoveservice/Device/Interface
    ${ROOT_DIR}/src/psmoveservice/Device/Manager
    ${ROOT_DIR}/src/psmoveservice/Device/USB
    ${ROOT_DIR}/src/psmoveservice/Platform
    ${ROOT_DIR}/src/psmoveservice/PSMoveConfig
    ${ROOT_DIR}/src/psmoveservice/PSMoveController
    ${ROOT_DIR}/src/psmoveservice/Utils)
list(APPEND TEST_PSMOVE_SRC
    ${ROOT_DIR}/src/psmovemath/MathAlignment.h
    ${ROOT_DIR}/src/psmovemath/MathAlignment.cpp
    ${ROOT_DIR}/src/psmovemath/MathEigen.h
    ${ROOT_DIR}/src/psmovemath/MathEigen.cpp
    ${ROOT_DIR}/src/psmovemath/MathUtility.h
    ${ROOT_DIR}/src/psmovemath/MathUtility.cpp
    ${ROOT_DIR}/src/psmoveservice/Server/ServerLog.h
    ${ROOT_DIR}/src/psmoveservice/Server/ServerLog.cpp
    ${ROOT_DIR}/src/psmoveservice/Server/ServerUtility.h
    ${ROOT_DIR}/src/psmoveservice/Server/ServerUtility.cpp
    ${ROOT_DIR}/src/psmoveservice/Device/Enumerator/ControllerDeviceEnumerator.h
    ${ROOT_DIR}/src/psmoveservice/Device/Enumerator/ControllerDeviceEnumerator.cpp
    ${ROOT_DIR}/src/psmoveservice/Device/Enumerator/ControllerGamepadEnumerator.h
    ${ROOT_DIR}/src/psmoveservice/Device/Enumerator/ControllerGamepadEnumerator.cpp
    ${ROOT_DIR}/src/psmoveservice/Device/Enumerator/ControllerHidDeviceEnumerator.h
    ${ROOT_DIR}/src/psmoveservice/Device/Enumerator/ControllerHidDeviceEnumerator.cpp
    ${ROOT_DIR}/src/psmoveservice/Device/Enumerator/ControllerUSBDeviceEnumerator.h
    ${ROOT_DIR}/src/psmoveservice/Device/Enumerator/ControllerUSBDeviceEnumerator.cpp
    ${ROOT_DIR}/src/psmoveservice/Device/Enumerator/VirtualControllerEnumerator.h
    ${ROOT_DIR}/src/psmoveservice/Device/Enumerator/VirtualControllerEnumerator.cpp
    ${ROOT_DIR}/src/psmoveservice/Device/Manager/USBDeviceManager.h
    ${ROOT_DIR}/src/psmoveservice/Device/Manager/USBDeviceManager.cpp
    ${ROOT_DIR}/src/psmoveservice/Device/USB/NullUSBApi.cpp
    ${ROOT_DIR}/src/psmoveservice/Device/USB/LibUSBApi.cpp
    ${ROOT_DIR}/src/psmoveservice/Device/USB/LibUSBBulkTransferBundle.cpp
    ${ROOT_DIR}/src/psmoveservice/Platform/BluetoothQueries.h
    ${ROOT_DIR}/src/psmoveservice/PSMoveConfig/PSMoveConfig.h
    ${ROOT_DIR}/src/psmoveservice/PSMoveConfig/PSMoveConfig.cpp
    ${ROOT_DIR}/src/psmoveservice/PSMoveController/PSMoveController.h
    ${ROOT_DIR}/src/psmoveservice/PSMoveController/PSMoveController.cpp
    ${ROOT_DIR}/src/psmoveservice/Utils/AtomicPrimitives.h
    ${ROOT_DIR}/src/psmoveservice/Utils/WorkerThread.h
    ${ROOT_DIR}/src/psmoveservice/Utils/WorkerThread.cpp)

# psmoveprotocol
# list(APPEND TEST_PSMOVE_REQ_LIBS protobuf::libprotoc protobuf::libprotobuf)
list(APPEND TEST_PSMOVE_INCL_DIRS ${ROOT_DIR}/src/psmoveprotocol)
list(APPEND TEST_PSMOVE_REQ_LIBS PSMoveProtocol)

add_executable(test_psmove_controller ${CMAKE_CURRENT_LIST_DIR}/test_psmove_controller.cpp ${TEST_PSMOVE_SRC})
target_include_directories(test_psmove_controller PUBLIC ${TEST_PSMOVE_INCL_DIRS})
target_link_libraries(test_psmove_controller ${PLATFORM_LIBS} ${TEST_PSMOVE_REQ_LIBS})
SET_TARGET_PROPERTIES(test_psmove_controller PROPERTIES FOLDER Test)

# Install
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    install(TARGETS test_psmove_controller
        CONFIGURATIONS Debug
        RUNTIME DESTINATION ${PSM_DEBUG_INSTALL_PATH}/bin
        LIBRARY DESTINATION ${PSM_DEBUG_INSTALL_PATH}/lib
        ARCHIVE DESTINATION ${PSM_DEBUG_INSTALL_PATH}/lib)
    install(TARGETS test_psmove_controller
        CONFIGURATIONS Release
        RUNTIME DESTINATION ${PSM_RELEASE_INSTALL_PATH}/bin
        LIBRARY DESTINATION ${PSM_RELEASE_INSTALL_PATH}/lib
        ARCHIVE DESTINATION ${PSM_RELEASE_INSTALL_PATH}/lib)        
ELSE() #Linux/Darwin
ENDIF()

#
# Test Navi Controller
#

SET(TEST_NAVI_SRC)
SET(TEST_NAVI_INCL_DIRS)
SET(TEST_NAVI_REQ_LIBS)

# Dependencies

# hidapi
list(APPEND TEST_NAVI_INCL_DIRS ${HIDAPI_INCLUDE_DIRS})
list(APPEND TEST_NAVI_SRC ${HIDAPI_SRC})
list(APPEND TEST_NAVI_REQ_LIBS ${HIDAPI_LIBRARIES})

# libusb
find_package(USB1 REQUIRED)
list(APPEND TEST_NAVI_INCL_DIRS ${LIBUSB_INCLUDE_DIR})
list(APPEND TEST_NAVI_REQ_LIBS ${LIBUSB_LIBRARIES})

#Bluetooth
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    list(APPEND TEST_NAVI_SRC ${ROOT_DIR}/src/psmoveservice/Platform/BluetoothQueriesOSX.mm)
ELSE()
    list(APPEND TEST_NAVI_SRC ${ROOT_DIR}/src/psmoveservice/Platform/BluetoothQueriesLinux.cpp)
ENDIF()

# libstem_gamepad
list(APPEND TEST_NAVI_INCL_DIRS ${LIBSTEM_GAMEPAD_INCLUDE_DIRS})
list(APPEND TEST_NAVI_SRC ${LIBSTEM_GAMEPAD_SRC})

# Boost
# TODO: Eliminate boost::filesystem with C++14
FIND_PACKAGE(Boost REQUIRED QUIET COMPONENTS atomic chrono filesystem program_options system thread)
list(APPEND TEST_NAVI_INCL_DIRS ${Boost_INCLUDE_DIRS})
list(APPEND TEST_NAVI_REQ_LIBS ${Boost_LIBRARIES})

# Eigen math library
list(APPEND TEST_NAVI_INCL_DIRS ${EIGEN3_INCLUDE_DIR})

# PSMoveController
# We are not including the PSMoveService target on purpose, because this only tests
# a small part of the service and should not depend on the whole thing building.
list(APPEND TEST_NAVI_INCL_DIRS
    ${ROOT_DIR}/src/psmovemath/
    ${ROOT_DIR}/src/psmoveservice/
    ${ROOT_DIR}/src/psmoveservice/Server
    ${ROOT_DIR}/src/psmoveservice/Device/Enumerator
    ${ROOT_DIR}/src/psmoveservice/Device/Interface
    ${ROOT_DIR}/src/psmoveservice/Device/Manager
    ${ROOT_DIR}/src/psmoveservice/Device/USB
    ${ROOT_DIR}/src/psmoveservice/Platform
    ${ROOT_DIR}/src/psmoveservice/PSMoveConfig
    ${ROOT_DIR}/src/psmoveservice/PSNaviController
    ${ROOT_DIR}/src/psmoveservice/Utils)
list(APPEND TEST_NAVI_SRC
    ${ROOT_DIR}/src/psmovemath/MathAlignment.h
    ${ROOT_DIR}/src/psmovemath/MathAlignment.cpp
    ${ROOT_DIR}/src/psmovemath/MathEigen.h
    ${ROOT_DIR}/src/psmovemath/MathEigen.cpp 
    ${ROOT_DIR}/src/psmovemath/MathUtility.h
    ${ROOT_DIR}/src/psmovemath/MathUtility.cpp
    ${ROOT_DIR}/src/psmoveservice/Server/ServerLog.h
    ${ROOT_DIR}/src/psmoveservice/Server/ServerLog.cpp
    ${ROOT_DIR}/src/psmoveservice/Server/ServerUtility.h
    ${ROOT_DIR}/src/psmoveservice/Server/ServerUtility.cpp
    ${ROOT_DIR}/src/psmoveservice/Device/Enumerator/ControllerDeviceEnumerator.h
    ${ROOT_DIR}/src/psmoveservice/Device/Enumerator/ControllerDeviceEnumerator.cpp
    ${ROOT_DIR}/src/psmoveservice/Device/Enumerator/ControllerGamepadEnumerator.h
    ${ROOT_DIR}/src/psmoveservice/Device/Enumerator/ControllerGamepadEnumerator.cpp
    ${ROOT_DIR}/src/psmoveservice/Device/Enumerator/ControllerHidDeviceEnumerator.h
    ${ROOT_DIR}/src/psmoveservice/Device/Enumerator/ControllerHidDeviceEnumerator.cpp
    ${ROOT_DIR}/src/psmoveservice/Device/Enumerator/ControllerUSBDeviceEnumerator.h
    ${ROOT_DIR}/src/psmoveservice/Device/Enumerator/ControllerUSBDeviceEnumerator.cpp
    ${ROOT_DIR}/src/psmoveservice/Device/Enumerator/VirtualControllerEnumerator.h
    ${ROOT_DIR}/src/psmoveservice/Device/Enumerator/VirtualControllerEnumerator.cpp
    ${ROOT_DIR}/src/psmoveservice/Device/Manager/USBDeviceManager.h
    ${ROOT_DIR}/src/psmoveservice/Device/Manager/USBDeviceManager.cpp
    ${ROOT_DIR}/src/psmoveservice/Device/USB/NullUSBApi.cpp
    ${ROOT_DIR}/src/psmoveservice/Device/USB/LibUSBApi.cpp
    ${ROOT_DIR}/src/psmoveservice/Device/USB/LibUSBBulkTransferBundle.cpp
    ${ROOT_DIR}/src/psmoveservice/Platform/BluetoothQueries.h
    ${ROOT_DIR}/src/psmoveservice/PSMoveConfig/PSMoveConfig.h
    ${ROOT_DIR}/src/psmoveservice/PSMoveConfig/PSMoveConfig.cpp
    ${ROOT_DIR}/src/psmoveservice/PSNaviController/PSNaviController.h
    ${ROOT_DIR}/src/psmoveservice/PSNaviController/PSNaviController.cpp
    ${ROOT_DIR}/src/psmoveservice/Utils/AtomicPrimitives.h
    ${ROOT_DIR}/src/psmoveservice/Utils/WorkerThread.h
    ${ROOT_DIR}/src/psmoveservice/Utils/WorkerThread.cpp)

# psmoveprotocol
# list(APPEND TEST_NAVI_REQ_LIBS protobuf::libprotoc protobuf::libprotobuf)
list(APPEND TEST_NAVI_INCL_DIRS ${ROOT_DIR}/src/psmoveprotocol)
list(APPEND TEST_NAVI_REQ_LIBS PSMoveProtocol)

add_executable(test_navi_controller ${CMAKE_CURRENT_LIST_DIR}/test_navi_controller.cpp ${TEST_NAVI_SRC})
target_include_directories(test_navi_controller PUBLIC ${TEST_NAVI_INCL_DIRS})
target_link_libraries(test_navi_controller ${PLATFORM_LIBS} ${TEST_NAVI_REQ_LIBS})
SET_TARGET_PROPERTIES(test_navi_controller PROPERTIES FOLDER Test)

# Install
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    install(TARGETS test_navi_controller
        CONFIGURATIONS Debug
        RUNTIME DESTINATION ${PSM_DEBUG_INSTALL_PATH}/bin
        LIBRARY DESTINATION ${PSM_DEBUG_INSTALL_PATH}/lib
        ARCHIVE DESTINATION ${PSM_DEBUG_INSTALL_PATH}/lib)
    install(TARGETS test_navi_controller
        CONFIGURATIONS Release
        RUNTIME DESTINATION ${PSM_RELEASE_INSTALL_PATH}/bin
        LIBRARY DESTINATION ${PSM_RELEASE_INSTALL_PATH}/lib
        ARCHIVE DESTINATION ${PSM_RELEASE_INSTALL_PATH}/lib)        
ELSE() #Linux/Darwin
ENDIF()

#
# Test DS4 Controller
#

SET(TEST_DS4_CTRLR_SRC)
SET(TEST_DS4_CTRLR_INCL_DIRS)
SET(TEST_DS4_CTRLR_REQ_LIBS)

# Dependencies

# Platform specific libraries
IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    #hid required for HidD_SetOutputReport() in DualShock4 controller
    list(APPEND TEST_DS4_CTRLR_REQ_LIBS bthprops hid)
ELSE() #Linux
ENDIF()

# hidapi
list(APPEND TEST_DS4_CTRLR_INCL_DIRS ${HIDAPI_INCLUDE_DIRS})
list(APPEND TEST_DS4_CTRLR_SRC ${HIDAPI_SRC})
list(APPEND TEST_DS4_CTRLR_REQ_LIBS ${HIDAPI_LIBRARIES})

# libusb
find_package(USB1 REQUIRED)
list(APPEND TEST_DS4_CTRLR_INCL_DIRS ${LIBUSB_INCLUDE_DIR})
list(APPEND TEST_DS4_CTRLR_REQ_LIBS ${LIBUSB_LIBRARIES})

#Bluetooth
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    list(APPEND TEST_DS4_CTRLR_SRC ${ROOT_DIR}/src/psmoveservice/Platform/BluetoothQueriesWin32.cpp)
ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    list(APPEND TEST_DS4_CTRLR_SRC ${ROOT_DIR}/src/psmoveservice/Platform/BluetoothQueriesOSX.mm)
ELSE()
    list(APPEND TEST_DS4_CTRLR_SRC ${ROOT_DIR}/src/psmoveservice/Platform/BluetoothQueriesLinux.cpp)
ENDIF()

# libstem_gamepad
list(APPEND TEST_DS4_CTRLR_INCL_DIRS ${LIBSTEM_GAMEPAD_INCLUDE_DIRS})
list(APPEND TEST_DS4_CTRLR_SRC ${LIBSTEM_GAMEPAD_SRC})

# Boost
# TODO: Eliminate boost::filesystem with C++14
FIND_PACKAGE(Boost REQUIRED QUIET COMPONENTS atomic chrono filesystem program_options system thread)
list(APPEND TEST_DS4_CTRLR_INCL_DIRS ${Boost_INCLUDE_DIRS})
list(APPEND TEST_DS4_CTRLR_REQ_LIBS ${Boost_LIBRARIES})

# Eigen math library
list(APPEND TEST_DS4_CTRLR_INCL_DIRS ${EIGEN3_INCLUDE_DIR})

# PSMoveController
# We are not including the PSMoveService target on purpose, because this only tests
# a small part of the service and should not depend on the whole thing building.
list(APPEND TEST_DS4_CTRLR_INCL_DIRS
    ${ROOT_DIR}/src/psmovemath/
    ${ROOT_DIR}/src/psmoveservice/
    ${ROOT_DIR}/src/psmoveservice/Server
    ${ROOT_DIR}/src/psmoveservice/Device/Enumerator
    ${ROOT_DIR}/src/psmoveservice/Device/Interface
    ${ROOT_DIR}/src/psmoveservice/Device/Manager
    ${ROOT_DIR}/src/psmoveservice/Device/USB
    ${ROOT_DIR}/src/psmoveservice/Platform
    ${ROOT_DIR}/src/psmoveservice/PSMoveConfig
    ${ROOT_DIR}/src/psmoveservice/PSDualShock4
	${ROOT_DIR}/src/psmoveservice/Utils)
list(APPEND TEST_DS4_CTRLR_SRC
    ${ROOT_DIR}/src/psmovemath/MathAlignment.h
    ${ROOT_DIR}/src/psmovemath/MathAlignment.cpp
    ${ROOT_DIR}/src/psmovemath/MathEigen.h
    ${ROOT_DIR}/src/psmovemath/MathEigen.cpp
    ${ROOT_DIR}/src/psmovemath/MathUtility.h
    ${ROOT_DIR}/src/psmovemath/MathUtility.cpp
    ${ROOT_DIR}/src/psmoveservice/Server/ServerLog.h
    ${ROOT_DIR}/src/psmoveservice/Server/ServerLog.cpp
    ${ROOT_DIR}/src/psmoveservice/Server/ServerUtility.h
    ${ROOT_DIR}/src/psmoveservice/Server/ServerUtility.cpp
    ${ROOT_DIR}/src/psmoveservice/Device/Enumerator/ControllerDeviceEnumerator.h
    ${ROOT_DIR}/src/psmoveservice/Device/Enumerator/ControllerDeviceEnumerator.cpp
    ${ROOT_DIR}/src/psmoveservice/Device/Enumerator/ControllerGamepadEnumerator.h
    ${ROOT_DIR}/src/psmoveservice/Device/Enumerator/ControllerGamepadEnumerator.cpp
    ${ROOT_DIR}/src/psmoveservice/Device/Enumerator/ControllerHidDeviceEnumerator.h
    ${ROOT_DIR}/src/psmoveservice/Device/Enumerator/ControllerHidDeviceEnumerator.cpp
    ${ROOT_DIR}/src/psmoveservice/Device/Enumerator/ControllerUSBDeviceEnumerator.h
    ${ROOT_DIR}/src/psmoveservice/Device/Enumerator/ControllerUSBDeviceEnumerator.cpp
    ${ROOT_DIR}/src/psmoveservice/Device/Enumerator/VirtualControllerEnumerator.h
    ${ROOT_DIR}/src/psmoveservice/Device/Enumerator/VirtualControllerEnumerator.cpp
    ${ROOT_DIR}/src/psmoveservice/Device/Manager/USBDeviceManager.h
    ${ROOT_DIR}/src/psmoveservice/Device/Manager/USBDeviceManager.cpp
    ${ROOT_DIR}/src/psmoveservice/Device/USB/NullUSBApi.cpp
    ${ROOT_DIR}/src/psmoveservice/Device/USB/LibUSBApi.cpp
    ${ROOT_DIR}/src/psmoveservice/Device/USB/LibUSBBulkTransferBundle.cpp
    ${ROOT_DIR}/src/psmoveservice/Platform/BluetoothQueries.h
    ${ROOT_DIR}/src/psmoveservice/PSMoveConfig/PSMoveConfig.h
    ${ROOT_DIR}/src/psmoveservice/PSMoveConfig/PSMoveConfig.cpp
    ${ROOT_DIR}/src/psmoveservice/PSDualShock4/PSDualShock4Controller.h
    ${ROOT_DIR}/src/psmoveservice/PSDualShock4/PSDualShock4Controller.cpp
    ${ROOT_DIR}/src/psmoveservice/Utils/AtomicPrimitives.h
    ${ROOT_DIR}/src/psmoveservice/Utils/WorkerThread.h
    ${ROOT_DIR}/src/psmoveservice/Utils/WorkerThread.cpp)

# psmoveprotocol
# list(APPEND TEST_DS4_CTRLR_REQ_LIBS protobuf::libprotoc protobuf::libprotobuf)
list(APPEND TEST_DS4_CTRLR_INCL_DIRS ${ROOT_DIR}/src/psmoveprotocol)
list(APPEND TEST_DS4_CTRLR_REQ_LIBS PSMoveProtocol)

add_executable(test_ds4_controller ${CMAKE_CURRENT_LIST_DIR}/test_ds4_controller.cpp ${TEST_DS4_CTRLR_SRC})
target_include_directories(test_ds4_controller PUBLIC ${TEST_DS4_CTRLR_INCL_DIRS})
target_link_libraries(test_ds4_controller ${PLATFORM_LIBS} ${TEST_DS4_CTRLR_REQ_LIBS})
SET_TARGET_PROPERTIES(test_ds4_controller PROPERTIES FOLDER Test)

# Install
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    install(TARGETS test_ds4_controller
        CONFIGURATIONS Debug
        RUNTIME DESTINATION ${PSM_DEBUG_INSTALL_PATH}/bin
        LIBRARY DESTINATION ${PSM_DEBUG_INSTALL_PATH}/lib
        ARCHIVE DESTINATION ${PSM_DEBUG_INSTALL_PATH}/lib)
    install(TARGETS test_ds4_controller
        CONFIGURATIONS Release
        RUNTIME DESTINATION ${PSM_RELEASE_INSTALL_PATH}/bin
        LIBRARY DESTINATION ${PSM_RELEASE_INSTALL_PATH}/lib
        ARCHIVE DESTINATION ${PSM_RELEASE_INSTALL_PATH}/lib)        
ELSE() #Linux/Darwin
ENDIF()

#
# TEST_CONSOLE_CAPI
#
add_executable(test_console_CAPI test_console_CAPI.cpp)
target_include_directories(test_console_CAPI PUBLIC 
    ${ROOT_DIR}/src/psmoveclient/
    ${ROOT_DIR}/src/psmoveprotocol/)
target_link_libraries(test_console_CAPI PSMoveClient_CAPI)
SET_TARGET_PROPERTIES(test_console_CAPI PROPERTIES FOLDER Test)
# Install
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
install(TARGETS test_console_CAPI
    CONFIGURATIONS Debug
    RUNTIME DESTINATION ${PSM_DEBUG_INSTALL_PATH}/bin
    LIBRARY DESTINATION ${PSM_DEBUG_INSTALL_PATH}/lib
    ARCHIVE DESTINATION ${PSM_DEBUG_INSTALL_PATH}/lib)
install(TARGETS test_console_CAPI
    CONFIGURATIONS Release
    RUNTIME DESTINATION ${PSM_RELEASE_INSTALL_PATH}/bin
    LIBRARY DESTINATION ${PSM_RELEASE_INSTALL_PATH}/lib
    ARCHIVE DESTINATION ${PSM_RELEASE_INSTALL_PATH}/lib)    
ELSE() #Linux/Darwin
ENDIF()

#
# TEST_KALMAN_FILTER
#

list(APPEND TEST_KALMAN_INCL_DIRS
    ${ROOT_DIR}/src/psmovemath/
    ${ROOT_DIR}/src/psmoveservice/Device/Interface
    ${ROOT_DIR}/src/psmoveservice/Filter/
    ${ROOT_DIR}/src/psmoveservice/PSMoveController
    ${ROOT_DIR}/src/psmoveservice/Server/)
list(APPEND TEST_KALMAN_SRC
    ${ROOT_DIR}/src/psmovemath/MathAlignment.h
    ${ROOT_DIR}/src/psmovemath/MathAlignment.cpp
    ${ROOT_DIR}/src/psmovemath/MathEigen.h
    ${ROOT_DIR}/src/psmovemath/MathEigen.cpp
    ${ROOT_DIR}/src/psmovemath/MathUtility.h
    ${ROOT_DIR}/src/psmovemath/MathUtility.cpp
    ${ROOT_DIR}/src/psmoveservice/Filter/CompoundPoseFilter.h
    ${ROOT_DIR}/src/psmoveservice/Filter/CompoundPoseFilter.cpp
    ${ROOT_DIR}/src/psmoveservice/Filter/KalmanOrientationFilter.h
    ${ROOT_DIR}/src/psmoveservice/Filter/KalmanOrientationFilter.cpp
    ${ROOT_DIR}/src/psmoveservice/Filter/KalmanPositionFilter.h
    ${ROOT_DIR}/src/psmoveservice/Filter/KalmanPositionFilter.cpp
    ${ROOT_DIR}/src/psmoveservice/Filter/KalmanPoseFilter.h
    ${ROOT_DIR}/src/psmoveservice/Filter/KalmanPoseFilter.cpp
    ${ROOT_DIR}/src/psmoveservice/Filter/OrientationFilter.h
    ${ROOT_DIR}/src/psmoveservice/Filter/OrientationFilter.cpp
    ${ROOT_DIR}/src/psmoveservice/Filter/PoseFilterInterface.h
    ${ROOT_DIR}/src/psmoveservice/Filter/PoseFilterInterface.cpp
    ${ROOT_DIR}/src/psmoveservice/Filter/PositionFilter.h
    ${ROOT_DIR}/src/psmoveservice/Filter/PositionFilter.cpp
    ${ROOT_DIR}/src/psmoveservice/Server/ServerLog.h
    ${ROOT_DIR}/src/psmoveservice/Server/ServerLog.cpp)
 
# Eigen math library
list(APPEND TEST_KALMAN_INCL_DIRS ${EIGEN3_INCLUDE_DIR})
list(APPEND TEST_KALMAN_INCL_DIRS ${ROOT_DIR}/thirdparty/kalman/include/)

add_executable(test_kalman_filter ${CMAKE_CURRENT_LIST_DIR}/test_kalman_filter.cpp ${TEST_KALMAN_SRC})
target_include_directories(test_kalman_filter PUBLIC ${TEST_KALMAN_INCL_DIRS})
SET_TARGET_PROPERTIES(test_kalman_filter PROPERTIES FOLDER Test)

# Install
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    install(TARGETS test_kalman_filter
        CONFIGURATIONS Debug
        RUNTIME DESTINATION ${PSM_DEBUG_INSTALL_PATH}/bin
        LIBRARY DESTINATION ${PSM_DEBUG_INSTALL_PATH}/lib
        ARCHIVE DESTINATION ${PSM_DEBUG_INSTALL_PATH}/lib)
    install(TARGETS test_kalman_filter
        CONFIGURATIONS Release
        RUNTIME DESTINATION ${PSM_RELEASE_INSTALL_PATH}/bin
        LIBRARY DESTINATION ${PSM_RELEASE_INSTALL_PATH}/lib
        ARCHIVE DESTINATION ${PSM_RELEASE_INSTALL_PATH}/lib)        
ELSE() #Linux/Darwin
ENDIF()

#
# UNIT_TESTS
#

list(APPEND UNIT_TEST_INCL_DIRS
    ${ROOT_DIR}/src/psmovemath/)

# Eigen math library
list(APPEND UNIT_TEST_INCL_DIRS ${EIGEN3_INCLUDE_DIR})

list(APPEND UNIT_TEST_SRC
    ${ROOT_DIR}/src/psmovemath/MathAlignment.h
    ${ROOT_DIR}/src/psmovemath/MathAlignment.cpp
    ${ROOT_DIR}/src/psmovemath/MathEigen.h
    ${ROOT_DIR}/src/psmovemath/MathEigen.cpp
    ${ROOT_DIR}/src/psmovemath/MathUtility.h
    ${ROOT_DIR}/src/psmovemath/MathUtility.cpp
    ${ROOT_DIR}/src/tests/math_alignment_unit_tests.cpp
    ${ROOT_DIR}/src/tests/math_eigen_unit_tests.cpp
    ${ROOT_DIR}/src/tests/math_utility_unit_tests.cpp
    ${ROOT_DIR}/src/tests/unit_test.h)

add_executable(unit_test_suite ${CMAKE_CURRENT_LIST_DIR}/unit_test_suite.cpp ${UNIT_TEST_SRC})
target_include_directories(unit_test_suite PUBLIC ${UNIT_TEST_INCL_DIRS})
SET_TARGET_PROPERTIES(unit_test_suite PROPERTIES FOLDER Test)

# Install
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    install(TARGETS unit_test_suite
        CONFIGURATIONS Debug
        RUNTIME DESTINATION ${PSM_DEBUG_INSTALL_PATH}/bin
        LIBRARY DESTINATION ${PSM_DEBUG_INSTALL_PATH}/lib
        ARCHIVE DESTINATION ${PSM_DEBUG_INSTALL_PATH}/lib)
    install(TARGETS unit_test_suite
        CONFIGURATIONS Debug
        RUNTIME DESTINATION ${PSM_RELEASE_INSTALL_PATH}/bin
        LIBRARY DESTINATION ${PSM_RELEASE_INSTALL_PATH}/lib
        ARCHIVE DESTINATION ${PSM_RELEASE_INSTALL_PATH}/lib)        
ELSE() #Linux/Darwin
ENDIF()


#
# Test hidapi in MacOS Sierra
#
IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    add_executable(test_hidapi_sierra
        ${CMAKE_CURRENT_LIST_DIR}/test_hidapi_sierra.cpp
        ${ROOT_DIR}/thirdparty/hidapi/mac/hid.c)
    target_include_directories(test_hidapi_sierra
        PUBLIC
        ${ROOT_DIR}/thirdparty/hidapi/hidapi)
        #/usr/local/opt/hidapi/include/hidapi
    target_link_libraries(test_hidapi_sierra ${PLATFORM_LIBS})
    #target_link_libraries(test_hidapi_sierra /usr/local/opt/hidapi/lib/libhidapi.dylib)
    SET_TARGET_PROPERTIES(test_hidapi_sierra PROPERTIES FOLDER Test)
ENDIF()
