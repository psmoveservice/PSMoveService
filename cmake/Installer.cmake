#------------------------
# Platform Specific Installers
#------------------------

IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
	find_package( InnoSetup )
	IF( INNOSETUP_FOUND )
		# Create a copy of the installer script with the version and source file path filled in
		string(REPLACE "/" "\\" APP_VERSION "${PSM_VERSION_STRING}")
		string(REPLACE "/" "\\" APP_INSTALLER_DIR "${PSM_RELEASE_INSTALL_PATH}")		
		configure_file(${ROOT_DIR}/templates/installer_win64.iss.in ${CMAKE_CURRENT_BINARY_DIR}/installer_win64.iss)	
		
		# Create a simple cmake script that invokes the Inno Install script compiler
		FILE( 
			WRITE ${CMAKE_CURRENT_BINARY_DIR}/CreateInstaller.cmake 
			"EXECUTE_PROCESS(
				COMMAND \"${INNOSETUP_COMPILER}\" \"/cc\" \"${CMAKE_CURRENT_BINARY_DIR}/installer_win64.iss\"
			)"
		)
		
		# Create a project build target that invokes the CreateInstaller cmake script
		ADD_CUSTOM_TARGET(
			CREATE_INSTALLER
			COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/CreateInstaller.cmake
			COMMENT "create installer"
		)		
	ELSE()
		message("Inno Setup Compiler not found. Skipping creation of CreateInstaller project.")
	ENDIF( INNOSETUP_FOUND )
ENDIF()