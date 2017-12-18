IF( NOT INNOSETUP_FOUND )
	SET(INNOSETUP_FOUND 0)
	
    # Can't use "$ENV{ProgramFiles(x86)}" to avoid violating CMP0053.  See
    # http://public.kitware.com/pipermail/cmake-developers/2014-October/023190.html
    set (ProgramFiles_x86 "ProgramFiles(x86)")	
	FIND_PATH(
	   INNOSETUP_DIR 
	   NAMES
	   Compil32.exe
	   PATHS
	   "$ENV{${ProgramFiles_x86}}/Inno Setup 5"
	   "$ENV{ProgramFiles}/Inno Setup 5"
	)

	IF( INNOSETUP_DIR )
		FIND_FILE( INNOSETUP_COMPILER Compil32.exe PATHS ${INNOSETUP_DIR} )
			
		SET(INNOSETUP_FOUND 1)
		
		MARK_AS_ADVANCED(
			INNOSETUP_DIR
			INNOSETUP_FOUND
			INNOSETUP_COMPILER
		)        
	ENDIF( INNOSETUP_DIR )
ENDIF( NOT INNOSETUP_FOUND )