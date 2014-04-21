################################################################################
#
# Description:
#	ACADO Toolkit package configuration file
#
#	Defines:
#		- Variable: ACADO_INCLUDE_DIRS
#		- Variable: ACADO_LIBRARY_DIRS
#		- Variable: ACADO_STATIC_LIBS_FOUND
#		- Variable: ACADO_STATIC_LIBRARIES
#		- Variable: ACADO_SHARED_LIBS_FOUND
#		- Variable: ACADO_SHARED_LIBRARIES
#
# Authors:
#	Milan Vukov, milan.vukov@esat.kuleuven.be
#
# Year:
#	2011 - 2013.
#
# NOTE:
#	- This script is for Linux/Unix use only.
#	- Use this script only (and only :)) if you do not want to install ACADO
#		toolkit. If you install ACADO toolkit you do not need this script.
#
#	- PREREQUISITE: sourced acado_env.sh in your ~/.bashrc file. This script
#		will try to find ACADO folders, libraries etc., but looking for them
#		in enviromental variables.
#
# Usage:
#	- Linux/Unix: TODO
#
################################################################################

################################################################################
#
# Search for package components
#
################################################################################

MESSAGE( STATUS "********************************************************************************" )
MESSAGE( STATUS "---> Looking for ACADO toolkit:" )

#
# Include folders
#
SET( ACADO_INCLUDE_DIRS $ENV{ACADO_ENV_INCLUDE_DIRS} )
IF( ACADO_INCLUDE_DIRS )
	MESSAGE( STATUS "Found ACADO toolkit include directories: ${ACADO_INCLUDE_DIRS}" )
	SET( ACADO_INCLUDE_DIRS_FOUND TRUE )
ELSE( ACADO_INCLUDE_DIRS )
	MESSAGE( STATUS "Could not find ACADO toolkit include directories" )
ENDIF( ACADO_INCLUDE_DIRS )

#
# Library folders
#
SET( ACADO_LIBRARY_DIRS $ENV{ACADO_ENV_LIBRARY_DIRS} )
IF( ACADO_LIBRARY_DIRS )
	MESSAGE( STATUS "Found ACADO toolkit library directories: ${ACADO_LIBRARY_DIRS}" )
	SET( ACADO_LIBRARY_DIRS_FOUND TRUE )
ELSE( ACADO_LIBRARY_DIRS )
	MESSAGE( STATUS "Could not find ACADO toolkit library directories" )
ENDIF( ACADO_LIBRARY_DIRS )

#
# Static libs
#

SET ( STATIC_TMP $ENV{ACADO_ENV_STATIC_LIBRARIES})
IF ( NOT STATIC_TMP )
	MESSAGE( STATUS "Could not find ACADO static library." )
	SET( ACADO_STATIC_LIBS_FOUND FALSE )
ELSE()
	SET( ACADO_STATIC_LIBS_FOUND TRUE )
	UNSET( ACADO_STATIC_LIBRARIES )
	FOREACH( LIB $ENV{ACADO_ENV_STATIC_LIBRARIES} )
		UNSET( ACADO_TOOLKIT_STATIC_${LIB} )
	
		FIND_LIBRARY( ACADO_TOOLKIT_STATIC_${LIB}
			NAMES ${LIB}
			PATHS ${ACADO_LIBRARY_DIRS}
			NO_DEFAULT_PATH
		)
	
		IF( ACADO_TOOLKIT_STATIC_${LIB} )
			MESSAGE( STATUS "Found ACADO static library: ${LIB}" )
			SET( ACADO_STATIC_LIBRARIES
				${ACADO_STATIC_LIBRARIES} ${ACADO_TOOLKIT_STATIC_${LIB}}
			)
		ELSE( )
			MESSAGE( STATUS "Could not find ACADO static library: ${LIB}" )
 			SET( ACADO_STATIC_LIBS_FOUND FALSE )
		ENDIF( )
	ENDFOREACH()
ENDIF()

SET( ACADO_BUILD_STATIC ${ACADO_STATIC_LIBS_FOUND} )

IF( VERBOSE )
	MESSAGE( STATUS "Static libraries: ${ACADO_STATIC_LIBRARIES}\n" )
ENDIF()

#
# Shared libs
#
SET( SHARED_TMP $ENV{ACADO_ENV_SHARED_LIBRARIES} )
IF ( NOT SHARED_TMP )
	MESSAGE( STATUS "Could not find ACADO shared library." )
	SET( ACADO_SHARED_LIBS_FOUND FALSE )
ELSE()
SET( ACADO_SHARED_LIBS_FOUND TRUE )
	UNSET( ACADO_SHARED_LIBRARIES )
	FOREACH( LIB $ENV{ACADO_ENV_SHARED_LIBRARIES} )
		UNSET( ACADO_TOOLKIT_SHARED_${LIB} )
	
		FIND_LIBRARY( ACADO_TOOLKIT_SHARED_${LIB}
			NAMES ${LIB}
			PATHS ${ACADO_LIBRARY_DIRS}
			NO_DEFAULT_PATH
		)
	
		IF( ACADO_TOOLKIT_SHARED_${LIB} )
			MESSAGE( STATUS "Found ACADO shared library: ${LIB}" )
			SET( ACADO_SHARED_LIBRARIES
				${ACADO_SHARED_LIBRARIES} ${ACADO_TOOLKIT_SHARED_${LIB}}
			)
		ELSE( )
			MESSAGE( STATUS "Could not find ACADO shared library: ${LIB}" )
 			SET( ACADO_SHARED_LIBS_FOUND FALSE )
		ENDIF( )
	ENDFOREACH()
ENDIF()

SET( ACADO_BUILD_SHARED ${ACADO_SHARED_LIBS_FOUND} )

IF( VERBOSE )
	MESSAGE( STATUS "${ACADO_SHARED_LIBRARIES}\n" )
ENDIF()

#
# qpOASES embedded source files and include folders
# TODO: checks
#
SET( ACADO_QPOASES_EMBEDDED_SOURCES $ENV{ACADO_ENV_QPOASES_EMBEDDED_SOURCES} )
SET( ACADO_QPOASES_EMBEDDED_INC_DIRS $ENV{ACADO_ENV_QPOASES_EMBEDDED_INC_DIRS} )


#
# And finally set found flag...
#
IF( ACADO_INCLUDE_DIRS_FOUND AND ACADO_LIBRARY_DIRS_FOUND 
		AND (ACADO_STATIC_LIBS_FOUND OR ACADO_SHARED_LIBS_FOUND) )
	SET( ACADO_FOUND TRUE )
ENDIF()

MESSAGE( STATUS "********************************************************************************" )

