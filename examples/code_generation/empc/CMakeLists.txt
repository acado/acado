################################################################################
#
# Description:
#	CMake scipt for building code-generation ACADO examples
#
# Authors:
#	Milan Vukov, milan.vukov@esat.kuleuven.be
#
# Year:
#	2012 - 2013.
#
# NOTE:
#	- It is assumed that one executable has only one source (.cpp) file.
#
# Usage:
#	- This file should be called from the main CMake script for examples.
#
################################################################################

################################################################################
#
# Compilation of examples
#
################################################################################

#
# Compiling examples
#
UNSET( SOURCES )
FILE( GLOB SOURCES ./*.cpp )
IF( NOT ACADO_DEVELOPER )
	#
	# Remove examples with dev_ prefix
	#
	FILE( GLOB DEV_SOURCES ./dev_*.cpp )
	IF ( DEV_SOURCES )
		LIST( REMOVE_ITEM SOURCES ${DEV_SOURCES} )
	ENDIF ( DEV_SOURCES )
ENDIF( NOT ACADO_DEVELOPER )

#
# Compile examples
#
FOREACH( SRC ${SOURCES} )
	GET_FILENAME_COMPONENT( EXEC_NAME ${SRC} NAME_WE )

	SET( CURR_EXE code_generation_${EXEC_NAME} )

	ADD_EXECUTABLE( ${CURR_EXE} ${SRC} )

	IF (ACADO_BUILD_SHARED)
		TARGET_LINK_LIBRARIES(
			${CURR_EXE}
			${ACADO_SHARED_LIBRARIES}
		)
	ELSE()
		TARGET_LINK_LIBRARIES(
			${CURR_EXE}
			${ACADO_STATIC_LIBRARIES}
		)
	ENDIF()
		
	SET_TARGET_PROPERTIES(
		${CURR_EXE}
		PROPERTIES
			OUTPUT_NAME "${EXEC_NAME}"
			RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
			# This one is Visual Studio specific
			FOLDER "examples"
	)
	
	IF ( ACADO_WITH_TESTING )
			
		ADD_TEST(
			NAME
				${CURR_EXE}_test
			WORKING_DIRECTORY
				"${CMAKE_CURRENT_SOURCE_DIR}"
			COMMAND
				${CURR_EXE}
			)
	ENDIF()

ENDFOREACH( SRC ${SOURCES} )
