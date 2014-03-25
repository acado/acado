################################################################################
#
# Description:
#	ACADO unit tests
#
# Authors:
#	Milan Vukov, milan.vukov@esat.kuleuven.be
#
# Year:
#	2013 - 2014.
#
# Usage:
#	- One cpp file is one app.
#	- This file is supposed to be called from the main CMake script.
#	- The script defines one additional target: type "make test_verbose" for
#	  verbose output. The default one "make test" is silent.
#
################################################################################

################################################################################
#
# Dependencies
#
################################################################################

IF( NOT UNIX )
	MESSAGE( ERROR "At the moment, unit tests are designed to work only on UNIX platforms" )
ENDIF()

FIND_PACKAGE( Boost COMPONENTS system filesystem unit_test_framework REQUIRED )

INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIRS} )

################################################################################
#
# Adding unit tests
#
################################################################################

FILE( GLOB SOURCES *.cpp )
FOREACH( SRC ${SOURCES} )
	GET_FILENAME_COMPONENT( EXEC_NAME ${SRC} NAME_WE )

	SET( CURR_EXE ${EXEC_NAME}_unittest )

	ADD_EXECUTABLE( ${CURR_EXE} ${SRC} )

	IF ( ACADO_BUILD_SHARED )
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
	
	TARGET_LINK_LIBRARIES(
		${CURR_EXE}	
		${Boost_LIBRARIES}
	)
	
	ADD_TEST(
		NAME
			${CURR_EXE}
		COMMAND
			${EXECUTABLE_OUTPUT_PATH}/${CURR_EXE}
	)
ENDFOREACH( SRC ${SOURCES} )

ADD_CUSTOM_TARGET( test_verbose
	COMMAND
		${CMAKE_CTEST_COMMAND} --verbose
)
