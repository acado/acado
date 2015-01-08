################################################################################
#
# Description:
#	CMake scipt for building code-generation ACADO examples
#
# Authors:
#	Milan Vukov, milan.vukov@esat.kuleuven.be
#
# Year:
#	2012 - 2014.
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
# Temporary patch: remove closed loop examples and test files
#
FILE( GLOB CLOSED_LOOP_SOURCES ./*_closed_loop.cpp )
FILE( GLOB CLOSED_LOOP_SOURCES ./*_test.cpp )
IF ( CLOSED_LOOP_SOURCES )
	LIST( REMOVE_ITEM SOURCES ${CLOSED_LOOP_SOURCES} )
ENDIF ( CLOSED_LOOP_SOURCES )

#
# Compile examples
#
FOREACH( SRC ${SOURCES} )
	GET_FILENAME_COMPONENT( EXEC_NAME ${SRC} NAME_WE )
	SET( CURR_EXE code_generation_${EXEC_NAME} )
	ACADO_APPLICATION( ${CURR_EXE} ${SRC} )
ENDFOREACH( SRC ${SOURCES} )

################################################################################
#
# Compilation of tests 
#
################################################################################

ACADO_GENERATE_COMPILE(
	# Target name
	code_generation_crane_kul_mhe
	# Code export folder name
	crane_kul_mhe_export
	# Test file source name
	crane_kul_mhe_test.cpp
)

ACADO_GENERATE_COMPILE(
	code_generation_pendulum_dae_nmpc
	pendulum_dae_nmpc_export
	pendulum_dae_nmpc_test.cpp
)

################################################################################
#
# Closed-loop Simulink example
#
################################################################################

ADD_SUBDIRECTORY( crane_closed_loop )
