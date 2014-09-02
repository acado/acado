################################################################################
#
# Description:
#	CMake scipt for building ACADO examples
#
# Authors:
#	Joel Andersson
#	Attila Kozma
#	Milan Vukov, milan.vukov@esat.kuleuven.be
#
# Year:
#	2011 - 2014
#
# NOTE:
#	- It is assumed that one executable has only one source (.cpp) file.
#
# Usage:
#	- TBD
#
#
################################################################################

################################################################################
#
# List of source folders
#
################################################################################

#
# Folders
#
IF (NOT ACADO_BUILD_CGT_ONLY)
	SET( EXAMPLE_DIRS
		basic_data_structures/curve
		basic_data_structures/function
		basic_data_structures/matrix_vector
		basic_data_structures/symbolics
		basic_data_structures/variables_grid
		controller
		getting_started
		integration_algorithm
		integrator
		validated_integrator
		multi_objective
		nlp
		ocp
		parameter_estimation
		process
		simulation_environment
	)
ENDIF()
SET( EXAMPLE_DIRS ${EXAMPLE_DIRS}
	code_generation/simulation
	my_examples
)


################################################################################
#
# ACADO macros
#
################################################################################

INCLUDE( UseACADO )

################################################################################
#
# Compilation of examples
#
################################################################################

SET( EXAMPLES_TESTING_BLACKLIST
	hydroscal
)

#
# Compiling examples from list EXAMPLE_DIRS
#
FOREACH( DIR ${EXAMPLE_DIRS} )
	FILE( GLOB SOURCES ./${DIR}/*.cpp )
	IF( NOT ACADO_DEVELOPER )
		#
		# Remove examples with dev_ prefix
		#
		FILE( GLOB DEV_SOURCES ./${DIR}/dev_*.cpp )
		IF ( DEV_SOURCES )
			LIST( REMOVE_ITEM SOURCES ${DEV_SOURCES} )
		ENDIF ( DEV_SOURCES )
	ENDIF( NOT ACADO_DEVELOPER )

#	MESSAGE( STATUS "SOURCES = ${SOURCES}" )
#	MESSAGE( STATUS "DEV_SOURCES = ${DEV_SOURCES}" )

	SET( SOURCE_PATH ${DIR} )
	STRING( REPLACE "/" "_" DIR "${DIR}" )
#	MESSAGE( STATUS "DIR = ${DIR}" )

	FOREACH( SRC ${SOURCES} )
		GET_FILENAME_COMPONENT( EXEC_NAME ${SRC} NAME_WE )
#		MESSAGE( STATUS "EXEC_NAME = ${EXEC_NAME}" )

		SET( CURR_EXE ${DIR}_${EXEC_NAME} )

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
		
		SET_TARGET_PROPERTIES(
			${CURR_EXE}
			PROPERTIES
				OUTPUT_NAME "${EXEC_NAME}"
				RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/${SOURCE_PATH}"
				# This one is Visual Studio specific
				FOLDER "examples"
		)
		
		IF ( ACADO_WITH_TESTING )
			UNSET( BLACKLISTED )
			
			FOREACH(BL_ITEM ${EXAMPLES_TESTING_BLACKLIST})
				IF (${EXEC_NAME} MATCHES ${BL_ITEM})
					SET( BLACKLISTED YES )
					BREAK()
				ENDIF()
			ENDFOREACH()
			
			# MO examples are not tested ATM
			IF (${SOURCE_PATH} MATCHES "multi_objective")
				SET( BLACKLISTED YES )
			ENDIF()
		
			IF (NOT BLACKLISTED)
				ADD_TEST(
					NAME
						${DIR}_${EXEC_NAME}_test
					WORKING_DIRECTORY
						"${CMAKE_CURRENT_SOURCE_DIR}/${SOURCE_PATH}"
					COMMAND
						${DIR}_${EXEC_NAME}
				)
			ENDIF (NOT BLACKLISTED)
		ENDIF()

	ENDFOREACH( SRC ${SOURCES} )
ENDFOREACH( DIR ${EXAMPLE_DIRS} )

#
# Build code generation examples
#
ADD_SUBDIRECTORY( ./code_generation/mpc_mhe )
ADD_SUBDIRECTORY( ./code_generation/empc )
