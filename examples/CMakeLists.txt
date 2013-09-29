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
#	2011 - 2013
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
# Project settings
#
################################################################################

#
# Minimum required version of cmake 
#
CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )

#
# Project name and programming languages used
#
PROJECT( ACADO_EXAMPLES CXX C )

#
# Folder path for generated executables
#
SET( EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin )

#
# Folder path for generated libraries
#
SET( LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib )

#
# CMake module(s) path
#
SET( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} )

################################################################################
#
# Prerequisites
#
################################################################################

IF ( NOT ACADO_BUILD )
	FIND_PACKAGE( ACADO REQUIRED )

	SET( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "$ENV{ACADO_ENV_CMAKE_DIR}" )

	INCLUDE( CompilerOptions )
	
	IF( ACADO_WITH_TESTING )
		ENABLE_TESTING()
	ENDIF( ACADO_WITH_TESTING )
ENDIF()

################################################################################
#
# User configuration
#
################################################################################

IF( NOT CMAKE_VERBOSE_MAKEFILE )
	SET( CMAKE_VERBOSE_MAKEFILE OFF )
ENDIF( NOT CMAKE_VERBOSE_MAKEFILE )

################################################################################
#
# Compiler settings
#
################################################################################

#
# Build type
#
IF( NOT CMAKE_BUILD_TYPE )
	SET(CMAKE_BUILD_TYPE Debug CACHE STRING
		"Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
		FORCE
	)
ENDIF( NOT CMAKE_BUILD_TYPE )

################################################################################
#
# List of source folders
#
################################################################################

#
# Folders
#
SET( EXAMPLE_DIRS
	basic_data_structures/curve
	basic_data_structures/function
	basic_data_structures/matrix_vector
	basic_data_structures/symbolics
	basic_data_structures/variables_grid
	code_generation/simulation
	controller
	getting_started
	integration_algorithm
	integrator
	validated_integrator
	multi_objective
	my_examples
	nlp
	ocp
	parameter_estimation
	process
	simulation_environment
)

################################################################################
#
# Include directories
#
################################################################################

IF ( NOT ACADO_BUILD )
	INCLUDE_DIRECTORIES( . ${ACADO_INCLUDE_DIRS} )
ENDIF( )

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
