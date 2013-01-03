#
# This file is part of ACADO Toolkit.
#
# ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
# Copyright (C) 2008-2011 by Boris Houska and Hans Joachim Ferreau.
# All rights reserved.
#
# ACADO Toolkit is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.
#
# ACADO Toolkit is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
# Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License along with ACADO Toolkit; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
#

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
#	2011 - 2012
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
	# CMake module(s) path
	SET( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/../cmake" )

	FIND_PACKAGE( ACADO REQUIRED )
	
	SET( ACADO_LIBS
		${ACADO_STATIC_LIBRARIES}
	)
	
ELSE()
	# If we are building withing main ACADO build, correct paths must be set
	SET( ACADO_QPOASES_EMBEDDED_SOURCES
		${ACADO_ENV_QPOASES_EMBEDDED_SOURCES}
	)
	SET( ACADO_QPOASES_EMBEDDED_INC_DIRS
		${ACADO_ENV_QPOASES_EMBEDDED_INC_DIRS}
	)
	
	SET( ACADO_LIBS
		acado_toolkit csparse qpOASESextras acado_casadi
	)
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
#	code_generation
	controller
	getting_started
	integration_algorithm
	integrator
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
# Compilation of examples
#
################################################################################

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

		ADD_EXECUTABLE( ${DIR}_${EXEC_NAME} ${SRC} )

		TARGET_LINK_LIBRARIES(
			${DIR}_${EXEC_NAME}
			${ACADO_LIBS}
		)
		
		SET_TARGET_PROPERTIES(
			${DIR}_${EXEC_NAME}
			PROPERTIES
				OUTPUT_NAME "${EXEC_NAME}"
				RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/${SOURCE_PATH}"
				# This one is Visual Studio specific
				FOLDER "examples"
		)		

	ENDFOREACH( SRC ${SOURCES} )
ENDFOREACH( DIR ${EXAMPLE_DIRS} )

#
# Build code generation examples
#
# ADD_SUBDIRECTORY( ./code_generation )
