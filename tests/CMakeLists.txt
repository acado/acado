################################################################################
#
# Description:
#	ACADO unit tests
#
# Authors:
#	Milan Vukov, milan.vukov@esat.kuleuven.be
#
# Year:
#	2013.
#
# Usage:
#	- TBD
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
PROJECT( ACADO_UNIT_TESTS CXX C )

#
# Folder path for generated executables
#
SET( EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin )

#
# Folder path for generated libraries
#
SET( LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/libs )

#
# CMake module(s) path
#
SET( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/../cmake" )

################################################################################
#
# User configuration
#
################################################################################

#
# Set this to true (ON) if you are using makefiles and want to see the full
# compile and link commands instead of only the shortened ones
#
IF( NOT CMAKE_VERBOSE_MAKEFILE )
	SET( CMAKE_VERBOSE_MAKEFILE OFF )
ENDIF( NOT CMAKE_VERBOSE_MAKEFILE )

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
# Dependencies
#
################################################################################

IF( NOT UNIX )
	MESSAGE( ERROR "At the moment, unit tests are designed to work only on UNIX platforms" )
ENDIF()

FIND_PACKAGE( Boost COMPONENTS system filesystem unit_test_framework REQUIRED )

IF( NOT ACADO_BUILD )
	FIND_PACKAGE( ACADO REQUIRED )
ENDIF()

################################################################################
#
# Adding unit tests
#
################################################################################
