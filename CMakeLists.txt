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
#	CMake script for building ACADO static and shared libraries
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
#	- In order to build the API documentation, you will need Doxygen installed.
#
# Usage:
#	- Linux
#
#		* If you want to build examples along with libraries, enable the option
#		WITH_ACADO_EXAMPLES before running CMake.
#
#		* For building type:
#
#		mkdir build
#		cd build
#		cmake ..
#		make clean all
#
#		* For installation type:
#
#		sudo make install 
#
#		Last command will install libraries, headers, etc., and you will need
#		admin privileges to execute it.
#
#		* If you want to build API documentation, type:
#
#		make doc
#
#		* For uninstallation type:
#
#		sudo make uninstall
#
#	- Apple: TODO
#
#	- Windows: TODO
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
PROJECT( ACADO CXX C )

#
# Project versioning information
#
SET( ACADO_VERSION_MAJOR  1 )
SET( ACADO_VERSION_MINOR  1 )
SET( ACADO_VERSION_PATCH  0 )
SET( ACADO_VERSION_STRING
		${ACADO_VERSION_MAJOR}.${ACADO_VERSION_MINOR}.${ACADO_VERSION_PATCH}
)

#
# Folder path for generated executables
#
SET( EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin )

#
# Folder path for generated libraries
#
SET( LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/libs )

#
# Folder path for testing
#
SET( TESTING_OUTPUT_PATH ${PROJECT_BINARY_DIR}/testing/automatic_testing )

#
# CMake module(s) path
#
SET( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake" )

#
# ACADO build flag
#
SET( ACADO_BUILD ON )

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
# Enable building of ...
#
OPTION( WITH_ACADO_EXAMPLES
	"Building ACADO examples"
	ON
)

#
# Compilation of testing suite
#
OPTION( WITH_TESTING
	"Compiling with tests"
	OFF
)

#
# ACADO developer flag
#
SET( ACADO_DEVELOPER OFF CACHE STRING
	"Choose developer/user building mode"
)

#
# Build type
#
IF( NOT CMAKE_BUILD_TYPE )
	SET(CMAKE_BUILD_TYPE Release CACHE STRING
		"Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
		FORCE
	)
ENDIF( NOT CMAKE_BUILD_TYPE )

#
# Offer the user the choice of overriding the installation directories
#
IF ( ${CMAKE_SYSTEM_NAME} MATCHES Windows )
	SET( LIB_DIR "lib/acado" )
#	SET( SHARE_DIR "" )
ELSE( )
	SET( LIB_DIR "/usr/lib" )
#	SET( SHARE_DIR "share" )
ENDIF( )

SET( INSTALL_LIB_DIR ${LIB_DIR} CACHE PATH
	"Installation directory for libraries"
)
SET( INSTALL_BIN_DIR bin CACHE PATH
	"Installation directory for executables"
)
SET( INSTALL_INCLUDE_DIR include/acado CACHE PATH
	"Installation directory for header files"
)

IF ( SHARE_DIR )
	SET( INSTALL_DATA_DIR ${SHARE_DIR}/acado CACHE PATH
		"Installation directory for data files"
	)
	SET( INSTALL_CMAKE_DIR ${SHARE_DIR}/acado/cmake CACHE PATH
		"Installation directory for CMake files"
	)
ELSE( SHARE_DIR )
	SET( INSTALL_DATA_DIR acado CACHE PATH
		"Installation directory for data files"
	)
	SET( INSTALL_CMAKE_DIR acado/cmake CACHE PATH
		"Installation directory for CMake files"
	)
ENDIF( SHARE_DIR )

# Make relative installation paths absolute (needed later on)
FOREACH( p LIB BIN INCLUDE DATA CMAKE )
  SET( var INSTALL_${p}_DIR )
  IF( NOT IS_ABSOLUTE "${${var}}" )
    SET( ${var} "${CMAKE_INSTALL_PREFIX}/${${var}}" )
  ENDIF()
ENDFOREACH()

################################################################################
#
# Dependencies
#
################################################################################

FIND_PACKAGE( Doxygen )

################################################################################
#
# Compiler settings
#
################################################################################

#
# Compiler options are in separate file
#
INCLUDE( CompilerOptions )

################################################################################
#
# Libraries - lists of source folders
#
################################################################################

SET( ACADO_SOURCE_DIRS
    # Integrators source folders
	./src/utils
	./src/clock
	./src/matrix_vector
	./src/variables_grid
	./src/curve
	./src/user_interaction
	./src/symbolic_operator
	./src/symbolic_expression
	./src/function
	./src/integrator
	./src/sparse_solver
	./external_packages/src/acado_csparse
	./external_packages/src/acado_gnuplot
	
	# Optimal Control source folders
	./src/code_generation
	./src/code_generation/integrators
	./src/code_generation/linear_solvers
	./src/dynamic_system
	./src/dynamic_discretization
	./src/objective
	./src/constraint
	./src/ocp
	./src/conic_program
	./src/conic_solver
	./src/nlp_solver
	./src/nlp_derivative_approximation
	./src/optimization_algorithm
	./external_packages/src/acado_qpoases
	
	# Simulation source folders
	./src/process
	./src/noise
	./src/transfer_device
	./src/controller
	./src/control_law
	./src/estimator
	./src/reference_trajectory
	./src/simulation_environment
)

################################################################################
#
# Include directories
#
################################################################################

INCLUDE_DIRECTORIES(
	${PROJECT_SOURCE_DIR}/include
	${PROJECT_SOURCE_DIR}/external_packages
	${PROJECT_SOURCE_DIR}/external_packages/qpOASES-3.0beta/include
	${PROJECT_BINARY_DIR}
)

################################################################################
#
# Configure templates
#
################################################################################

ADD_SUBDIRECTORY( ./src/code_generation/templates )

################################################################################
#
# Building of stand-alone libraries
#
################################################################################

#
# Build ACADO Toolkit library
#
UNSET( ACADO_SOURCES )
FOREACH( DIR ${ACADO_SOURCE_DIRS} )
	FILE( GLOB SRC ${DIR}/*.cpp )
	SET( ACADO_SOURCES ${ACADO_SOURCES} ${SRC} )
ENDFOREACH( DIR ${ACADO_SOURCE_DIRS} )
ADD_LIBRARY( acado_toolkit STATIC ${ACADO_SOURCES} )
TARGET_LINK_LIBRARIES(
	acado_toolkit
	qpOASESextras csparse
)
ADD_LIBRARY( acado_toolkit_s SHARED ${ACADO_SOURCES} )
SET_TARGET_PROPERTIES( acado_toolkit_s
	PROPERTIES
		VERSION ${ACADO_VERSION_STRING}
		SOVERSION ${ACADO_VERSION_MAJOR}
)

# Dependencies, dependencies...
IF( NOT MSVC )
	# Linux is assumed
	TARGET_LINK_LIBRARIES(
		acado_toolkit_s
		qpOASESextras_s csparse_s
	)
ELSE( NOT MSVC )
	# Windows, Visual Studio environment specifics...
	TARGET_LINK_LIBRARIES(
		acado_toolkit_s
		qpOASESextras csparse
	)
ENDIF( NOT MSVC )

################################################################################
#
# Building external packages
#
################################################################################

#
# Build qpOASES and qpOASESextras libraries
#
ADD_SUBDIRECTORY( ./external_packages/qpOASES-3.0beta )

#
# Build csparse library
#
ADD_SUBDIRECTORY( ./external_packages/csparse )

################################################################################
#
# TODO Automatic testing
#
################################################################################

#
# Tests
#
#IF( WITH_TESTING )
#	ENABLE_TESTING()
#	INCLUDE( CTest )
#	SET( DBUILD_DASHBOARD_REPORTS = ON )
#	ADD_SUBDIRECTORY( ./testing/automatic_testing ${TESTING_OUTPUT_PATH} )
#ENDIF( WITH_TESTING )

################################################################################
#
# Installation rules
#
################################################################################

#
# Install ACADO Toolkit libs
#
INSTALL(
	TARGETS
		acado_toolkit acado_toolkit_s
	DESTINATION
		${INSTALL_LIB_DIR}
)

#
# Install ACADO include folder
#
INSTALL(
	DIRECTORY
		include/
	DESTINATION
		${INSTALL_INCLUDE_DIR}
	PATTERN ".svn" EXCLUDE
)

#
# Install (parts of) external packages and interfaces to those packages
#
SET( EXTERNAL_PACKAGES_DIRS
	external_packages/include
	external_packages/qpoases
)	
INSTALL(
	DIRECTORY
		${EXTERNAL_PACKAGES_DIRS}
	DESTINATION
		${INSTALL_DATA_DIR}/external_packages
	PATTERN ".svn" EXCLUDE
	# Temporary patch
	PATTERN "xml" EXCLUDE
)

#
# Configure and install CMake package configuration file
#
SET( ACADO_INCLUDE_DIRS
	${INSTALL_INCLUDE_DIR}
	${INSTALL_DATA_DIR}/external_packages
	${INSTALL_DATA_DIR}/external_packages/csparse/
	${INSTALL_DATA_DIR}/external_packages/qpOASES-3.0beta/include
)
SET( ACADO_LIBRARY_DIRS
	${INSTALL_LIB_DIR}
)
SET( ACADO_EXTERNAL_PACKAGES_DIR
	${INSTALL_DATA_DIR}/external_packages
)
SET( ACADO_CMAKE_DIR
	${INSTALL_CMAKE_DIR}
)

SET( ACADO_STATIC_LIBRARIES
	acado_toolkit acado_qpOASESextras acado_csparse
)
SET( ACADO_SHARED_LIBRARIES
	acado_toolkit_s acado_qpOASESextras_s acado_csparse_s
)

SET( ACADO_QPOASES_EMBEDDED_SOURCES
	${INSTALL_DATA_DIR}/external_packages/qpoases/SRC/Bounds.cpp
	${INSTALL_DATA_DIR}/external_packages/qpoases/SRC/CyclingManager.cpp 
	${INSTALL_DATA_DIR}/external_packages/qpoases/SRC/MessageHandling.cpp
	${INSTALL_DATA_DIR}/external_packages/qpoases/SRC/QProblem.cpp
	${INSTALL_DATA_DIR}/external_packages/qpoases/SRC/Utils.cpp
	${INSTALL_DATA_DIR}/external_packages/qpoases/SRC/Constraints.cpp
	${INSTALL_DATA_DIR}/external_packages/qpoases/SRC/Indexlist.cpp	     
	${INSTALL_DATA_DIR}/external_packages/qpoases/SRC/QProblemB.cpp
	${INSTALL_DATA_DIR}/external_packages/qpoases/SRC/SubjectTo.cpp
	${INSTALL_DATA_DIR}/external_packages/qpoases/SRC/EXTRAS/SolutionAnalysis.cpp
)
SET( ACADO_QPOASES_EMBEDDED_INC_DIRS
	${INSTALL_DATA_DIR}/external_packages/qpoases/
	${INSTALL_DATA_DIR}/external_packages/qpoases/INCLUDE
	${INSTALL_DATA_DIR}/external_packages/qpoases/SRC
)

CONFIGURE_FILE(
	"${PROJECT_SOURCE_DIR}/cmake/ACADOConfig.cmake.in"
	"${PROJECT_BINARY_DIR}/ACADOConfig.cmake"
	@ONLY
)
INSTALL(
	FILES
		${PROJECT_BINARY_DIR}/ACADOConfig.cmake
	DESTINATION
		${INSTALL_CMAKE_DIR}
)

################################################################################
#
# Rules for un-installing ACADO toolkit
#
################################################################################

CONFIGURE_FILE(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/ACADOUninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/ACADOUninstall.cmake"
    IMMEDIATE @ONLY
)

ADD_CUSTOM_TARGET( uninstall
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/ACADOUninstall.cmake
)

################################################################################
#
# Building API documentation
#
################################################################################

IF( DOXYGEN_FOUND )
	ADD_CUSTOM_TARGET( doc
		${DOXYGEN_EXECUTABLE} ${PROJECT_SOURCE_DIR}/doc/doxygen.config
		WORKING_DIRECTORY
			${PROJECT_SOURCE_DIR}/doc
		COMMENT
			"Generating API documentation with Doxygen" VERBATIM
	)
ENDIF( DOXYGEN_FOUND )

################################################################################
#
# Configuration of the bash script
#
################################################################################

SET( ACADO_ENV_INCLUDE_DIRS
	${PROJECT_SOURCE_DIR}/include
	${PROJECT_SOURCE_DIR}/external_packages
	${PROJECT_SOURCE_DIR}/external_packages/csparse/
	${PROJECT_SOURCE_DIR}/external_packages/qpOASES-3.0beta/include
)

SET( ACADO_ENV_LIBRARY_DIRS
	${LIBRARY_OUTPUT_PATH}
	${qpOASES_BINARY_DIR}/libs
	${CSPARSE_BINARY_DIR}/libs
)

SET( ACADO_ENV_EXTERNAL_PACKAGES_DIR
	${PROJECT_SOURCE_DIR}/external_packages
)

SET( ACADO_ENV_BUILD_DIR
	${PROJECT_BINARY_DIR}
)

SET( ACADO_ENV_STATIC_LIBRARIES 
	${ACADO_STATIC_LIBRARIES}
)

SET( ACADO_ENV_SHARED_LIBRARIES
	${ACADO_SHARED_LIBRARIES}
)

SET( ACADO_ENV_QPOASES_EMBEDDED_SOURCES
	${PROJECT_SOURCE_DIR}/external_packages/qpoases/SRC/Bounds.cpp
	${PROJECT_SOURCE_DIR}/external_packages/qpoases/SRC/CyclingManager.cpp 
	${PROJECT_SOURCE_DIR}/external_packages/qpoases/SRC/MessageHandling.cpp
	${PROJECT_SOURCE_DIR}/external_packages/qpoases/SRC/QProblem.cpp
	${PROJECT_SOURCE_DIR}/external_packages/qpoases/SRC/Utils.cpp
	${PROJECT_SOURCE_DIR}/external_packages/qpoases/SRC/Constraints.cpp
	${PROJECT_SOURCE_DIR}/external_packages/qpoases/SRC/Indexlist.cpp	     
	${PROJECT_SOURCE_DIR}/external_packages/qpoases/SRC/QProblemB.cpp
	${PROJECT_SOURCE_DIR}/external_packages/qpoases/SRC/SubjectTo.cpp
	${PROJECT_SOURCE_DIR}/external_packages/qpoases/SRC/EXTRAS/SolutionAnalysis.cpp
)

SET( ACADO_ENV_QPOASES_EMBEDDED_INC_DIRS
	${PROJECT_SOURCE_DIR}/external_packages/qpoases/
	${PROJECT_SOURCE_DIR}/external_packages/qpoases/INCLUDE
	${PROJECT_SOURCE_DIR}/external_packages/qpoases/SRC
)

CONFIGURE_FILE(
	"${PROJECT_SOURCE_DIR}/cmake/acado_env.sh.in"
	"${PROJECT_BINARY_DIR}/acado_env.sh"
	@ONLY
)

################################################################################
#
# Build examples
#
################################################################################

IF( WITH_ACADO_EXAMPLES )
	ADD_SUBDIRECTORY( ./examples )
ENDIF( WITH_ACADO_EXAMPLES )
