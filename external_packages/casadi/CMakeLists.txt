################################################################################
#
# Description:
#	CMake script for building a part of CasADi (www.casadi.org) software package
#
# Authors:
#	Milan Vukov, milan.vukov@esat.kuleuven.be
#
#
################################################################################

#
# Minimum required version of cmake 
#
CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )

#
# Project name and programming languages used
#
PROJECT( ACADO_CASADI CXX )

#
# Project info
#
SET( ACADO_CASADI_VERSION "1.5.0" )
SET( ACADO_CASADI_SOVERSION "1" )

#
# Project library output directory
#
SET( LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/libs )

################################################################################
#
# Compilation of stand-alone libraries
#
################################################################################

#
# Collect all source and header files
#
FILE( GLOB ACADO_CASADI_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/symbolic/*.cpp )
FILE( GLOB ACADO_CASADI_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/symbolic/*.hpp )

#
# Make static libraries
#
ADD_LIBRARY( acado_casadi STATIC ${ACADO_CASADI_SOURCES} )

INSTALL(
	FILES
		${ACADO_CASADI_HEADERS}
	DESTINATION
		${INSTALL_DATA_DIR}/external_packages/casadi/symbolic
)
