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
PROJECT( SOME_PROJECT CXX C )

#
# Folder path for generated executables
#
SET( EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR} )

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

FIND_PACKAGE( ACADO REQUIRED )
#FIND_PACKAGE( MHEExport REQUIRED )

################################################################################
#
# User configuration
#
################################################################################

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
# Compiler settings
#
################################################################################

ADD_DEFINITIONS( -DACADO_CMAKE_BUILD )

#
# C compiler, C99 support
#
SET( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99" )

################################################################################
#
# Include directories
#
################################################################################

INCLUDE_DIRECTORIES(
    .
    ${ACADO_INCLUDE_DIRS}
    ${MHE_EXPORT_INCLUDE_DIRS}
)

################################################################################
#
# Build applications, libraries and examples
#
################################################################################

#
# Build crane example
#
ADD_EXECUTABLE(
    crane
    crane.cpp
)
TARGET_LINK_LIBRARIES(
	crane
	${ACADO_SHARED_LIBRARIES}
	${MHE_EXPORT_SHARED_LIBRARIES}
)

#
# Build externModel example
#
ADD_EXECUTABLE(
    extern_model
    extern_model.cpp
)
TARGET_LINK_LIBRARIES(
	extern_model
	${ACADO_SHARED_LIBRARIES}
	${MHE_EXPORT_SHARED_LIBRARIES}
)

SET( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR} )
