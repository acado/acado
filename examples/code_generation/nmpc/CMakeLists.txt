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
#	CMake scipt for building code-generation ACADO examples
#
# Authors:
#	Milan Vukov, milan.vukov@esat.kuleuven.be
#
# Year:
#	2012
#
# NOTE:
#	- It is assumed that one executable has only one source (.cpp) file.
#
# Usage:
#	- This file should be called from the main CMake script for examples.
#
# TODO:
#	- Simplify build rules for closed loop example(s).
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
# Temporary patch: remove closed loop examples
#
FILE( GLOB CLOSED_LOOP_SOURCES ./*_closed_loop.cpp )
IF ( CLOSED_LOOP_SOURCES )
	LIST( REMOVE_ITEM SOURCES ${CLOSED_LOOP_SOURCES} )
ENDIF ( CLOSED_LOOP_SOURCES )

#
# Compile examples
#
FOREACH( SRC ${SOURCES} )
	GET_FILENAME_COMPONENT( EXEC_NAME ${SRC} NAME_WE )

	ADD_EXECUTABLE( code_generation_${EXEC_NAME} ${SRC} )

	TARGET_LINK_LIBRARIES(
		code_generation_${EXEC_NAME}
		${ACADO_LIBS}
	)
		
	SET_TARGET_PROPERTIES(
		code_generation_${EXEC_NAME}
		PROPERTIES
			OUTPUT_NAME "${EXEC_NAME}"
			RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
			# This one is Visual Studio specific
			FOLDER "examples"
	)		

ENDFOREACH( SRC ${SOURCES} )

################################################################################
#
# Compilation of the closed loop example. The following code first calls an
# executable that generates the code, and afterwards a test application is built
# that performs a closed loop simulation with the auto-generated NMPC. 
#
################################################################################

#
# Compile the closed loop example
#

# Build library out of auto-generated code
SET( GENERATED_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/getting_started_export/acado.h
    ${CMAKE_CURRENT_SOURCE_DIR}/getting_started_export/condensing.c
    ${CMAKE_CURRENT_SOURCE_DIR}/getting_started_export/gauss_newton_method.c
    ${CMAKE_CURRENT_SOURCE_DIR}/getting_started_export/integrator.c
    ${CMAKE_CURRENT_SOURCE_DIR}/getting_started_export/qpoases/solver.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/getting_started_export/qpoases/solver.hpp
)

# Add the additional folders to the build process
INCLUDE_DIRECTORIES(
	${CMAKE_CURRENT_SOURCE_DIR}/getting_started_export/qpoases
	${ACADO_QPOASES_EMBEDDED_INC_DIRS}
)

# Add a patch definition
ADD_DEFINITIONS( -DACADO_CMAKE_BUILD )

# Get the full name of the executable getting_started
GET_TARGET_PROPERTY(
    MY_GENERATOR_EXE
        code_generation_getting_started LOCATION
)

# Create a command that will generate the code for us
ADD_CUSTOM_COMMAND(
    OUTPUT
        ${GENERATED_FILES}       
    COMMAND
        ${MY_GENERATOR_EXE}
	WORKING_DIRECTORY
		${CMAKE_CURRENT_SOURCE_DIR}
	DEPENDS
        code_generation_getting_started
)

# Build a test application
ADD_EXECUTABLE(
	# Name of the executable
    getting_started_closed_loop
    
    # A test application source
    getting_started_closed_loop.cpp
    
    # Auto-generated sources
    ${GENERATED_FILES}
    
    # qpOASES embedded sources
	${ACADO_QPOASES_EMBEDDED_SOURCES}
)
TARGET_LINK_LIBRARIES(
	# Name of the executable
    getting_started_closed_loop
    
    # Link against ACADO libraries
    ${ACADO_LIBS}
)
SET_TARGET_PROPERTIES(
	getting_started_closed_loop
	PROPERTIES
		RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)
