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
#	- This file should be called from the main CMake script for examples
#
# TODO:
#	- Closed loop example(s) -- this can work with static libraries only
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

FOREACH( SRC ${SOURCES} )
	GET_FILENAME_COMPONENT( EXEC_NAME ${SRC} NAME_WE )

	ADD_EXECUTABLE( code_generation_${EXEC_NAME} ${SRC} )

	TARGET_LINK_LIBRARIES(
		code_generation_${EXEC_NAME}
		${ACADO_STATIC_LIBRARIES}
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
