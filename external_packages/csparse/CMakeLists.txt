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
PROJECT( CSPARSE C )

#
# Project info from file cs.h
#
SET( CSPARSE_VERSION "2.2.3" )
SET( CSPARSE_SOVERSION "2" )

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
FILE( GLOB CSPARSE_SOURCES *.c )
FILE( GLOB CSPARSE_HEADERS *.h )

#
# Make static & shared libraries
#
ADD_LIBRARY( csparse STATIC ${CSPARSE_SOURCES} )
SET_TARGET_PROPERTIES( csparse
	PROPERTIES
		OUTPUT_NAME acado_csparse		
)

ADD_LIBRARY( csparse_s SHARED ${CSPARSE_SOURCES} )
SET_TARGET_PROPERTIES( csparse_s
	PROPERTIES
		OUTPUT_NAME acado_csparse_s
		VERSION ${CSPARSE_VERSION}
		SOVERSION ${CSPARSE_SOVERSION}
)

################################################################################
#
# Installation rules
#
################################################################################

INSTALL(
	TARGETS
		csparse csparse_s
	DESTINATION
		${INSTALL_LIB_DIR}
)

INSTALL(
	FILES
		${CSPARSE_HEADERS}
	DESTINATION
		${INSTALL_DATA_DIR}/external_packages/csparse
)
