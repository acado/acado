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
