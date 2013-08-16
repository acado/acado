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
#	Compiler Options for building ACADO static and shared libraries
#
# Authors:
#	Joel Andersson
#	Attila Kozma
#	Milan Vukov, milan.vukov@esat.kuleuven.be
#
# Credits:
#	Enrico Bertolazzi, Universita` degli Studi di Trento
#
# Year:
#	2011 - 2013
#
# Usage:
#	- /
#
################################################################################

################################################################################
#
# Compiler settings - General
#
################################################################################

#
# ACADO build flag
#
SET( ACADO_BUILD ON )

#
# Temporary patch, needed for compilation of the qpOASES embedded (at least)
#
ADD_DEFINITIONS( -DACADO_CMAKE_BUILD )

#
# CMake RPATH handling, http://www.cmake.org/Wiki/CMake_RPATH_handling
#

# Use, i.e. don't skip the full RPATH for the build tree
SET(CMAKE_SKIP_BUILD_RPATH  FALSE)

# When building, don't use the install RPATH already
# (but later on when installing)
SET(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE) 

SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

# Add the automatically determined parts of the RPATH
# which point to directories outside the build tree to the install RPATH
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)


# The RPATH to be used when installing, but only if it's not a system directory
LIST(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/lib" isSystemDir)
IF("${isSystemDir}" STREQUAL "-1")
   SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
ENDIF("${isSystemDir}" STREQUAL "-1")

#
# Includes
#
INCLUDE( CompilerOptionsSSE )

# define __DEBUG__ for debug builds
SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -D__DEBUG__")
SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -D__DEBUG__")

################################################################################
#
# Compiler settings - GCC/G++; Linux, Apple
#
################################################################################
IF( CMAKE_COMPILER_IS_GNUCXX OR CMAKE_COMPILER_IS_GNUCC OR "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang" )

	#
	# Compiler options from original Makefiles
	#
	
	# Cygwin complains in about the -fPIC flag...
	IF( NOT CYGWIN )
		SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC" )
		SET( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC" )
	ENDIF( NOT CYGWIN )
	
	SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pedantic -Wfloat-equal -Wshadow -DLINUX" )
	SET( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -pedantic -Wfloat-equal -Wshadow -DLINUX" )
	
	IF( ACADO_DEVELOPER )
#		SET( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Winline" )
#		SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Winline" )
	ELSE()
		SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-overloaded-virtual" )
	ENDIF()

	#
	# Some common stuff...
	#
	SET(CMAKE_CXX_FLAGS_DEBUG			"${CMAKE_CXX_FLAGS_DEBUG}")
	SET(CMAKE_CXX_FLAGS_MINSIZEREL		"${CMAKE_CXX_FLAGS_MINSIZEREL} ")
	SET(CMAKE_CXX_FLAGS_RELEASE			"${CMAKE_CXX_FLAGS_RELEASE} -O3")
	SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO	"${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -DNDEBUG -O3")
	SET(CMAKE_C_FLAGS_DEBUG				"${CMAKE_C_FLAGS_DEBUG}")
	SET(CMAKE_C_FLAGS_MINSIZEREL		"${CMAKE_C_FLAGS_MINSIZEREL} ")
	SET(CMAKE_C_FLAGS_RELEASE			"${CMAKE_C_FLAGS_RELEASE} -O3")
	SET(CMAKE_C_FLAGS_RELWITHDEBINFO	"${CMAKE_C_FLAGS_RELWITHDEBINFO} -DNDEBUG -O3")
	
	IF ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
		SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-comparison" )
	ENDIF()

	#
	# Apple specifics
	#
	IF( APPLE )
		SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -g")
		SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")
		
#		SET( CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -funroll-loops -ftree-vectorize  -pthtread" )
#		SET( CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -funroll-loops -ftree-vectorize -gstabs+ -pthtread" )

		# Build only for 64-bit arch. This requires (at least) OS X version >= 10.7.
		IF ( NOT CMAKE_OSX_ARCHITECTURES ) 
			SET( CMAKE_OSX_ARCHITECTURES "x86_64;" )
		ENDIF()

	ENDIF( APPLE )

	#
	# Adding SSE compilation flags
	#
	SET( CMAKE_C_FLAGS_RELEASE   "${CMAKE_C_FLAGS_RELEASE} ${SSE_FLAGS}" )
	SET( CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}  ${SSE_FLAGS}" )
	
	#
	# If we are running test suite, then turn off plotting
	#
	IF( ACADO_WITH_TESTING )
		ADD_DEFINITIONS( -D__NO_PIPES__ )
		ADD_DEFINITIONS( -D__NO_PLOTTING__ )
	ENDIF()

################################################################################
#
# Compiler settings - MS Visual Studio; Windows
#
################################################################################
ELSEIF( MSVC )
	SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -nologo -EHsc " )
	
	#
	# Check for Gnuplot installation
	#
	SET(GNUPLOT_EXECUTABLE_PATH "C:/gnuplot/bin" CACHE STRING
		"Abosulute path of the Gnuplot executable."
	)
	FIND_PROGRAM( GNUPLOT_EXECUTABLE
		NAMES gnuplot
		PATHS ${GNUPLOT_EXECUTABLE_PATH}
		NO_DEFAULT_PATH
	)
	IF( GNUPLOT_EXECUTABLE )
		MESSAGE( STATUS "Looking for Gnuplot executable: found." )
	ELSE ()
		MESSAGE( STATUS "Looking for Gnuplot executable: not found." )
	ENDIF()
	MARK_AS_ADVANCED( FORCE GNUPLOT_EXECUTABLE )
	
	#
	# Some common definitions
	#
	ADD_DEFINITIONS( -DWIN32 )
	ADD_DEFINITIONS( -D__NO_COPYRIGHT__ )
	ADD_DEFINITIONS( -Dsnprintf=_snprintf )
	ADD_DEFINITIONS( -Dusleep=Sleep )
	ADD_DEFINITIONS( -Dsleep=Sleep )
	ADD_DEFINITIONS( -D_CRT_SECURE_NO_WARNINGS )
	ADD_DEFINITIONS( -D__NO_PIPES__ )
	
	IF( GNUPLOT_EXECUTABLE )
		ADD_DEFINITIONS( -DGNUPLOT_EXECUTABLE="${GNUPLOT_EXECUTABLE}" )
	ENDIF()

	#
	# Enable project grouping when making Visual Studio solution
	# NOTE: This feature is NOT supported in Express editions
	#
	SET_PROPERTY( GLOBAL PROPERTY USE_FOLDERS ON )

ENDIF( )

