##
##	This file is part of qpOASES.
##
##	qpOASES -- An Implementation of the Online Active Set Strategy.
##	Copyright (C) 2007-2011 by Hans Joachim Ferreau, Andreas Potschka,
##	Christian Kirches et al. All rights reserved.
##
##	qpOASES is free software; you can redistribute it and/or
##	modify it under the terms of the GNU Lesser General Public
##	License as published by the Free Software Foundation; either
##	version 2.1 of the License, or (at your option) any later version.
##
##	qpOASES is distributed in the hope that it will be useful,
##	but WITHOUT ANY WARRANTY; without even the implied warranty of
##	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
##	See the GNU Lesser General Public License for more details.
##
##	You should have received a copy of the GNU Lesser General Public
##	License along with qpOASES; if not, write to the Free Software
##	Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
##



##
##	Filename:  CMakeLists.txt
##	Authors:   Hans Joachim Ferreau, Milan Vukov
##	Version:   3.0beta
##	Date:      2007-2013.
##


CMAKE_MINIMUM_REQUIRED( VERSION 2.6 )

PROJECT( qpOASES CXX )

SET( qpOASES_VERSION_MAJOR 3 )
SET( qpOASES_VERSION_MINOR 0 )
SET( qpOASES_VERSION_PATCH beta )
SET( qpOASES_VERSION_STRING
	${qpOASES_VERSION_MAJOR}.${qpOASES_VERSION_MINOR}.${qpOASES_VERSION_PATCH}
)

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/libs)

IF ( UNIX )
	SET(CMAKE_CXX_FLAGS "-Wall -pedantic -Wshadow -O3 -finline-functions -DLINUX -D__NO_COPYRIGHT__")
ELSEIF( WINDOWS )
	SET(CMAKE_CXX_FLAGS "-nologo -EHsc -DWIN32 -Dsnprintf=_snprintf -D__NO_COPYRIGHT__")
ENDIF()

# SET(CMAKE_VERBOSE_MAKEFILE ON)

OPTION( QPOASES_WITH_EXAMPLES
	"Build qpOASES examples"
	OFF
)
OPTION( QPOASES_WITH_LAPACK
	"Build qpOASES using original LAPACK routines"
	OFF
)
OPTION( QPOASES_WITH_BLAS
	"Build qpOASES using original BLAS routines"
	OFF
)

IF ( ACADO_BUILD )
	SET( QPOASES_PREFIX acado_ )
ENDIF()

INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/include )

#
# compile qpOASES libraries
#

SET( QPOASES_SOURCES
	src/Bounds.cpp
	src/Constraints.cpp
	src/Flipper.cpp
	src/Indexlist.cpp
	src/Matrices.cpp
	src/MessageHandling.cpp
	src/Options.cpp
	src/QProblem.cpp
	src/QProblemB.cpp
	src/SQProblem.cpp
	src/SubjectTo.cpp
	src/Utils.cpp
)

# find LAPACK
IF ( QPOASES_WITH_LAPACK )
	ENABLE_LANGUAGE( C )
	ENABLE_LANGUAGE( Fortran )

	FIND_PACKAGE( LAPACK )

	IF ( NOT LAPACK_FOUND )
		SET( QPOASES_SOURCES ${QPOASES_SOURCES} src/LAPACKReplacement.cpp )
	ENDIF ( NOT LAPACK_FOUND )

ELSE ( QPOASES_WITH_LAPACK )
	SET( QPOASES_SOURCES ${QPOASES_SOURCES} src/LAPACKReplacement.cpp )
ENDIF ( QPOASES_WITH_LAPACK )

# find BLAS
IF ( QPOASES_WITH_BLAS )
	ENABLE_LANGUAGE( C )
	ENABLE_LANGUAGE( Fortran )

	FIND_PACKAGE( BLAS )

	IF ( NOT BLAS_FOUND )
		SET( QPOASES_SOURCES ${QPOASES_SOURCES} src/BLASReplacement.cpp )
	ENDIF ( NOT BLAS_FOUND )

ELSE ( QPOASES_WITH_BLAS )
	SET( QPOASES_SOURCES ${QPOASES_SOURCES} src/BLASReplacement.cpp )
ENDIF ( QPOASES_WITH_BLAS )

SET( QPOASES_EXTRAS_SOURCES
	src/extras/OQPinterface.cpp
	src/extras/SolutionAnalysis.cpp
)

ADD_LIBRARY(${QPOASES_PREFIX}qpOASES STATIC ${QPOASES_SOURCES})
ADD_LIBRARY(${QPOASES_PREFIX}qpOASESextras STATIC ${QPOASES_SOURCES} ${QPOASES_EXTRAS_SOURCES})

ADD_LIBRARY(${QPOASES_PREFIX}qpOASES_s SHARED ${QPOASES_SOURCES})
SET_TARGET_PROPERTIES( ${QPOASES_PREFIX}qpOASES_s
	PROPERTIES
		VERSION ${qpOASES_VERSION_STRING}
		SOVERSION ${qpOASES_VERSION_MAJOR}
)

ADD_LIBRARY(${QPOASES_PREFIX}qpOASESextras_s SHARED ${QPOASES_SOURCES} ${QPOASES_EXTRAS_SOURCES})
SET_TARGET_PROPERTIES( ${QPOASES_PREFIX}qpOASESextras_s
	PROPERTIES
		VERSION ${qpOASES_VERSION_STRING}
		SOVERSION ${qpOASES_VERSION_MAJOR}
)

IF ( QPOASES_WITH_LAPACK AND LAPACK_FOUND )
	TARGET_LINK_LIBRARIES(
		${QPOASES_PREFIX}qpOASES
		${LAPACK_LIBRARIES}
	)

	TARGET_LINK_LIBRARIES(
		${QPOASES_PREFIX}qpOASESextras
		${LAPACK_LIBRARIES}
	)

	TARGET_LINK_LIBRARIES(
		${QPOASES_PREFIX}qpOASES_s
		${LAPACK_LIBRARIES}
	)

	TARGET_LINK_LIBRARIES(
		${QPOASES_PREFIX}qpOASESextras_s
		${LAPACK_LIBRARIES}
	)

ENDIF()

IF ( QPOASES_WITH_BLAS AND BLAS_FOUND )
	TARGET_LINK_LIBRARIES(
		${QPOASES_PREFIX}qpOASES
		${BLAS_LIBRARIES}
	)

	TARGET_LINK_LIBRARIES(
		${QPOASES_PREFIX}qpOASESextras
		${BLAS_LIBRARIES}
	)

	TARGET_LINK_LIBRARIES(
		${QPOASES_PREFIX}qpOASES_s
		${BLAS_LIBRARIES}
	)

	TARGET_LINK_LIBRARIES(
		${QPOASES_PREFIX}qpOASESextras_s
		${BLAS_LIBRARIES}
	)
ENDIF()

# compile examples only if the option is enabled (set to ON)
SET(EXAMPLE_NAMES
example1
example1a
example1b
example4
example5
example6
exampleLP
qrecipe
)

SET(EXTRAS_EXAMPLE_NAMES
example2
example3
example3b
)

IF( QPOASES_WITH_EXAMPLES )
	FOREACH(ELEMENT ${EXAMPLE_NAMES})
		ADD_EXECUTABLE(${ELEMENT} examples/${ELEMENT}.cpp)
		TARGET_LINK_LIBRARIES(${ELEMENT} ${QPOASES_PREFIX}qpOASES)
	ENDFOREACH(ELEMENT ${EXAMPLE_NAMES})

	FOREACH(ELEMENT ${EXTRAS_EXAMPLE_NAMES})
		ADD_EXECUTABLE(${ELEMENT} examples/${ELEMENT}.cpp)
		TARGET_LINK_LIBRARIES(${ELEMENT} ${QPOASES_PREFIX}qpOASESextras)
	ENDFOREACH(ELEMENT ${EXTRAS_EXAMPLE_NAMES})
ENDIF( QPOASES_WITH_EXAMPLES )

# ACADO toolkit related stuff
IF( ACADO_BUILD )
	# Change the output names of the libraries
	SET_TARGET_PROPERTIES( ${QPOASES_PREFIX}qpOASESextras
		PROPERTIES
			OUTPUT_NAME acado_qpOASESextras
	)

	SET_TARGET_PROPERTIES( ${QPOASES_PREFIX}qpOASESextras_s
		PROPERTIES
			OUTPUT_NAME acado_qpOASESextras_s
	)
	
	# installation rules
	INSTALL(
		TARGETS
			${QPOASES_PREFIX}qpOASESextras ${QPOASES_PREFIX}qpOASESextras_s
		DESTINATION
			${INSTALL_LIB_DIR}
	)

	INSTALL(
		DIRECTORY
			include
		DESTINATION
			${INSTALL_DATA_DIR}/external_packages/qpOASES-3.0
		PATTERN ".svn" EXCLUDE
	)
ENDIF( ACADO_BUILD )

##
##   end of file
##
