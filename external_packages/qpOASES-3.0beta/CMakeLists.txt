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
##
##  NOTE:      This is (a very) modified version of the original qpOASES build
##             script. 


CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )

PROJECT( qpOASES CXX )

SET( qpOASES_VERSION_MAJOR 3 )
SET( qpOASES_VERSION_MINOR 0 )
SET( qpOASES_VERSION_PATCH beta )
SET( qpOASES_VERSION_STRING
	${qpOASES_VERSION_MAJOR}.${qpOASES_VERSION_MINOR}.${qpOASES_VERSION_PATCH}
)

#
# Project library output directory
#
SET( LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/libs )

IF ( UNIX )
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D__NO_COPYRIGHT__")
ELSEIF( WINDOWS )
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -nologo -EHsc -DWIN32 -Dsnprintf=_snprintf -D__NO_COPYRIGHT__")
ENDIF()


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

#
# Build only the static library with extra features
#
ADD_LIBRARY(acado_qpOASESextras STATIC ${QPOASES_SOURCES} ${QPOASES_EXTRAS_SOURCES})

IF ( QPOASES_WITH_LAPACK AND LAPACK_FOUND )
	TARGET_LINK_LIBRARIES(
		acado_qpOASESextras
		${LAPACK_LIBRARIES}
	)
ENDIF()

IF ( QPOASES_WITH_BLAS AND BLAS_FOUND )
	TARGET_LINK_LIBRARIES(
		acado_qpOASESextras
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
		TARGET_LINK_LIBRARIES(${ELEMENT} acado_qpOASESextras)
	ENDFOREACH(ELEMENT ${EXAMPLE_NAMES})

	FOREACH(ELEMENT ${EXTRAS_EXAMPLE_NAMES})
		ADD_EXECUTABLE(${ELEMENT} examples/${ELEMENT}.cpp)
		TARGET_LINK_LIBRARIES(${ELEMENT} acado_qpOASESextras)
	ENDFOREACH(ELEMENT ${EXTRAS_EXAMPLE_NAMES})
ENDIF( QPOASES_WITH_EXAMPLES )

#
# Installation rules
#

INSTALL(
	DIRECTORY
		include
	DESTINATION
		${INSTALL_DATA_DIR}/external_packages/qpOASES-3.0
)


##
##   end of file
##
