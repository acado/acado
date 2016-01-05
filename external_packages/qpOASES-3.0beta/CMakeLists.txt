################################################################################
#
# Description:
#	CMake script for building qpOASES library and examples
#
# Authors:
#   Hans Joachim Ferreau
#	Milan Vukov, milan.vukov@esat.kuleuven.be
#
# Year:
#	2011 - 2014
#
# NOTE:
#   - This is a quite modified version of the original qpOASES build script.
#
# Usage:
#	- TBD
#
#
################################################################################

SET( qpOASES_VERSION_MAJOR 3 )
SET( qpOASES_VERSION_MINOR 0 )
SET( qpOASES_VERSION_PATCH beta )
SET( qpOASES_VERSION_STRING
	${qpOASES_VERSION_MAJOR}.${qpOASES_VERSION_MINOR}.${qpOASES_VERSION_PATCH}
)

IF ( UNIX )
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D__NO_COPYRIGHT__")
ELSEIF( WINDOWS )
    IF(MSVC_VERSION LESS 1900)
	   SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -nologo -EHsc -DWIN32 -Dsnprintf=_snprintf -D__NO_COPYRIGHT__")
    ELSE()
       SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -nologo -EHsc -DWIN32 -D__NO_COPYRIGHT__")
    ENDIF()
ENDIF()

OPTION( QPOASES_WITH_LAPACK
	"Build qpOASES using original LAPACK routines"
	OFF
)
OPTION( QPOASES_WITH_BLAS
	"Build qpOASES using original BLAS routines"
	OFF
)

#
# Compile qpOASES libraries
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

#
# Find LAPACK
#
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

#
# Find BLAS
#
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
ADD_LIBRARY(acado_qpoases STATIC ${QPOASES_SOURCES} ${QPOASES_EXTRAS_SOURCES})

IF ( QPOASES_WITH_LAPACK AND LAPACK_FOUND )
	TARGET_LINK_LIBRARIES(
		acado_qpoases
		${LAPACK_LIBRARIES}
	)
ENDIF()

IF ( QPOASES_WITH_BLAS AND BLAS_FOUND )
	TARGET_LINK_LIBRARIES(
		acado_qpoases
		${BLAS_LIBRARIES}
	)
ENDIF()
