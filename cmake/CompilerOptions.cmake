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
#	2011 - 2014
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

INCLUDE( CheckCXXCompilerFlag )

#
# ACADO build flags, used internally for conditional code compilation
#
SET( ACADO_BUILD ON )
ADD_DEFINITIONS( -DACADO_CMAKE_BUILD )

# Define __DEBUG__ for debug builds
SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -D__DEBUG__")
SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -D__DEBUG__")

#
# Define a testing flag and turn off the plotting functionality
#
IF ( ACADO_WITH_TESTING )
	ADD_DEFINITIONS( -DACADO_WITH_TESTING )
	ADD_DEFINITIONS( -D__NO_PIPES__ )
	ADD_DEFINITIONS( -D__NO_PLOTTING__ )
ENDIF()

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

################################################################################
#
# Compiler settings - GCC/G++; Linux, Apple
#
################################################################################
IF (    "${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU"
     OR "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang"
     OR "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Intel" )
	
	# Cygwin complains in about the -fPIC flag...
	IF( CYGWIN )
		ADD_DEFINITIONS( -D__NO_PIPES__ )
	ENDIF( )

	IF( NOT (CYGWIN OR WIN32) )
		SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC" )
		SET( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC" )
	ENDIF( )
	
	SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pedantic -Wfloat-equal -Wshadow -DLINUX" )
	SET( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -pedantic -Wfloat-equal -Wshadow -DLINUX" )
	
	IF( NOT ACADO_DEVELOPER )
		SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-overloaded-virtual" )
	ENDIF()

	IF (NOT (APPLE AND "${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU") AND (NOT MINGW))
		SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")
		SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
	ENDIF()
	
	IF ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
		SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-comparison -Dregister=''" )
	ENDIF()

	#
	# Apple specifics
	#
	IF( APPLE )
		# Build only for 64-bit arch. This requires (at least) OS X version >= 10.7.
		IF ( NOT CMAKE_OSX_ARCHITECTURES ) 
			SET( CMAKE_OSX_ARCHITECTURES "x86_64;" )
		ENDIF()
		
		IF ( "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang" )
			SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++" )
		ENDIF()

	ENDIF( APPLE )
	
	#
	# Check for C++11/C++0x support
	# NOTE: This is only done in the case when we don't use OSX with Clang
	#       (version >= 4.0) since there are namespace problems (std::tr1 is removed).
	#
		CHECK_CXX_COMPILER_FLAG(-std=c++11 COMPILER_SUPPORTS_CXX11 )
		CHECK_CXX_COMPILER_FLAG(-std=gnu++11 COMPILER_SUPPORTS_GNU11)
		CHECK_CXX_COMPILER_FLAG(-std=c++0x COMPILER_SUPPORTS_CXX0X)
		CHECK_CXX_COMPILER_FLAG(-std=gnu++0x COMPILER_SUPPORTS_GNU0X)

		IF( COMPILER_SUPPORTS_CXX11 )
			SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -DACADO_HAS_CXX11" )
		ELSEIF( COMPILER_SUPPORTS_GNU11 )
			SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11 -DACADO_HAS_CXX11" )
		ELSEIF( COMPILER_SUPPORTS_CXX0X)
			SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -DACADO_HAS_CXX0X" )
		ELSEIF( COMPILER_SUPPORTS_GNU0X)
			SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++0x -DACADO_HAS_CXX0X" )
		ENDIF()
	
	IF ( MINGW )
        SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -static")
		SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -static")
		SET(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS "${CMAKE_SHARED_LIBRARY_LINK_C_FLAGS} -static -s")
		SET(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "${CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS} -static -s")
	ENDIF( )

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
		"Absolute path of the Gnuplot executable."
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
	#ADD_DEFINITIONS( -Dsnprintf=_snprintf )
	ADD_DEFINITIONS( -Dusleep=Sleep )
	ADD_DEFINITIONS( -Dsleep=Sleep )
	ADD_DEFINITIONS( -D_CRT_SECURE_NO_WARNINGS )
	ADD_DEFINITIONS( -D_SCL_SECURE_NO_WARNINGS )
	ADD_DEFINITIONS( -D__NO_PIPES__ )
	ADD_DEFINITIONS( "/wd4068")

    IF(MSVC_VERSION LESS 1900)
        ADD_DEFINITIONS( -Dsnprintf=_snprintf )
    ENDIF()

	IF( GNUPLOT_EXECUTABLE )
		ADD_DEFINITIONS( -DGNUPLOT_EXECUTABLE="${GNUPLOT_EXECUTABLE}" )
	ENDIF()

	#
	# Enable project grouping when making Visual Studio solution
	# NOTE: This feature is NOT supported in Express editions
	#
	SET_PROPERTY( GLOBAL PROPERTY USE_FOLDERS ON )

ENDIF( )

