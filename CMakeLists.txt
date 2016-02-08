################################################################################
#
# Description:
#	Main build system script for the ACADO toolkit.
#
# Authors:
#	Joel Andersson
#	Attila Kozma
#	Milan Vukov, milan.vukov@esat.kuleuven.be
#
# Year:
#	2011 - 2014.
#
# Usage:
#	- Linux, OS X, Windows: more info can be found on www.acadotoolkit.org
#
################################################################################

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
PROJECT( ACADO CXX C )

#
# Project version information
#
SET( ACADO_VERSION_MAJOR  1 )
SET( ACADO_VERSION_MINOR  2 )
SET( ACADO_VERSION_PATCH  2 )
SET( ACADO_VERSION_STRING
		${ACADO_VERSION_MAJOR}.${ACADO_VERSION_MINOR}.${ACADO_VERSION_PATCH}beta
)

#
# Folder path for generated executables
#
SET( EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin )

#
# Folder path for generated libraries
#
SET( LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib )

#
# CMake module(s) path
#
SET( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake" )

################################################################################
#
# User configuration
#
################################################################################

#
# Set this to true (ON) if you are using makefiles and want to see the full
# compile and link commands instead of only the shortened ones
#
IF( NOT CMAKE_VERBOSE_MAKEFILE )
	SET( CMAKE_VERBOSE_MAKEFILE OFF )
ENDIF( NOT CMAKE_VERBOSE_MAKEFILE )

#
# Enable building of shared and static libraries
#
IF ( MSVC OR CYGWIN OR MINGW )
	# All Windows builds that we currently support
	SET( ACADO_BUILD_SHARED OFF )
	SET( ACADO_BUILD_STATIC ON )
ELSE()
	# Linux or OS X
	OPTION( ACADO_BUILD_SHARED "Build shared libraries" ON )
	OPTION( ACADO_BUILD_STATIC "Build static libraries" OFF )
ENDIF()

IF((NOT ACADO_BUILD_SHARED) AND (NOT ACADO_BUILD_STATIC))
	MESSAGE( ERROR "You must build some libraries..." )
ENDIF()

#
# Enable building of ...
#
OPTION( ACADO_WITH_EXAMPLES "Building ACADO examples" ON )

#
# Compilation of testing suite
#
OPTION( ACADO_WITH_TESTING "Building the testing framework" OFF )

#
# ACADO developer flag
#
OPTION ( ACADO_DEVELOPER "Choose developer/user building mode" OFF )

#
# ACADO internal stuff
#
OPTION( ACADO_INTERNAL "Select building of ACADO internal stuff" OFF )

#
# Compile only the code generation tool
#
OPTION( ACADO_BUILD_CGT_ONLY "Build only the code generation tool" OFF )

#
# Build type
#
IF( NOT CMAKE_BUILD_TYPE )
	SET(CMAKE_BUILD_TYPE Release CACHE STRING
		"Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
		FORCE
	)
ENDIF( NOT CMAKE_BUILD_TYPE )

#
# Installation folders
#
SET( INSTALL_LIB_DIR lib CACHE PATH
	"Installation directory for libraries"
)
SET( INSTALL_BIN_DIR bin CACHE PATH
	"Installation directory for executables"
)
SET( INSTALL_INCLUDE_DIR include/acado CACHE PATH
	"Installation directory for header files"
)
SET( INSTALL_DATA_DIR share/acado CACHE PATH
	"Installation directory for data files"
)
SET( INSTALL_CMAKE_DIR share/acado/cmake CACHE PATH
	"Installation directory for CMake files"
)

################################################################################
#
# Dependencies
#
################################################################################

FIND_PACKAGE( Doxygen )

################################################################################
#
# Compiler settings
#
################################################################################

# Suppressing some stupid warnings from CMake 3.0
IF (POLICY CMP0026)
	CMAKE_POLICY(SET CMP0026 OLD)
ENDIF( )
IF (POLICY CMP0042)
	CMAKE_POLICY(SET CMP0042 OLD)
ENDIF( )

#
# Compiler options are in separate file
#
INCLUDE( CompilerOptions )

################################################################################
#
# Include directories
#
################################################################################

INCLUDE_DIRECTORIES(
	${PROJECT_SOURCE_DIR}
	${PROJECT_SOURCE_DIR}/acado
	${PROJECT_SOURCE_DIR}/external_packages
	${PROJECT_SOURCE_DIR}/external_packages/qpOASES-3.2.0/include
	${PROJECT_BINARY_DIR}
)

################################################################################
#
# Build the ACADO toolkit library
#
################################################################################

ADD_SUBDIRECTORY( acado )

################################################################################
#
# Building external packages
#
################################################################################

IF (NOT ACADO_BUILD_CGT_ONLY)
	#
	# Build qpOASES library
	#
	ADD_SUBDIRECTORY( ./external_packages/qpOASES-3.2.0 )
	
	#
	# Build csparse library
	#
	ADD_SUBDIRECTORY( ./external_packages/csparse )
ENDIF()

#
# Build (a part of) CasADi library
#
ADD_SUBDIRECTORY( ./external_packages/casadi )

################################################################################
#
# Automatic testing
#
################################################################################

#
# Tests
#
IF( ACADO_WITH_TESTING )
	ENABLE_TESTING()
ENDIF( ACADO_WITH_TESTING )

################################################################################
#
# Some global installation rules
#
################################################################################

#
# Install ACADO headers
#
INSTALL(
	DIRECTORY
		acado/
	DESTINATION
		${INSTALL_INCLUDE_DIR}
	FILES_MATCHING PATTERN "*.hpp"
)
INSTALL(
	DIRECTORY
		acado/
	DESTINATION
		${INSTALL_INCLUDE_DIR}
	FILES_MATCHING PATTERN "*.ipp"
)

#
# Install Eigen3
#
INSTALL(
	DIRECTORY
		external_packages/eigen3
	DESTINATION
		${INSTALL_INCLUDE_DIR}/external_packages
)

#
# Install qpOASES embedded
#
INSTALL(
	DIRECTORY
		external_packages/qpoases
	DESTINATION
		${INSTALL_DATA_DIR}/external_packages
)

#
# Configure and install CMake package configuration file
#
SET( ACADO_INSTALL_INCLUDE_DIRS
	${CMAKE_INSTALL_PREFIX}/${INSTALL_INCLUDE_DIR}
)
SET( ACADO_INSTALL_LIBRARY_DIRS
	${CMAKE_INSTALL_PREFIX}/${INSTALL_LIB_DIR}
)
SET( ACADO_INSTALL_EXTERNAL_PACKAGES_DIR
	${CMAKE_INSTALL_PREFIX}/${INSTALL_DATA_DIR}/external_packages
)
SET( ACADO_INSTALL_CMAKE_DIR
	${CMAKE_INSTALL_PREFIX}/${INSTALL_CMAKE_DIR}
)

IF ( ACADO_BUILD_STATIC )
	SET( ACADO_INSTALL_STATIC_LIBRARIES
		acado_toolkit
	)
ENDIF()

IF ( ACADO_BUILD_SHARED )
	SET( ACADO_INSTALL_SHARED_LIBRARIES
		acado_toolkit_s
	)
ENDIF()

SET( ACADO_INSTALL_QPOASES_EMBEDDED_SOURCES
	${CMAKE_INSTALL_PREFIX}/${INSTALL_DATA_DIR}/external_packages/qpoases/SRC/Bounds.cpp
	${CMAKE_INSTALL_PREFIX}/${INSTALL_DATA_DIR}/external_packages/qpoases/SRC/CyclingManager.cpp 
	${CMAKE_INSTALL_PREFIX}/${INSTALL_DATA_DIR}/external_packages/qpoases/SRC/MessageHandling.cpp
	${CMAKE_INSTALL_PREFIX}/${INSTALL_DATA_DIR}/external_packages/qpoases/SRC/QProblem.cpp
	${CMAKE_INSTALL_PREFIX}/${INSTALL_DATA_DIR}/external_packages/qpoases/SRC/Utils.cpp
	${CMAKE_INSTALL_PREFIX}/${INSTALL_DATA_DIR}/external_packages/qpoases/SRC/Constraints.cpp
	${CMAKE_INSTALL_PREFIX}/${INSTALL_DATA_DIR}/external_packages/qpoases/SRC/Indexlist.cpp	     
	${CMAKE_INSTALL_PREFIX}/${INSTALL_DATA_DIR}/external_packages/qpoases/SRC/QProblemB.cpp
	${CMAKE_INSTALL_PREFIX}/${INSTALL_DATA_DIR}/external_packages/qpoases/SRC/SubjectTo.cpp
	${CMAKE_INSTALL_PREFIX}/${INSTALL_DATA_DIR}/external_packages/qpoases/SRC/EXTRAS/SolutionAnalysis.cpp
)
SET( ACADO_INSTALL_QPOASES_EMBEDDED_INC_DIRS
	${CMAKE_INSTALL_PREFIX}/${INSTALL_DATA_DIR}/external_packages/qpoases/
	${CMAKE_INSTALL_PREFIX}/${INSTALL_DATA_DIR}/external_packages/qpoases/INCLUDE
	${CMAKE_INSTALL_PREFIX}/${INSTALL_DATA_DIR}/external_packages/qpoases/SRC
)

CONFIGURE_FILE(
	"${PROJECT_SOURCE_DIR}/cmake/ACADOConfig.cmake.in"
	"${PROJECT_BINARY_DIR}/ACADOConfig.cmake"
	@ONLY
)
INSTALL(
	FILES
		${PROJECT_BINARY_DIR}/ACADOConfig.cmake
	DESTINATION
		${INSTALL_CMAKE_DIR}
)
INSTALL(
	FILES
		${PROJECT_SOURCE_DIR}/cmake/FindACADO.cmake
		${PROJECT_SOURCE_DIR}/cmake/UseACADO.cmake
	DESTINATION
		${INSTALL_CMAKE_DIR}
)

################################################################################
#
# Rules for un-installing ACADO toolkit
#
################################################################################

CONFIGURE_FILE(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/ACADOUninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/ACADOUninstall.cmake"
    IMMEDIATE @ONLY
)

ADD_CUSTOM_TARGET( uninstall
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/ACADOUninstall.cmake
)

################################################################################
#
# Building API documentation
#
################################################################################

IF( DOXYGEN_FOUND )
	ADD_CUSTOM_TARGET( doc
		${DOXYGEN_EXECUTABLE} ${PROJECT_SOURCE_DIR}/doc/doxygen.config
		WORKING_DIRECTORY
			${PROJECT_SOURCE_DIR}/doc
		COMMENT
			"Generating API documentation with Doxygen" VERBATIM
	)
ENDIF( DOXYGEN_FOUND )

################################################################################
#
# Configuration of the bash script
#
################################################################################

SET( ACADO_INCLUDE_DIRS
	${PROJECT_SOURCE_DIR}
	${PROJECT_SOURCE_DIR}/acado
	${PROJECT_SOURCE_DIR}/external_packages
	${PROJECT_SOURCE_DIR}/external_packages/qpOASES-3.2.0/include
	${PROJECT_BINARY_DIR}
)

SET( ACADO_LIBRARY_DIRS
	${LIBRARY_OUTPUT_PATH}
)

SET( ACADO_EXTERNAL_PACKAGES_DIR
	${PROJECT_SOURCE_DIR}/external_packages
)

SET( ACADO_CMAKE_DIR
	${PROJECT_SOURCE_DIR}/cmake
)

SET( ACADO_STATIC_LIBRARIES 
	${ACADO_INSTALL_STATIC_LIBRARIES}
)

SET( ACADO_SHARED_LIBRARIES
	${ACADO_INSTALL_SHARED_LIBRARIES}
)

SET( ACADO_QPOASES_EMBEDDED_FOLDER 
	${PROJECT_SOURCE_DIR}/external_packages/qpoases
)

SET( ACADO_QPOASES_EMBEDDED_SOURCES
	${PROJECT_SOURCE_DIR}/external_packages/qpoases/SRC/Bounds.cpp
	${PROJECT_SOURCE_DIR}/external_packages/qpoases/SRC/CyclingManager.cpp 
	${PROJECT_SOURCE_DIR}/external_packages/qpoases/SRC/MessageHandling.cpp
	${PROJECT_SOURCE_DIR}/external_packages/qpoases/SRC/QProblem.cpp
	${PROJECT_SOURCE_DIR}/external_packages/qpoases/SRC/Utils.cpp
	${PROJECT_SOURCE_DIR}/external_packages/qpoases/SRC/Constraints.cpp
	${PROJECT_SOURCE_DIR}/external_packages/qpoases/SRC/Indexlist.cpp	     
	${PROJECT_SOURCE_DIR}/external_packages/qpoases/SRC/QProblemB.cpp
	${PROJECT_SOURCE_DIR}/external_packages/qpoases/SRC/SubjectTo.cpp
	${PROJECT_SOURCE_DIR}/external_packages/qpoases/SRC/EXTRAS/SolutionAnalysis.cpp
)

SET( ACADO_QPOASES_EMBEDDED_INC_DIRS
	${PROJECT_SOURCE_DIR}/external_packages/qpoases/
	${PROJECT_SOURCE_DIR}/external_packages/qpoases/INCLUDE
	${PROJECT_SOURCE_DIR}/external_packages/qpoases/SRC
)

CONFIGURE_FILE(
	"${PROJECT_SOURCE_DIR}/cmake/acado_env.sh.in"
	"${PROJECT_BINARY_DIR}/acado_env.sh"
	@ONLY
)

################################################################################
#
# Configure pkg-config script
#
################################################################################

SET( PC_VERSION
	${ACADO_VERSION_STRING}
)

STRING(REPLACE ";" " -I" PC_INCLUDE_DIRS "${ACADO_INCLUDE_DIRS}")
SET( PC_INCLUDE_DIRS 
	"-I${PC_INCLUDE_DIRS}"
)

STRING(REPLACE ";" " -L" PC_LIBRARY_DIRS "${ACADO_LIBRARY_DIRS}")
SET( PC_LIBRARY_DIRS
	"-L${PC_LIBRARY_DIRS}"
)

# The following code favors shared libraries.
IF ( ACADO_BUILD_SHARED )
	STRING(REPLACE ";" " -l" PC_LIBS "${ACADO_SHARED_LIBRARIES}")
ELSEIF ( ACADO_BUILD_STATIC )
	STRING(REPLACE ";" " -l" PC_LIBS "${ACADO_STATIC_LIBRARIES}")
ENDIF()
SET( PC_LIBS
	"-l${PC_LIBS}"
)

STRING(REPLACE ";" " -I" PC_QPOASES_EMBEDDED_INC_DIRS "${ACADO_QPOASES_EMBEDDED_INC_DIRS}")
SET( PC_QPOASES_EMBEDDED_INC_DIRS 
	"-I${PC_QPOASES_EMBEDDED_INC_DIRS}"
)

STRING(REPLACE ";" " " PC_QPOASES_EMBEDDED_SOURCES "${ACADO_QPOASES_EMBEDDED_SOURCES}")

CONFIGURE_FILE(
	"${PROJECT_SOURCE_DIR}/cmake/acado.pc.in"
	"${PROJECT_BINARY_DIR}/acado.pc"
	@ONLY
)

################################################################################
#
# Build examples
#
################################################################################

IF( ACADO_WITH_EXAMPLES )
	ADD_SUBDIRECTORY( examples )
ENDIF( ACADO_WITH_EXAMPLES )

################################################################################
#
# Unit tests
#
################################################################################

IF( ACADO_WITH_TESTING )
	ADD_SUBDIRECTORY( tests )
ENDIF( ACADO_WITH_TESTING )

################################################################################
#
# Internal stuff
#
################################################################################

IF( ACADO_INTERNAL )
	ADD_SUBDIRECTORY( experimental )
ENDIF( ACADO_INTERNAL )
