################################################################################
#
# Description:
#	CMake script for building ACADO static and shared libraries, etc.
#
# Authors:
#	Joel Andersson
#	Attila Kozma
#	Milan Vukov, milan.vukov@esat.kuleuven.be
#
# Year:
#	2011 - 2013.
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
SET( ACADO_VERSION_PATCH  0 )
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
SET( LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/libs )

#
# Folder path for testing
#
SET( TESTING_OUTPUT_PATH ${PROJECT_BINARY_DIR}/testing/automatic_testing )

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
IF ( MSVC OR CYGWIN )
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
# Build type
#
IF( NOT CMAKE_BUILD_TYPE )
	SET(CMAKE_BUILD_TYPE Release CACHE STRING
		"Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
		FORCE
	)
ENDIF( NOT CMAKE_BUILD_TYPE )

#
# Offer the user the choice of overriding the installation directories
#
IF ( ${CMAKE_SYSTEM_NAME} MATCHES Windows )
	SET( LIB_DIR "lib/acado" )
#	SET( SHARE_DIR "" )
ELSE( )
	SET( LIB_DIR "lib" )
#	SET( SHARE_DIR "share" )
ENDIF( )

SET( INSTALL_LIB_DIR lib CACHE PATH
	"Installation directory for libraries"
)
SET( INSTALL_BIN_DIR bin CACHE PATH
	"Installation directory for executables"
)
SET( INSTALL_INCLUDE_DIR include/acado CACHE PATH
	"Installation directory for header files"
)

IF ( SHARE_DIR )
	SET( INSTALL_DATA_DIR ${SHARE_DIR}/acado CACHE PATH
		"Installation directory for data files"
	)
	SET( INSTALL_CMAKE_DIR ${SHARE_DIR}/acado/cmake CACHE PATH
		"Installation directory for CMake files"
	)
ELSE( SHARE_DIR )
	SET( INSTALL_DATA_DIR acado CACHE PATH
		"Installation directory for data files"
	)
	SET( INSTALL_CMAKE_DIR acado/cmake CACHE PATH
		"Installation directory for CMake files"
	)
ENDIF( SHARE_DIR )

# Make relative installation paths absolute (needed later on)
FOREACH( p LIB BIN INCLUDE DATA CMAKE )
  SET( var INSTALL_${p}_DIR )
  IF( NOT IS_ABSOLUTE "${${var}}" )
    SET( ${var} "${CMAKE_INSTALL_PREFIX}/${${var}}" )
  ENDIF()
ENDFOREACH()

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

#
# Compiler options are in separate file
#
INCLUDE( CompilerOptions )

################################################################################
#
# Libraries - lists of source folders
#
################################################################################

SET( ACADO_SOURCE_DIRS
	# Integrators source folders
	./src/utils
	./src/clock
	./src/matrix_vector
	./src/set_arithmetics
	./src/variables_grid
	./src/curve
	./src/user_interaction
	./src/symbolic_operator
	./src/symbolic_expression
	./src/function
	./src/integrator
	./src/validated_integrator
	./src/sparse_solver
	./external_packages/src/acado_csparse
	./external_packages/src/acado_gnuplot
	
	# Optimal Control source folders
	./src/code_generation
	./src/code_generation/integrators
	./src/code_generation/linear_solvers
	./src/dynamic_system
	./src/dynamic_discretization
	./src/objective
	./src/constraint
	./src/ocp
	./src/conic_program
	./src/conic_solver
	./src/nlp_solver
	./src/nlp_derivative_approximation
	./src/optimization_algorithm
	./external_packages/src/acado_qpoases
	
	# Simulation source folders
	./src/process
	./src/noise
	./src/transfer_device
	./src/controller
	./src/control_law
	./src/estimator
	./src/reference_trajectory
	./src/simulation_environment
)

################################################################################
#
# Include directories
#
################################################################################

INCLUDE_DIRECTORIES(
	${PROJECT_SOURCE_DIR}/include
	${PROJECT_SOURCE_DIR}/external_packages
	${PROJECT_SOURCE_DIR}/external_packages/qpOASES-3.0beta/include
	${PROJECT_BINARY_DIR}/include
)

################################################################################
#
# Configure templates
#
################################################################################

ADD_SUBDIRECTORY( ./src/code_generation/templates )

################################################################################
#
# Building of stand-alone libraries
#
################################################################################

#
# Build ACADO Toolkit library
#
UNSET( ACADO_SOURCES )

FOREACH( DIR ${ACADO_SOURCE_DIRS} )
	FILE( GLOB SRC ${DIR}/*.cpp )
	SET( ACADO_SOURCES ${ACADO_SOURCES} ${SRC} )
ENDFOREACH( DIR ${ACADO_SOURCE_DIRS} )

IF ( ACADO_BUILD_STATIC )	
	ADD_LIBRARY( acado_toolkit STATIC ${ACADO_SOURCES} )
	TARGET_LINK_LIBRARIES(
		acado_toolkit
		acado_qpOASESextras acado_csparse acado_casadi
	)
ENDIF ( ACADO_BUILD_STATIC )

IF( ACADO_BUILD_SHARED )
	ADD_LIBRARY( acado_toolkit_s SHARED ${ACADO_SOURCES} )
	SET_TARGET_PROPERTIES( acado_toolkit_s
		PROPERTIES
			VERSION ${ACADO_VERSION_STRING}
			SOVERSION ${ACADO_VERSION_MAJOR}
	)
	TARGET_LINK_LIBRARIES(
		acado_toolkit_s
		acado_qpOASESextras acado_csparse acado_casadi
	)
ENDIF( ACADO_BUILD_SHARED )

################################################################################
#
# Building external packages
#
################################################################################

#
# Build qpOASES library
#
ADD_SUBDIRECTORY( ./external_packages/qpOASES-3.0beta )

#
# Build csparse library
#
ADD_SUBDIRECTORY( ./external_packages/csparse )

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
# Installation rules
#
################################################################################

#
# Install ACADO Toolkit libs
#
IF ( ACADO_BUILD_STATIC )
	INSTALL(
		TARGETS
			acado_toolkit
		DESTINATION
			${INSTALL_LIB_DIR}
	)
ENDIF()

IF( ACADO_BUILD_SHARED )
	INSTALL(
		TARGETS
			acado_toolkit_s
		DESTINATION
			${INSTALL_LIB_DIR}
	)
ENDIF()

#
# Install ACADO include folder
#
INSTALL(
	DIRECTORY
		include/
	DESTINATION
		${INSTALL_INCLUDE_DIR}
)

#
# Install (parts of) external packages and interfaces to those packages
#
SET( EXTERNAL_PACKAGES_DIRS
	external_packages/include
	external_packages/qpoases
)	
INSTALL(
	DIRECTORY
		${EXTERNAL_PACKAGES_DIRS}
	DESTINATION
		${INSTALL_DATA_DIR}/external_packages
)

#
# Configure and install CMake package configuration file
#
SET( ACADO_INSTALL_INCLUDE_DIRS
	${INSTALL_INCLUDE_DIR}
	${INSTALL_DATA_DIR}/external_packages
	${INSTALL_DATA_DIR}/external_packages/csparse/
	${INSTALL_DATA_DIR}/external_packages/qpOASES-3.0beta/include
)
SET( ACADO_INSTALL_LIBRARY_DIRS
	${INSTALL_LIB_DIR}
)
SET( ACADO_INSTALL_EXTERNAL_PACKAGES_DIR
	${INSTALL_DATA_DIR}/external_packages
)
SET( ACADO_INSTALL_CMAKE_DIR
	${INSTALL_CMAKE_DIR}
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
	${INSTALL_DATA_DIR}/external_packages/qpoases/SRC/Bounds.cpp
	${INSTALL_DATA_DIR}/external_packages/qpoases/SRC/CyclingManager.cpp 
	${INSTALL_DATA_DIR}/external_packages/qpoases/SRC/MessageHandling.cpp
	${INSTALL_DATA_DIR}/external_packages/qpoases/SRC/QProblem.cpp
	${INSTALL_DATA_DIR}/external_packages/qpoases/SRC/Utils.cpp
	${INSTALL_DATA_DIR}/external_packages/qpoases/SRC/Constraints.cpp
	${INSTALL_DATA_DIR}/external_packages/qpoases/SRC/Indexlist.cpp	     
	${INSTALL_DATA_DIR}/external_packages/qpoases/SRC/QProblemB.cpp
	${INSTALL_DATA_DIR}/external_packages/qpoases/SRC/SubjectTo.cpp
	${INSTALL_DATA_DIR}/external_packages/qpoases/SRC/EXTRAS/SolutionAnalysis.cpp
)
SET( ACADO_INSTALL_QPOASES_EMBEDDED_INC_DIRS
	${INSTALL_DATA_DIR}/external_packages/qpoases/
	${INSTALL_DATA_DIR}/external_packages/qpoases/INCLUDE
	${INSTALL_DATA_DIR}/external_packages/qpoases/SRC
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
	${PROJECT_SOURCE_DIR}/include
	${PROJECT_SOURCE_DIR}/external_packages
	${PROJECT_SOURCE_DIR}/external_packages/csparse/
	${PROJECT_SOURCE_DIR}/external_packages/qpOASES-3.0beta/include
	${PROJECT_BINARY_DIR}/include
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
# Configure include.mk script for backward compatibility
#
################################################################################

SET( INCLUDE_MK_CPP_EXE
	${CMAKE_CXX_COMPILER}
)

STRING(REPLACE ";" " " INCLUDE_MK_CPP_FLAGS "${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_DEBUG}")

STRING(REPLACE ";" " -I" INCLUDE_MK_INCLUDE_DIRS "${ACADO_INCLUDE_DIRS}")
SET( INCLUDE_MK_INCLUDE_DIRS 
	"-I${INCLUDE_MK_INCLUDE_DIRS}"
)

STRING(REPLACE ";" " -L" INCLUDE_MK_LIBRARY_DIRS "${ACADO_LIBRARY_DIRS}")
SET( INCLUDE_MK_LIBRARY_DIRS
	"-L${INCLUDE_MK_LIBRARY_DIRS}"
)

# The following code favors shared libraries.
IF ( ACADO_BUILD_SHARED )
	STRING(REPLACE ";" " -l" INCLUDE_MK_LIBS "${ACADO_SHARED_LIBRARIES}")
ELSEIF ( ACADO_BUILD_STATIC )
	STRING(REPLACE ";" " -l" INCLUDE_MK_LIBS "${ACADO_STATIC_LIBRARIES}")
ENDIF()
SET( INCLUDE_MK_LIBS
	"-l${INCLUDE_MK_LIBS}"
)

CONFIGURE_FILE(
	"${PROJECT_SOURCE_DIR}/cmake/include.mk.in"
	"${PROJECT_BINARY_DIR}/include.mk"
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

SET( PC_INCLUDE_DIRS 
	${INCLUDE_MK_INCLUDE_DIRS}
)

SET( PC_LIBRARY_DIRS
	${INCLUDE_MK_LIBRARY_DIRS}
)

SET( PC_LIBS
	${INCLUDE_MK_LIBS}
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
	ADD_SUBDIRECTORY( ./examples )
ENDIF( ACADO_WITH_EXAMPLES )

################################################################################
#
# Internal stuff
#
################################################################################

IF( ACADO_INTERNAL )
	ADD_SUBDIRECTORY( ./experimental )
ENDIF( ACADO_INTERNAL )
