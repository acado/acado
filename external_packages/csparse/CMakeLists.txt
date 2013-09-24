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
ADD_LIBRARY( acado_csparse STATIC ${CSPARSE_SOURCES} )

################################################################################
#
# Installation rules
#
################################################################################

INSTALL(
	FILES
		${CSPARSE_HEADERS}
	DESTINATION
		${INSTALL_DATA_DIR}/external_packages/csparse
)
