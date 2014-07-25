#
# Project info from file cs.h
#
SET( CSPARSE_VERSION "2.2.3" )
SET( CSPARSE_SOVERSION "2" )

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
