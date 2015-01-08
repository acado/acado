#
# Project info
#
SET( ACADO_CASADI_VERSION "1.5.0" )
SET( ACADO_CASADI_SOVERSION "1" )

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
