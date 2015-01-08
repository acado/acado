################################################################################
#
# Description:
#	CMake script for building ACADO static and shared libraries, etc.
#
# Authors:
#	Milan Vukov, milan.vukov@esat.kuleuven.be
#
# Year:
#	2011 - 2014.
#
# Usage:
#	- Must be called from the main CMake script
#
################################################################################

################################################################################
#
# Configure templates
#
################################################################################

ADD_SUBDIRECTORY( code_generation/templates )

################################################################################
#
# List of source folders
#
################################################################################

SET( ACADO_SOURCE_DIRS
	# Core 
	utils
	matrix_vector
	variables_grid
	curve
	user_interaction
	symbolic_operator
	symbolic_expression
	function
)

IF (NOT ACADO_BUILD_CGT_ONLY)
	SET ( ACADO_SOURCE_DIRS ${ACADO_SOURCE_DIRS}
		# Integrators
		clock
		integrator
		set_arithmetics
		validated_integrator
		sparse_solver
		bindings/acado_csparse
		bindings/acado_gnuplot
	)
ENDIF()
	
# Optimal Control and Simulation source folders.
## Optimal Control
SET ( ACADO_SOURCE_DIRS ${ACADO_SOURCE_DIRS}
	objective
	constraint
	ocp
)

IF (NOT ACADO_BUILD_CGT_ONLY)
	SET ( ACADO_SOURCE_DIRS ${ACADO_SOURCE_DIRS}
		dynamic_system
		dynamic_discretization
		conic_program
		conic_solver
		nlp_solver
		nlp_derivative_approximation
		optimization_algorithm
		bindings/acado_qpoases
	)
ENDIF()

## Simulation
IF (NOT ACADO_BUILD_CGT_ONLY)
	SET ( ACADO_SOURCE_DIRS ${ACADO_SOURCE_DIRS}
		process
		noise
		transfer_device
		controller
		control_law
		estimator
		reference_trajectory
		simulation_environment
	)
ENDIF()

SET ( ACADO_SOURCE_DIRS ${ACADO_SOURCE_DIRS}
	# Code Generation Tool
	code_generation
	code_generation/linear_solvers	
	code_generation/integrators
)

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
		acado_casadi
	)
	IF (NOT ACADO_BUILD_CGT_ONLY)
		TARGET_LINK_LIBRARIES(
			acado_toolkit
			acado_qpoases acado_csparse
		)
	ENDIF()
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
		acado_casadi
	)
	IF (NOT ACADO_BUILD_CGT_ONLY)
		TARGET_LINK_LIBRARIES(
			acado_toolkit_s
			acado_qpoases acado_csparse
		)
	ENDIF()
ENDIF( ACADO_BUILD_SHARED )

#
# Installation rules
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

################################################################################
#
# CPP lint check target
#
################################################################################

INCLUDE( CppLint )
ADD_STYLE_CHECK_TARGET(lint ${ACADO_SOURCES})
