################################################################################
#
# Description:
#	Configuration of paths for templates
#
# Authors:
#	Milan Vukov, milan.vukov@esat.kuleuven.be
# 	Rien Quirynen
#
# Year:
#	2011 - 2013.
#
# NOTE:
#	- For each template we need three definitions, ATM.
#   - Export pure template file name and list of possible paths; the paths
#     should be stored in the ExportTemplatedFile class (the base one) and
#     this guy should open the template somehow later
#
#
################################################################################

#
# Template paths
#

SET( INSTALL_TEMPLATE_PATH
	${INSTALL_INCLUDE_DIR}/acado/code_generation/templates
)

SET( TEMPLATE_PATHS ${CMAKE_CURRENT_SOURCE_DIR} ${INSTALL_TEMPLATE_PATH} )

#
# Templates
#

SET( INTEGRATOR_MEX_TEMPLATE ${CMAKE_CURRENT_SOURCE_DIR}/integrator_mex.c.in)
SET( RHS_MEX_TEMPLATE ${CMAKE_CURRENT_SOURCE_DIR}/rhs_mex.c.in)

SET( FORCES_TEMPLATE ${CMAKE_CURRENT_SOURCE_DIR}/forces_interface.in)
SET( FORCES_GENERATOR ${CMAKE_CURRENT_SOURCE_DIR}/acado_forces_generator.m.in)
SET( QPOASES_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/qpoases_solver.hpp.in)
SET( QPOASES_SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/qpoases_solver.cpp.in)
SET( QPDUNES_TEMPLATE ${CMAKE_CURRENT_SOURCE_DIR}/qpdunes_interface.in)

SET( AUXILIARY_FUNCTIONS_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/acado_auxiliary_functions.h.in)
SET( AUXILIARY_FUNCTIONS_SOURCE ${CMAKE_CURRENT_SOURCE_DIR}acado_auxiliary_functions.c.in)
SET( SOLVER_MEX ${CMAKE_CURRENT_SOURCE_DIR}/acado_solver_mex.c.in)
SET( MAKE_MEX_FORCES ${CMAKE_CURRENT_SOURCE_DIR}/make_acado_solver_forces.m.in)
SET( MAKE_MEX_QPOASES ${CMAKE_CURRENT_SOURCE_DIR}/make_acado_solver_qpoases.m.in)
SET( MAKEFILE_FORCES ${CMAKE_CURRENT_SOURCE_DIR}/makefile.forces.in)
SET( MAKEFILE_QPOASES ${CMAKE_CURRENT_SOURCE_DIR}/makefile.qpoases.in)

SET( MAKEFILE_SFUN_QPOASES ${CMAKE_CURRENT_SOURCE_DIR}/make_acado_solver_sfunction.m.in)
SET( SOLVER_SFUN_SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/acado_solver_sfunction.c.in)
SET( SOLVER_SFUN_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/acado_solver_sfunction.h.in)

SET( DUMMY_TEST_FILE ${CMAKE_CURRENT_SOURCE_DIR}/dummy_test_file.in)

#
# Configure templates
#

SET( CG_TEMPLATES_PATH
	${PROJECT_BINARY_DIR}/include/acado/code_generation/templates
)

CONFIGURE_FILE(
	"${CMAKE_CURRENT_SOURCE_DIR}/templates.hpp.in"
	"${CG_TEMPLATES_PATH}/templates.hpp"
	@ONLY
)

#
# Install templates
#

FILE( GLOB TMP ./*.in )

INSTALL(
	FILES
		"${CG_TEMPLATES_PATH}/templates.hpp"
		${TMP}
	DESTINATION
		${INSTALL_INCLUDE_DIR}/acado/code_generation/templates
)
