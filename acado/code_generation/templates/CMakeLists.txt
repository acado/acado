################################################################################
#
# Description:
#	Configuration of templates
#
# Authors:
#	Milan Vukov, milan.vukov@esat.kuleuven.be
# 	Rien Quirynen
#
# Year:
#	2011 - 2014
#
#
################################################################################

#
# Templates
#

SET( INTEGRATOR_MEX_TEMPLATE integrator_mex.c.in)
SET( RHS_MEX_TEMPLATE rhs_mex.c.in)

SET( HESSIAN_REG_SOURCE acado_hessian_regularization.c.in)

SET( FORCES_TEMPLATE forces_interface.in)
SET( FORCES_GENERATOR acado_forces_generator.m.in)
SET( FORCES_GENERATOR_PYTHON acado_forces_generator.py.in)
SET( QPOASES_HEADER qpoases_interface.hpp.in)
SET( QPOASES_SOURCE qpoases_interface.cpp.in)
SET( QPOASES3_HEADER qpoases3_interface.h.in)
SET( QPOASES3_SOURCE qpoases3_interface.c.in)
SET( QPDUNES_TEMPLATE qpdunes_interface.in)
SET( QPDUNES_SPLIT_TEMPLATE qpdunes_split_interface.in)

SET( AUXILIARY_FUNCTIONS_HEADER acado_auxiliary_functions.h.in)
SET( AUXILIARY_FUNCTIONS_SOURCE acado_auxiliary_functions.c.in)
SET( AUXILIARY_SIM_FUNCTIONS_HEADER acado_auxiliary_sim_functions.h.in)
SET( AUXILIARY_SIM_FUNCTIONS_SOURCE acado_auxiliary_sim_functions.c.in)
SET( SOLVER_MEX acado_solver_mex.c.in)
SET( EH_SOLVER_MEX acado_EH_solver_mex.c.in)
SET( MAKE_MEX_FORCES make_acado_solver_forces.m.in)
SET( MAKE_MEX_QPOASES make_acado_solver_qpoases.m.in)
SET( MAKE_MEX_EH_QPOASES make_acado_EH_solver_qpoases.m.in)
SET( MAKE_MEX_QPOASES3 make_acado_solver_qpoases3.m.in)
SET( MAKE_MEX_EH_QPOASES3 make_acado_EH_solver_qpoases3.m.in)
SET( MAKE_MEX_QPDUNES make_acado_solver_qpdunes.m.in)
SET( MAKE_MEX_EH_QPDUNES make_acado_EH_solver_qpdunes.m.in)
SET( MAKE_MEX_BLOCK_QPDUNES make_acado_block_solver_qpdunes.m.in)
SET( MAKE_MEX_INTEGRATOR make_acado_integrator.m.in)
SET( MAKE_MEX_MODEL make_acado_model.m.in)
SET( MAKEFILE_FORCES makefile.forces.in)
SET( MAKEFILE_QPOASES makefile.qpoases.in)
SET( MAKEFILE_QPOASES3 makefile.qpoases3.in)
SET( MAKEFILE_QPDUNES makefile.qpdunes.in)
SET( MAKEFILE_EH_QPOASES makefile.EH_qpoases.in)
SET( MAKEFILE_EH_QPDUNES makefile.EH_qpdunes.in)
SET( MAKEFILE_HPMPC makefile.hpmpc.in)
SET( MAKEFILE_INTEGRATOR makefile.integrator.in)

SET( MAKEFILE_SFUN_QPOASES make_acado_solver_sfunction.m.in)
SET( MAKEFILE_SFUN_QPOASES3 make_acado_solver_sfunction.m.in)
SET( SOLVER_SFUN_SOURCE acado_solver_sfunction.c.in)
SET( SOLVER_SFUN_HEADER acado_solver_sfunction.h.in)

SET( DUMMY_TEST_FILE dummy_test_file.in)

SET( COMMON_HEADER_TEMPLATE acado_common_header.h.in)

SET( HPMPC_INTERFACE acado_hpmpc_interface.c.in)

#
# Template paths
#

SET( INSTALL_TEMPLATE_PATH
	${CMAKE_INSTALL_PREFIX}/${INSTALL_INCLUDE_DIR}/code_generation/templates
)

SET( TEMPLATE_PATHS ${CMAKE_CURRENT_SOURCE_DIR} ${INSTALL_TEMPLATE_PATH} )

#
# Configure templates
#

SET( CG_TEMPLATES_PATH
	${PROJECT_BINARY_DIR}/acado/code_generation/templates
)

CONFIGURE_FILE(
	"${CMAKE_CURRENT_SOURCE_DIR}/templates.hpp.in"
	"${CG_TEMPLATES_PATH}/templates.hpp"
	@ONLY
)

#
# Install templates' header
#
FILE( GLOB TEMPLATES "*.in" )

INSTALL(
	FILES
		${TEMPLATES}
		"${CG_TEMPLATES_PATH}/templates.hpp"
	DESTINATION
		${INSTALL_INCLUDE_DIR}/code_generation/templates
)
