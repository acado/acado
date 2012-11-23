
################################################################################
#
# Configure templates
#
################################################################################

SET( INTEGRATOR_MEX_TEMPLATE ${CMAKE_CURRENT_SOURCE_DIR}/integrator_mex.c.in)
SET( RHS_MEX_TEMPLATE ${CMAKE_CURRENT_SOURCE_DIR}/rhs_mex.c.in)
SET( MPC_MEX_TEMPLATE ${CMAKE_CURRENT_SOURCE_DIR}/MPC_mex.c.in)
SET( MHE_MEX_TEMPLATE ${CMAKE_CURRENT_SOURCE_DIR}/MHE_mex.c.in)

SET( CG_TEMPLATES_PATH
	${PROJECT_BINARY_DIR}/include/acado/code_generation/templates
)

CONFIGURE_FILE(
	"${CMAKE_CURRENT_SOURCE_DIR}/templates.hpp.in"
	"${CG_TEMPLATES_PATH}/templates.hpp"
	@ONLY
)
