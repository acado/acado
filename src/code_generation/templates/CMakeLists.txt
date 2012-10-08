
################################################################################
#
# Configure templates
#
################################################################################

SET( INTEGRATOR_MEX_TEMPLATE ${CMAKE_CURRENT_SOURCE_DIR}/integrator_mex.c.in)

SET( CG_TEMPLATES_PATH
	${PROJECT_BINARY_DIR}/include/acado/code_generation/templates
)

CONFIGURE_FILE(
	"${CMAKE_CURRENT_SOURCE_DIR}/templates.hpp.in"
	"${CG_TEMPLATES_PATH}/templates.hpp"
	@ONLY
)