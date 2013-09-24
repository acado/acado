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
#
# Usage:
#	- TDB
#
################################################################################

SET( INTEGRATOR_MEX_TEMPLATE ${CMAKE_CURRENT_SOURCE_DIR}/integrator_mex.c.in)
SET( RHS_MEX_TEMPLATE ${CMAKE_CURRENT_SOURCE_DIR}/rhs_mex.c.in)

SET( CG_TEMPLATES_PATH
	${PROJECT_BINARY_DIR}/include/acado/code_generation/templates
)

CONFIGURE_FILE(
	"${CMAKE_CURRENT_SOURCE_DIR}/templates.hpp.in"
	"${CG_TEMPLATES_PATH}/templates.hpp"
	@ONLY
)

#
# Installation rules
#

SET( INSTALL_TEMPLATE_PATH
	${INSTALL_INCLUDE_DIR}/acado/code_generation/templates
)

SET( INTEGRATOR_MEX_TEMPLATE ${INSTALL_TEMPLATE_PATH}/integrator_mex.c.in)
SET( RHS_MEX_TEMPLATE ${INSTALL_TEMPLATE_PATH}/rhs_mex.c.in)

CONFIGURE_FILE(
	"${CMAKE_CURRENT_SOURCE_DIR}/templates.hpp.in"
	"${PROJECT_BINARY_DIR}/install/templates.hpp"
	@ONLY
)

INSTALL(
	FILES
		${PROJECT_BINARY_DIR}/install/templates.hpp
		integrator_mex.c.in
		rhs_mex.c.in
	DESTINATION
		${INSTALL_INCLUDE_DIR}/acado/code_generation/templates
)
