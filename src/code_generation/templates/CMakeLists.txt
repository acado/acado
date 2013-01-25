#
# This file is part of ACADO Toolkit.
#
# ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
# Copyright (C) 2008-2011 by Boris Houska and Hans Joachim Ferreau.
# All rights reserved.
#
# ACADO Toolkit is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.
#
# ACADO Toolkit is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
# Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License along with ACADO Toolkit; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
#

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
#	- Linux/Unix only.
#	- For each template we need three definitions, ATM.
#
# Usage:
#	- TDB
#
################################################################################

SET( INTEGRATOR_MEX_TEMPLATE ${CMAKE_CURRENT_SOURCE_DIR}/integrator_mex.c.in)
SET( RHS_MEX_TEMPLATE ${CMAKE_CURRENT_SOURCE_DIR}/rhs_mex.c.in)
SET( MPC_MEX_TEMPLATE ${CMAKE_CURRENT_SOURCE_DIR}/mpc_mex.c.in)
SET( MHE_MEX_TEMPLATE ${CMAKE_CURRENT_SOURCE_DIR}/mhe_mex.c.in)

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
SET( MPC_MEX_TEMPLATE ${INSTALL_TEMPLATE_PATH}/mpc_mex.c.in)
SET( MHE_MEX_TEMPLATE ${INSTALL_TEMPLATE_PATH}/mhe_mex.c.in)

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
		mpc_mex.c.in
		mhe_mex.c.in
	DESTINATION
		${INSTALL_INCLUDE_DIR}/acado/code_generation/templates
)
