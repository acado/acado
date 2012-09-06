##
##    This file is part of ACADO Toolkit.
##
##    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
##    Copyright (C) 2008-2009 by Boris Houska and Hans Joachim Ferreau, K.U.Leuven.
##    Developed within the Optimization in Engineering Center (OPTEC) under
##    supervision of Moritz Diehl. All rights reserved.
##
##    ACADO Toolkit is free software; you can redistribute it and/or
##    modify it under the terms of the GNU Lesser General Public
##    License as published by the Free Software Foundation; either
##    version 3 of the License, or (at your option) any later version.
##
##    ACADO Toolkit is distributed in the hope that it will be useful,
##    but WITHOUT ANY WARRANTY; without even the implied warranty of
##    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
##    Lesser General Public License for more details.
##
##    You should have received a copy of the GNU Lesser General Public
##    License along with ACADO Toolkit; if not, write to the Free Software
##    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
##
##


##
##	Filename:  src/code_generation/Makefile
##	Author:    Boris Houska and Hans Joachim Ferreau
##	Date:      2010-2011
##


##
##	include global settings
##
LOCAL_PATH_PREFIX = ../..
include ${LOCAL_PATH_PREFIX}/include/acado/include.mk



##
##	flags
##

IFLAGS    =  -I. \
             ${HEADER_PATHS}

CPPFLAGS  =  ${IFLAGS} ${CPP_GLOBAL_FLAGS}



OBJECTS = \
	linear_solver_export.${OBJEXT} \
	householder_qr_export.${OBJEXT} \
	gaussian_elimination_export.${OBJEXT} \
	integrator_export.${OBJEXT} \
	rk_export.${OBJEXT} \
	erk_export.${OBJEXT} \
	explicit_euler_export.${OBJEXT} \
	erk2_export.${OBJEXT} \
	erk3_export.${OBJEXT} \
	erk4_export.${OBJEXT} \
	irk_export.${OBJEXT} \
	gauss_legendre2_export.${OBJEXT} \
	gauss_legendre4_export.${OBJEXT} \
	gauss_legendre6_export.${OBJEXT} \
	gauss_legendre8_export.${OBJEXT} \
	radau_IIA1_export.${OBJEXT} \
	radau_IIA3_export.${OBJEXT} \
	radau_IIA5_export.${OBJEXT} \
	condensing_export.${OBJEXT} \
	gauss_newton_export.${OBJEXT} \
	auxiliary_functions_export.${OBJEXT} \
	export_algorithm.${OBJEXT} \
	mpc_export.${OBJEXT} \
	sim_export.${OBJEXT} \
	export_module.${OBJEXT} \
	export_data.${OBJEXT} \
	export_index.${OBJEXT} \
	export_argument.${OBJEXT} \
	export_variable.${OBJEXT} \
	export_argument_list.${OBJEXT} \
	export_function.${OBJEXT} \
	export_ode_function.${OBJEXT} \
	export_statement.${OBJEXT} \
	export_arithmetic_statement.${OBJEXT} \
	export_statement_block.${OBJEXT} \
	export_file.${OBJEXT} \
	export_for_loop.${OBJEXT} \
	export_function_call.${OBJEXT} \
	export_statement_string.${OBJEXT} \
	export_data_declaration.${OBJEXT} \
	export_function_declaration.${OBJEXT} \
	export_printf.${OBJEXT}

##
##	targets
##

all: libacado_code_generation.${LIBEXT}


libacado_code_generation.${LIBEXT}: ${OBJECTS}
	${AR} $@ $?


%.${OBJEXT}: %.cpp
	echo "Creating" $@
	@ ${CPP} ${DEF_TARGET} -c ${IFLAGS} ${CPPFLAGS} $<


clean:
	${RM} -f *.${LIBEXT} *.${OBJEXT}

clobber: clean



##
##	end of file
##
