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
##	Filename:  src/symbolic_expression/Makefile
##	Author:    Boris Houska and Hans Joachim Ferreau
##	Date:      2010
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
	acado_syntax.${OBJEXT} \
	algebraic_state.${OBJEXT} \
	control.${OBJEXT} \
	constraint_component.${OBJEXT} \
	differential_state.${OBJEXT} \
	differential_state_derivative.${OBJEXT} \
	disturbance.${OBJEXT} \
	expression.${OBJEXT} \
	integer_control.${OBJEXT} \
	integer_parameter.${OBJEXT} \
	intermediate_state.${OBJEXT} \
	parameter.${OBJEXT} \
	vt_time.${OBJEXT} \
	output.${OBJEXT}\
	lyapunov.${OBJEXT}


##
##	targets
##

all: libacado_symbolic_expression.${LIBEXT}


libacado_symbolic_expression.${LIBEXT}: ${OBJECTS}
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
