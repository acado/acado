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
##	Filename:  src/optimization_algorithm/Makefile
##	Author:    Boris Houska and Hans Joachim Ferreau
##	Date:      2008
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
	optimization_algorithm_base.${OBJEXT} \
	optimization_algorithm.${OBJEXT} \
	real_time_algorithm.${OBJEXT} \
	parameter_estimation_algorithm.${OBJEXT} \
	multi_objective_algorithm.${OBJEXT} \
	weight_generation.${OBJEXT} \
	mhe_algorithm.${OBJEXT}


	
##
##	targets
##

all: libacado_optimization_algorithm.${LIBEXT}


libacado_optimization_algorithm.${LIBEXT}: ${OBJECTS}
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
