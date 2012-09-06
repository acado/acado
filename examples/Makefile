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
##	Filename:  examples/Makefile
##	Author:    Boris Houska, Hans Joachim Ferreau
##	Date:      2009
##


##
##	include global settings
##
LOCAL_PATH_PREFIX = ..
include ${LOCAL_PATH_PREFIX}/include/acado/include.mk
include ${LOCAL_PATH_PREFIX}/include/acado/include_aux.mk


##
##	targets
##

ifeq ($(VERBOSE),NO)
.SILENT:
endif

all: ${PACKAGE_MAKE_TARGET}


toolkit:
	@echo ""
	@echo "Creating examples..."
	@  cd getting_started        && ${MAKE} all && cd .. \
	&& cd ocp                    && ${MAKE} all && cd .. \
	&& cd multi_objective        && ${MAKE} all && cd .. \
	&& cd parameter_estimation   && ${MAKE} all && cd .. \
	&& cd process                && ${MAKE} all && cd .. \
	&& cd simulation_environment && ${MAKE} all && cd ..
#	&& cd basic_data_structures  && ${MAKE} all && cd ..
	@echo "Creating examples... done."

optimal_control:
	@echo ""
	@echo "Creating examples..."
	@  cd ocp                    && ${MAKE} all && cd .. \
	&& cd multi_objective        && ${MAKE} all && cd .. \
	&& cd parameter_estimation   && ${MAKE} all && cd ..
#	&& cd basic_data_structures  && ${MAKE} all && cd ..
	@echo "Creating examples... done."
 
integrators:
	@echo ""
	@echo "Creating examples..."
	@  cd integrator             && ${MAKE} all && cd ..
#	&& cd basic_data_structures  && ${MAKE} all && cd ..
	@echo "Creating examples... done."


clean: clean_${PACKAGE_MAKE_TARGET}

clean_toolkit:
	@  cd getting_started        && ${MAKE} clean && cd .. \
	&& cd ocp                    && ${MAKE} clean && cd .. \
	&& cd multi_objective        && ${MAKE} clean && cd .. \
	&& cd parameter_estimation   && ${MAKE} clean && cd .. \
	&& cd process                && ${MAKE} clean && cd .. \
	&& cd simulation_environment && ${MAKE} clean && cd ..
#	&& cd basic_data_structures  && ${MAKE} clean && cd ..

clean_optimal_control:
	@  cd getting_started        && ${MAKE} clean && cd .. \
	&& cd ocp                    && ${MAKE} clean && cd .. \
	&& cd multi_objective        && ${MAKE} clean && cd .. \
	&& cd parameter_estimation   && ${MAKE} clean && cd ..
#	&& cd basic_data_structures  && ${MAKE} clean && cd ..

clean_integrators:
	@  cd integrator             && ${MAKE} clean && cd ..
#	&& cd basic_data_structures  && ${MAKE} clean && cd ..



clobber: clobber_${PACKAGE_MAKE_TARGET}

clobber_toolkit: clean_toolkit

clobber_optimal_control: clean_optimal_control

clobber_integrators: clean_integrators


.PHONY: all clean clobber


##
##	end of file
##
