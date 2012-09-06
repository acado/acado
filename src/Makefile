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
##	Filename:  src/Makefile
##	Author:    Boris Houska and Hans Joachim Ferreau and Joel Andersson
##	Date:      2009
##


##
##	include global settings
##
LOCAL_PATH_PREFIX = ..
include ${LOCAL_PATH_PREFIX}/include/acado/include.mk
include ${LOCAL_PATH_PREFIX}/include/acado/include_aux.mk

INTEGRATOR_DIRS = \
	utils \
	clock \
	matrix_vector \
	variables_grid \
	curve \
	user_interaction \
	symbolic_operator \
	symbolic_expression \
	function \
	integrator \
	sparse_solver

OPTIMAL_CONTROL_DIRS = \
	${INTEGRATOR_DIRS} \
	code_generation \
	dynamic_system \
	dynamic_discretization \
	objective \
	constraint \
	ocp \
	conic_program \
	conic_solver \
	nlp_solver \
	nlp_derivative_approximation \
	optimization_algorithm


TOOLKIT_DIRS = \
	${OPTIMAL_CONTROL_DIRS} \
	process \
	noise \
	transfer_device \
	controller \
	control_law \
	estimator \
	reference_trajectory \
	simulation_environment



##
##	flags
##

IFLAGS    =  -I. \
             ${HEADER_PATHS}

CPPFLAGS  =  ${IFLAGS} ${CPP_GLOBAL_FLAGS}


##
##	targets
##

ifeq ($(VERBOSE),NO)
.SILENT:
endif

all: ../libs/libacado_${PACKAGE_MAKE_TARGET}.${LIBEXT}

../libs/libacado_toolkit.${LIBEXT}: ../libs/libacado_optimal_control.${LIBEXT}
	@echo ""
	@echo "Creating acado_toolkit library..."
	@for i in ${TOOLKIT_DIRS} ; do       \
	   cd $${i} && ${MAKE} && ${AR} ../$@ *.${OBJEXT} && cd .. ; \
	done
	@echo "Creating acado_toolkit library... done."

../libs/libacado_optimal_control.${LIBEXT}: ../libs/libacado_integrators.${LIBEXT}
	@echo ""
	@echo "Creating acado_optimal_control library..."
	@for i in ${OPTIMAL_CONTROL_DIRS} ; do       \
	   cd $${i} && ${MAKE} && ${AR} ../$@ *.${OBJEXT} && cd .. ; \
	done
	@echo "Creating acado_optimal_control library... done."

../libs/libacado_integrators.${LIBEXT}:
	@echo ""
	@echo "Creating acado_integrators library..."
	@for i in ${INTEGRATOR_DIRS} ; do       \
	   cd $${i} && ${MAKE} && ${AR} ../$@ *.${OBJEXT} && cd .. ; \
	done
	@echo "Creating acado_integrators library... done."

###############################################################################
# CLEAN
###############################################################################

clean: clean_${PACKAGE_MAKE_TARGET}

clean_toolkit:
	@for i in ${TOOLKIT_DIRS} ; do       \
	   cd $${i} && ${MAKE} clean && cd .. ; \
	done
	${RM} -rf ../libs/libacado_toolkit.${LIBEXT}
	${RM} -rf ../libs/libacado_optimal_control.${LIBEXT}
	${RM} -rf ../libs/libacado_integrators.${LIBEXT}

clean_optimal_control:
	@for i in ${OPTIMAL_CONTROL_DIRS} ; do       \
	   cd $${i} && ${MAKE} clean && cd .. ; \
	done
	${RM} -rf ../libs/libacado_optimal_control.${LIBEXT}
	${RM} -rf ../libs/libacado_integrators.${LIBEXT}

clean_integrators:
	@for i in ${INTEGRATOR_DIRS} ; do       \
	   cd $${i} && ${MAKE} clean && cd .. ; \
	done
	${RM} -rf ../libs/libacado_integrators.${LIBEXT}


###############################################################################
# CLOBBER
###############################################################################

clobber: clobber_${PACKAGE_MAKE_TARGET}

clobber_toolkit: clean_toolkit

clobber_optimal_control: clean_optimal_control

clobber_integrators: clean_integrators


###############################################################################
# LIBS
###############################################################################

libs:
	@${RM} -f ../libs/libacado_toolkit.${LIBEXT} ../libs/libacado_optimal_control.${LIBEXT} ../libs/libacado_integrators.${LIBEXT} && \
	${MAKE} all


.PHONY: all clean clobber libs


##
##   end of file
##
