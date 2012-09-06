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
##    Filename:  examples/integrator/Makefile
##    Author:    Boris Houska and Hans Joachim Ferreau
##    Date:      2008
##


##
##    include global settings
##

LOCAL_PATH_PREFIX = ../..
include ${LOCAL_PATH_PREFIX}/include/acado/include.mk

##
##    flags
##


IFLAGS    =  ${HEADER_PATHS}
CPPFLAGS  =  ${CPP_GLOBAL_FLAGS}

LIBS      =  ${INTEGRATORS_LIBS}

SRCS = \
	getting_started.cpp \
	harmonic_oscillator.cpp \
	forward_differentiation.cpp \
	backward_differentiation.cpp \
	pendulum.cpp \
	pendulum_C.cpp \
	chemical_rate_equations.cpp \
	getting_started_discretized.cpp \
	simple_dae.cpp \
	simple_dae_c.cpp \
	hydroscal.cpp \
	wave_energy.cpp \
	transition.cpp \
        batch.cpp \
	cstr.cpp


DEV_SRCS = \
	dev_pendulum_C.cpp \
	dev_linking_jacobian_C.cpp \
	dev_kite0.cpp \
	dev_liebouds_system.cpp \
	dev_pendulum_CPP.cpp \
	dev_active_damping_discretized.cpp

#	dev_sparse_dae.cpp \
#	dev_debug.cpp \
#	dev_order_cross_check.cpp \
#	dev_simple_dae1.cpp \
#	dev_simple_dae2.cpp \
#	dev_3dof_robot.cpp \
#	dev_mass_spring_system2.cpp \
#	dev_liebouds_system.cpp \
#	dev_integrator_options_test.cpp \
#	dev_bio_reactor.cpp \
#	dev_two_models.cpp \
#	dev_crane.cpp \
#	dev_car.cpp \
#	dev_lotka_volterra.cpp \
#	dev_kite1.cpp \
#	dev_segway.cpp \
#	dev_simple_dae.cpp \
#	dev_double_pendulum.cpp \
#	dev_glycemia_p01.cpp \
#	dev_temperature_oscillation_calorimetry.cpp \
#	dev_lorentz_attractor.cpp \
#	dev_fitz_hugh_nagumo.cpp \
#	dev_mass_spring_system.cpp \
#	dev_intelligent_cruise_control.cpp \
#	dev_furuta.cpp \
#	dev_debug2.cpp \
#	dev_debug3.cpp \
#	dev_linking_jacobian_C.cpp \
#	dev_langrange.cpp \
#	dev_lagrange_double_pendulum.cpp \
#	dev_newton_euler_tutorial.cpp \
#	dev_debug4.cpp \
#	dev_kite0.cpp \
#	steady_state.cpp



OBJS = $(SRCS:.cpp=.${OBJEXT})
EXECS = $(SRCS:.cpp=${EXE})

DEV_OBJS = $(DEV_SRCS:.cpp=.${OBJEXT})
DEV_EXECS = $(DEV_SRCS:.cpp=${EXE})


##
##    targets
##

ifeq ($(VERBOSE),NO)
.SILENT:
endif

all: ${EXECS}

developer: ${EXECS} ${DEV_EXECS}

${EXECS} : %${EXE} : %.${OBJEXT}
	@echo "Creating" $@
	${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${LIBS}

${DEV_EXECS} : %${EXE} : %.${OBJEXT}
	@echo "Creating" $@
	${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${LIBS}

${OBJS} : %.${OBJEXT}: %.cpp
	${CPP} ${DEF_TARGET} -c ${IFLAGS} ${CPPFLAGS} $<

${DEV_OBJS} : %.${OBJEXT}: %.cpp
	${CPP} ${DEF_TARGET} -c ${IFLAGS} ${CPPFLAGS} $<

clean:
	${RM} -f ${OBJS} ${EXECS} ${DEV_OBJS} ${DEV_EXECS}

clobber: clean




##
##    end of file
##
