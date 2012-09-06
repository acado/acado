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
##	Filename:  examples/ocp/Makefile
##	Author:    Boris Houska and Hans Joachim Ferreau
##	Date:      2008-2009
##


##
##	include global settings
##
LOCAL_PATH_PREFIX = ../..
include ${LOCAL_PATH_PREFIX}/include/acado/include.mk


##
##	flags
##

IFLAGS    =    ${HEADER_PATHS}

CPPFLAGS    =  ${CPP_GLOBAL_FLAGS} ${NO_PARENTHESES_WARNING}

LIBS =  ${OPTIMAL_CONTROL_LIBS}


SRCS = \
	getting_started.cpp \
	rocket.cpp \
	rocket_c.cpp \
	time_optimal_rocket.cpp \
	discrete_time_rocket.cpp \
	discrete_time_rocket_c.cpp \
	rocket_with_logging.cpp \
	lsq_term.cpp \
	powerkite.cpp \
	powerkite_c.cpp \
	van_der_pol.cpp \
	dae_optimization_tutorial.cpp \
	dae_optimization_tutorial2.cpp \
	active_damping.cpp \
	matrix_vector_ocp.cpp \
	wave_energy.cpp \
	simple_dae_c.cpp \
	hydroscal.cpp \
	cstr.cpp

#rocket2.cpp
#dev_vpsquare.cpp


DEV_SRCS = \
	dev_crane.cpp \
	dev_lieboud_crane.cpp \
	dev_lieboud_crane2.cpp \
	dev_lieboud_crane3.cpp \
	dev_simple_linear.cpp \
	dev_lotka_volterra.cpp \
	dev_oscillator.cpp \
	dev_pendulum.cpp \
	dev_storage.cpp \
	dev_bio_reactor.cpp \
	dev_carmotion.cpp \
	dev_rocket_with_templates.cpp \
	dev_optimal_distance_rocket.cpp \
	dev_forward_mode.cpp \
	dev_random_long_horizon.cpp \
	dev_dcmotor.cpp \
	dev_getting_started_discretized.cpp \
	dev_discrete_time_rocket_c.cpp \
	dev_init_bug.cpp
#	dev_sim_OCP_tradeoff_qcar_EBW.cpp
#	dev_kite0.cpp \
#	car_ws_SiS.cpp \
#	dev_machine_tool.cpp \
#	dev_machine_tool2.cpp \
#	dev_cart_pole.cpp \
#	dev_discrete_time.cpp
#	building_ground_vc2_COPfix3_new.cpp \
#	lsq_term_c.cpp
#	quartercar_example.cpp \
#	dev_getting_started_discretized2.cpp \
#	dev_convection.cpp \
#	

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
##	end of file
##
