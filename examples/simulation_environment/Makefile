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
##	Filename:  examples/simulation_environment/Makefile
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

IFLAGS    =    ${HEADER_PATHS}

CPPFLAGS    =  ${CPP_GLOBAL_FLAGS} ${NO_PARENTHESES_WARNING}

EXPORTED_RTI_LIBS = 
LIBS = ${TOOLKIT_LIBS} ${EXPORTED_RTI_LIBS}


SRCS = \
	getting_started.cpp \
	getting_started_classical.cpp \
	periodic_tracking.cpp \
	powerkite_on.cpp \
	fourtankNMPC.cpp \
	active_damping_stepped.cpp


DEV_SRCS = \
	dev_kite_carousel.cpp \
	dev_dcmotor.cpp \
	dev_dcmotor2.cpp \
	dev_powerkite_nominal.cpp \
	dev_sienginempc1fninode.cpp \
	dev_powerkite_on_C.cpp \
	dev_active_damping_C.cpp \
	dev_discrete_time_rocket.cpp

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
