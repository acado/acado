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
##	Filename:  examples/multi_objective/Makefile
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

LIBS =  ${OPTIMAL_CONTROL_LIBS}


SRCS = \
    car_ws.cpp                    \
    car_nnc.cpp                   \
    car_nbi.cpp                   \
    catalyst_mixing_ws.cpp        \
    catalyst_mixing_nnc.cpp       \
    catalyst_mixing_nbi.cpp       \
    crane_ws.cpp                  \
    crane_nnc.cpp                 \
    crane_nbi.cpp                 \
    fed_batch_bioreactor_ws.cpp   \
    fed_batch_bioreactor_nnc.cpp  \
    fed_batch_bioreactor_nbi.cpp  \
    fed_batch_bioreactor2_ws.cpp  \
    fed_batch_bioreactor2_nnc.cpp \
    fed_batch_bioreactor2_nbi.cpp \
    plug_flow_reactor_ws.cpp      \
    plug_flow_reactor_nnc.cpp     \
    plug_flow_reactor_nbi.cpp     \
    plug_flow_reactor2_ws.cpp     \
    plug_flow_reactor2_nnc.cpp    \
    plug_flow_reactor2_nbi.cpp    \
    scalar2_ws.cpp		  \
    scalar2_nnc.cpp		  \
    scalar2_nbi.cpp		  \
    scalar3_ws.cpp		  \
    scalar3_nnc.cpp		  \
    scalar3_nbi.cpp


#DEV_SRCS = \
#    scalar2.cpp 		  \
#    scalar3.cpp


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

${EXECS} : %${EXE} : %.${OBJEXT} #L_OPTIMAL_CONTROL
	@echo "Creating" $@
	${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${LIBS}

${DEV_EXECS} : %${EXE} : %.${OBJEXT}
	@echo "Creating" $@
	${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${LIBS}

${OBJS} : %.${OBJEXT}: %.cpp
	${CPP} ${DEF_TARGET} -c ${IFLAGS} ${CPPFLAGS} $<

${DEV_OBJS} : %.${OBJEXT}: %.cpp
	${CPP} ${DEF_TARGET} -c ${IFLAGS} ${CPPFLAGS} $<

#L_OPTIMAL_CONTROL:
#	@ cd ${LOCAL_PATH_PREFIX}/src && ${MAKE} libs && cd ../examples/multi_objective

clean:
	${RM} -f ${OBJS} ${EXECS} ${DEV_OBJS} ${DEV_EXECS}

clobber: clean


##
##	end of file
##
