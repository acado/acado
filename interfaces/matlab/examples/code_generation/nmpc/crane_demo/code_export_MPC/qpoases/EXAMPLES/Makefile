##
##	This file is part of qpOASES.
##
##	qpOASES -- An Implementation of the Online Active Set Strategy.
##	Copyright (C) 2007-2008 by Hans Joachim Ferreau et al. All rights reserved.
##
##	qpOASES is free software; you can redistribute it and/or
##	modify it under the terms of the GNU Lesser General Public
##	License as published by the Free Software Foundation; either
##	version 2.1 of the License, or (at your option) any later version.
##
##	qpOASES is distributed in the hope that it will be useful,
##	but WITHOUT ANY WARRANTY; without even the implied warranty of
##	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
##	Lesser General Public License for more details.
##
##	You should have received a copy of the GNU Lesser General Public
##	License along with qpOASES; if not, write to the Free Software
##	Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
##



##
##	Filename:  EXAMPLES/Makefile
##	Author:    Hans Joachim Ferreau
##	Version:   1.3embedded
##	Date:      2007-2008
##



##
##	bins
##

CPP = g++
RM  = rm



##
##	flags
##


LIBS_PATH           =  ../SRC
QPOASES_LIB         =  qpOASES


IDIR        =  ../INCLUDE

IFLAGS      =  -I. \
               -I${IDIR} \
               -I${LIBS_PATH}

CPPFLAGS    =  ${IFLAGS} -Wall -O3 -finline-functions -DLINUX


QPOASES_HEADERS = \
	${IDIR}/QProblem.hpp \
	${IDIR}/QProblemB.hpp \
	${IDIR}/Bounds.hpp \
	${IDIR}/Constraints.hpp \
	${IDIR}/SubjectTo.hpp \
	${IDIR}/Indexlist.hpp \
	${IDIR}/CyclingManager.hpp \
	${IDIR}/ExternalQPsolver.hpp \
	${IDIR}/Utils.hpp \
	${IDIR}/Types.hpp \
	${IDIR}/MessageHandling.hpp


##
##	targets
##

all: example1 example1b memory


example1: example1.o
	${CPP} -o $@ ${CPPFLAGS} $@.o -L${LIBS_PATH} -l${QPOASES_LIB}

example1b: example1b.o
	${CPP} -o $@ ${CPPFLAGS} $@.o -L${LIBS_PATH} -l${QPOASES_LIB}


memory: memory.o
	${CPP} -o $@ ${CPPFLAGS} $@.o -L${LIBS_PATH} -l${QPOASES_LIB}


clean:
	${RM} -f *.o


.o: \
	$*.cpp \
	${QPOASES_HEADERS} 
	@echo "Creating" $@
	${CPP} -o $@ -c ${CPPFLAGS} $*.cpp



##
##	end of file
##
