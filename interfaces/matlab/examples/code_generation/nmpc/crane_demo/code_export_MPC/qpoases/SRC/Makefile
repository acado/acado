##
##	This file is part of qpOASES.
##
##	qpOASES -- An Implementation of the Online Active Set Strategy.
##	Copyright (C) 2007 by Hans Joachim Ferreau et al. All rights reserved.
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
##	Filename:  SRC/Makefile
##	Author:    Hans Joachim Ferreau
##	Version:   1.3embedded
##	Date:      2007-2008
##



##
##	bins
##

CPP = g++
AR  = ar
RM  = rm



##
##	flags
##

IDIR        =  ../INCLUDE
SRCDIR      =  ../SRC

IFLAGS      =  -I. \
               -I${IDIR} \
               -I${SRCDIR}

CPPFLAGS    =  ${IFLAGS} -Wall -Wextra -Wshadow -Winline -Wfloat-equal -pedantic -O3 -finline-functions -DLINUX 
##-DPC_DEBUG -D__PERFORM_KKT_TEST__
## -D__PERFORM_KKT_TEST__


QPOASES_OBJECTS = \
	QProblem.o \
	QProblemB.o \
	Bounds.o \
	Constraints.o \
	SubjectTo.o \
	Indexlist.o \
	CyclingManager.o \
	Utils.o \
	MessageHandling.o



##
##	targets
##

all: libqpOASES.a


libqpOASES.a: ${QPOASES_OBJECTS}
	${AR} r $@ $?


QProblem.o: \
	QProblem.cpp \
	${IDIR}/QProblem.hpp \
	${IDIR}/QProblemB.hpp \
	${IDIR}/Bounds.hpp \
	${IDIR}/Constraints.hpp \
	${IDIR}/SubjectTo.hpp \
	${IDIR}/Indexlist.hpp \
	${IDIR}/CyclingManager.hpp \
	${IDIR}/Utils.hpp \
	${IDIR}/Constants.hpp \
	${IDIR}/Types.hpp \
	${IDIR}/MessageHandling.hpp
	@echo "Creating" $@
	${CPP} -o $@ -c ${CPPFLAGS} QProblem.cpp

QProblemB.o: \
	QProblemB.cpp \
	${IDIR}/QProblemB.hpp \
	${IDIR}/Bounds.hpp \
	${IDIR}/Constraints.hpp \
	${IDIR}/SubjectTo.hpp \
	${IDIR}/Indexlist.hpp \
	${IDIR}/Utils.hpp \
	${IDIR}/Constants.hpp \
	${IDIR}/Types.hpp \
	${IDIR}/MessageHandling.hpp
	@echo "Creating" $@
	${CPP} -o $@ -c ${CPPFLAGS} QProblemB.cpp
	
Bounds.o: Bounds.cpp \
	${IDIR}/Bounds.hpp \
	${IDIR}/SubjectTo.hpp \
	${IDIR}/Indexlist.hpp \
	${IDIR}/Utils.hpp \
	${IDIR}/Constants.hpp \
	${IDIR}/Types.hpp \
	${IDIR}/MessageHandling.hpp
	@echo "Creating" $@
	${CPP} -o $@ -c ${CPPFLAGS} Bounds.cpp

Constraints.o: \
	Constraints.cpp \
	${IDIR}/Constraints.hpp \
	${IDIR}/SubjectTo.hpp \
	${IDIR}/Indexlist.hpp \
	${IDIR}/Utils.hpp \
	${IDIR}/Constants.hpp \
	${IDIR}/Types.hpp \
	${IDIR}/MessageHandling.hpp
	@echo "Creating" $@
	${CPP} -o $@ -c ${CPPFLAGS} Constraints.cpp

SubjectTo.o: \
	SubjectTo.cpp \
	${IDIR}/SubjectTo.hpp \
	${IDIR}/Indexlist.hpp \
	${IDIR}/Utils.hpp \
	${IDIR}/Constants.hpp \
	${IDIR}/Types.hpp \
	${IDIR}/MessageHandling.hpp
	@echo "Creating" $@
	${CPP} -o $@ -c ${CPPFLAGS} SubjectTo.cpp

Indexlist.o: \
	Indexlist.cpp \
	${IDIR}/Indexlist.hpp \
	${IDIR}/Utils.hpp \
	${IDIR}/Constants.hpp \
	${IDIR}/Types.hpp \
	${IDIR}/MessageHandling.hpp
	@echo "Creating" $@
	${CPP} -o $@ -c ${CPPFLAGS} Indexlist.cpp

CyclingManager.o: \
	CyclingManager.cpp \
	${IDIR}/CyclingManager.hpp \
	${IDIR}/Utils.hpp \
	${IDIR}/Constants.hpp \
	${IDIR}/Types.hpp \
	${IDIR}/MessageHandling.hpp
	@echo "Creating" $@
	${CPP} -o $@ -c ${CPPFLAGS} CyclingManager.cpp

Utils.o: \
	Utils.cpp \
	${IDIR}/Utils.hpp \
	${IDIR}/Constants.hpp \
	${IDIR}/Types.hpp \
	${IDIR}/MessageHandling.hpp
	@echo "Creating" $@
	${CPP} -o $@ -c ${CPPFLAGS} Utils.cpp

MessageHandling.o: \
	MessageHandling.cpp \
	${IDIR}/Utils.hpp \
	${IDIR}/Constants.hpp \
	${IDIR}/Types.hpp \
	${IDIR}/MessageHandling.hpp
	@echo "Creating" $@
	${CPP} -o $@ -c ${CPPFLAGS} MessageHandling.cpp


clean:
	${RM} -f *.o */*.o


clobber:
	${MAKE} clean; ${RM} -f *.a



##
##	end of file
##
