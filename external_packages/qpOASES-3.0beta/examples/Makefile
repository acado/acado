##
##	This file is part of qpOASES.
##
##	qpOASES -- An Implementation of the Online Active Set Strategy.
##	Copyright (C) 2007-2011 by Hans Joachim Ferreau, Andreas Potschka,
##	Christian Kirches et al. All rights reserved.
##
##	qpOASES is free software; you can redistribute it and/or
##	modify it under the terms of the GNU Lesser General Public
##	License as published by the Free Software Foundation; either
##	version 2.1 of the License, or (at your option) any later version.
##
##	qpOASES is distributed in the hope that it will be useful,
##	but WITHOUT ANY WARRANTY; without even the implied warranty of
##	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
##	See the GNU Lesser General Public License for more details.
##
##	You should have received a copy of the GNU Lesser General Public
##	License along with qpOASES; if not, write to the Free Software
##	Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
##



##
##	Filename:  examples/Makefile
##	Author:    Hans Joachim Ferreau
##	Version:   3.0beta
##	Date:      2007-2011
##



# select your operating system here!
include ../src/make_linux.mk
#include ../src/make_windows.mk


##
##	flags
##

IDIR        =  ../include
SRCDIR      =  ../src
EXTRAS_DIR  =  extras

IFLAGS      =  -I. \
               -I${IDIR}

QPOASES_EXES = \
	example1${EXE} \
	example1a${EXE} \
	example1b${EXE} \
	example2${EXE} \
	example3${EXE} \
	example3b${EXE} \
	example4${EXE} \
	example5${EXE} \
	example6${EXE} \
	exampleLP${EXE} \
	qrecipe${EXE}

QPOASES_DEV_EXES = \
	dev_dieseltest${EXE} \
	dev_example7${EXE} \
	dev_idhessian${EXE}


QPOASES_HEADERS = \
	${IDIR}/SQProblem.hpp \
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
	${IDIR}/MessageHandling.hpp \
	${IDIR}/${EXTRAS_DIR}/SolutionAnalysis.hpp \
	${IDIR}/${EXTRAS_DIR}/OQPinterface.hpp



##
##	targets
##

all: ${QPOASES_EXES}

developer: ${QPOASES_EXES} ${QPOASES_DEV_EXES}


example1${EXE}: example1.${OBJEXT} ../src/libqpOASES.a
	${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${QPOASES_LIB}  #${LIB_LAPACK} ${LIB_BLAS}

example1a${EXE}: example1a.${OBJEXT} ../src/libqpOASES.a
	${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${QPOASES_LIB}  #${LIB_LAPACK} ${LIB_BLAS}

example1b${EXE}: example1b.${OBJEXT} ../src/libqpOASES.a
	${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${QPOASES_LIB}  #${LIB_LAPACK} ${LIB_BLAS}

example2${EXE}: example2.${OBJEXT} ../src/libqpOASESextras.a
	${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${QPOASES_EXTRAS_LIB} #${LIB_LAPACK} ${LIB_BLAS}

example3${EXE}: example3.${OBJEXT} ../src/libqpOASESextras.a
	${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${QPOASES_EXTRAS_LIB} #${LIB_LAPACK} ${LIB_BLAS}

example3b${EXE}: example3b.${OBJEXT} ../src/libqpOASESextras.a
	${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${QPOASES_EXTRAS_LIB} #${LIB_LAPACK} ${LIB_BLAS}

example4${EXE}: example4.${OBJEXT} ../src/libqpOASESextras.a
	${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${QPOASES_EXTRAS_LIB} #${LIB_LAPACK} ${LIB_BLAS}

example5${EXE}: example5.${OBJEXT} ../src/libqpOASESextras.a
	${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${QPOASES_EXTRAS_LIB} #${LIB_LAPACK} ${LIB_BLAS}

example6${EXE}: example6.${OBJEXT} ../src/libqpOASESextras.a
	${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${QPOASES_EXTRAS_LIB} #${LIB_LAPACK} ${LIB_BLAS}
	
example7${EXE}: example7.${OBJEXT} ../src/libqpOASESextras.a
	${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${QPOASES_EXTRAS_LIB} #${LIB_LAPACK} ${LIB_BLAS}

exampleLP${EXE}: exampleLP.${OBJEXT} ../src/libqpOASES.a
	${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${QPOASES_LIB} #${LIB_LAPACK} ${LIB_BLAS}

qrecipe${EXE}: qrecipe.${OBJEXT} ../src/libqpOASESextras.a
	${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${QPOASES_EXTRAS_LIB} #${LIB_LAPACK} ${LIB_BLAS}


dev_dieseltest${EXE}: dev_dieseltest.${OBJEXT} ../src/libqpOASESextras.a
	${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${QPOASES_EXTRAS_LIB} #${LIB_LAPACK} ${LIB_BLAS}
	
dev_example7${EXE}: dev_example7.${OBJEXT} ../src/libqpOASES.a
	${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${QPOASES_LIB} #${LIB_LAPACK} ${LIB_BLAS}
	
dev_idhessian${EXE}: dev_idhessian.${OBJEXT} ../src/libqpOASES.a
	${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${QPOASES_LIB} #${LIB_LAPACK} ${LIB_BLAS}


clean:
	${RM} -f *.${OBJEXT} ${QPOASES_EXES}

clobber: clean


%.${OBJEXT}: %.cpp
	@echo "Creating" $@
	${CPP} ${DEF_TARGET} ${IFLAGS} ${CPPFLAGS} -c $<


##
##	end of file
##
