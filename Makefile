##
##    This file is part of ACADO Toolkit.
##
##    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
##    Copyright (C) 2008-2009 by Boris Houska and Hans Joachim Ferreau.
##    All rights reserved.
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
##   Filename:  Makefile
##   Author:    Hans Joachim Ferreau, Boris Houska
##   Date:      2009
##


##
##   include global settings
##
LOCAL_PATH_PREFIX = .
include ${LOCAL_PATH_PREFIX}/include/acado/include.mk
include ${LOCAL_PATH_PREFIX}/include/acado/include_aux.mk




##
##   flags
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

all:
	@  cd src               && ${MAKE} && cd .. \
	&& cd external_packages && ${MAKE} && cd .. \
	&& cd examples          && ${MAKE} && cd ..


clean:
	@  cd src               && ${MAKE} clean && cd .. \
	&& cd external_packages && ${MAKE} clean && cd .. \
	&& cd examples          && ${MAKE} clean && cd .. \
	&& ${RM} -f ./libs/*.${LIBEXT}


clobber: clean


doc:
	@  cd doc               && ${MAKE} && cd ..


.PHONY: all clean clobber doc


##
##   end of file
##
