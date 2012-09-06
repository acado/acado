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
##	Filename:  examples/basic_data_structures/Makefile
##	Author:    Boris Houska, Hans Joachim Ferreau
##	Date:      2010
##


##
##	include global settings
##
LOCAL_PATH_PREFIX = ../..
include ${LOCAL_PATH_PREFIX}/include/acado/include.mk
include ${LOCAL_PATH_PREFIX}/include/acado/include_aux.mk


##
##	targets
##

ifeq ($(VERBOSE),NO)
.SILENT:
endif

all:
	@  cd matrix_vector          && ${MAKE} all && cd .. \
	&& cd function               && ${MAKE} all && cd .. \
	&& cd variables_grid         && ${MAKE} all && cd .. \
	&& cd curve                  && ${MAKE} all && cd .. \
	&& cd symbolics              && ${MAKE} all && cd ..


clean: 
	@  cd matrix_vector          && ${MAKE} clean && cd .. \
	&& cd function               && ${MAKE} clean && cd .. \
	&& cd variables_grid         && ${MAKE} clean && cd .. \
	&& cd curve                  && ${MAKE} clean && cd .. \
	&& cd symbolics              && ${MAKE} clean && cd ..


clobber: clean


##
##	end of file
##
