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
##   Filename:  external_packages/Makefile
##   Author:    Boris Houska and Hans Joachim Ferreau
##   Date:      2009
##




## ======================================================================= ##
##                                                                         ##
##                        INCLUDE GLOBAL SETTINGS:                         ##
##                                                                         ##
## ======================================================================= ##

   LOCAL_PATH_PREFIX = ..
   include ${LOCAL_PATH_PREFIX}/include/acado/include.mk
   include ${LOCAL_PATH_PREFIX}/include/acado/include_aux.mk



## ======================================================================= ##
##                                                                         ##
##                                FLAGS:                                   ##
##                                                                         ##
## ======================================================================= ##


   IFLAGS    =  -I. \
                ${HEADER_PATHS}

   CPPFLAGS  =  ${IFLAGS} ${CPP_GLOBAL_FLAGS}



## ======================================================================= ##
##                                                                         ##
##                           PACKAGE SELECTION:                            ##
##                                                                         ##
## ======================================================================= ##


   PACKAGES =


   ifeq (${QPOASES}, YES)

       PACKAGES += libqpOASES3.0beta.${LIBEXT}

   endif


   ifeq (${GNUPLOT}, YES)

   endif


   ifeq (${CSPARSE}, YES)

       PACKAGES +=  libcsparse.${LIBEXT}

   endif



## ======================================================================= ##
##                                                                         ##
##                                TARGETS:                                 ##
##                                                                         ##
## ======================================================================= ##


.SILENT:

all: ${PACKAGES}


ifeq (${QPOASES}, YES)

    libqpOASES3.0beta.${LIBEXT}:
		@echo ""
		@echo "Creating qpOASES..."
		@ cd qpOASES-3.0beta \
		&& ${MAKE} all \
		&& cp src/libqpOASES.${LIBEXT} ../../libs/libqpOASES3.0beta.${LIBEXT} \
		&& cp src/libqpOASESextras.${LIBEXT} ../../libs/libqpOASESextras3.0beta.${LIBEXT} \
		&& cd ..
		@echo "Creating qpOASES... done."

endif


ifeq (${GNUPLOT}, YES)

endif


ifeq (${CSPARSE}, YES)

    libcsparse.${LIBEXT}:
		@echo ""
		@echo "Creating csparse..."
		@ cd  csparse \
		&& ${MAKE} all \
		&& cp libcsparse.${LIBEXT} ../../libs \
		&& cd ..
		@echo "Creating csparse... done."

endif



clean:
	@  cd qpOASES-3.0beta   && ${MAKE} clean   && cd .. \
	&& cd csparse           && ${MAKE} clean   && cd ..


clobber: 
	@  cd qpOASES-3.0beta   && ${MAKE} clobber && cd .. \
	&& cd csparse           && ${MAKE} clobber && cd ..


.PHONY: all clean clobber


##
##   end of file
##
