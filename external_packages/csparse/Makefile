
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
##   Filename:  external_packages/csparse/Makefile
##   Author:    Boris Houska and Hans Joachim Ferreau
##   Date:      2008
##

##
##   include global settings
##
LOCAL_PATH_PREFIX = ../..
include ${LOCAL_PATH_PREFIX}/include/acado/include.mk


##
##   flags
##

IFLAGS    =  -I. \
             ${HEADER_PATHS}

CPPFLAGS  =  ${IFLAGS} ${CPP_GLOBAL_FLAGS}


all: libcsparse.a

CS = cs_add.${OBJEXT} cs_amd.${OBJEXT} cs_chol.${OBJEXT} cs_cholsol.${OBJEXT} cs_counts.${OBJEXT} cs_cumsum.${OBJEXT} \
	cs_droptol.${OBJEXT} cs_dropzeros.${OBJEXT} cs_dupl.${OBJEXT} cs_entry.${OBJEXT} \
	cs_etree.${OBJEXT} cs_fkeep.${OBJEXT} cs_gaxpy.${OBJEXT} cs_happly.${OBJEXT} cs_house.${OBJEXT} cs_ipvec.${OBJEXT} \
	cs_lsolve.${OBJEXT} cs_ltsolve.${OBJEXT} cs_lu.${OBJEXT} cs_lusol.${OBJEXT} cs_util.${OBJEXT} cs_multiply.${OBJEXT} \
	cs_permute.${OBJEXT} cs_pinv.${OBJEXT} cs_post.${OBJEXT} cs_pvec.${OBJEXT} cs_qr.${OBJEXT} cs_qrsol.${OBJEXT} \
	cs_scatter.${OBJEXT} cs_schol.${OBJEXT} cs_sqr.${OBJEXT} cs_symperm.${OBJEXT} cs_tdfs.${OBJEXT} cs_malloc.${OBJEXT} \
	cs_transpose.${OBJEXT} cs_compress.${OBJEXT} cs_usolve.${OBJEXT} cs_utsolve.${OBJEXT} cs_scc.${OBJEXT} \
	cs_maxtrans.${OBJEXT} cs_dmperm.${OBJEXT} cs_updown.${OBJEXT} cs_print.${OBJEXT} cs_norm.${OBJEXT} cs_load.${OBJEXT} \
	cs_dfs.${OBJEXT} cs_reach.${OBJEXT} cs_spsolve.${OBJEXT} cs_ereach.${OBJEXT} cs_leaf.${OBJEXT} cs_randperm.${OBJEXT}

$(CS): cs.h Makefile

%.${OBJEXT}: \
	%.c cs.h
	@ echo "Creating" $@
	@ ${CPP} $(CPPFLAGS) -c $<

libcsparse.${LIBEXT}: ${CS}
	${AR} $@ $?

clean:
	${RM} -f *.${LIBEXT} *.${OBJEXT}

clobber: clean

