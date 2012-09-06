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



## ======================================================================= ##
##                                                                         ##
##    FILENAME:   include/include.mk                                       ##
##    AUTHORS :   Boris Houska, Hans Joachim Ferreau, and Joel Andersson   ##
##    DATE    :   2008/2009                                                ##
##                                                                         ##
## ======================================================================= ##




## ======================================================================= ##
##                                                                         ##
##                           GENERAL SETTINGS:                             ##
##                                                                         ##
## ======================================================================= ##


   ## SYSTEM (LINUX or WIN32)
   ## -----------------------
      SYSTEM   = LINUX


   ## COMPILER  ( GNU or VC )
   ## -----------------------
      COMPILER  = GNU


   ## VERBOSE   ( YES or _NO_ )
   ## -----------------------
      VERBOSE   = NO


   ## DEBUG     ( _YES_ or NO )
   ## -----------------------
      DEBUG     = YES


   ## INLINE    ( YES or NO )
   ## -----------------------
      INLINE    = YES


   ## NAMESPACE ( YES or NO )
   ## -----------------------
      NAMESPACE = YES


   ## PIPES     ( YES or NO )
   ## -----------------------
      USE_PIPES = YES



## ======================================================================= ##
##                                                                         ##
##                     OPTIONAL EXTERNAL PACKAGES:                         ##
##                                                                         ##
## ======================================================================= ##


   ## GNUPLOT  ( YES or NO )
   ## ----------------------
#      GNUPLOT = YES


   ## QPOASES  ( YES or NO )
   ## ----------------------
      QPOASES = YES


   ## CSPARSE  ( YES or NO )
   ## ----------------------
      CSPARSE = YES


   ## XML      ( YES or NO )
   ## ----------------------
      XML     = NO




## ======================================================================= ##
##                                                                         ##
##                              WARNINGS                                   ##
##                                                                         ##
## ======================================================================= ##


      WARNINGS               = -Wall            \
                               -pedantic        \
                               -Wfloat-equal    \
                               -Wshadow         \
                               -Winline


      NO_PARENTHESES_WARNING = -Wno-parentheses




## ======================================================================= ##
##                                                                         ##
##                    COMPILER SETTINGS (GNU COMPILER)                     ##
##                                                                         ##
## ======================================================================= ##


   ifeq (${COMPILER}, GNU)


      ## DEFINITION OF STANDARD COMPILER SYNTAX:
      ## -----------------------------------------------------------------

      CPP         = g++
      AR          = ar r
      ARX         = ar x
      RM          = rm
      OBJEXT      = o
      DEF_TARGET  = -o $@

      LIBEXT      = a
      lib         = -l
      a           =


      ## THE COMPILER FLAGS:
      ## -----------------------------------------------------------------

      CPP_GLOBAL_FLAGS          = -DLINUX ${WARNINGS}

      ifeq (${NAMESPACE}, NO)
          CPP_GLOBAL_FLAGS     += -D__WITHOUT_NAMESPACE__
      endif
      ifeq (${DEBUG}, YES)
          CPP_GLOBAL_FLAGS     += -D__DEBUG__ -g -O0
      else
          CPP_GLOBAL_FLAGS     += -O3
      endif
      ifeq (${INLINE}, YES)
          CPP_GLOBAL_FLAGS     += -finline-functions
      endif


   endif


## ======================================================================= ##
##                                                                         ##
##                    COMPILER SETTINGS (VC COMPILER)                      ##
##                                                                         ##
## ======================================================================= ##


   ifeq (${COMPILER}, VC)


      ## DEFINITION OF STANDARD COMPILER SYNTAX:
      ## -----------------------------------------------------------------

      CPP        = cl
      AR         = ar r
      ARX        = ar x
      RM         = rm
      OBJEXT     = obj
      DEF_TARGET =

      LIBEXT     = lib
      lib        = ${LIBS_DIR}/lib
      a          = .${LIBEXT}


      ## THE COMPILER FLAGS:
      ## -----------------------------------------------------------------

      CPP_GLOBAL_FLAGS       =  -D__DEBUG__ -DWIN32                  \
                                -nologo -D__NO_COPYRIGHT__ -EHsc     \
                                -Dsnprintf=_snprintf -Dusleep=Sleep

   endif


## ======================================================================= ##
##                                                                         ##
##                          EXTENSIONS OF FILES                            ##
##                                                                         ##
## ======================================================================= ##


   ifeq (${SYSTEM}, WIN32)
         EXE = .exe
   else
         EXE =
   endif



## ======================================================================= ##
##                                                                         ##
##                                 PATHS                                   ##
##                                                                         ##
## ======================================================================= ##


   INCLUDE_PATHS  = -I${LOCAL_PATH_PREFIX}/include \
                    -I${LOCAL_PATH_PREFIX}/new_features/include


   EXTERNAL_PATHS = -I${LOCAL_PATH_PREFIX}/external_packages \
                    -I${LOCAL_PATH_PREFIX}/external_packages/qpOASES-3.0beta/include \


   TINYXML_INC    = -I${LOCAL_PATH_PREFIX}/external_packages/tinyxml


   MODELICA_INC   = -I"C:\Program Files\Dymola 7.2\bin\external\source"


   CSPARSE_PATH   = -I${LOCAL_PATH_PREFIX}/external_packages/include/acado_csparse \
                    -I${LOCAL_PATH_PREFIX}/external_packages/csparse


   HEADER_PATHS   = ${INCLUDE_PATHS} \
                    ${EXTERNAL_PATHS}

   LIBS_DIR       = ${LOCAL_PATH_PREFIX}/libs




## ======================================================================= ##
##                                                                         ##
##                            CORE LIBRARIES                               ##
##                                                                         ##
## ======================================================================= ##



      L_INTEGRATOR       = ${lib}acado_integrators${a}
      L_OPTIMAL_CONTROL  = ${lib}acado_optimal_control${a}
      L_TOOLKIT          = ${lib}acado_toolkit${a}





## ======================================================================= ##
##                                                                         ##
##                           EXTERNAL LIBRARIES                            ##
##                                                                         ##
## ======================================================================= ##


#      ifeq (${GNUPLOT}, YES)
#           GNUPLOT_LIBS       = ${lib}acado_acado2gnuplot${a}
#      endif

      ifeq (${QPOASES}, YES)
           QPOASES_LIBS       = ${lib}qpOASESextras3.0beta${a}
      endif

      ifeq (${CSPARSE}, YES)
           CSPARSE_LIBS       = ${lib}csparse${a}
      endif

      ifeq (${XML}, YES)
           XML_LIBS           = ${lib}acado_xml${a} ${lib}tinyxml${a}
      endif



## ======================================================================= ##
##                                                                         ##
##                           LIBRARY SHORTCUTS                             ##
##                                                                         ##
## ======================================================================= ##


      INTEGRATORS_LIBS     = -L${LIBS_DIR}          \
                               ${XML_LIBS}          \
                               ${GNUPLOT_LIBS}      \
                               ${L_INTEGRATOR}      \
                               ${CSPARSE_LIBS}

      CODE_GENERATION_LIBS = -L${LIBS_DIR}          \
                               ${XML_LIBS}          \
                               ${GNUPLOT_LIBS}      \
                               ${L_OPTIMAL_CONTROL} \
                               ${CSPARSE_LIBS}

      OPTIMAL_CONTROL_LIBS = -L${LIBS_DIR}          \
                               ${XML_LIBS}          \
                               ${GNUPLOT_LIBS}      \
                               ${L_OPTIMAL_CONTROL} \
                               ${QPOASES_LIBS}      \
                               ${CSPARSE_LIBS}

      TOOLKIT_LIBS         = -L${LIBS_DIR}          \
                               ${XML_LIBS}          \
                               ${GNUPLOT_LIBS}      \
                               ${L_TOOLKIT}         \
                               ${QPOASES_LIBS}      \
                               ${CSPARSE_LIBS}



## ======================================================================= ##
##  end of file
