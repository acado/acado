#
# This file is part of ACADO Toolkit.
#
# ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
# Copyright (C) 2008-2011 by Boris Houska and Hans Joachim Ferreau.
# All rights reserved.
#
# ACADO Toolkit is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.
#
# ACADO Toolkit is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
# Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License along with ACADO Toolkit; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
#

################################################################################
#
# Description:
#	Script for discovering SSE CPU flags
#
# Authors:
#	Enrico Bertolazzi, Universita` degli Studi di Trento
#	Milan Vukov, milan.vukov@esat.kuleuven.be
#
# Year:
#	2011.
#
# Usage:
#	- /
#
################################################################################

################################################################################
#
# Modified version of CHECK_CXX_SOURCE_RUNS
#
################################################################################
MACRO(CHECK_CXX_SOURCE_RUNS SOURCE VAR)
  IF("${VAR}" MATCHES "^${VAR}$")
    SET(MACRO_CHECK_FUNCTION_DEFINITIONS "-D${VAR} ${CMAKE_REQUIRED_FLAGS}")
    IF(CMAKE_REQUIRED_LIBRARIES)
      SET(CHECK_CXX_SOURCE_COMPILES_ADD_LIBRARIES "-DLINK_LIBRARIES:STRING=${CMAKE_REQUIRED_LIBRARIES}")
    ELSE(CMAKE_REQUIRED_LIBRARIES)
      SET(CHECK_CXX_SOURCE_COMPILES_ADD_LIBRARIES)
    ENDIF(CMAKE_REQUIRED_LIBRARIES)
    IF(CMAKE_REQUIRED_INCLUDES)
      SET(CHECK_CXX_SOURCE_COMPILES_ADD_INCLUDES "-DINCLUDE_DIRECTORIES:STRING=${CMAKE_REQUIRED_INCLUDES}")
    ELSE(CMAKE_REQUIRED_INCLUDES)
      SET(CHECK_CXX_SOURCE_COMPILES_ADD_INCLUDES)
    ENDIF(CMAKE_REQUIRED_INCLUDES)

    MESSAGE(STATUS "Performing Test ${VAR}")
    TRY_RUN(${VAR}_EXITCODE ${VAR}_COMPILED
      ${CMAKE_BINARY_DIR}
      ${SOURCE}
      COMPILE_DEFINITIONS ${CMAKE_REQUIRED_DEFINITIONS}
      CMAKE_FLAGS -DCOMPILE_DEFINITIONS:STRING=${MACRO_CHECK_FUNCTION_DEFINITIONS}
      -DCMAKE_SKIP_RPATH:BOOL=${CMAKE_SKIP_RPATH}
      "${CHECK_CXX_SOURCE_COMPILES_ADD_LIBRARIES}"
      "${CHECK_CXX_SOURCE_COMPILES_ADD_INCLUDES}"
      COMPILE_OUTPUT_VARIABLE OUTPUT)

    # if it did not compile make the return value fail code of 1
    IF(NOT ${VAR}_COMPILED)
      SET(${VAR}_EXITCODE 1)
    ENDIF(NOT ${VAR}_COMPILED)
    # if the return value was 0 then it worked
    IF("${${VAR}_EXITCODE}" EQUAL 0)
      SET(${VAR} 1 CACHE INTERNAL "Test ${VAR}")
      MESSAGE(STATUS "Performing Test ${VAR} - Success")
      FILE(APPEND ${CMAKE_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/CMakeOutput.log 
        "Performing C++ SOURCE FILE Test ${VAR} succeded with the following output:\n"
        "${OUTPUT}\n" 
        "Return value: ${${VAR}}\n"
        "Source file was:\n${SOURCE}\n")
    ELSE("${${VAR}_EXITCODE}" EQUAL 0)
      IF(CMAKE_CROSSCOMPILING AND "${${VAR}_EXITCODE}" MATCHES  "FAILED_TO_RUN")
        SET(${VAR} "${${VAR}_EXITCODE}")
      ELSE(CMAKE_CROSSCOMPILING AND "${${VAR}_EXITCODE}" MATCHES  "FAILED_TO_RUN")
        SET(${VAR} "" CACHE INTERNAL "Test ${VAR}")
      ENDIF(CMAKE_CROSSCOMPILING AND "${${VAR}_EXITCODE}" MATCHES  "FAILED_TO_RUN")

      MESSAGE(STATUS "Performing Test ${VAR} - Failed")
      FILE(APPEND ${CMAKE_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/CMakeError.log 
        "Performing C++ SOURCE FILE Test ${VAR} failed with the following output:\n"
        "${OUTPUT}\n"  
        "Return value: ${${VAR}_EXITCODE}\n"
        "Source file was:\n${SOURCE}\n")
    ENDIF("${${VAR}_EXITCODE}" EQUAL 0)
  ENDIF("${VAR}" MATCHES "^${VAR}$")
ENDMACRO(CHECK_CXX_SOURCE_RUNS)

################################################################################
#
# Part for detecting SSE CPU flags
#
################################################################################
IF( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX OR "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang" )
	#
	# Linux/Apple + GCC
	#

	MESSAGE(STATUS "********************************************************************************")
	MESSAGE(STATUS "Detecting SSE ...")

	GET_FILENAME_COMPONENT(scriptPath ${CMAKE_CURRENT_LIST_FILE} PATH)
	SET( CPUINFO "${scriptPath}/cpuInfo.cc" )

	SET( CMAKE_REQUIRED_FLAGS "-fPIC -msse4.2" )
	SET( CMAKE_REQUIRED_DEFINITIONS -DTEST=m_bSSE42 )
	CHECK_CXX_SOURCE_RUNS( ${CPUINFO} HAS_SSE42_EXTENSIONS)

	SET( CMAKE_REQUIRED_FLAGS "-fPIC -msse4.1" )
	SET( CMAKE_REQUIRED_DEFINITIONS -DTEST=m_bSSE41 )
	CHECK_CXX_SOURCE_RUNS( ${CPUINFO} HAS_SSE41_EXTENSIONS )

	SET( CMAKE_REQUIRED_FLAGS "-fPIC -msse4a" )
	SET( CMAKE_REQUIRED_DEFINITIONS -DTEST=m_bSSE4a )
	CHECK_CXX_SOURCE_RUNS( ${CPUINFO} HAS_SSE4A_EXTENSIONS )

	SET( CMAKE_REQUIRED_FLAGS "-fPIC -mssse3" )
	SET( CMAKE_REQUIRED_DEFINITIONS -DTEST=m_bSSSE3 )
	CHECK_CXX_SOURCE_RUNS( ${CPUINFO} HAS_SSSE3_EXTENSIONS )

	SET( CMAKE_REQUIRED_FLAGS "-fPIC -msse3" )
	SET( CMAKE_REQUIRED_DEFINITIONS -DTEST=m_bSSE3 )
	CHECK_CXX_SOURCE_RUNS( ${CPUINFO} HAS_SSE3_EXTENSIONS )

	SET( CMAKE_REQUIRED_FLAGS "-fPIC -msse2" )
	SET( CMAKE_REQUIRED_DEFINITIONS -DTEST=m_bSSE2 )
	CHECK_CXX_SOURCE_RUNS( ${CPUINFO} HAS_SSE2_EXTENSIONS )

	SET( CMAKE_REQUIRED_FLAGS "-fPIC -msse" )
	SET( CMAKE_REQUIRED_DEFINITIONS -DTEST=m_bSSE )
	CHECK_CXX_SOURCE_RUNS( ${CPUINFO} HAS_SSE_EXTENSIONS )

	SET( CMAKE_REQUIRED_FLAGS "-fPIC -mmmx" )
	SET( CMAKE_REQUIRED_DEFINITIONS -DTEST=m_bMMX )
	CHECK_CXX_SOURCE_RUNS( ${CPUINFO} HAS_MMX_EXTENSIONS )
	
	SET( CMAKE_REQUIRED_FLAGS "" ) 
	
	UNSET( SSE_FLAGS )
	
	IF ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
		SET (SSE_FLAGS "-mfpmath=sse")
	ENDIF()
	
	IF (HAS_SSE42_EXTENSIONS) 
		SET(SSE_FLAGS "${SSE_FLAGS} -msse4.2") 
	ENDIF()
	IF (HAS_SSE41_EXTENSIONS)
		SET(SSE_FLAGS "${SSE_FLAGS} -msse4.1") 
	ENDIF()
	IF (HAS_SSE4A_EXTENSIONS)
		SET(SSE_FLAGS "${SSE_FLAGS} -msse4a") 
	ENDIF()
	IF (HAS_SSSE3_EXTENSIONS)
		SET(SSE_FLAGS "${SSE_FLAGS} -mssse3") 
	ENDIF() 
	IF (HAS_SSE3_EXTENSIONS) 
		SET(SSE_FLAGS "${SSE_FLAGS} -msse3") 
	ENDIF() 
	IF (HAS_SSE2_EXTENSIONS) 
		SET(SSE_FLAGS "${SSE_FLAGS} -msse2") 
	ENDIF() 
	IF (HAS_SSE_EXTENSIONS) 
		SET(SSE_FLAGS "${SSE_FLAGS} -msse") 
	ENDIF() 
	IF (HAS_MMX_EXTENSIONS) 
    	SET(SSE_FLAGS "${SSE_FLAGS} -mmmx") 
	ENDIF()

	MESSAGE(STATUS "Using ${SSE_FLAGS} extensions")
	MESSAGE(STATUS "********************************************************************************")

ELSEIF(MSVC)
	#
	# MS Windows + MS Visual Studio 
	#

	GET_FILENAME_COMPONENT( scriptPath ${CMAKE_CURRENT_LIST_FILE} PATH )
  	SET( CPUINFO "${scriptPath}/cpuInfoWin.c" )

	CHECK_CXX_SOURCE_RUNS( ${CPUINFO} HAS_SSE2_EXTENSIONS ) 
	IF( HAS_SSE2_EXTENSIONS )
		MESSAGE( STATUS "Using SSE2 extensions" ) 
		ADD_DEFINITIONS( "/arch:SSE2 /fp:fast -D__SSE__ -D__SSE2__" ) 
	ENDIF()
ENDIF()
