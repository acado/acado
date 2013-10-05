################################################################################
#
# Description:
#	CMake scipt for building closed loop crane example to be used with
#	Simulink.
#
# Authors:
#	Milan Vukov, milan.vukov@esat.kuleuven.be
#
# Year:
#	2013.
#
# Usage:
#	- This file should be called from the main CMake script for examples.
#
################################################################################

################################################################################
#
# Compilation of examples
#
################################################################################

IF ( NOT ACADO_BUILD )
	RETURN()
ENDIF()

#
# Crane NMPC
#
SET( CURR_EXE crane_cl_nmpc )
ADD_EXECUTABLE( ${CURR_EXE} crane_cl_nmpc.cpp )

IF (ACADO_BUILD_SHARED)
	TARGET_LINK_LIBRARIES(
		${CURR_EXE}
		${ACADO_SHARED_LIBRARIES}
	)
ELSE()
	TARGET_LINK_LIBRARIES(
		${CURR_EXE}
		${ACADO_STATIC_LIBRARIES}
	)
ENDIF()
		
SET_TARGET_PROPERTIES(
	${CURR_EXE}
	PROPERTIES
		RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
		# This one is Visual Studio specific
		FOLDER "examples"
)

FILE( MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/crane_cl_nmpc_export )
FILE(
	COPY
		${ACADO_QPOASES_EMBEDDED_FOLDER}
	DESTINATION
		${CMAKE_CURRENT_SOURCE_DIR}/crane_cl_nmpc_export
)

#
# Crane MHE
#
SET ( CURR_EXE crane_cl_mhe )
ADD_EXECUTABLE( ${CURR_EXE} crane_cl_mhe.cpp )

IF (ACADO_BUILD_SHARED)
	TARGET_LINK_LIBRARIES(
		${CURR_EXE}
		${ACADO_SHARED_LIBRARIES}
	)
ELSE()
	TARGET_LINK_LIBRARIES(
		${CURR_EXE}
		${ACADO_STATIC_LIBRARIES}
	)
ENDIF()
		
SET_TARGET_PROPERTIES(
	${CURR_EXE}
	PROPERTIES
		RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
		# This one is Visual Studio specific
		FOLDER "examples"
)

FILE( MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/crane_cl_mhe_export )
FILE(
	COPY
		${ACADO_QPOASES_EMBEDDED_FOLDER}
	DESTINATION
		${CMAKE_CURRENT_SOURCE_DIR}/crane_cl_mhe_export
)
