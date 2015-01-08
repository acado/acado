################################################################################
#
# Description:
#	Some useful macros used in ACADO
#
# Authors:
#	Milan Vukov, milan.vukov@esat.kuleuven.be
#
# Year:
#	2013 - 2014
#
# NOTE:
#
# Usage:
#	- TBD
#
################################################################################

MACRO( ACADO_GENERATE_COMPILE generator exportFolder testFile )
	# generator:    target used for code generation
	# exportFolder: export folder name. At the moment, only relative folder
	#               names are supported
	# testFile:     C/C++ source name of a test file
	
	# NOTE: works only with qpOASES based OCP solvers
	
	IF (NOT ("${CMAKE_VERSION}" VERSION_LESS "2.8.10"))
	
#		MESSAGE( STATUS "Generating and compiling the code." )              

		SET( ${generator}_GENERATED_FILES
			${CMAKE_CURRENT_SOURCE_DIR}/${exportFolder}/acado_common.h
			${CMAKE_CURRENT_SOURCE_DIR}/${exportFolder}/acado_solver.c
			${CMAKE_CURRENT_SOURCE_DIR}/${exportFolder}/acado_integrator.c
			${CMAKE_CURRENT_SOURCE_DIR}/${exportFolder}/acado_qpoases_interface.hpp
			${CMAKE_CURRENT_SOURCE_DIR}/${exportFolder}/acado_qpoases_interface.cpp
			${CMAKE_CURRENT_SOURCE_DIR}/${exportFolder}/acado_auxiliary_functions.h
			${CMAKE_CURRENT_SOURCE_DIR}/${exportFolder}/acado_auxiliary_functions.c
		)
		
		# Get the full name of the executable getting_started
		GET_TARGET_PROPERTY(
			${generator}_EXE
				${generator} LOCATION
		)
		
		# Create a command that will generate the code for us
		ADD_CUSTOM_COMMAND(
			OUTPUT
				${${generator}_GENERATED_FILES}       
			COMMAND
				${${generator}_EXE}
			WORKING_DIRECTORY
				${CMAKE_CURRENT_SOURCE_DIR}
			DEPENDS
				${generator}
		)
		
		# Build a test application
		GET_FILENAME_COMPONENT( EXEC_NAME ${testFile} NAME_WE )
		ADD_EXECUTABLE(
			# Name of the executable
			${EXEC_NAME}
			
			# A test application source
			${testFile}
			
			# Auto-generated sources
			${${generator}_GENERATED_FILES}
			     
			# qpOASES embedded sources
			${ACADO_QPOASES_EMBEDDED_SOURCES}
		)

		IF( ${CMAKE_SYSTEM_NAME} MATCHES "Linux" )
			TARGET_LINK_LIBRARIES(
				${EXEC_NAME}
				rt
			)
		ENDIF( ${CMAKE_SYSTEM_NAME} MATCHES "Linux" )
		
		SET_TARGET_PROPERTIES(
			${EXEC_NAME}
			PROPERTIES
				RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
		)
		
		SET_PROPERTY(
			TARGET
				${EXEC_NAME}
			PROPERTY
				INCLUDE_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR}/${exportFolder} ${CMAKE_CURRENT_SOURCE_DIR}/${exportFolder}/qpoases ${ACADO_QPOASES_EMBEDDED_INC_DIRS}
		)
		
		IF ( ACADO_WITH_TESTING )
			ADD_TEST(
				NAME
					${EXEC_NAME}_test
				WORKING_DIRECTORY
					"${CMAKE_CURRENT_SOURCE_DIR}"
				COMMAND
					${EXEC_NAME}
			)
		ENDIF()
		
	ELSE(NOT ("${CMAKE_VERSION}" VERSION_LESS "2.8.10"))
		MESSAGE( WARNING "Your CMake is old, thus we cannot generate and compile the code with a macro -- i.e. you have to do it by hand.")
	
	ENDIF (NOT ("${CMAKE_VERSION}" VERSION_LESS "2.8.10"))
	
ENDMACRO()

MACRO( ACADO_APPLICATION name )
	# name: Name of the application (target)
	# NOTE: All other arguments are considered as source files

	ADD_EXECUTABLE( ${name} ${ARGN} )
	IF ( ACADO_BUILD_SHARED )
		TARGET_LINK_LIBRARIES(
			${name}
			${ACADO_SHARED_LIBRARIES}
		)
	ELSE()
		TARGET_LINK_LIBRARIES(
			${name}
			${ACADO_STATIC_LIBRARIES}
		)
	ENDIF()
	
	SET_TARGET_PROPERTIES(
		${name}
		PROPERTIES
			RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
			# This one is Visual Studio specific
			FOLDER "examples"
	)
	
	IF ( ACADO_WITH_TESTING )
		ADD_TEST(
			NAME
				${name}_test
			WORKING_DIRECTORY
				"${CMAKE_CURRENT_SOURCE_DIR}"
			COMMAND
				${name}
		)
	ENDIF()
ENDMACRO()

MACRO( ACADO_COPY_QPOASES_EMBEDDED folder )
	# folder: A folder (relative path) where to copy the qpOASES embedded source

	FILE( MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${folder} )
	FILE(
		COPY
			${ACADO_QPOASES_EMBEDDED_FOLDER}
		DESTINATION
			${CMAKE_CURRENT_SOURCE_DIR}/${folder}
	)
ENDMACRO()
