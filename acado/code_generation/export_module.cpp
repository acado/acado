/*
 *    This file is part of ACADO Toolkit.
 *
 *    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
 *    Copyright (C) 2008-2014 by Boris Houska, Hans Joachim Ferreau,
 *    Milan Vukov, Rien Quirynen, KU Leuven.
 *    Developed within the Optimization in Engineering Center (OPTEC)
 *    under supervision of Moritz Diehl. All rights reserved.
 *
 *    ACADO Toolkit is free software; you can redistribute it and/or
 *    modify it under the terms of the GNU Lesser General Public
 *    License as published by the Free Software Foundation; either
 *    version 3 of the License, or (at your option) any later version.
 *
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *    Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public
 *    License along with ACADO Toolkit; if not, write to the Free Software
 *    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */



/**
 *    \file src/code_generation/mpc_module.cpp
 *    \author Hans Joachim Ferreau, Boris Houska, Milan Vukov
 *    \date 2010 - 2013
 */

#include <acado/code_generation/export_module.hpp>
#include <acado/code_generation/integrators/integrator_export.hpp>

BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

ExportModule::ExportModule( ) : UserInteraction( )
{ 
	setupOptions( );

	commonHeaderName = "acado.h";
}


ExportModule::~ExportModule( )
{}

//
// PROTECTED MEMBER FUNCTIONS:
//

returnValue ExportModule::setupOptions( )
{
	addOption( HESSIAN_APPROXIMATION,       GAUSS_NEWTON    );
	addOption( HESSIAN_REGULARIZATION, 		BLOCK_REG 		);
	addOption( CG_CONDENSED_HESSIAN_CHOLESKY, EXTERNAL		);
	addOption( DISCRETIZATION_TYPE,         MULTIPLE_SHOOTING );
	addOption( INTEGRATOR_TYPE,             INT_RK4         );
	addOption( DYNAMIC_SENSITIVITY,         FORWARD         );
	addOption( LINEAR_ALGEBRA_SOLVER,       GAUSS_LU        );
	addOption( UNROLL_LINEAR_SOLVER,       	false	    	);
	addOption( NUM_INTEGRATOR_STEPS,        30              );
	addOption( MEASUREMENT_GRID, 			OFFLINE_GRID	);
	addOption( INTEGRATOR_DEBUG_MODE, 		0				);
	addOption( IMPLICIT_INTEGRATOR_MODE,	IFTR 			);
//	addOption( LIFTED_INTEGRATOR_MODE,		1 				);
	addOption( LIFTED_GRADIENT_UPDATE, 		false			);
	addOption( IMPLICIT_INTEGRATOR_NUM_ITS,	5				);
	addOption( IMPLICIT_INTEGRATOR_NUM_ITS_INIT, 0			);
	addOption( SPARSE_QP_SOLUTION,          FULL_CONDENSING );
	addOption( CONDENSING_BLOCK_SIZE,       0 				);
	addOption( FIX_INITIAL_STATE,           true         	);
	addOption( QP_SOLVER,                   QP_QPOASES      );
	addOption( MAX_NUM_QP_ITERATIONS,       -1              );
	addOption( HOTSTART_QP,                 false        	);
	addOption( LEVENBERG_MARQUARDT,         0.0             );
	addOption( GENERATE_TEST_FILE,          true         	);
	addOption( GENERATE_MAKE_FILE,          true         	);
	addOption( GENERATE_SIMULINK_INTERFACE, false        	);
	addOption( GENERATE_MATLAB_INTERFACE, 	false        	);
	addOption( MEX_ITERATION_STEPS, 		1        		);
	addOption( MEX_VERBOSE, 				0       		);
	addOption( USE_SINGLE_PRECISION,        false        	);
	addOption( PRINTLEVEL,                  MEDIUM          );

	addOption( CG_USE_C99,                       NO         );
	addOption( CG_USE_VARIABLE_WEIGHTING_MATRIX, NO         );
	addOption( CG_COMPUTE_COVARIANCE_MATRIX,     NO         );
	addOption( CG_USE_OPENMP,					 NO         );
	addOption( CG_HARDCODE_CONSTRAINT_VALUES,    YES        );
	addOption( CG_USE_ARRIVAL_COST,              NO         );

	addOption( CG_CONDENSED_HESSIAN_CHOLESKY,    EXTERNAL   );
	addOption( CG_FORCE_DIAGONAL_HESSIAN,        NO         );

	addOption( CG_MODULE_NAME, "acado"						);
    addOption( CG_MODULE_PREFIX, "ACADO"                	);
	addOption( CG_EXPORT_FOLDER_NAME, "acado_export"		);

	return SUCCESSFUL_RETURN;
}

CLOSE_NAMESPACE_ACADO
