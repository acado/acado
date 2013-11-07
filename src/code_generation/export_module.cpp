/*
 *    This file is part of ACADO Toolkit.
 *
 *    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
 *    Copyright (C) 2008-2013 by Boris Houska, Hans Joachim Ferreau,
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


BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

ExportModule::ExportModule( ) : UserInteraction( )
{ 
	setupOptions( );

	commonHeaderName = "acado.h";
	name = "acado";
	exportFolderName = "acado_exported_code";
}


ExportModule::~ExportModule( )
{}

std::string ExportModule::getCommonHeaderName( ) const
{
	return commonHeaderName;
}


//
// PROTECTED MEMBER FUNCTIONS:
//

returnValue ExportModule::setupOptions( )
{
	addOption( HESSIAN_APPROXIMATION,       GAUSS_NEWTON    );
	addOption( DISCRETIZATION_TYPE,         SINGLE_SHOOTING );
	addOption( INTEGRATOR_TYPE,             INT_RK4         );
	addOption( DYNAMIC_SENSITIVITY,         FORWARD         );
	addOption( LINEAR_ALGEBRA_SOLVER,       GAUSS_LU        );
	addOption( UNROLL_LINEAR_SOLVER,       	false	    );
	addOption( NUM_INTEGRATOR_STEPS,        30              );
	addOption( MEASUREMENT_GRID, 			OFFLINE_GRID);
	addOption( INTEGRATOR_DEBUG_MODE, 		0				);
	addOption( IMPLICIT_INTEGRATOR_MODE,	IFTR 			);
	addOption( IMPLICIT_INTEGRATOR_NUM_ITS,	5				);
	addOption( IMPLICIT_INTEGRATOR_NUM_ITS_INIT, 0			);
	addOption( SPARSE_QP_SOLUTION,          FULL_CONDENSING );
	addOption( FIX_INITIAL_STATE,           true         );
	addOption( QP_SOLVER,                   QP_QPOASES      );
	addOption( MAX_NUM_QP_ITERATIONS,       -1              );
	addOption( HOTSTART_QP,                 false        );
	addOption( LEVENBERG_MARQUARDT,         0.0             );
	addOption( GENERATE_TEST_FILE,          true         );
	addOption( GENERATE_MAKE_FILE,          true         );
	addOption( GENERATE_SIMULINK_INTERFACE, false        );
	addOption( GENERATE_MATLAB_INTERFACE, 	false        );
	addOption( MEX_ITERATION_STEPS, 		1        		);
	addOption( MEX_VERBOSE, 				0       		);
	addOption( OPERATING_SYSTEM,            OS_DEFAULT      );
	addOption( USE_SINGLE_PRECISION,        false        );
	addOption( PRINTLEVEL,                  MEDIUM          );

	addOption( CG_USE_C99,                       NO         );
	addOption( CG_USE_VARIABLE_WEIGHTING_MATRIX, NO         );
	addOption( CG_COMPUTE_COVARIANCE_MATRIX,     NO         );
	addOption( CG_USE_OPENMP,					 NO         );
	addOption( CG_HARDCODE_CONSTRAINT_VALUES,    YES        );
	addOption( CG_USE_ARRIVAL_COST,              NO         );

	return SUCCESSFUL_RETURN;
}

returnValue ExportModule::setName(const std::string& _name)
{
	if ( _name.empty() == true )
		return ACADOERROR( RET_INVALID_ARGUMENTS );

	name = _name;

	return SUCCESSFUL_RETURN;
}

std::string ExportModule::getName() const
{
	return name;
}

const std::string& ExportModule::getExportFolderName() const
{
	return exportFolderName;
}

void ExportModule::setExportFolderName(const std::string& _name)
{
	exportFolderName = _name;
}

CLOSE_NAMESPACE_ACADO
