/*
 *    This file is part of ACADO Toolkit.
 *
 *    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
 *    Copyright (C) 2008-2009 by Boris Houska and Hans Joachim Ferreau, K.U.Leuven.
 *    Developed within the Optimization in Engineering Center (OPTEC) under
 *    supervision of Moritz Diehl. All rights reserved.
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
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 2010-2011
 */

#include <acado/code_generation/export_module.hpp>


BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

ExportModule::ExportModule( ) : UserInteraction( )
{ 
	setupOptions( );

	NX = 0; 
	NXA = 0; 
	NU = 0; 
	NP = 0; 
	N  = 0;

	setCommonHeaderName( "acado.h" );
}


ExportModule::ExportModule(	const OCP& _ocp
							) : UserInteraction( )
{ 
	setupOptions( );

	NX = 0; 
	NXA = 0; 
	NU = 0; 
	NP = 0; 
	N  = 0;

	setCommonHeaderName( "acado.h" );

	returnValue returnvalue = setOCP( _ocp );
	ASSERT( returnvalue == SUCCESSFUL_RETURN );
}


ExportModule::ExportModule(	const ExportModule& arg
							) : UserInteraction( arg )
{
	copy( arg );
}


ExportModule::~ExportModule( )
{
}


ExportModule& ExportModule::operator=(	const ExportModule& arg
										)
{
	if( this != &arg )
	{
		UserInteraction::operator=( arg );
		copy( arg );
	}

	return *this;
}



returnValue ExportModule::setOCP(	const OCP& _ocp	
									)
{
	ocp = _ocp;
	return SUCCESSFUL_RETURN;
}



uint ExportModule::getNX( ) const
{
	return NX;
}


uint ExportModule::getNXA( ) const
{
	return NXA;
}


uint ExportModule::getNU( ) const
{
	return NU;
}


uint ExportModule::getNP( ) const
{
	return NP;
}


uint ExportModule::getN( ) const
{
	return N;
}



returnValue	ExportModule::setCommonHeaderName(	const String& _name
												)
{
	if ( _name.isEmpty() == BT_TRUE )
		return ACADOERROR( RET_INVALID_ARGUMENTS );
	
	commonHeaderName = _name;
	return SUCCESSFUL_RETURN;
}


String ExportModule::getCommonHeaderName( ) const
{
	return commonHeaderName;
}



//
// PROTECTED MEMBER FUNCTIONS:
//


returnValue ExportModule::copy(	const ExportModule& arg
								)
{
	ocp = arg.ocp;

	NX = arg.NX;
	NXA = arg.NXA; 
	NU = arg.NU;
	NP = arg.NP;
	N  = arg.N;
	
	commonHeaderName = arg.commonHeaderName;

	return SUCCESSFUL_RETURN;
}


returnValue ExportModule::setupOptions( )
{
	addOption( HESSIAN_APPROXIMATION,       GAUSS_NEWTON    );
	addOption( DISCRETIZATION_TYPE,         SINGLE_SHOOTING );
	addOption( INTEGRATOR_TYPE,             INT_RK4         );
	addOption( LINEAR_ALGEBRA_SOLVER,       GAUSS_LU        );
	addOption( UNROLL_LINEAR_SOLVER,       	BT_FALSE	    );
	addOption( NUM_INTEGRATOR_STEPS,        30              );
	addOption( IMPLICIT_INTEGRATOR_MODE,	IFTR 			);
	addOption( IMPLICIT_INTEGRATOR_NUM_ITS,	3				);
	addOption( IMPLICIT_INTEGRATOR_NUM_ITS_INIT, 0			);
	addOption( IMPLICIT_INTEGRATOR_NUM_ALG_ITS,	1			);
	addOption( IMPLICIT_INTEGRATOR_NUM_ALG_ITS_INIT, 2		);
	addOption( SPARSE_QP_SOLUTION,          FULL_CONDENSING );
	addOption( FIX_INITIAL_STATE,           BT_TRUE         );
	addOption( QP_SOLVER,                   QP_QPOASES      );
	addOption( MAX_NUM_QP_ITERATIONS,       -1              );
	addOption( HOTSTART_QP,                 BT_FALSE        );
	addOption( LEVENBERG_MARQUARDT,         0.0             );
	addOption( GENERATE_TEST_FILE,          BT_TRUE         );
	addOption( GENERATE_MAKE_FILE,          BT_TRUE         );
	addOption( GENERATE_SIMULINK_INTERFACE, BT_FALSE        );
	addOption( OPERATING_SYSTEM,            OS_DEFAULT      );
	addOption( USE_SINGLE_PRECISION,        BT_FALSE        );
	addOption( PRINTLEVEL,                  MEDIUM          );

	addOption( CG_USE_C99,                       NO         );
	addOption( CG_USE_VARIABLE_WEIGHTING_MATRIX, NO         );
	addOption( CG_COMPUTE_COVARIANCE_MATRIX,     NO         );


	return SUCCESSFUL_RETURN;
}



CLOSE_NAMESPACE_ACADO

// end of file.
