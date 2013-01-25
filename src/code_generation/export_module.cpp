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

	setCommonHeaderName( "acado.h" );
}


ExportModule::ExportModule(	const OCP& _ocp
							) : UserInteraction( )
{ 
	setupOptions( );

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


returnValue ExportModule::exportAcadoHeader(	const String& _dirName,
											const String& _fileName,
											const String& _realString,
											const String& _intString,
											int _precision
											) const
{
	int qpSolver;
	get( QP_SOLVER,qpSolver );

	int operatingSystem;
	get( OPERATING_SYSTEM,operatingSystem );

	int useSinglePrecision;
	get( USE_SINGLE_PRECISION,useSinglePrecision );

	int fixInitialState;
	get( FIX_INITIAL_STATE,fixInitialState );


	String fileName( _dirName );
	fileName << "/" << _fileName;
	ExportFile acadoHeader( fileName,"", _realString,_intString,_precision );

	acadoHeader.addStatement( "#include <stdio.h>\n" );
	acadoHeader.addStatement( "#include <math.h>\n" );

	if ( (OperatingSystem)operatingSystem == OS_WINDOWS )
	{
		acadoHeader.addStatement( "#include <windows.h>\n" );
	}
	else
	{
		// OS_UNIX
		acadoHeader.addStatement( "#include <time.h>\n" );
		acadoHeader.addStatement( "#include <sys/stat.h>\n" );
		acadoHeader.addStatement( "#include <sys/time.h>\n" );
	}
	acadoHeader.addLinebreak( );

	acadoHeader.addStatement( "#ifndef ACADO_H\n" );
	acadoHeader.addStatement( "#define ACADO_H\n" );
	acadoHeader.addLinebreak( );

	switch ( (QPSolverName)qpSolver )
	{
		case QP_CVXGEN:
			acadoHeader.addStatement( "#define USE_CVXGEN\n" );
			acadoHeader.addStatement( "#include \"cvxgen/solver.h\"\n" );
			acadoHeader.addLinebreak( 2 );

			if ( (BooleanType)useSinglePrecision == BT_TRUE )
				acadoHeader.addStatement( "typedef float real_t;\n" );
			else
				acadoHeader.addStatement( "typedef double real_t;\n" );
			acadoHeader.addLinebreak( 2 );
			break;

		case QP_QPOASES:
			acadoHeader.addStatement( "#ifndef __MATLAB__\n" );
			acadoHeader.addStatement( "#ifdef __cplusplus\n" );
			acadoHeader.addStatement( "extern \"C\"\n" );
			acadoHeader.addStatement( "{\n" );
			acadoHeader.addStatement( "#endif\n" );
			acadoHeader.addStatement( "#endif\n" );
			acadoHeader.addStatement( "#include \"qpoases/solver.hpp\"\n" );
			acadoHeader.addLinebreak( 2 );
			break;

		case QP_QPOASES3:
			acadoHeader.addStatement( "#include \"qpoases3/solver.h\"\n" );
			acadoHeader.addLinebreak( 2 );
			break;

		case QP_NONE:
			if ( (BooleanType)useSinglePrecision == BT_TRUE )
				acadoHeader.addStatement( "typedef float real_t;\n" );
			else
				acadoHeader.addStatement( "typedef double real_t;\n" );
			acadoHeader.addLinebreak( 2 );
			break;

		default:
			return ACADOERROR( RET_INVALID_OPTION );
	}

	Vector nMeasV = ocp.getNumMeas();
	Vector nOutV = ocp.getDimOutputs();
	if( nMeasV.getDim() != nOutV.getDim() ) return ACADOERROR( RET_INVALID_OPTION );

	//
	// Some common defines
	//
	acadoHeader.addComment( "COMMON DEFINITIONS:             " );
	acadoHeader.addComment( "--------------------------------" );
	acadoHeader.addLinebreak( 2 );
	if( (uint)nOutV.getDim() > 0 ) {
		acadoHeader.addComment( "Dimension of the output functions" );
		acadoHeader.addDeclaration( ExportVariable( "NOUT",nOutV,STATIC_CONST_INT ) );
		acadoHeader.addComment( "Measurements of the output functions per shooting interval" );
		acadoHeader.addDeclaration( ExportVariable( "NMEAS",nMeasV,STATIC_CONST_INT ) );
	}
	acadoHeader.addLinebreak( 2 );

	acadoHeader.addComment( "Number of control intervals" );
	acadoHeader.addStatement( (String)"#define ACADO_N   " << ocp.getN() << "\n");
	acadoHeader.addComment( "Number of differential states" );
	acadoHeader.addStatement( (String)"#define ACADO_NX  " << ocp.getNX() << "\n" );
	acadoHeader.addComment( "Number of differential state derivatives" );
	acadoHeader.addStatement( (String)"#define ACADO_NDX  " << ocp.getNDX() << "\n" );
	acadoHeader.addComment( "Number of algebraic states" );
	acadoHeader.addStatement( (String)"#define ACADO_NXA  " << ocp.getNXA() << "\n" );
	acadoHeader.addComment( "Number of controls" );
	acadoHeader.addStatement( (String)"#define ACADO_NU  " << ocp.getNU() << "\n" );
	acadoHeader.addComment( "Number of parameters" );
	acadoHeader.addStatement( (String)"#define ACADO_NP  " << ocp.getNP() << "\n" );
	acadoHeader.addComment( "Number of output functions" );
	acadoHeader.addStatement( (String)"#define NUM_OUTPUTS  " << (uint)nOutV.getDim() << "\n" );
	acadoHeader.addLinebreak( 2 );

	acadoHeader.addComment( "GLOBAL VARIABLES:               " );
	acadoHeader.addComment( "--------------------------------" );
	acadoHeader.addStatement( "typedef struct ACADOvariables_ {\n" );

	if ( collectDataDeclarations( acadoHeader,ACADO_VARIABLES ) != SUCCESSFUL_RETURN )
		return ACADOERROR( RET_UNABLE_TO_EXPORT_CODE );

	acadoHeader.addLinebreak( );
	acadoHeader.addStatement( "} ACADOvariables;\n" );
	acadoHeader.addLinebreak( 2 );

	acadoHeader.addComment( "GLOBAL WORKSPACE:               " );
	acadoHeader.addComment( "--------------------------------" );
	acadoHeader.addStatement( "typedef struct ACADOworkspace_ {\n" );

	if ( collectDataDeclarations( acadoHeader,ACADO_WORKSPACE ) != SUCCESSFUL_RETURN )
		return ACADOERROR( RET_UNABLE_TO_EXPORT_CODE );

	acadoHeader.addLinebreak( );
	acadoHeader.addStatement( "} ACADOworkspace;\n" );
	acadoHeader.addLinebreak( 2 );

	acadoHeader.addComment( "GLOBAL FORWARD DECLARATIONS:         " );
	acadoHeader.addComment( "-------------------------------------" );

	if ( collectFunctionDeclarations( acadoHeader ) != SUCCESSFUL_RETURN )
		return ACADOERROR( RET_UNABLE_TO_EXPORT_CODE );

	acadoHeader.addComment( "-------------------------------------" );
	acadoHeader.addLinebreak( 2 );

	acadoHeader.addComment( "EXTERN DECLARATIONS:                 " );
	acadoHeader.addComment( "-------------------------------------" );
	acadoHeader.addStatement( "extern ACADOworkspace acadoWorkspace;\n" );
	acadoHeader.addStatement( "extern ACADOvariables acadoVariables;\n" );
	acadoHeader.addComment( "-------------------------------------" );

	switch ( (QPSolverName) qpSolver )
	{
		case QP_CVXGEN:
			break;

		case QP_QPOASES:
			acadoHeader.addStatement( "#ifndef __MATLAB__\n");
			acadoHeader.addStatement( "#ifdef __cplusplus\n" );
			acadoHeader.addLinebreak( );
			acadoHeader.addStatement( "} /* extern \"C\" */\n" );
			acadoHeader.addStatement( "#endif\n" );
			acadoHeader.addStatement( "#endif\n" );
			break;

		case QP_QPOASES3:
			break;

		case QP_NONE:
			break;

		default:
			return ACADOERROR( RET_INVALID_OPTION );
	}

	acadoHeader.addStatement( "#endif\n" );
	acadoHeader.addLinebreak( );
    acadoHeader.addComment( "END OF FILE." );
	acadoHeader.addLinebreak( );

	return acadoHeader.exportCode( );
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
	addOption( MEASUREMENT_GRID, 			EQUIDISTANT_GRID);
	addOption( INTEGRATOR_DEBUG_MODE, 		0				);
	addOption( IMPLICIT_INTEGRATOR_MODE,	IFTR 			);
	addOption( IMPLICIT_INTEGRATOR_NUM_ITS,	3				);
	addOption( IMPLICIT_INTEGRATOR_NUM_ITS_INIT, 0			);
	addOption( SPARSE_QP_SOLUTION,          FULL_CONDENSING );
	addOption( FIX_INITIAL_STATE,           BT_TRUE         );
	addOption( QP_SOLVER,                   QP_QPOASES      );
	addOption( MAX_NUM_QP_ITERATIONS,       -1              );
	addOption( HOTSTART_QP,                 BT_FALSE        );
	addOption( LEVENBERG_MARQUARDT,         0.0             );
	addOption( GENERATE_TEST_FILE,          BT_TRUE         );
	addOption( GENERATE_MAKE_FILE,          BT_TRUE         );
	addOption( GENERATE_SIMULINK_INTERFACE, BT_FALSE        );
	addOption( GENERATE_MATLAB_INTERFACE, 	BT_FALSE        );
	addOption( MEX_ITERATION_STEPS, 		1        		);
	addOption( MEX_VERBOSE, 				0       		);
	addOption( OPERATING_SYSTEM,            OS_DEFAULT      );
	addOption( USE_SINGLE_PRECISION,        BT_FALSE        );
	addOption( PRINTLEVEL,                  MEDIUM          );

	addOption( CG_USE_C99,                       NO         );
	addOption( CG_USE_VARIABLE_WEIGHTING_MATRIX, NO         );
	addOption( CG_COMPUTE_COVARIANCE_MATRIX,     NO         );
	addOption( CG_USE_OPENMP,					 NO         );

	return SUCCESSFUL_RETURN;
}



CLOSE_NAMESPACE_ACADO

// end of file.
