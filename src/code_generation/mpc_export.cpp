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
 *    \file src/code_generation/mpc_export.cpp
 *    \author Hans Joachim Ferreau, Boris Houska, Milan Vukov
 *    \date 2010-2012
 */

#include <acado/code_generation/mpc_export.hpp>

#include <acado/code_generation/export_algorithm_factory.hpp>

#ifdef __MATLAB__
#include "mex.h"
#endif

BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

MPCexport::MPCexport( ) : ExportModule( )
{
	integrator  = 0;
	condenser   = 0;
	gaussNewton = 0;
	auxFcns     = 0;

	setStatus( BS_NOT_INITIALIZED );
}


MPCexport::MPCexport(	const OCP& _ocp
						) : ExportModule( _ocp )
{
	integrator  = 0;
	condenser   = 0;
	gaussNewton = 0;
	auxFcns     = 0;

	setStatus( BS_NOT_INITIALIZED );
}


MPCexport::MPCexport(	const MPCexport& arg
						) : ExportModule( arg )
{
	copy( arg );
}


MPCexport::~MPCexport( )
{
	clear( );
}


MPCexport& MPCexport::operator=(	const MPCexport& arg
									)
{
	if( this != &arg )
	{
		clear( );
		ExportModule::operator=( arg );
		copy( arg );
	}

	return *this;
}



returnValue MPCexport::exportCode(	const String& dirName,
									const String& _realString,
									const String& _intString,
									int _precision
									)
{
	if (!MODEL_DIMENSIONS_SET) return ACADOERROR( RET_UNABLE_TO_EXPORT_CODE );
	if ( setup( ) != SUCCESSFUL_RETURN )
		return ACADOERROR( RET_UNABLE_TO_EXPORT_CODE );

	int printLevel;
	get( PRINTLEVEL,printLevel );

	// export mandatory source code files
	if ( exportAcadoHeader( dirName,commonHeaderName,_realString,_intString,_precision ) != SUCCESSFUL_RETURN )
		return ACADOERROR( RET_UNABLE_TO_EXPORT_CODE );

	if( integrator != 0 )
	{
		String fileName( dirName );
		fileName << "/integrator.c";

		ExportFile integratorFile( fileName,commonHeaderName,_realString,_intString,_precision );
		integrator->getCode( integratorFile );
		
		if ( integratorFile.exportCode( ) != SUCCESSFUL_RETURN )
			return ACADOERROR( RET_UNABLE_TO_EXPORT_CODE );

		int generateMatlabInterface;
		get( GENERATE_MATLAB_INTERFACE, generateMatlabInterface );
		if ( (BooleanType)generateMatlabInterface == BT_TRUE ) {
			// TODO
		}
	}

	if( condenser != 0 )
	{
		String fileName( dirName );
		fileName << "/condensing.c";

		ExportFile condenserFile( fileName,commonHeaderName,_realString,_intString,_precision );
		condenser->getCode( condenserFile );
		
		if ( condenserFile.exportCode( ) != SUCCESSFUL_RETURN )
			return ACADOERROR( RET_UNABLE_TO_EXPORT_CODE );
	}

	if( gaussNewton != 0 )
	{
		String fileName( dirName );
		fileName << "/gauss_newton_method.c";

		ExportFile gaussNewtonFile( fileName,commonHeaderName,_realString,_intString,_precision );
		gaussNewton->getCode( gaussNewtonFile );
		
		if ( gaussNewtonFile.exportCode( ) != SUCCESSFUL_RETURN )
			return ACADOERROR( RET_UNABLE_TO_EXPORT_CODE );
	}

	if( auxFcns != 0 )
	{
		String fileName( dirName );
		fileName << "/auxiliary_functions.c";

		ExportFile auxFcnsFile( fileName,"",_realString,_intString,_precision );
		auxFcns->getCode( auxFcnsFile );
		
		if ( auxFcnsFile.exportCode( ) != SUCCESSFUL_RETURN )
			return ACADOERROR( RET_UNABLE_TO_EXPORT_CODE );
	}

	if ( exportQPsolverInterface( dirName ) != SUCCESSFUL_RETURN )
		return ACADOERROR( RET_UNABLE_TO_EXPORT_CODE );


	// export template for main file, if desired
	if ( (PrintLevel)printLevel >= HIGH ) 
		acadoPrintf( "--> Exporting remaining files... " );
	
	int generateTestFile;
	get( GENERATE_TEST_FILE,generateTestFile );
	if ( (BooleanType)generateTestFile == BT_TRUE )
		if ( exportTemplateMain( dirName,"test.c",_realString,_intString,_precision ) != SUCCESSFUL_RETURN )
			return ACADOERROR( RET_UNABLE_TO_EXPORT_CODE );

	// export a basic Makefile, if desired
	int generateMakeFile;
	get( GENERATE_MAKE_FILE,generateMakeFile );
	if ( (BooleanType)generateMakeFile == BT_TRUE )
		if ( exportMakefile( dirName,"Makefile",_realString,_intString,_precision ) != SUCCESSFUL_RETURN )
			return ACADOERROR( RET_UNABLE_TO_EXPORT_CODE );

	// export a simulink s function interface, if desired
	int generateSimulinkInterface;
	get( GENERATE_SIMULINK_INTERFACE,generateSimulinkInterface );
	if ( (BooleanType)generateSimulinkInterface == BT_TRUE )
		if ( exportSimulinkInterface( dirName,"sfunction.cpp","make_sfunction.m",_realString,_intString,_precision ) != SUCCESSFUL_RETURN )
			return ACADOERROR( RET_UNABLE_TO_EXPORT_CODE );

	// export a Matlab mex function interface, if desired
	int generateMatlabInterface;
	get( GENERATE_MATLAB_INTERFACE, generateMatlabInterface );
	if ( (BooleanType)generateMatlabInterface == BT_TRUE ) {
		String mpcInterface( dirName );
		mpcInterface << "/MPCstep.c";
		ExportMatlabMPC exportMexFun( MPC_MEX_TEMPLATE, mpcInterface, commonHeaderName,_realString,_intString,_precision );

		int mexSteps, verbose;
		get( MEX_ITERATION_STEPS, mexSteps );
		get( MEX_VERBOSE, verbose );
		exportMexFun.configure(mexSteps, verbose);
		exportMexFun.exportCode();
	}

	if ( (PrintLevel)printLevel >= HIGH ) 
		acadoPrintf( "done.\n" );

	if ( (PrintLevel)printLevel > NONE )
		ACADOINFO( RET_CODE_EXPORT_SUCCESSFUL );

    return SUCCESSFUL_RETURN;
}



returnValue MPCexport::printDimensionsQP( )
{
	if ( getStatus() != BS_READY )
		if ( setup( ) != SUCCESSFUL_RETURN )
			return ACADOERROR( RET_UNABLE_TO_EXPORT_CODE );

	acadoPrintf( "\n***********************  ACADO CODE GENERATION  ***********************\n\n" );
	acadoPrintf( " The condensed QP comprises " );
	acadoPrintf( "%d variables and ", gaussNewton->getNumQPvars() );
	acadoPrintf( "%d constraints.\n", gaussNewton->getNumStateBounds() );
	acadoPrintf( "\n***********************************************************************\n\n" );

	return SUCCESSFUL_RETURN;
}



//
// PROTECTED MEMBER FUNCTIONS:
//

returnValue MPCexport::copy(	const MPCexport& arg
								)
{
	// TODO: why like this? Why the copy is made this way?
	if ( arg.integrator != 0 )
		integrator = arg.integrator;
	else
		integrator = 0;
	
	if ( arg.condenser != 0 )
		condenser = new CondensingExport( *(arg.condenser) );
	else
		condenser = 0;
	
	if ( arg.gaussNewton != 0 )
		gaussNewton = new GaussNewtonExport( *(arg.gaussNewton) );
	else
		gaussNewton = 0;

	if ( arg.auxFcns != 0 )
		auxFcns = new AuxiliaryFunctionsExport( *(arg.auxFcns) );
	else
		auxFcns = 0;

	return SUCCESSFUL_RETURN;
}


returnValue MPCexport::clear( )
{
	if ( integrator != 0 )
		delete integrator;

	if ( condenser != 0 )
		delete condenser;

	if ( gaussNewton != 0 )
		delete gaussNewton;

	if ( auxFcns != 0 )
		delete auxFcns;
	return SUCCESSFUL_RETURN;
}



returnValue MPCexport::setup( )
{
	// nothing to do as object is up-to-date
	if ( getStatus() == BS_READY )
		return SUCCESSFUL_RETURN;

	returnValue returnvalue = checkConsistency( );
	if ( returnvalue != SUCCESSFUL_RETURN )
		return ACADOERROR( returnvalue );

	Grid grid;
	ocp.getGrid( grid );

	N = grid.getNumIntervals();

	int numSteps;
	get( NUM_INTEGRATOR_STEPS, numSteps );

	int integratorType;
	get( INTEGRATOR_TYPE, integratorType );

	if ( integrator != NULL )
		delete integrator;

	integrator = IntegratorExportFactory::instance().createAlgorithm(this, commonHeaderName, static_cast<IntegratorType>(integratorType));

	if ( integrator == NULL )
		return ACADOERROR( RET_INVALID_OPTION );

	integrator->setDimensions( NX,NDX,NXA,NU,NP,N );

	if( EXPORT_RHS ) {
		Expression rhs;
		f.getExpression( rhs );
		if ( integrator->setDifferentialEquation( rhs ) != SUCCESSFUL_RETURN )
			return RET_UNABLE_TO_EXPORT_CODE;
	}
	else {
		if ( integrator->setModel( rhs_ODE, diffs_ODE ) != SUCCESSFUL_RETURN )
			return RET_UNABLE_TO_EXPORT_CODE;
	}

	if( !integrationGrid.isEmpty() ) {
		if ( integrator->setGrid(integrationGrid) != SUCCESSFUL_RETURN ) {
			return RET_UNABLE_TO_EXPORT_CODE;
		}
	}
	else {
		if ( integrator->setGrid(grid, numSteps) != SUCCESSFUL_RETURN ) {
			return RET_UNABLE_TO_EXPORT_CODE;
		}
	}

	integrator->setup( );

	if( ocp.hasOutputFunctions() ) {
		std::vector<OutputFcn> outputFcns;
		ocp.getOutputFunctions( outputFcns );

		std::vector<Expression> rhsOutput;
		uint i;
		for( i = 0; i < outputFcns.size(); i++ ) {
			Expression next;
			outputFcns[i].getExpression( next );
			rhsOutput.push_back( next );
		}

		std::vector<Grid> outputGrids_;
		ocp.getOutputGrids( outputGrids_ );

		int steps;
		if( ocp.hasEquidistantGrid() ) {
			steps = numSteps;
		}
		else {
			Vector _numSteps;
			ocp.getNumSteps( _numSteps );
			steps = 0;
			for( i = 0; i < _numSteps.getDim(); i++ ) {
				steps += (int)_numSteps(i);
			}
		}

		std::vector<Grid> newGrids_;
		for( i = 0; i < outputGrids_.size(); i++ ) {
			Grid nextGrid( 0.0, 1.0, (int) ceil((double)outputGrids_[i].getNumIntervals()/((double) steps) - 10.0*EPS) + 1 );
			newGrids_.push_back( nextGrid );
		}

		integrator->setupOutput( newGrids_, rhsOutput );
	}


	// extract control/state bounds (no free parameters yet!)
	// TODO: extension to DAE systems
	VariablesGrid ugrid( NU, grid );
	VariablesGrid xgrid( NX, grid );

	OCPiterate tmp;
	tmp.init( &xgrid,0,0,&ugrid,0 );

	Constraint constraint;
	ocp.getConstraint( constraint );
	constraint.getBounds( tmp );


	// setup condensing
	double levenbergMarquardt;
	get( LEVENBERG_MARQUARDT,levenbergMarquardt );

	if ( condenser != 0 )
		delete condenser;

	condenser = new CondensingExport( this,commonHeaderName );
	condenser->setDimensions( NX,NU,NP,N );

	condenser->setIntegratorExport( integrator );

	ExportVariable Q, R, QF, QS, QS2;

	ocp.getQRmatrices( Q,R,QF,QS,QS2 );

	// check for positive semi-definiteness of Q, QF and R
	if ( Q.isGiven() == BT_TRUE )
		if ( Q.getGivenMatrix().isPositiveSemiDefinite() == BT_FALSE )
			return ACADOERROR( RET_NONPOSITIVE_WEIGHT );

	if ( R.isGiven() == BT_TRUE )
	{
		if ( R.getGivenMatrix().isPositiveSemiDefinite() == BT_FALSE )
			return ACADOERROR( RET_NONPOSITIVE_WEIGHT );

		// automatically regularise in case R==0
		if ( ( R.getGivenMatrix().isZero( ) == BT_TRUE ) && ( acadoIsZero( levenbergMarquardt ) == BT_TRUE ) )
		{
			levenbergMarquardt = 1.0e-5;
			set( LEVENBERG_MARQUARDT,levenbergMarquardt );
		}
	}

	if ( QF.isGiven() == BT_TRUE )
		if ( QF.getGivenMatrix().isPositiveSemiDefinite() == BT_FALSE )
			return ACADOERROR( RET_NONPOSITIVE_WEIGHT );

	if( Q.getDim() == 0 )
		Q = zeros( NX,NX );

	if( R.getDim() == 0 )
		R = zeros( NU,NU );

	if( QF.getDim() == 0 )
		QF = zeros( NX,NX );

	if( QS.getDim() == 0 )
		QS = zeros( NX,NX );

	if( QS2.getDim() == 0 )
		QS2 = zeros( NX,NX );

	if ( ( Q.getNumRows( ) != QF.getNumRows( ) ) || ( Q.getNumCols( ) != QF.getNumCols( ) ) )
		return ACADOERROR( RET_VECTOR_DIMENSION_MISMATCH );

	condenser->setWeightingMatrices( Q,R,QF,QS,QS2 );
	condenser->setStateBounds( *(tmp.x) );
	condenser->setLevenbergMarquardt( levenbergMarquardt );
	condenser->setup( );


	// setup Gauss-Newton algorithm
	if ( gaussNewton != 0 )
		delete gaussNewton;

	gaussNewton = new GaussNewtonExport( this,commonHeaderName );
	gaussNewton->setDimensions( NX,NDX,NXA,NU,NP,N );

	gaussNewton->setCondensingExport( condenser );
	gaussNewton->setControlBounds( *(tmp.u) );
	gaussNewton->setup( );


	// setup auxiliary functions
	if ( auxFcns != 0 )
		delete auxFcns;

	auxFcns = new AuxiliaryFunctionsExport( this );
	auxFcns->setDimensions( NX,NDX,NXA,NU,NP,N );

	auxFcns->setup( );

	setStatus( BS_READY );

	return SUCCESSFUL_RETURN;
}


returnValue MPCexport::checkConsistency( ) const
{
	// consistency checks:
	// only standard LSQ objective supported!
	if ( ocp.hasObjective( ) == BT_TRUE )
		return ACADOERROR( RET_INVALID_OBJECTIVE_FOR_CODE_EXPORT );
	
	// only time-continuous ODEs without parameter and disturbances supported!
	if ( f.isODE( ) == BT_FALSE )
		return ACADOERROR( RET_ONLY_ODE_FOR_CODE_EXPORT );

	if ( f.isDiscretized( ) == BT_TRUE )
		return ACADOERROR( RET_NO_DISCRETE_ODE_FOR_CODE_EXPORT );
	
	if ( ( f.getNUI( ) > 0 ) || 
		 /*( f.getNP( ) > 0 ) ||*/ ( f.getNPI( ) > 0 ) || ( f.getNW( ) > 0 ) )
		return ACADOERROR( RET_ONLY_STATES_AND_CONTROLS_FOR_CODE_EXPORT );

	// only equidistant evaluation grids supported!
	Grid grid;
	ocp.getGrid( grid );
	
	//if ( grid.isEquidistant( ) == BT_FALSE )
		//return ACADOERROR( RET_ONLY_EQUIDISTANT_GRID_FOR_CODE_EXPORT );

	// only state or control BOUNDS supported!
	Constraint constraint;
	ocp.getConstraint( constraint );
	
	if ( constraint.isBoxConstraint( ) == BT_FALSE )
		return ACADOERROR( RET_ONLY_BOUNDS_FOR_CODE_EXPORT );


	int sparseQPsolution;
	get( SPARSE_QP_SOLUTION,sparseQPsolution );

	if ( ( (SparseQPsolutionMethods)sparseQPsolution != FULL_CONDENSING ) && 
		 ( (SparseQPsolutionMethods)sparseQPsolution != CONDENSING ) )
		return ACADOERROR( RET_INVALID_OPTION );
	
	int hessianApproximation;
	get( HESSIAN_APPROXIMATION,hessianApproximation );
	if ( (HessianApproximationMode)hessianApproximation != GAUSS_NEWTON )
		return ACADOERROR( RET_INVALID_OPTION );

	int discretizationType;
	get( DISCRETIZATION_TYPE,discretizationType );
//	if ( (StateDiscretizationType)discretizationType != SINGLE_SHOOTING )
//		return ACADOERROR( RET_INVALID_OPTION );

	return SUCCESSFUL_RETURN;
}



returnValue MPCexport::collectDataDeclarations(	ExportStatementBlock& declarations,
												ExportStruct dataStruct
												) const
{
	if ( integrator->getDataDeclarations( declarations,dataStruct ) != SUCCESSFUL_RETURN )
		return RET_UNABLE_TO_EXPORT_CODE;

	if ( condenser->getDataDeclarations( declarations,dataStruct ) != SUCCESSFUL_RETURN )
		return RET_UNABLE_TO_EXPORT_CODE;
	
	if ( gaussNewton->getDataDeclarations( declarations,dataStruct ) != SUCCESSFUL_RETURN )
		return RET_UNABLE_TO_EXPORT_CODE;

	if ( auxFcns->getDataDeclarations( declarations,dataStruct ) != SUCCESSFUL_RETURN )
		return RET_UNABLE_TO_EXPORT_CODE;

	return SUCCESSFUL_RETURN;
}


returnValue MPCexport::collectFunctionDeclarations(	ExportStatementBlock& declarations
													) const
{
	if ( integrator->getFunctionDeclarations( declarations ) != SUCCESSFUL_RETURN )
		return RET_UNABLE_TO_EXPORT_CODE;

	if ( condenser->getFunctionDeclarations( declarations ) != SUCCESSFUL_RETURN )
		return RET_UNABLE_TO_EXPORT_CODE;
	
	if ( gaussNewton->getFunctionDeclarations( declarations ) != SUCCESSFUL_RETURN )
		return RET_UNABLE_TO_EXPORT_CODE;

	if ( auxFcns->getFunctionDeclarations( declarations ) != SUCCESSFUL_RETURN )
		return RET_UNABLE_TO_EXPORT_CODE;

	return SUCCESSFUL_RETURN;
}


returnValue MPCexport::exportTemplateMain(	const String& _dirName,
											const String& _fileName,
											const String& _realString,
											const String& _intString,
											int _precision
											) const
{
    String fileName( _dirName );
    fileName << "/" << _fileName;

	ExportFile main( fileName,"acado.h", _realString,_intString,_precision );

	int qpSolver;
	get( QP_SOLVER,qpSolver );

	int fixInitialState;
	get( FIX_INITIAL_STATE,fixInitialState );

	main.addStatement( "#include \"auxiliary_functions.c\"\n" );
	main.addLinebreak( 2 );
	main.addComment( "SOME CONVENIENT DEFINTIONS:" );
	main.addComment( "---------------------------------------------------------------" );
	main.addStatement( (String)"   #define NX          " << NX << "      /* number of differential states  */\n" );
	main.addStatement( (String)"   #define NU          " << NU << "      /* number of control inputs       */\n" );

	if ( NP > 0 )
		main.addStatement( (String)"   #define NP          " << NP << "      /* number of fixed parameters     */\n" );

	main.addStatement( (String)"   #define N           " << N  << "      /* number of control intervals    */\n" );
	main.addStatement( "   #define NUM_STEPS   5      /* number of real time iterations */\n" );
	main.addStatement( "   #define VERBOSE     1      /* show iterations: 1, silent: 0  */\n" );
	main.addComment( "---------------------------------------------------------------" );
	main.addLinebreak( 2 );
	main.addComment( "GLOBAL VARIABLES FOR THE ACADO REAL-TIME ALGORITHM:" );
	main.addComment( "---------------------------------------------------" );
	main.addStatement( "   ACADOvariables acadoVariables;\n" );
	main.addStatement( "   ACADOworkspace acadoWorkspace;\n" );
	main.addLinebreak( );
	main.addComment( "GLOBAL VARIABLES FOR THE QP SOLVER:" );
	main.addComment( "-----------------------------------" );
	main.addStatement( "   Vars         vars;\n" );
	main.addStatement( "   Params       params;\n" );

	switch ( (QPSolverName) qpSolver )
	{
		case QP_CVXGEN:
			main.addStatement( "   Workspace    work;\n" );
			main.addStatement( "   Settings     settings;\n" );
			break;
		
		default:
			break;
	}

    main.addLinebreak( 2 );
	main.addComment( "A TEMPLATE FOR TESTING THE REAL-TIME IMPLEMENTATION:" );
    main.addComment( "----------------------------------------------------" );
    main.addStatement( "int main(){\n" );
    main.addLinebreak( );
    main.addComment( 3,"INTRODUCE AUXILIARY VAIRABLES:" );
    main.addComment( 3,"------------------------------" );
    main.addStatement( "      int    i, iter        ;\n" );
    main.addStatement( "      real_t measurement[NX];\n" );
    main.addLinebreak( 2 );

	switch ( (QPSolverName) qpSolver )
	{
		case QP_CVXGEN:
			main.addComment( 3,"CUSTOMIZE THE SOLVER SETTINGS:" );
			main.addComment( 3,"------------------------------" );
			main.addStatement( "      set_defaults();\n" );
			main.addStatement( "      if( !VERBOSE ) settings.verbose = 0;\n" );
			main.addLinebreak( 2 );
			break;
		
		default:
			break;
	}

    main.addComment( 3,"INITIALIZE THE STATES AND CONTROLS:" );
    main.addComment( 3,"----------------------------------------" );
    main.addStatement( "      for( i = 0; i < NX*N; ++i )  acadoVariables.x[i] = 0.0;\n" );
    main.addStatement( "      for( i = 0; i < NU*N; ++i )  acadoVariables.u[i] = 0.0;\n" );

	if ( NP > 0 )
		main.addStatement( "      for( i = 0; i < NP; ++i )    acadoVariables.p[i] = 0.0;\n" );

    main.addLinebreak( );
    main.addComment( 3,"// INITIALIZE THE STATES AND CONTROL REFERENCE:" );
    main.addComment( 3,"// --------------------------------------------" );

	if ( (BooleanType)fixInitialState == BT_FALSE )
	{
		main.addStatement( "      for( i = 0; i < NX;   ++i )  acadoVariables.x0Ref[i]  =  0.0;\n" );
		main.addStatement( "      for( i = 0; i < NX;   ++i )  acadoVariables.x0Ref2[i] =  0.0;\n" );
	}

	main.addStatement( "      for( i = 0; i < NX*N; ++i )  acadoVariables.xRef[i]  =  0.0;\n" );
    main.addStatement( "      for( i = 0; i < NU*N; ++i )  acadoVariables.uRef[i]  =  0.0;\n" );
    main.addLinebreak( 2 );
    main.addComment( 3,"SETUP THE FIRST STATE MEASUREMENT:" );
    main.addComment( 3,"------------------------------------------------" );
    main.addStatement( "      for( i = 0; i < NX; ++i )  measurement[i] = 0.0;\n" );
    main.addLinebreak( );
    main.addStatement( "      acadoVariables.x[0] = 1.0;\n" );
    main.addStatement( "      measurement     [0] = 1.0;\n" );
    main.addLinebreak( );
    main.addStatement( "      if( VERBOSE ) printHeader();\n" );
    main.addLinebreak( 2 );
	main.addComment( 3,"PREPARE FIRST STEP:" );
    main.addComment( 3,"-------------------" );
    main.addStatement( "      preparationStep();\n" ); 
	main.addLinebreak( 2 );
    main.addComment( 3,"GET THE TIME BEFORE START THE LOOP:" );
    main.addComment( 3,"----------------------------------------------" );
    main.addStatement( "      real_t t1 = getTime();\n" );
    main.addLinebreak( 2 );
    main.addComment( 3,"THE REAL-TIME ITERATION LOOP:" );
    main.addComment( 3,"----------------------------------------------" );
    main.addStatement( "      for( iter = 0; iter < NUM_STEPS; ++iter ){\n" );
    main.addLinebreak( );
    main.addComment( 8,"TAKE A MEASUREMENT:" );
    main.addComment( 8,"-----------------------------" );
    main.addStatement( "           /// meausrement = ...\n" );
    main.addLinebreak( );
    main.addComment( 8,"PERFORM THE FEEDBACK STEP:" );
    main.addComment( 8,"-----------------------------" );
    main.addStatement( "           feedbackStep( measurement );\n" );
    main.addLinebreak( );
    main.addComment( 8,"APPLY THE NEW CONTROL IMMEDIATELY TO THE PROCESS:" );
    main.addComment( 8,"-------------------------------------------------" );
    main.addStatement( "           /// send first piece of acadoVariables.u to process;\n" );
    main.addStatement( "           if( VERBOSE ) printf(\"=================================================================\\n\\n\" );\n" );
    main.addStatement( "           if( VERBOSE ) printf(\"      Real-Time Iteration %d:  KKT Tolerance = %.3e\\n\", iter, getKKT() );\n" );
    main.addStatement( "           if( VERBOSE ) printf(\"\\n=================================================================\\n\" );\n" );
    main.addLinebreak( );
    main.addComment( 8,"OPTIONAL: SHIFT THE INITIALIZATION:" );
    main.addComment( 8,"-----------------------------------" );
    main.addStatement( "           /// shiftControls( acadoVariables.uRef );\n" );
    main.addStatement( "           /// shiftStates  ( acadoVariables.xRef );\n" );
	main.addLinebreak( );
	main.addComment( 8,"PREPARE NEXT STEP:" );
    main.addComment( 8,"------------------" );
    main.addStatement( "           preparationStep();\n" ); 
    main.addStatement( "      }\n" );
    main.addStatement( "      if( VERBOSE ) printf(\"\\n\\n              END OF THE REAL-TIME LOOP. \\n\\n\\n\");\n" );
    main.addLinebreak( );
    main.addLinebreak( );
    main.addComment( 3,"GET THE TIME AT THE END OF THE LOOP:" );
    main.addComment( 3,"----------------------------------------------" );
    main.addStatement( "      real_t t2 = getTime();\n" );
    main.addLinebreak( );
    main.addLinebreak( );
    main.addComment( 3,"PRINT DURATION AND RESULTS:" );
    main.addComment( 3,"--------------------------------------------------------------------------------------------------" );
    main.addStatement( "      if( !VERBOSE )\n" );
    main.addStatement( "      printf(\"\\n\\n AVERAGE DURATION OF ONE REAL-TIME ITERATION:   %.3g μs\\n\\n\", 1e6*(t2-t1)/NUM_STEPS );\n" );
    main.addLinebreak( );
	main.addStatement( "      printStates();\n" );
	main.addStatement( "      printControls();\n" );
	main.addLinebreak( );
    main.addStatement( "    return 0;\n" );
    main.addStatement( "}" );
	main.addLinebreak( );

	return main.exportCode( );
}


returnValue MPCexport::exportMakefile(	const String& _dirName,
										const String& _fileName,
										const String& _realString,
										const String& _intString,
										int _precision
										) const
{
	String fileName( _dirName );
	fileName << "/" << _fileName;

	ExportFile Makefile( fileName,"", _realString,_intString,_precision,"##" );

    int qpSolver;
	get( QP_SOLVER,qpSolver );

	switch ( (QPSolverName) qpSolver )
	{
		case QP_CVXGEN:
			Makefile.addStatement( "LDLIBS = -lm\n" );
			Makefile.addStatement( "CFLAGS = -Os\n" );
			Makefile.addStatement( "CC     = gcc\n" );
			Makefile.addLinebreak( );
			Makefile.addStatement( "OBJECTS = ./cvxgen/solver.o       \\\n" );
			Makefile.addStatement( "\t./cvxgen/matrix_support.o      \\\n" );
			Makefile.addStatement( "\t./cvxgen/ldl.o                 \\\n" );
			Makefile.addStatement( "\t./cvxgen/util.o                \\\n" );
			Makefile.addStatement( "\tintegrator.o                   \\\n" );
			if( !EXPORT_RHS ) {
				Makefile.addStatement( (String)"\t" << externModel << ".o \\\n" );
			}
			Makefile.addStatement( "\tcondensing.o                   \\\n" );
			Makefile.addStatement( "\tgauss_newton_method.o          \n" );
			Makefile.addLinebreak( );
			Makefile.addStatement( ".PHONY: all\n" );
			Makefile.addStatement( "all: test libacado_exported_rti.a\n" );
			Makefile.addLinebreak( );
			Makefile.addStatement( "test: ${OBJECTS} test.o\n" );
			Makefile.addLinebreak( );
			Makefile.addStatement( "./cvxgen/solver.o     : ./cvxgen/solver.h\n" );
			Makefile.addStatement( "integrator.o          : acado.h\n" );
			if( !EXPORT_RHS ) {
				Makefile.addStatement( (String)externModel << ".o             : acado.h\n" );
			}
			Makefile.addStatement( "condensing.o          : acado.h\n" );
			Makefile.addStatement( "gauss_newton_method.o : acado.h   ./cvxgen/solver.h\n" );
			Makefile.addStatement( "test.o                : acado.h   ./cvxgen/solver.h\n" );
			Makefile.addLinebreak( );
			Makefile.addStatement( "libacado_exported_rti.a: ${OBJECTS}\n" );
			Makefile.addStatement( "\tar r $@ $\?\n" );
			Makefile.addLinebreak( );
			Makefile.addStatement( ".PHONY : clean\n" );
			Makefile.addStatement( "clean :\n" );
			Makefile.addStatement( "\t-rm -f *.o *.a ./cvxgen/*.o test\n" );
			Makefile.addLinebreak( );

			break;
		
		case QP_QPOASES:
			Makefile.addStatement( "LDLIBS = -lm \n" );
			Makefile.addStatement( "CXXFLAGS = -O3 -finline-functions -I. -I./qpoases/INCLUDE -I./qpoases/SRC\n" );
			Makefile.addStatement( "CFLAGS = -O3\n" );
			Makefile.addStatement( "CC     = g++\n" );
			Makefile.addLinebreak( );
			Makefile.addStatement( "OBJECTS = \\\n" );

			if ( gaussNewton->getNumStateBounds() > 0 )
				Makefile.addStatement( "\t./qpoases/SRC/QProblem.o        \\\n" );

			Makefile.addStatement( "\t./qpoases/SRC/QProblemB.o       \\\n" );
			Makefile.addStatement( "\t./qpoases/SRC/Bounds.o          \\\n" );
			Makefile.addStatement( "\t./qpoases/SRC/Constraints.o     \\\n" );
			Makefile.addStatement( "\t./qpoases/SRC/SubjectTo.o       \\\n" );
			Makefile.addStatement( "\t./qpoases/SRC/Indexlist.o       \\\n" );
			Makefile.addStatement( "\t./qpoases/SRC/CyclingManager.o  \\\n" );
			Makefile.addStatement( "\t./qpoases/SRC/Utils.o           \\\n" );
			Makefile.addStatement( "\t./qpoases/SRC/MessageHandling.o \\\n" );
			Makefile.addStatement( "\t./qpoases/solver.o              \\\n" );
			Makefile.addStatement( "\tintegrator.o                    \\\n" );
			if( !EXPORT_RHS ) {
				Makefile.addStatement( (String)"\t" << externModel << ".o \\\n" );
			}
			Makefile.addStatement( "\tcondensing.o                    \\\n" );
			Makefile.addStatement( "\tgauss_newton_method.o \n" );
			Makefile.addLinebreak( );
			Makefile.addStatement( ".PHONY: all\n" );
			Makefile.addStatement( "all: test libacado_exported_rti.a\n" );
			Makefile.addLinebreak( );
			Makefile.addStatement( "test: ${OBJECTS} test.o\n" );
			Makefile.addLinebreak( );
			Makefile.addStatement( "./qpoases/solver.o    : ./qpoases/solver.hpp\n" );
			Makefile.addStatement( "integrator.o          : acado.h\n" );
			if( !EXPORT_RHS ) {
				Makefile.addStatement( (String)externModel << ".o             : acado.h\n" );
			}
			Makefile.addStatement( "condensing.o          : acado.h\n" );
			Makefile.addStatement( "gauss_newton_method.o : acado.h   ./qpoases/solver.hpp\n" );
			Makefile.addStatement( "test.o                : acado.h   ./qpoases/solver.hpp\n" );
			Makefile.addLinebreak( );
			Makefile.addStatement( "libacado_exported_rti.a: ${OBJECTS}\n" );
			Makefile.addStatement( "\tar r $@ $\?\n" );
			Makefile.addLinebreak( );
			Makefile.addStatement( "${OBJECTS} : ./qpoases/solver.hpp\n" );
			Makefile.addLinebreak( );
			Makefile.addStatement( ".PHONY : clean\n" );
			Makefile.addStatement( "clean :\n" );
			Makefile.addStatement( "\t-rm -f *.o *.a ./qpoases/SRC/*.o ./qpoases/SRC/*.a test\n" );
			Makefile.addLinebreak( );
			break;

		case QP_QPOASES3:
			Makefile.addStatement( "LDLIBS = -lm \n" );
			Makefile.addStatement( "CFLAGS = -O3 -finline-functions -D__CODE_GENERATION__ -I. -I./qpoases3/include -I./qpoases3/src\n" );
			Makefile.addStatement( "CC     = gcc\n" );
			Makefile.addLinebreak( );
			Makefile.addStatement( "OBJECTS = \\\n" );

			if ( gaussNewton->getNumStateBounds() > 0 )
				Makefile.addStatement( "\t./qpoases3/src/QProblem.o        \\\n" );
			else
				Makefile.addStatement( "\t./qpoases3/src/QProblemB.o       \\\n" );

			Makefile.addStatement( "\t./qpoases3/src/Bounds.o          \\\n" );
			Makefile.addStatement( "\t./qpoases3/src/Constraints.o     \\\n" );
			Makefile.addStatement( "\t./qpoases3/src/Indexlist.o       \\\n" );
			Makefile.addStatement( "\t./qpoases3/src/Options.o         \\\n" );
			Makefile.addStatement( "\t./qpoases3/src/Matrices.o        \\\n" );
			Makefile.addStatement( "\t./qpoases3/src/Utils.o           \\\n" );
			Makefile.addStatement( "\t./qpoases3/src/MessageHandling.o \\\n" );
			Makefile.addStatement( "\t./qpoases3/solver.o              \\\n" );
			Makefile.addStatement( "\tintegrator.o                     \\\n" );
			if( !EXPORT_RHS ) {
				Makefile.addStatement( (String)"\t" << externModel << ".o \\\n" );
			}
			Makefile.addStatement( "\tcondensing.o                     \\\n" );
			Makefile.addStatement( "\tgauss_newton_method.o \n" );
			Makefile.addLinebreak( );
			Makefile.addStatement( ".PHONY: all\n" );
			Makefile.addStatement( "all: test libacado_exported_rti.a\n" );
			Makefile.addLinebreak( );
			Makefile.addStatement( "test: ${OBJECTS} test.o\n" );
			Makefile.addLinebreak( );
			Makefile.addStatement( "./qpoases3/solver.o    : ./qpoases3/solver.h\n" );
			Makefile.addStatement( "integrator.o          : acado.h\n" );
			if( !EXPORT_RHS ) {
				Makefile.addStatement( (String)externModel << ".o             : acado.h\n" );
			}
			Makefile.addStatement( "condensing.o          : acado.h\n" );
			Makefile.addStatement( "gauss_newton_method.o : acado.h   ./qpoases3/solver.h\n" );
			Makefile.addStatement( "test.o                : acado.h   ./qpoases3/solver.h\n" );
			Makefile.addLinebreak( );
			Makefile.addStatement( "libacado_exported_rti.a: ${OBJECTS}\n" );
			Makefile.addStatement( "\tar r $@ $\?\n" );
			Makefile.addLinebreak( );
			Makefile.addStatement( "${OBJECTS} : ./qpoases3/solver.h\n" );
			Makefile.addLinebreak( );
			Makefile.addStatement( ".PHONY : clean\n" );
			Makefile.addStatement( "clean :\n" );
			Makefile.addStatement( "\t-rm -f *.o *.a ./qpoases3/src/*.o ./qpoases3/src/*.a test\n" );
			Makefile.addLinebreak( );
			break;

		default:
			return ACADOERROR( RET_INVALID_OPTION );
	}

	return Makefile.exportCode( );
}


returnValue MPCexport::exportQPsolverInterface(	const String& _dirName,
												const String& _realString,
												const String& _intString,
												int _precision
												) const
{
	int qpSolver;
	get( QP_SOLVER,qpSolver );

	int maxNumQPiterations;
	get( MAX_NUM_QP_ITERATIONS,maxNumQPiterations );

	// if not specified, use default value
	if ( maxNumQPiterations <= 0 )
		maxNumQPiterations = 3*(gaussNewton->getNumQPvars()+gaussNewton->getNumStateBounds());

	int hotstartQP;
	get( HOTSTART_QP,hotstartQP );
	
	int useSinglePrecision;
	get( USE_SINGLE_PRECISION,useSinglePrecision );

	String fileNameHeader( _dirName );
	String fileNameSource( _dirName );
	
	switch ( (QPSolverName)qpSolver )
	{
		case QP_CVXGEN:
			if ( (BooleanType)hotstartQP == BT_TRUE )
				return ACADOERROR( RET_UNABLE_TO_HOTSTART_QP );
			else
				return SUCCESSFUL_RETURN;

			if ( (BooleanType)useSinglePrecision == BT_TRUE )
				return ACADOERROR( RET_INVALID_OPTION );
			else
				return SUCCESSFUL_RETURN;

		case QP_QPOASES:
			fileNameHeader += "/qpoases/solver.hpp";
			fileNameSource += "/qpoases/solver.cpp";
			break;

		case QP_QPOASES3:
			fileNameHeader += "/qpoases3/solver.h";
			fileNameSource += "/qpoases3/solver.c";
			break;

		default:
			return ACADOERROR( RET_INVALID_OPTION );
	}


	// generate header file
    ExportFile qpSolverHeader( fileNameHeader,"", _realString,_intString,_precision );
	
    qpSolverHeader.addStatement( "#ifndef SOLVER_QPOASES_H\n" );
	qpSolverHeader.addStatement( "#define SOLVER_QPOASES_H\n" );
	qpSolverHeader.addLinebreak( );
	qpSolverHeader.addStatement( "#include <stdio.h>\n" );
	qpSolverHeader.addStatement( "#include <math.h>\n" );
	qpSolverHeader.addLinebreak( 2 );
	qpSolverHeader.addStatement( (String)"#define QPOASES_NVMAX      " << gaussNewton->getNumQPvars() << "\n" );
	qpSolverHeader.addStatement( (String)"#define QPOASES_NCMAX      " << acadoMax( gaussNewton->getNumStateBounds(),1 ) << "\n" );
	qpSolverHeader.addStatement( (String)"#define QPOASES_NWSRMAX    " << acadoMax( maxNumQPiterations,1 ) << "\n" );
	qpSolverHeader.addStatement( (String)"#define QPOASES_PRINTLEVEL " << "PL_NONE" << "\n" );

	if ( (BooleanType)useSinglePrecision == BT_TRUE )
	{
		qpSolverHeader.addStatement( (String)"#define QPOASES_EPS        " << 1.193e-07 << "\n" );
		qpSolverHeader.addLinebreak( );
		qpSolverHeader.addStatement( "typedef float real_t;\n" );
	}
	else
	{
		qpSolverHeader.addStatement( (String)"#define QPOASES_EPS        " << 2.221e-16 << "\n" );
		qpSolverHeader.addLinebreak( );
		qpSolverHeader.addStatement( "typedef double real_t;\n" );
	}

	qpSolverHeader.addLinebreak( );
	qpSolverHeader.addStatement( "typedef struct Params_t {\n" );

	if ( collectDataDeclarations( qpSolverHeader,ACADO_PARAMS ) != SUCCESSFUL_RETURN )
		return ACADOERROR( RET_UNABLE_TO_EXPORT_CODE );

	qpSolverHeader.addStatement( "} Params;\n" );
	qpSolverHeader.addLinebreak( );
	qpSolverHeader.addStatement( "typedef struct Vars_t {\n" );

	if ( collectDataDeclarations( qpSolverHeader,ACADO_VARS ) != SUCCESSFUL_RETURN )
		return ACADOERROR( RET_UNABLE_TO_EXPORT_CODE );

	qpSolverHeader.addStatement( "} Vars;\n" );
	qpSolverHeader.addLinebreak( );
	qpSolverHeader.addStatement( "extern Params params;\n" );
	qpSolverHeader.addStatement( "extern Vars vars;\n" );
	qpSolverHeader.addLinebreak( );
	qpSolverHeader.addStatement( "long solve(void);\n" );
	qpSolverHeader.addLinebreak( );
	qpSolverHeader.addStatement( "#endif\n" );
	
	qpSolverHeader.exportCode( );


	// generate source file
	ExportFile qpSolverSource( fileNameSource,"", _realString,_intString,_precision );

	if ( (QPSolverName)qpSolver == QP_QPOASES )
	{
		qpSolverSource.addStatement( "extern \"C\"{\n" );
		qpSolverSource.addStatement( "#include \"solver.hpp\"\n" );
		qpSolverSource.addStatement( "}\n" );
		qpSolverSource.addStatement( "#include \"INCLUDE/QProblem.hpp\"\n" );
	}
	else
	{
		qpSolverSource.addStatement( "#include \"solver.h\"\n" );
		
		if ( gaussNewton->getNumStateBounds() > 0 )
			qpSolverSource.addStatement( "#include \"include/qpOASES/QProblem.h\"\n" );
		else
			qpSolverSource.addStatement( "#include \"include/qpOASES/QProblemB.h\"\n" );
	}
    qpSolverSource.addLinebreak( 2 );

    qpSolverSource.addStatement( "int logNWSR;" );

    qpSolverSource.addLinebreak( 2 );
	qpSolverSource.addStatement( "long solve( void )\n" );
	qpSolverSource.addStatement( "{\n" );
	qpSolverSource.addStatement( "  int nWSR = QPOASES_NWSRMAX;\n" );
	qpSolverSource.addLinebreak( );

	if ( (QPSolverName)qpSolver == QP_QPOASES )
	{
		if ( gaussNewton->getNumStateBounds() > 0 )
		{
			qpSolverSource.addStatement( (String)"  QProblem qp( " << gaussNewton->getNumQPvars() << "," << gaussNewton->getNumStateBounds() << " );\n" );
			if ( (BooleanType)hotstartQP == BT_TRUE )
				qpSolverSource.addStatement( "  returnValue retVal = qp.init( params.H,params.g,params.A,params.lb,params.ub,params.lbA,params.ubA, nWSR,vars.y );\n" );
			else
				qpSolverSource.addStatement( "  returnValue retVal = qp.init( params.H,params.g,params.A,params.lb,params.ub,params.lbA,params.ubA, nWSR );\n" );
		}
		else
		{
			qpSolverSource.addStatement( (String)"  QProblemB qp( " << gaussNewton->getNumQPvars() << " );\n" );
			if ( (BooleanType)hotstartQP == BT_TRUE )
				qpSolverSource.addStatement( "  returnValue retVal = qp.init( params.H,params.g,params.lb,params.ub, nWSR,vars.y );\n" );
			else
				qpSolverSource.addStatement( "  returnValue retVal = qp.init( params.H,params.g,params.lb,params.ub, nWSR );\n" );
		}

//		qpSolverSource.addComment( " if ( (int)retVal != 0 )\n    printf( \"WARNING: QP solver returned with status %d\", retVal );" );
		qpSolverSource.addLinebreak( );
		qpSolverSource.addStatement( "  qp.getPrimalSolution( vars.x );\n" );
		qpSolverSource.addStatement( "  qp.getDualSolution( vars.y );\n" );
	}
	else
	{
		if ( gaussNewton->getNumStateBounds() > 0 )
		{
			qpSolverSource.addStatement( "  QProblem qp;\n" );
			qpSolverSource.addStatement( (String)"  QProblemCON( &qp," << gaussNewton->getNumQPvars() << "," << gaussNewton->getNumStateBounds() << ",HST_POSDEF );\n" );
			if ( (BooleanType)hotstartQP == BT_TRUE )
				qpSolverSource.addStatement( "  returnValue retVal = QProblem_initW( &qp,params.H,params.g,params.A,params.lb,params.ub,params.lbA,params.ubA, &nWSR,0, 0,vars.y,0,0 );\n" );
			else
				qpSolverSource.addStatement( "  returnValue retVal = QProblem_init( &qp,params.H,params.g,params.A,params.lb,params.ub,params.lbA,params.ubA, &nWSR,0 );\n" );

//			qpSolverSource.addComment( " if ( (int)retVal != 0 )\n    printf( \"WARNING: QP solver returned with status %d\", retVal );" );
			qpSolverSource.addLinebreak( );
			qpSolverSource.addStatement( "  QProblem_getPrimalSolution( &qp,vars.x );\n" );
			qpSolverSource.addStatement( "  QProblem_getDualSolution( &qp,vars.y );\n" );
		}
		else
		{
			qpSolverSource.addStatement( "  QProblemB qp;\n" );
			qpSolverSource.addStatement( (String)"  QProblemBCON ( &qp," << gaussNewton->getNumQPvars() << ",HST_POSDEF );\n" );
			if ( (BooleanType)hotstartQP == BT_TRUE )
				qpSolverSource.addStatement( "  returnValue retVal = QProblemB_init( &qp,params.H,params.g,params.lb,params.ub, &nWSR,0, 0,vars.y,0 );\n" );
			else
				qpSolverSource.addStatement( "  returnValue retVal = QProblemB_init( &qp,params.H,params.g,params.lb,params.ub, &nWSR,0 );\n" );

//			qpSolverSource.addComment( " if ( (int)retVal != 0 )\n    printf( \"WARNING: QP solver returned with status %d\", retVal );" );
			qpSolverSource.addLinebreak( );
			qpSolverSource.addStatement( "  QProblemB_getPrimalSolution( &qp,vars.x );\n" );
			qpSolverSource.addStatement( "  QProblemB_getDualSolution( &qp,vars.y );\n" );
		}
	}
	
	qpSolverSource.addStatement( "\n\n  logNWSR = nWSR;\n\n" );

	qpSolverSource.addLinebreak( );
	qpSolverSource.addStatement( "  return (long) retVal;\n" );
	qpSolverSource.addStatement( "}\n" );


//	qpSolverSource.addLinebreak( );
//	qpSolverSource.addStatement( "  return 0;\n" );
//	qpSolverSource.addStatement( "}\n" );

	qpSolverSource.exportCode( );

	return SUCCESSFUL_RETURN;
}


returnValue MPCexport::exportSimulinkInterface(	const String& _dirName,
												const String& _sFcnFileName,
												const String& _makeFileName,
												const String& _realString,
												const String& _intString,
												int _precision
												) const
{
	// Export Simulink interface (S function)
	String fileName( _dirName );
    fileName << "/" << _sFcnFileName;

	ExportFile sfunction( fileName,"", _realString,_intString,_precision );

	int qpSolver;
	get( QP_SOLVER,qpSolver );

	sfunction.addStatement( "#define S_FUNCTION_NAME   sfunction\n" );
	sfunction.addStatement( "#define S_FUNCTION_LEVEL  2\n" );
	sfunction.addLinebreak( );
	sfunction.addStatement( "#define MDL_START\n" );
	sfunction.addLinebreak( 2 );
	sfunction.addStatement( "#ifdef __cplusplus\n" );
	sfunction.addStatement( "extern \"C\" {\n" );
	sfunction.addStatement( "#endif\n" );
	sfunction.addLinebreak( );
	sfunction.addStatement( "#include \"acado.h\"\n" );
	sfunction.addStatement( "#include \"auxiliary_functions.c\"\n" );
	sfunction.addStatement( "#include \"simstruc.h\"\n" );
	sfunction.addLinebreak( );
	sfunction.addStatement( "ACADOvariables acadoVariables;\n" );
	sfunction.addStatement( "ACADOworkspace acadoWorkspace;\n" );
	sfunction.addStatement( "Vars   vars;\n" );
	sfunction.addStatement( "Params params;\n" );

	switch ( (QPSolverName) qpSolver )
	{
		case QP_CVXGEN:
			sfunction.addStatement( "Workspace work;\n" );
			sfunction.addStatement( "Settings  settings;\n" );
			break;
			
		case QP_QPOASES3:
			return ACADOERROR( RET_INVALID_OPTION );
		
		default:
			break;
	}

	sfunction.addLinebreak( );
	sfunction.addStatement( (String)"#define NX           " << getNX() << "\n" );
	sfunction.addStatement( (String)"#define NU           " << getNU() << "\n" );
	sfunction.addStatement( (String)"#define NP           " << getNP() << "\n" );
	sfunction.addStatement( (String)"#define N            " << getN()  << "\n" );
	sfunction.addStatement( "#define SAMPLINGTIME 0.1\n" );
	sfunction.addLinebreak( 2 );
	
	int numInputs = 3;
	if ( getNP() > 0 )
		++numInputs;

	sfunction.addStatement( "static void mdlInitializeSizes (SimStruct *S)\n" );
	sfunction.addStatement( "{\n" );
	sfunction.addStatement( "    /* Specify the number of continuous and discrete states */\n" );
	sfunction.addStatement( "    ssSetNumContStates(S, 0);\n" );
	sfunction.addStatement( "    ssSetNumDiscStates(S, 0);\n" );
	sfunction.addLinebreak( );
	sfunction.addStatement( "    /* Specify the number of intput ports */\n" );
	sfunction.addStatement( (String)"    if ( !ssSetNumInputPorts(S, " << numInputs << ") )\n" );
	sfunction.addStatement( "        return;\n" );
	sfunction.addLinebreak( );
	sfunction.addStatement( "    /* Specify the number of output ports */\n" );
	sfunction.addStatement( "    if ( !ssSetNumOutputPorts(S, 2) )\n" );
	sfunction.addStatement( "        return;\n" );
	sfunction.addLinebreak( );
	sfunction.addStatement( "    /* Specify the number of parameters */\n" );
	sfunction.addStatement( "    ssSetNumSFcnParams(S, 2);\n" );
	sfunction.addStatement( "    if ( ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S) )\n" );
	sfunction.addStatement( "        return;\n" );
	sfunction.addLinebreak( );
	sfunction.addStatement( "    /* Specify dimension information for the input ports */\n" );
	sfunction.addStatement( "    ssSetInputPortVectorDimension(S, 0, NX);\n" );
	sfunction.addStatement( "    ssSetInputPortVectorDimension(S, 1, NX*N);\n" );
	sfunction.addStatement( "    ssSetInputPortVectorDimension(S, 2, NU*N);\n" );

	if ( getNP() > 0 )
		sfunction.addStatement( "    ssSetInputPortVectorDimension(S, 3, NP);\n" );

	sfunction.addLinebreak( );
	sfunction.addStatement( "    /* Specify dimension information for the output ports */\n" );
	sfunction.addStatement( "    ssSetOutputPortVectorDimension(S, 0, NU );\n" );
	sfunction.addStatement( "    ssSetOutputPortVectorDimension(S, 1, 1 );\n" );
	sfunction.addLinebreak( );
	sfunction.addStatement( "    /* Specify the direct feedthrough status */\n" );
	sfunction.addStatement( "    ssSetInputPortDirectFeedThrough(S, 0, 1);\n" );
	sfunction.addStatement( "    ssSetInputPortDirectFeedThrough(S, 1, 1);\n" );
	sfunction.addStatement( "    ssSetInputPortDirectFeedThrough(S, 2, 1);\n" );

	if ( getNP() > 0 )
		sfunction.addStatement( "    ssSetInputPortDirectFeedThrough(S, 3, 1);\n" );

	sfunction.addLinebreak( );
	sfunction.addStatement( "    /* One sample time */\n" );
	sfunction.addStatement( "    ssSetNumSampleTimes(S, 1);\n" );
	sfunction.addStatement( "    }\n" );
	sfunction.addLinebreak( 2 );
	
	sfunction.addStatement( "#if defined(MATLAB_MEX_FILE)\n" );
	sfunction.addLinebreak( );
	sfunction.addStatement( "#define MDL_SET_INPUT_PORT_DIMENSION_INFO\n" );
	sfunction.addStatement( "#define MDL_SET_OUTPUT_PORT_DIMENSION_INFO\n" );
	sfunction.addLinebreak( );
	sfunction.addStatement( "static void mdlSetInputPortDimensionInfo(SimStruct *S, int_T port, const DimsInfo_T *dimsInfo)\n" );
	sfunction.addStatement( "{\n" );
	sfunction.addStatement( "    if ( !ssSetInputPortDimensionInfo(S, port, dimsInfo) )\n" );
	sfunction.addStatement( "         return;\n" );
	sfunction.addStatement( "}\n" );
	sfunction.addLinebreak( );
	sfunction.addStatement( "static void mdlSetOutputPortDimensionInfo(SimStruct *S, int_T port, const DimsInfo_T *dimsInfo)\n" );
	sfunction.addStatement( "{\n" );
	sfunction.addStatement( "    if ( !ssSetOutputPortDimensionInfo(S, port, dimsInfo) )\n" );
	sfunction.addStatement( "         return;\n" );
	sfunction.addStatement( "}\n" );
	sfunction.addLinebreak( );
	sfunction.addStatement( "    #endif /* MATLAB_MEX_FILE */\n" );
	sfunction.addLinebreak( 2 );
	
	sfunction.addStatement( "static void mdlInitializeSampleTimes(SimStruct *S)\n" );
	sfunction.addStatement( "{\n" );
	sfunction.addStatement( "    ssSetSampleTime(S, 0, SAMPLINGTIME);\n" );
	sfunction.addStatement( "    ssSetOffsetTime(S, 0, 0.0);\n" );
	sfunction.addStatement( "}\n" );
	sfunction.addLinebreak( 2 );
	sfunction.addStatement( "static void mdlStart(SimStruct *S)\n" );
	sfunction.addStatement( "{\n" );
	sfunction.addStatement( "    int i;\n" );
	sfunction.addLinebreak( );

	if ( getNP() > 0 )
		sfunction.addStatement( "    InputRealPtrsType in_xRef, in_uRef, in_p;\n" );
	else
		sfunction.addStatement( "    InputRealPtrsType in_xRef, in_uRef;\n" );

	sfunction.addStatement( "    double *xInit, *uInit;\n" );
	sfunction.addLinebreak( );
	sfunction.addStatement( "    /* get inputs and perform feedback step */\n" );
	sfunction.addStatement( "    in_xRef = ssGetInputPortRealSignalPtrs(S, 1);\n" );
	sfunction.addStatement( "    in_uRef = ssGetInputPortRealSignalPtrs(S, 2);\n" );
	
	if ( getNP() > 0 )
		sfunction.addStatement( "    in_p    = ssGetInputPortRealSignalPtrs(S, 3);\n" );

	sfunction.addLinebreak( );
	sfunction.addStatement( "    xInit = mxGetPr( ssGetSFcnParam(S, 0) );\n" );
	sfunction.addStatement( "    uInit = mxGetPr( ssGetSFcnParam(S, 1) );\n" );
	sfunction.addLinebreak( );
	sfunction.addStatement( "    for( i=0; i < NX*N; ++i ) acadoVariables.x[i] = xInit[i];\n" );
	sfunction.addStatement( "    for( i=0; i < NU*N; ++i ) acadoVariables.u[i] = uInit[i];\n" );
	sfunction.addLinebreak( );
	sfunction.addStatement( "    for( i=0; i < NX*N; ++i ) acadoVariables.xRef[i] = (double)(*in_xRef)[i];\n" );
	sfunction.addStatement( "    for( i=0; i < NU*N; ++i ) acadoVariables.uRef[i] = (double)(*in_uRef)[i];\n" );

	if ( getNP() > 0 )
		sfunction.addStatement( "    for( i = 0; i < NP;   ++i ) acadoVariables.p[i]    = (double)(*in_p)[i];\n" );

	sfunction.addLinebreak( );
	sfunction.addStatement( "    preparationStep( );\n" );
	sfunction.addStatement( "}\n" );
	sfunction.addLinebreak( 2 );
	
	sfunction.addStatement( "static void mdlOutputs(SimStruct *S, int_T tid)\n" );
	sfunction.addStatement( "{\n" );
	sfunction.addStatement( "    int i;\n" );
	sfunction.addStatement( "    double measurement[NX];\n" );
	sfunction.addLinebreak( );

	if ( getNP() > 0 )
		sfunction.addStatement( "    InputRealPtrsType in_x, in_xRef, in_uRef, in_p;\n" );
	else
		sfunction.addStatement( "    InputRealPtrsType in_x, in_xRef, in_uRef;\n" );

	sfunction.addStatement( "    real_t *out_u0, *out_kktTol;\n" );
	sfunction.addLinebreak( );
	sfunction.addStatement( "    /* get inputs and perform feedback step */\n" );
	sfunction.addStatement( "    in_x    = ssGetInputPortRealSignalPtrs(S, 0);\n" );
	sfunction.addStatement( "    in_xRef = ssGetInputPortRealSignalPtrs(S, 1);\n" );
	sfunction.addStatement( "    in_uRef = ssGetInputPortRealSignalPtrs(S, 2);\n" );

	if ( getNP() > 0 )
		sfunction.addStatement( "    in_p    = ssGetInputPortRealSignalPtrs(S, 3);\n" );

	sfunction.addLinebreak( );
	sfunction.addStatement( "    for( i=0; i < NX;   ++i ) measurement[i]         = (double)(*in_x)[i];\n" );
	sfunction.addStatement( "    for( i=0; i < NX*N; ++i ) acadoVariables.xRef[i] = (double)(*in_xRef)[i];\n" );
	sfunction.addStatement( "    for( i=0; i < NU*N; ++i ) acadoVariables.uRef[i] = (double)(*in_uRef)[i];\n" );

	if ( getNP() > 0 )
		sfunction.addStatement( "    for( i = 0; i < NP;   ++i ) acadoVariables.p[i]    = (double)(*in_p)[i];\n" );

	sfunction.addLinebreak( );
	sfunction.addStatement( "    feedbackStep( measurement );\n" );
	sfunction.addLinebreak( );
	sfunction.addStatement( "    /* return outputs and prepare next iteration */\n" );
	sfunction.addStatement( "    out_u0     = ssGetOutputPortRealSignal(S, 0);\n" );
	sfunction.addStatement( "    out_kktTol = ssGetOutputPortRealSignal(S, 1);\n" );
	sfunction.addLinebreak( );
	sfunction.addStatement( "    for( i=0; i < NU; ++i ) out_u0[i] = acadoVariables.u[i];\n" );
	sfunction.addStatement( "    out_kktTol[0] = getKKT( );\n" );
	sfunction.addLinebreak( );
	sfunction.addStatement( "    preparationStep( );\n" );
	sfunction.addStatement( "}\n" );
	sfunction.addLinebreak( 2 );
	
	sfunction.addStatement( "static void mdlTerminate(SimStruct *S)\n" );
	sfunction.addStatement( "{\n" );
	sfunction.addStatement( "}\n" );
	sfunction.addLinebreak( 2 );
	
	sfunction.addStatement( "#ifdef  MATLAB_MEX_FILE\n" );
	sfunction.addStatement( "#include \"simulink.c\"\n" );
	sfunction.addStatement( "#else\n" );
	sfunction.addStatement( "#include \"cg_sfun.h\"\n" );
	sfunction.addStatement( "#endif\n" );
	sfunction.addLinebreak( 2 );
	
	sfunction.addStatement( "#ifdef __cplusplus\n" );
	sfunction.addStatement( "}\n" );
	sfunction.addStatement( "#endif\n" );

	sfunction.exportCode( );


	// Export Simulink make script
	fileName = _dirName;
	fileName << "/" << _makeFileName;

	ExportFile make_sfunction( fileName,"", _realString,_intString,_precision,"%%" );


	make_sfunction.addStatement( "FCN_NAME = 'sfunction';\n" );
	make_sfunction.addLinebreak( );

	switch ( (QPSolverName) qpSolver )
	{
		case QP_CVXGEN:
			make_sfunction.addStatement( "IFLAGS  = [ '-I. -I./cvxgen ' ];\n" );
			break;
		
		case QP_QPOASES:
			make_sfunction.addStatement( "IFLAGS  = [ '-I. -I./qpoases/INCLUDE  -I./qpoases/SRC ' ];\n" );
			break;
			
		case QP_QPOASES3:
			make_sfunction.addStatement( "IFLAGS  = [ '-I. -I./qpoases3/include -I./qpoases3/src ' ];\n" );
			break;

		default:
			return ACADOERROR( RET_INVALID_OPTION );
	}

	make_sfunction.addLinebreak( );
	make_sfunction.addStatement( "if ( ispc == 0 )\n" );
	make_sfunction.addStatement( "    CPPFLAGS  = [ IFLAGS, '-D__cpluplus -O -DLINUX ' ];\n" );
	make_sfunction.addStatement( "else\n" );
	make_sfunction.addStatement( "    CPPFLAGS  = [ IFLAGS, '-D__cpluplus -O -DWIN32 ' ];\n" );
	make_sfunction.addStatement( "end\n" );
	make_sfunction.addLinebreak( );
	
	switch ( (QPSolverName) qpSolver )
	{
		case QP_CVXGEN:
			make_sfunction.addStatement( "OBJECTS = [ './cvxgen/solver.c ',...\n" );
			make_sfunction.addStatement( "            './cvxgen/matrix_support.c ',...\n" );
			make_sfunction.addStatement( "            './cvxgen/ldl.c ',...\n" );
			make_sfunction.addStatement( "            './cvxgen/util.c ',...\n" );
			break;
		
		case QP_QPOASES:
			make_sfunction.addStatement( "OBJECTS = [ './qpoases/SRC/QProblem.cpp ',...\n" );
			make_sfunction.addStatement( "            './qpoases/SRC/QProblemB.cpp ',...\n" );
			make_sfunction.addStatement( "            './qpoases/SRC/Bounds.cpp ',...\n" );
			make_sfunction.addStatement( "            './qpoases/SRC/Constraints.cpp ',...\n" );
			make_sfunction.addStatement( "            './qpoases/SRC/SubjectTo.cpp ',...\n" );
			make_sfunction.addStatement( "            './qpoases/SRC/Indexlist.cpp ',...\n" );
			make_sfunction.addStatement( "            './qpoases/SRC/CyclingManager.cpp ',...\n" );
			make_sfunction.addStatement( "            './qpoases/SRC/Utils.cpp ',...\n" );
			make_sfunction.addStatement( "            './qpoases/SRC/MessageHandling.cpp ',...\n" );
			make_sfunction.addStatement( "            './qpoases/solver.cpp ',...\n" );
			break;
	
		case QP_QPOASES3:
			make_sfunction.addStatement( "OBJECTS = [ './qpoases3/src/QProblem.c ',...\n" );
			make_sfunction.addStatement( "            './qpoases3/src/QProblemB.c ',...\n" );
			make_sfunction.addStatement( "            './qpoases3/src/Bounds.c ',...\n" );
			make_sfunction.addStatement( "            './qpoases3/src/Constraints.c ',...\n" );
			make_sfunction.addStatement( "            './qpoases3/src/Indexlist.c ',...\n" );
			make_sfunction.addStatement( "            './qpoases3/src/Options.c ',...\n" );
			make_sfunction.addStatement( "            './qpoases3/src/Matrices.c ',...\n" );
			make_sfunction.addStatement( "            './qpoases3/src/Utils.c ',...\n" );
			make_sfunction.addStatement( "            './qpoases3/src/MessageHandling.c ',...\n" );
			make_sfunction.addStatement( "            './qpoases3/solver.c ',...\n" );
			break;

		default:
			return ACADOERROR( RET_INVALID_OPTION );
	}

	make_sfunction.addStatement( "            'integrator.c ',...\n" );
	make_sfunction.addStatement( "            'condensing.c ',...\n" );
	make_sfunction.addStatement( "            'gauss_newton_method.c ' ];\n" );
	make_sfunction.addLinebreak( 2 );
	make_sfunction.addStatement( "eval( [ 'mex -output ', FCN_NAME, ' ', CPPFLAGS, ' ', FCN_NAME, '.cpp ', OBJECTS] );\n" );
	make_sfunction.addStatement( "disp( [ FCN_NAME, '.', eval('mexext'), ' successfully created!'] );\n" );
	make_sfunction.addLinebreak( );
	make_sfunction.addStatement( "clear IFLAGS CPPFLAGS OBJECTS\n" );
	make_sfunction.addLinebreak( );

	make_sfunction.exportCode( );

	return SUCCESSFUL_RETURN;
}


CLOSE_NAMESPACE_ACADO

// end of file.
