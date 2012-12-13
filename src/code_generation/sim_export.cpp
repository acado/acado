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
 *    \file src/code_generation/sim_export.cpp
 *    \author Rien Quirynen
 *    \date 2012
 */

#include <acado/code_generation/sim_export.hpp>

#include <acado/code_generation/templates/templates.hpp>

#include <acado/code_generation/export_algorithm_factory.hpp>

#ifdef WIN32
#include <windows.h>
#endif


BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

SIMexport::SIMexport( const uint simIntervals, const double totalTime ) : ExportModule( )
{
	N = simIntervals;
	T = totalTime;
	integrator  = 0;
	timingSteps = 100;
	timingCalls = 0;
	
	_initStates = String( "initStates.txt" );
	_controls = String( "controls.txt" );
	_results = String( "results.txt" );
	_ref = String( "ref.txt" );
	referenceProvided = BT_FALSE;
	PRINT_DETAILS = BT_TRUE;
	MODEL_DIMENSIONS_SET = BT_FALSE;

	setStatus( BS_NOT_INITIALIZED );
}


SIMexport::SIMexport(	const SIMexport& arg
						) : ExportModule( arg )
{
	copy( arg );
}


SIMexport::~SIMexport( )
{
	clear( );
}


SIMexport& SIMexport::operator=(	const SIMexport& arg
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



returnValue SIMexport::exportCode(	const String& dirName,
									const String& _realString,
									const String& _intString,
									int _precision
									)
{
	if (!MODEL_DIMENSIONS_SET) return ACADOERROR( RET_UNABLE_TO_EXPORT_CODE );
	set( QP_SOLVER, QP_NONE );

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

		int measGrid;
		get( MEASUREMENT_GRID, measGrid );
		int generateMatlabInterface;
		get( GENERATE_MATLAB_INTERFACE, generateMatlabInterface );
		if ( (BooleanType)generateMatlabInterface == BT_TRUE ) {
			String integrateInterface( dirName );
			integrateInterface << "/integrate.c";
			ExportMatlabIntegrator exportMexFun( INTEGRATOR_MEX_TEMPLATE, integrateInterface, commonHeaderName,_realString,_intString,_precision );
			exportMexFun.configure((MeasurementGrid)measGrid == ONLINE_GRID);
			exportMexFun.exportCode();

			String rhsInterface( dirName );
			rhsInterface << "/rhs.c";
			ExportMatlabRhs exportMexFun2( RHS_MEX_TEMPLATE, rhsInterface, commonHeaderName,_realString,_intString,_precision );
			exportMexFun2.configure(integrator->getNameODE());
			exportMexFun2.exportCode();
		}
	}


	// export template for main file, if desired
	if ( (PrintLevel)printLevel >= HIGH ) 
		acadoPrintf( "--> Exporting remaining files... " );

	// export a basic Makefile, if desired
	int generateMakeFile;
	get( GENERATE_MAKE_FILE,generateMakeFile );
	if ( (BooleanType)generateMakeFile == BT_TRUE )
		if ( exportMakefile( dirName,"Makefile",_realString,_intString,_precision ) != SUCCESSFUL_RETURN )
			return ACADOERROR( RET_UNABLE_TO_EXPORT_CODE );
			
	// export the evaluation file
	if ( exportEvaluation( dirName, String( "compare.c" ) ) != SUCCESSFUL_RETURN )
		return ACADOERROR( RET_UNABLE_TO_EXPORT_CODE );

	if ( (PrintLevel)printLevel >= HIGH ) 
		acadoPrintf( "done.\n" );

	if ( (PrintLevel)printLevel > NONE )
		ACADOINFO( RET_CODE_EXPORT_SUCCESSFUL );

    return SUCCESSFUL_RETURN;
}


returnValue SIMexport::addOutput( const OutputFcn& outputEquation_ ){

	if( rhs_ODE.isEmpty() && outputNames.size() == 0 ) {
		Expression next;
		outputEquation_.getExpression( next );
		outputExpressions.push_back( next );
		dim_outputs.push_back( next.getDim() );
	}
	else {
		return ACADOERROR( RET_INVALID_OPTION );
	}

    return SUCCESSFUL_RETURN;
}


returnValue SIMexport::addOutput( const String& output, const String& diffs_output, const uint dim ){

	if( outputExpressions.size() == 0 && f.getNumDynamicEquations() == 0 ) {
		outputNames.push_back( output );
		diffs_outputNames.push_back( diffs_output );
		dim_outputs.push_back( dim );
	}
	else {
		return ACADOERROR( RET_INVALID_OPTION );
	}

    return SUCCESSFUL_RETURN;
}


returnValue SIMexport::addOutput( 	const String& output, const String& diffs_output, const uint dim,
									const String& colInd, const String& rowPtr	){


	Vector colIndV = readFromFile( colInd.getName() );
	Vector rowPtrV = readFromFile( rowPtr.getName() );
	if( rowPtrV.getDim() != (dim+1) ) {
		return ACADOERROR( RET_INVALID_OPTION );
	}
	colInd_outputs.push_back( colIndV );
	rowPtr_outputs.push_back( rowPtrV );

	addOutput( output, diffs_output, dim );

    return SUCCESSFUL_RETURN;
}


returnValue SIMexport::setMeasurements( const Vector& numberMeasurements ){

	int i;
	if( outputExpressions.size() != numberMeasurements.getDim() && outputNames.size() != numberMeasurements.getDim() ) {
		return ACADOERROR( RET_INVALID_OPTION );
	}
	outputGrids.clear();
	for( i = 0; i < (int)numberMeasurements.getDim(); i++ ) {
		Grid nextGrid( 0.0, 1.0, (int)numberMeasurements(i) + 1 );
		outputGrids.push_back( nextGrid );
	}

    return SUCCESSFUL_RETURN;
}


BooleanType SIMexport::hasOutputs() const{

	if( outputExpressions.size() == 0 && outputNames.size() == 0 ) return BT_FALSE;
	return BT_TRUE;
}



//
// PROTECTED MEMBER FUNCTIONS:
//

returnValue SIMexport::copy(	const SIMexport& arg
								)
{
	integrator = arg.integrator;
		
	_initStates = arg._initStates;
	_controls = arg._controls;
	_results = arg._results;
	_ref = arg._ref;
	_refOutputFiles = arg._refOutputFiles;
	referenceProvided = arg.referenceProvided;
	PRINT_DETAILS = arg.PRINT_DETAILS;
	timingSteps = arg.timingSteps;
	timingCalls = arg.timingCalls;

	return SUCCESSFUL_RETURN;
}


returnValue SIMexport::clear( )
{
	if ( integrator != 0 )
		delete integrator;

	return SUCCESSFUL_RETURN;
}



returnValue SIMexport::setup( )
{
	returnValue returnvalue = checkConsistency( );
	if ( returnvalue != SUCCESSFUL_RETURN )
		return ACADOERROR( returnvalue );

	int numSteps;
    get( NUM_INTEGRATOR_STEPS, numSteps );

	if ( numSteps <= 0 )
		return ACADOERROR( RET_INVALID_OPTION );

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

	Grid ocpGrid( 0.0, T, N+1 );
	if ( integrator->setGrid(ocpGrid, numSteps) != SUCCESSFUL_RETURN ) {
		return RET_UNABLE_TO_EXPORT_CODE;
	}

	integrator->setup( );
	
	if( hasOutputs() ) {
		uint i;

		std::vector<Grid> newGrids_;
		if( !referenceProvided ) _refOutputFiles.clear();
		_outputFiles.clear();
		num_meas.clear();
		for( i = 0; i < outputGrids.size(); i++ ) {
			uint numOuts = (int) ceil((double)outputGrids[i].getNumIntervals()/((double) N) - 10.0*EPS);
			Grid nextGrid( 0.0, 1.0, numOuts + 1 );
			newGrids_.push_back( nextGrid );

			if( !referenceProvided ) _refOutputFiles.push_back( (String)"refOutput" << i << ".txt" );
			_outputFiles.push_back( (String)"output" << i << ".txt" );
			num_meas.push_back( numOuts );
		}
		
		if( outputExpressions.size() > 0 ) {
			integrator->setupOutput( newGrids_, outputExpressions );
		}
		else {
			if( colInd_outputs.size() == 0 ) {
				integrator->setupOutput( newGrids_, outputNames, diffs_outputNames, dim_outputs );
			}
			else {
				integrator->setupOutput( newGrids_, outputNames, diffs_outputNames, dim_outputs, getOutputDependencies() );
			}
		}
	}
	
	if( !integrator->hasEquidistantGrid() ) return ACADOERROR( RET_INVALID_OPTION );
	
	setStatus( BS_READY );

	return SUCCESSFUL_RETURN;
}


std::vector<Matrix> SIMexport::getOutputDependencies( ) {
	std::vector<Matrix> outputDependencies;
	for( int i = 0; i < outputNames.size(); i++ ) {
		Vector colIndV = colInd_outputs[i];
		Vector rowPtrV = rowPtr_outputs[i];

		Matrix dependencyMat = zeros( dim_outputs[i],NX+NXA+NU+NDX );
		int index = 1;
		for( int j = 0; j < dim_outputs[i]; j++ ) {
			int upper = rowPtrV(j+1);
			for( int k = rowPtrV(j); k < upper; k++ ) {
				dependencyMat(j,colIndV(k-1)-1) = index++;
			}
		}

		outputDependencies.push_back( dependencyMat );
	}
	return outputDependencies;
}


returnValue SIMexport::checkConsistency( ) const
{
	// Number of differential state derivatives must be either zero or equal to the number of differential states:
	if( NDX > 0 && NDX != NX ) {
		return ACADOERROR( RET_INVALID_OPTION );
	}

	// consistency checks:
	// only time-continuous DAEs without parameter and disturbances supported!
	if ( f.isDiscretized( ) == BT_TRUE )
		return ACADOERROR( RET_NO_DISCRETE_ODE_FOR_CODE_EXPORT );
	
	if ( ( f.getNUI( ) > 0 ) || 
		 /*( f.getNP( ) > 0 ) ||*/ ( f.getNPI( ) > 0 ) || ( f.getNW( ) > 0 ) )
		return ACADOERROR( RET_ONLY_STATES_AND_CONTROLS_FOR_CODE_EXPORT );

	// only equidistant evaluation grids supported!

	return SUCCESSFUL_RETURN;
}



returnValue SIMexport::collectDataDeclarations(	ExportStatementBlock& declarations,
												ExportStruct dataStruct
												) const
{
	if ( integrator->getDataDeclarations( declarations,dataStruct ) != SUCCESSFUL_RETURN )
		return RET_UNABLE_TO_EXPORT_CODE;

	return SUCCESSFUL_RETURN;
}


returnValue SIMexport::collectFunctionDeclarations(	ExportStatementBlock& declarations
													) const
{
	if ( integrator->getFunctionDeclarations( declarations ) != SUCCESSFUL_RETURN )
		return RET_UNABLE_TO_EXPORT_CODE;

	return SUCCESSFUL_RETURN;
}


returnValue SIMexport::exportTest(	const String& _dirName,
									const String& _fileName,
									const String& _resultsFile,
									const std::vector<String>& outputFiles,
									const BooleanType& TIMING,
									const uint jumpReference
											) const
{
	int i;
	
	if( outputFiles.size() != outputGrids.size() || (outputFiles.size() != outputExpressions.size() && outputFiles.size() != outputNames.size()) ) {
		return ACADOERROR( RET_INVALID_OPTION );
	}

    String fileName( _dirName );
    fileName << "/" << _fileName;

	ExportFile main( fileName,"acado.h" );
	main.addLinebreak( 2 );

	main.addComment( "SOME CONVENIENT DEFINTIONS:" );
	main.addComment( "---------------------------------------------------------------" );
	main.addStatement( (String)"   #define JUMP           " << jumpReference  << "      /* jump for the output reference    */\n" );
	main.addStatement( (String)"   #define h           " << T/N  << "      /* length of one simulation interval    */\n" );
	if( TIMING == BT_TRUE ) main.addStatement( (String)"   #define STEPS_TIMING   " << timingSteps << "      /* number of steps for the timing */\n" );
	if( TIMING == BT_TRUE ) main.addStatement( (String)"   #define CALLS_TIMING   " << timingCalls << "      /* number of calls for the timing */\n" );
	main.addStatement( "   #define RESULTS_NAME	  \"" << _resultsFile << "\"\n" );
	for( i = 0; i < (int)outputGrids.size(); i++ ) {
		main.addStatement( (String)"   #define OUTPUT" << i << "_NAME	  \"" << outputFiles[i] << "\"\n" );
	}
	main.addStatement( "   #define CONTROLS_NAME  \"" << _controls << "\"\n" );
	main.addStatement( "   #define INIT_NAME	  \"" << _initStates << "\"\n" );
	main.addComment( "---------------------------------------------------------------" );
	main.addLinebreak( 2 );
	main.addComment( "GLOBAL VARIABLES FOR THE ACADO REAL-TIME ALGORITHM:" );
	main.addComment( "---------------------------------------------------" );
	main.addStatement( "   ACADOworkspace acadoWorkspace;\n" );
	main.addStatement( "   ACADOvariables acadoVariables;\n" );
	main.addLinebreak( );

    main.addLinebreak( 2 );
	main.addComment( "A TEMPLATE FOR TESTING THE INTEGRATOR:" );
    main.addComment( "----------------------------------------------------" );
    main.addStatement( "int main(){\n" );
    main.addLinebreak( );
    main.addComment( 3,"INTRODUCE AUXILIARY VAIRABLES:" );
    main.addComment( 3,"------------------------------" );
    main.addStatement( "      FILE *file, *controls, *initStates;\n" );
    for( i = 0; i < (int)outputGrids.size(); i++ ) {
		main.addStatement( (String)"      FILE *output" << i << ";\n" );
	}
    main.addStatement( "      int i,j,k,nil;\n" );
    for( i = 0; i < (int)outputGrids.size(); i++ ) {
		main.addStatement( (String)"      const int dimOut" << i << " = NOUT[" << i << "]*(1+ACADO_NX+ACADO_NU);\n" );
	}
    main.addStatement( "      real_t x[(ACADO_NX+ACADO_NXA)*(1+ACADO_NX+ACADO_NU)+ACADO_NU];\n" );
    for( i = 0; i < (int)outputGrids.size(); i++ ) {
		main.addStatement( (String)"      real_t out" << i << "[NMEAS[" << i << "]*dimOut" << i << "];\n" );
	}
    main.addStatement( "      real_t u[ACADO_NU];\n" );
    if( NXA > 0 ) main.addStatement( "      real_t norm;\n" );
    for( i = 0; i < (int)outputGrids.size(); i++ ) {
		main.addStatement( (String)"      real_t step" << i << " = h/NMEAS[" << i << "];\n" );
	}
    if( TIMING == BT_TRUE ) {
		main.addStatement( "      struct timeval theclock;\n" );
		main.addStatement( "      real_t start, end, time;\n" );
		main.addStatement( "      real_t xT[(ACADO_NX+ACADO_NXA)*(1+ACADO_NX+ACADO_NU)+ACADO_NU];\n" );
	}
    main.addStatement( "      const ACADOworkspace_ nullWork2 = {0};\n" );
    main.addStatement( "      const ACADOvariables_ nullVars2 = {0};\n" );
    main.addStatement( " 	  acadoWorkspace = nullWork2;\n" );
    main.addStatement( " 	  acadoVariables = nullVars2;\n" );
    main.addLinebreak( 2 );

    main.addComment( 3,"INITIALIZATION:" );
    main.addComment( 3,"----------------------------------------" );
    main.addStatement( "      initStates = fopen( INIT_NAME,\"r\" );\n" );
    main.addStatement( "      for( j = 0; j < ACADO_NX+ACADO_NXA; j++) {\n" );
    main.addStatement( "      		nil = fscanf( initStates, \"%lf\", &x[j] );\n" );
    main.addStatement( "      }\n" );
    main.addStatement( "      fclose( initStates );\n" );
    main.addLinebreak( 1 );
    main.addStatement( "      for( i = 0; i < (ACADO_NX+ACADO_NXA); i++ ) {\n" );
    main.addStatement( "      		for( j = 0; j < ACADO_NX; j++ ) {\n" );
    main.addStatement( "      			if( i == j ) {\n" );
    main.addStatement( "      				x[ACADO_NX+ACADO_NXA+i*ACADO_NX+j] = 1;\n" );
    main.addStatement( "      			} else {\n" );
    main.addStatement( "      				x[ACADO_NX+ACADO_NXA+i*ACADO_NX+j] = 0;\n" );
    main.addStatement( "      			}\n" );
    main.addStatement( "      		}\n" );
    main.addStatement( "      }\n" );
    main.addStatement( "      for( i = 0; i < (ACADO_NX+ACADO_NXA); i++ ) {\n" );
    main.addStatement( "      		for( j = 0; j < ACADO_NU; j++ ) {\n" );
    main.addStatement( "      			x[ACADO_NX+ACADO_NXA+(ACADO_NX+ACADO_NXA)*ACADO_NX+i*ACADO_NU+j] = 0;\n" );
    main.addStatement( "      		}\n" );
    main.addStatement( "      }\n" );
    main.addStatement( "      for( i = 0; i < ACADO_NU; i++ ) {\n" );
    main.addStatement( "      		x[(ACADO_NX+ACADO_NXA)*(1+ACADO_NX+ACADO_NU)+i] = 0;\n" );
    main.addStatement( "      }\n" );
    main.addLinebreak( 1 );

    main.addComment( 3,"RUN INTEGRATOR:" );
    main.addComment( 3,"----------------------------------------" );
    main.addStatement( "      file = fopen(RESULTS_NAME,\"w\");\n" );
    for( i = 0; i < (int)outputGrids.size(); i++ ) {
		main.addStatement( (String)"      output" << i << " = fopen(OUTPUT" << i << "_NAME,\"w\");\n" );
	}
    main.addStatement( "      controls = fopen(CONTROLS_NAME,\"r\");\n" );
    main.addStatement( "      for( i = 0; i < ACADO_N; i++ ) {\n" );
    main.addStatement( "      		fprintf(file, \"%.16f \", i*h);\n" );
    main.addStatement( "      		for( j = 0; j < (ACADO_NX+ACADO_NXA)*(1+ACADO_NX+ACADO_NU); j++) {\n" );
    main.addStatement( "      			fprintf(file, \"%.16f \", x[j]);\n" );
    main.addStatement( "      		}\n" );
    main.addStatement( "      		fprintf(file, \"\\n\");\n" );
    main.addLinebreak( );
    main.addStatement( "      		nil = fscanf( controls, \"%lf\", &x[(ACADO_NX+ACADO_NXA)*(1+ACADO_NX+ACADO_NU)] );\n" );
    main.addStatement( "      		for( j = 0; j < ACADO_NU; j++) {\n" );
    main.addStatement( "      			nil = fscanf( controls, \"%lf\", &x[(ACADO_NX+ACADO_NXA)*(1+ACADO_NX+ACADO_NU)+j] );\n" );
    main.addStatement( "      		}\n" );
    main.addLinebreak( );
    if( TIMING == BT_TRUE ) {
		main.addStatement( "      		if( i == 0 ) {\n" );
		main.addStatement( "      			for( j=0; j < (ACADO_NX+ACADO_NXA)*(1+ACADO_NX+ACADO_NU)+ACADO_NU; j++ ) {\n" );
		main.addStatement( "      				xT[j] = x[j];\n" );
		main.addStatement( "     			}\n" );
		main.addStatement( "      		}\n" );
	}
    main.addLinebreak( );
    String integrate( "      		integrate( x" );
    for( i = 0; i < (int)outputGrids.size(); i++ ) {
		integrate << ", out" << i;
	}
    main.addStatement( integrate << " );\n" );
    main.addLinebreak( );
    for( i = 0; i < (int)outputGrids.size(); i++ ) {
		main.addStatement( (String)"      		for( j = JUMP-1; j < NMEAS[" << i << "]; j=j+JUMP ) {\n" );
		main.addStatement( (String)"      			fprintf(output" << i << ", \"%.16f \", i*h+(j+1)*step" << i << ");\n" );
		main.addStatement( (String)"      			for( k = 0; k < dimOut" << i << "; k++ ) {\n" );
		main.addStatement( (String)"      				fprintf(output" << i << ", \"%.16f \", out" << i << "[j*dimOut" << i << "+k]);\n" );
		main.addStatement( "      			}\n" );
		main.addStatement( (String)"      			fprintf(output" << i << ", \"%s\", \"\\n\");\n" );
		main.addStatement( "      		}\n" );
	}
    main.addStatement( "      }\n" );
    main.addStatement( "      fprintf(file, \"%.16f \", ACADO_N*h);\n" );
    main.addStatement( "      for( j = 0; j < (ACADO_NX+ACADO_NXA)*(1+ACADO_NX+ACADO_NU); j++) {\n" );
    main.addStatement( "      		fprintf(file, \"%.16f \", x[j]);\n" );
    main.addStatement( "      }\n" );
    main.addStatement( "      fprintf(file, \"\\n\");\n" );
    main.addLinebreak( );
    main.addStatement( "      fclose(file);\n" );
    for( i = 0; i < (int)outputGrids.size(); i++ ) {
		main.addStatement( (String)"      fclose(output" << i << ");\n" );
	}
    main.addStatement( "      fclose(controls);\n" );
    if( TIMING == BT_TRUE ) {
		main.addStatement( "      gettimeofday( &theclock,0 );\n" );
		main.addStatement( "      start = 1.0*theclock.tv_sec + 1.0e-6*theclock.tv_usec;\n" );
		main.addStatement( "      for( i=0; i < CALLS_TIMING; i++ ) {\n" );
		main.addStatement( "      		for( j=0; j < (ACADO_NX+ACADO_NXA); j++ ) {\n" );
		main.addStatement( "      			x[j] = xT[j];\n" );
		main.addStatement( "      		}\n" );
		integrate = String( "      		integrate( x" );
		for( i = 0; i < (int)outputGrids.size(); i++ ) {
			integrate << ", out" << i;
		}
		main.addStatement( integrate << " );\n" );
		main.addStatement( "      }\n" );
		main.addStatement( "      gettimeofday( &theclock,0 );\n" );
		main.addStatement( "      end = 1.0*theclock.tv_sec + 1.0e-6*theclock.tv_usec;\n" );
		main.addStatement( "      time = (end-start);\n" );
		main.addLinebreak( );
		main.addStatement( "      printf( \"\\n\\n AVERAGE DURATION OF ONE INTEGRATION STEP:   %.3g Î¼s\\n\\n\", 1e6*time/STEPS_TIMING );\n" );
	}
    main.addLinebreak( );
	main.addStatement( "      return 0;\n" );
	main.addStatement( "}\n" );
    
    
	return main.exportCode( );
}


returnValue SIMexport::exportEvaluation(	const String& _dirName,
											const String& _fileName
											) const
{
	int i;
	
	Vector nMeasV( outputGrids.size() );
	Vector nOutV( outputGrids.size() );
	for( i = 0; i < (int)outputGrids.size(); i++ ) {
		nMeasV(i) = num_meas[i];
		nOutV(i) = getDimOutput( (uint) i );
	}

    String fileName( _dirName );
    fileName << "/" << _fileName;

	ExportFile main( fileName,"acado.h" );
	
    main.addLinebreak( 2 );
	main.addComment( "SOME CONVENIENT DEFINTIONS:" );
	main.addComment( "---------------------------------------------------------------" );
	main.addStatement( (String)"   #define h           " << T/N  << "      /* length of one simulation interval   */\n" );
	main.addStatement( "   #define RESULTS_NAME	  \"" << _results << "\"\n" );
	for( i = 0; i < (int)outputGrids.size(); i++ ) {
		main.addStatement( (String)"   #define OUTPUT" << i << "_NAME	  \"" << _outputFiles[i] << "\"\n" );
		main.addStatement( (String)"   #define REF_OUTPUT" << i << "_NAME	  \"" << _refOutputFiles[i] << "\"\n" );
	}
	main.addStatement( "   #define REF_NAME  \"" << _ref << "\"\n" );
	main.addComment( "---------------------------------------------------------------" );
	main.addLinebreak( 2 );
	main.addComment( "GLOBAL VARIABLES FOR THE ACADO REAL-TIME ALGORITHM:" );
	main.addComment( "---------------------------------------------------" );
	main.addStatement( "   ACADOworkspace acadoWorkspace;\n" );
	main.addStatement( "   ACADOvariables acadoVariables;\n" );
	main.addLinebreak( );

    main.addLinebreak( 2 );
	main.addComment( "A TEMPLATE FOR TESTING THE INTEGRATOR:" );
    main.addComment( "----------------------------------------------------" );
    main.addStatement( "int main(){\n" );
    main.addLinebreak( );
    main.addComment( 3,"INTRODUCE AUXILIARY VAIRABLES:" );
    main.addComment( 3,"------------------------------" );
    main.addStatement( "      FILE *file, *ref;\n" );
    for( i = 0; i < (int)outputGrids.size(); i++ ) {
		main.addStatement( (String)"      FILE *output" << i << ";\n" );
		main.addStatement( (String)"      FILE *refOutput" << i << ";\n" );
	}
    main.addStatement( "      int i, j, nil;\n" );
    main.addStatement( "      real_t x[ACADO_NX+ACADO_NXA];\n" );
    main.addStatement( "      real_t xRef[ACADO_NX+ACADO_NXA];\n" );
    for( i = 0; i < (int)outputGrids.size(); i++ ) {
		main.addStatement( (String)"      real_t step" << i << " = h/NMEAS[" << i << "];\n" );
		main.addStatement( (String)"      real_t out" << i << "[NMEAS[" << i << "]*NOUT[" << i << "]];\n" );
		main.addStatement( (String)"      real_t refOut" << i << "[NMEAS[" << i << "]*NOUT[" << i << "]];\n" );
	}
    main.addStatement( "      real_t maxErr, meanErr, maxErrX, meanErrX, maxErrXA, meanErrXA, temp;\n" );
    main.addStatement( "      const ACADOworkspace_ nullWork2 = {0};\n" );
    main.addStatement( "      const ACADOvariables_ nullVars2 = {0};\n" );
    main.addStatement( " 	  acadoWorkspace = nullWork2;\n" );
    main.addStatement( " 	  acadoVariables = nullVars2;\n" );
    main.addLinebreak( 2 );

    main.addComment( 3,"START EVALUATION RESULTS:" );
    main.addComment( 3,"----------------------------------------" );
	main.addStatement( "      meanErrX = 0;\n" );
	main.addStatement( "      meanErrXA = 0;\n" );
    main.addStatement( "      file = fopen(RESULTS_NAME,\"r\");\n" );
    main.addStatement( "      ref = fopen(REF_NAME,\"r\");\n" );
    main.addStatement( "      for( i = 0; i < (ACADO_NX+ACADO_NXA)*(1+ACADO_NX+ACADO_NU)+1; i++ ) {\n" );
    main.addStatement( "      		nil = fscanf( file, \"%lf\", &temp );\n" );
    main.addStatement( "      		nil = fscanf( ref, \"%lf\", &temp );\n" );
    main.addStatement( "      }\n" );
	main.addStatement( "      printf( \" STATES:\\n\" );\n" );
    main.addLinebreak( );
    main.addStatement( "      for( i = 1; i <= ACADO_N; i++ ) {\n" );
    main.addStatement( "      		nil = fscanf( file, \"%lf\", &temp );\n" );
    main.addStatement( "      		nil = fscanf( ref, \"%lf\", &temp );\n" );
    main.addLinebreak( );
    main.addStatement( "      		maxErrX = 0;\n" );
    main.addStatement( "      		for( j = 0; j < ACADO_NX; j++ ) {\n" );
    main.addStatement( "      			nil = fscanf( file, \"%lf\", &x[j] );\n" );
    main.addStatement( "      			nil = fscanf( ref, \"%lf\", &xRef[j] );\n" );
    main.addStatement( "      			temp = fabs(x[j] - xRef[j])/fabs(xRef[j]);\n" );
    main.addStatement( "      			if( temp > maxErrX ) maxErrX = temp;\n" );
    main.addStatement( "      			if( isnan(x[j]) ) maxErrX = sqrt(-1);\n" );
    main.addStatement( "      		}\n" );
    main.addLinebreak( );
    main.addStatement( "      		maxErrXA = 0;\n" );
    main.addStatement( "      		for( j = 0; j < ACADO_NXA; j++ ) {\n" );
    main.addStatement( "      			nil = fscanf( file, \"%lf\", &x[ACADO_NX+j] );\n" );
    main.addStatement( "      			nil = fscanf( ref, \"%lf\", &xRef[ACADO_NX+j] );\n" );
    main.addStatement( "      			temp = fabs(x[ACADO_NX+j] - xRef[ACADO_NX+j])/fabs(xRef[ACADO_NX+j]);\n" );
    main.addStatement( "      			if( temp > maxErrXA ) maxErrXA = temp;\n" );
    main.addStatement( "      			if( isnan(x[ACADO_NX+j]) ) maxErrXA = sqrt(-1);\n" );
    main.addStatement( "      		}\n" );
    main.addLinebreak( );
    if( PRINT_DETAILS && NXA > 0 ) {
    	main.addStatement( "      		printf( \"MAX ERROR AT %.3f s:   %.4e   %.4e \\n\", i*h, maxErrX, maxErrXA );\n" );
    }
    else if( PRINT_DETAILS ) {
    	main.addStatement( "      		printf( \"MAX ERROR AT %.3f s:   %.4e \\n\", i*h, maxErrX );\n" );
    }
    main.addStatement( "			meanErrX += maxErrX;\n" );
    main.addStatement( "			meanErrXA += maxErrXA;\n" );
    main.addLinebreak( );
    main.addStatement( "      		for( j = 0; j < (ACADO_NX+ACADO_NXA)*(ACADO_NX+ACADO_NU); j++ ) {\n" );
    main.addStatement( "      			nil = fscanf( file, \"%lf\", &temp );\n" );
    main.addStatement( "      			nil = fscanf( ref, \"%lf\", &temp );\n" );
    main.addStatement( "      		}\n" );
    main.addStatement( "      }\n" );
    main.addStatement( "	  meanErrX = meanErrX/ACADO_N;\n" );
    main.addStatement( "	  meanErrXA = meanErrXA/ACADO_N;\n" );
    if( PRINT_DETAILS ) main.addStatement( "      printf( \"\\n\" );\n" );
    if( NXA > 0 ) {
    	main.addStatement( "      printf( \"TOTAL MEAN ERROR:   %.4e   %.4e \\n\", meanErrX, meanErrXA );\n" );
    }
    else {
    	main.addStatement( "      printf( \"TOTAL MEAN ERROR:   %.4e \\n\", meanErrX );\n" );
    }
    main.addStatement( "      printf( \"\\n\\n\" );\n" );
    for( i = 0; i < (int)outputGrids.size(); i++ ) {
		main.addLinebreak( );
		main.addStatement( (String)"      printf( \" OUTPUT FUNCTION " << (i+1) << ":\\n\" );\n" );
		main.addStatement( (String)"      meanErr = 0;\n" );
		main.addStatement( (String)"      output" << i << " = fopen(OUTPUT" << i << "_NAME,\"r\");\n" );
		main.addStatement( (String)"      refOutput" << i << " = fopen(REF_OUTPUT" << i << "_NAME,\"r\");\n" );
		main.addLinebreak( );
		main.addStatement( (String)"      for( i = 1; i <= ACADO_N*NMEAS[" << i << "]; i++ ) {\n" );
		main.addStatement( (String)"      		nil = fscanf( output" << i << ", \"%lf\", &temp );\n" );
		main.addStatement( (String)"      		nil = fscanf( refOutput" << i << ", \"%lf\", &temp );\n" );
		main.addLinebreak( );
		main.addStatement( "      		maxErr = 0;\n" );
		main.addStatement( (String)"      		for( j = 0; j < NOUT[" << i << "]; j++ ) {\n" );
		main.addStatement( (String)"      			nil = fscanf( output" << i << ", \"%lf\", &out" << i << "[j] );\n" );
		main.addStatement( (String)"      			nil = fscanf( refOutput" << i << ", \"%lf\", &refOut" << i << "[j] );\n" );
		main.addStatement( (String)"      			temp = fabs(out" << i << "[j] - refOut" << i << "[j])/fabs(refOut" << i << "[j]);\n" );
		main.addStatement( "      			if( temp > maxErr ) maxErr = temp;\n" );
		main.addStatement( (String)"      			if( isnan(out" << i << "[j]) ) maxErr = sqrt(-1);\n" );
		main.addStatement( "      		}\n" );
		main.addLinebreak( );
		if( PRINT_DETAILS ) main.addStatement( (String)"      		printf( \"MAX ERROR AT %.3f s:   %.4e \\n\", i*step" << i << ", maxErr );\n" );
		main.addStatement( "      		meanErr += maxErr;\n" );
		main.addLinebreak( );
		main.addStatement( (String)"      		for( j = 0; j < NOUT[" << i << "]*(ACADO_NX+ACADO_NU); j++ ) {\n" );
		main.addStatement( (String)"      			nil = fscanf( output" << i << ", \"%lf\", &temp );\n" );
		main.addStatement( (String)"      			nil = fscanf( refOutput" << i << ", \"%lf\", &temp );\n" );
		main.addStatement( "      		}\n" );
		main.addStatement( "      }\n" );
		main.addStatement( (String)"	  meanErr = meanErr/(ACADO_N*NMEAS[" << i << "]);\n" );
		if( PRINT_DETAILS ) main.addStatement( "      printf( \"\\n\" );\n" );
		main.addStatement( "      printf( \"TOTAL MEAN ERROR:   %.4e \\n\", meanErr );\n" );
		main.addStatement( "      printf( \"\\n\\n\" );\n" );
	}
    main.addLinebreak( );
    main.addStatement( "      return 0;\n" );
    main.addStatement( "}\n" );
    
	return main.exportCode( );
}



returnValue SIMexport::exportAndRun(	const String& dirName,
							const String& initStates,
							const String& controls,
							const String& results,
							const String& ref
										)
{
	_initStates = initStates;
	_controls = controls;
	_results = results;
	_ref = ref;
	uint i;
	Vector meas( (uint)outputGrids.size() );
	Vector measRef( (uint)outputGrids.size() );
	for( i = 0; i < outputGrids.size(); i++ ) {
		meas(i) = (double)outputGrids[i].getNumIntervals();
		measRef(i) = (double)outputGrids[i].getNumIntervals()*factorRef;
	}
	
	int numSteps;
    get( NUM_INTEGRATOR_STEPS, numSteps );
	timingCalls = (uint) ceil((double)(timingSteps*N)/((double) numSteps) - 10.0*EPS);
	timingSteps = (uint) ceil((double)timingCalls*((double) numSteps/((double) N)) - 10.0*EPS);
    
    if( !referenceProvided ) {
	    // REFERENCE:
	    setMeasurements( measRef );
		set( NUM_INTEGRATOR_STEPS,  (int)factorRef*numSteps );
		exportCode(	dirName );
		exportTest(	dirName, String( "test.c" ), _ref, _refOutputFiles, BT_FALSE, factorRef );
		executeTest( dirName );
	}
    
    // THE INTEGRATOR:
    setMeasurements( meas );
	set( NUM_INTEGRATOR_STEPS,  numSteps );
	exportCode(	dirName );
	if(timingSteps > 0 && timingCalls > 0) 	exportTest(	dirName, String( "test.c" ), _results, _outputFiles, BT_TRUE, 1 );
	else 									exportTest(	dirName, String( "test.c" ), _results, _outputFiles, BT_FALSE, 1 );
	executeTest( dirName );
	
	// THE EVALUATION:
	int nil;
	nil = system( (String(dirName) << "/./compare").getName() );
	
	return SUCCESSFUL_RETURN;
}


returnValue SIMexport::exportMakefile(	const String& _dirName,
										const String& _fileName,
										const String& _realString,
										const String& _intString,
										int _precision
										) const
{
	String fileName( _dirName );
	fileName << "/" << _fileName;

	ExportFile Makefile( fileName,"", _realString,_intString,_precision,"##" );

	Makefile.addStatement( "LDLIBS = -lm \n" );
	Makefile.addStatement( "CXXFLAGS = -O3 -finline-functions -I. \n" );
	Makefile.addStatement( "CFLAGS = -O3\n" );
	Makefile.addStatement( "CC     = g++\n" );
	Makefile.addLinebreak( );
	Makefile.addStatement( "OBJECTS = \\\n" );
	Makefile.addStatement( "\tintegrator.o \\\n" );
	if( !EXPORT_RHS ) {
		Makefile.addStatement( (String)"\t" << externModel << ".o \n" );
	}
	Makefile.addLinebreak( 2 );
	Makefile.addStatement( ".PHONY: all\n" );
	Makefile.addStatement( "all: test compare \n" );
	Makefile.addLinebreak( );
	Makefile.addStatement( "test: ${OBJECTS} test.o\n" );
	Makefile.addLinebreak( );
	Makefile.addStatement( "compare: ${OBJECTS} compare.o\n" );
	Makefile.addLinebreak( );
	Makefile.addStatement( "integrator.o          : acado.h\n" );
	Makefile.addStatement( "test.o                : acado.h\n" );
	Makefile.addStatement( "compare.o             : acado.h\n" );
	if( !EXPORT_RHS ) {
		Makefile.addStatement( (String)externModel << ".o             : acado.h\n" );
	}
	Makefile.addLinebreak( );
	Makefile.addStatement( "${OBJECTS} : \n" );
	Makefile.addLinebreak( );
	Makefile.addStatement( ".PHONY : clean\n" );
	Makefile.addStatement( "clean :\n" );
	Makefile.addStatement( "\t-rm -f *.o *.a test\n" );
	Makefile.addLinebreak( );

	return Makefile.exportCode( );
}


returnValue SIMexport::setReference( const String& reference, const std::vector<String>& outputReference ) {
	if( hasOutputs() && outputReference.size() == 0 ) {
		referenceProvided = BT_FALSE;
		return RET_UNABLE_TO_EXPORT_CODE;
	}
	referenceProvided = BT_TRUE;
	_ref = reference;
	if( outputReference.size() > 0 ) _refOutputFiles = outputReference;
	
	return SUCCESSFUL_RETURN;
}


returnValue SIMexport::setTimingSteps( uint _timingSteps ) {
	timingSteps = _timingSteps;
	
	return SUCCESSFUL_RETURN;
}


returnValue SIMexport::printDetails( BooleanType details ) {
	PRINT_DETAILS = details;
	
	return SUCCESSFUL_RETURN;
}


returnValue SIMexport::executeTest( const String& _dirName ) {
	//sleep(2); does not compile on windows!!
	int nil;
	nil = system( ((String) String("make clean -s -C ") << _dirName).getName() );
	nil = system( ((String) String("make -s -C ") << _dirName).getName() );
	nil = system( (String(_dirName) << "/./test").getName() );
	
	return SUCCESSFUL_RETURN;
}


uint SIMexport::getDimOutput( uint index ) const {
	if( outputExpressions.size() > 0 ) {
		return outputExpressions[index].getDim();
	}
	else {
		return dim_outputs[index];
	}
}


CLOSE_NAMESPACE_ACADO

// end of file.
