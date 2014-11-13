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
 *    \file src/code_generation/sim_export.cpp
 *    \author Rien Quirynen
 *    \date 2012
 */

#include <acado/code_generation/sim_export.hpp>
#include <acado/code_generation/integrators/export_matlab_integrator.hpp>
#include <acado/code_generation/integrators/integrator_generation.hpp>
#include <acado/code_generation/export_common_header.hpp>
#include <acado/code_generation/templates/templates.hpp>
#include <acado/code_generation/integrators/export_auxiliary_sim_functions.hpp>
#include <acado/code_generation/export_algorithm_factory.hpp>

#ifdef WIN32
#include <windows.h>
#endif

using namespace std;

BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

SIMexport::SIMexport( const uint simIntervals, const double totalTime ) : ExportModule( )
{
	setN(simIntervals);
	T = totalTime;
	integrator  = 0;
	timingSteps = 100;
	
	_initStates = "initStates.txt";
	_controls = "controls.txt";
	_results = "results.txt";
	_ref = "ref.txt";
	referenceProvided = false;
	PRINT_DETAILS = true;

	timingCalls = 0;

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



returnValue SIMexport::exportCode(	const std::string& dirName,
									const std::string& _realString,
									const std::string& _intString,
									int _precision
									)
{
	if (!modelDimensionsSet() && !exportRhs()) return ACADOERROR( RET_UNABLE_TO_EXPORT_CODE );
	set( QP_SOLVER, QP_NONE );

	string moduleName;
	get(CG_MODULE_NAME, moduleName);

	//
	// Create the export folders
	//

	set(CG_EXPORT_FOLDER_NAME, dirName);

	returnValue dirStatus = acadoCreateFolder( dirName );
	if (dirStatus != SUCCESSFUL_RETURN)
		return dirStatus;

	if ( setup( ) != SUCCESSFUL_RETURN )
		return ACADOERROR( RET_UNABLE_TO_EXPORT_CODE );

	int printLevel;
	get( PRINTLEVEL,printLevel );

	// export mandatory source code files
	if ( exportAcadoHeader( dirName,commonHeaderName,_realString,_intString,_precision ) != SUCCESSFUL_RETURN )
		return ACADOERROR( RET_UNABLE_TO_EXPORT_CODE );

	if( integrator != 0 )
	{
		std::string fileName( dirName );
		fileName += "/acado_integrator.c";

		ExportFile integratorFile( fileName,commonHeaderName,_realString,_intString,_precision );
		integrator->getCode( integratorFile );
		
		if ( integratorFile.exportCode( ) != SUCCESSFUL_RETURN )
			return ACADOERROR( RET_UNABLE_TO_EXPORT_CODE );

		int sensGen;
		get( DYNAMIC_SENSITIVITY, sensGen );
		int measGrid;
		get( MEASUREMENT_GRID, measGrid );
		int generateMatlabInterface;
		get( GENERATE_MATLAB_INTERFACE, generateMatlabInterface );
		int debugMode;
		get( INTEGRATOR_DEBUG_MODE, debugMode );
		if ( (bool)generateMatlabInterface == true ) {
			std::string integrateInterface =  dirName;
			integrateInterface += "/acado_integrate.c";
			ExportMatlabIntegrator exportMexFun( INTEGRATOR_MEX_TEMPLATE, integrateInterface, commonHeaderName,_realString,_intString,_precision );
			exportMexFun.configure((ExportSensitivityType)sensGen, (MeasurementGrid)measGrid == ONLINE_GRID, (bool)debugMode, timingCalls, ((RungeKuttaExport*)integrator)->getNumStages());
			exportMexFun.exportCode();

			integrateInterface = dirName + std::string("/make_acado_integrator.m");
			acadoCopyTemplateFile(MAKE_MEX_INTEGRATOR, integrateInterface, "%", true);

			// NOT SUPPORTED ANYMORE:
//			std::string rhsInterface = dirName;
//			rhsInterface += "/acado_rhs.c";
//			ExportMatlabRhs exportMexFun2( RHS_MEX_TEMPLATE, rhsInterface, commonHeaderName,_realString,_intString,_precision );
//			exportMexFun2.configure(integrator->getNameFullRHS());
//			exportMexFun2.exportCode();
//
//			rhsInterface = dirName + std::string("/make_acado_model.m");
//			acadoCopyTempateFile(MAKE_MEX_MODEL, rhsInterface, "%", true);
		}
	}


	// export template for main file, if desired
	if ( (PrintLevel)printLevel >= HIGH ) 
		cout <<  "--> Exporting remaining files... ";

	// Export auxiliary functions, always
	//
	ExportAuxiliarySimFunctions eaf(
			dirName + string("/") + moduleName + "_auxiliary_sim_functions.h",
			dirName + string("/") + moduleName + "_auxiliary_sim_functions.c",
			moduleName );
	eaf.configure();
	eaf.exportCode();

	// export a basic Makefile, if desired
	int generateMakeFile;
	get( GENERATE_MAKE_FILE,generateMakeFile );
	if ( (bool)generateMakeFile == true )
		if ( exportMakefile( dirName,"Makefile",_realString,_intString,_precision ) != SUCCESSFUL_RETURN )
			return ACADOERROR( RET_UNABLE_TO_EXPORT_CODE );
			
	// export the evaluation file
	int exportTestFile;
	get( GENERATE_TEST_FILE, exportTestFile );
	if ( exportTestFile && exportEvaluation( dirName, std::string( "acado_compare.c" ) ) != SUCCESSFUL_RETURN )
		return ACADOERROR( RET_UNABLE_TO_EXPORT_CODE );

	if ( (PrintLevel)printLevel >= HIGH ) 
		cout <<  "done.\n";

	if ( (PrintLevel)printLevel > NONE )
		ACADOINFO( RET_CODE_EXPORT_SUCCESSFUL );

    return SUCCESSFUL_RETURN;
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

 	//
 	// Set common header name
 	//
	string moduleName;
	get(CG_MODULE_NAME, moduleName);
 	commonHeaderName = moduleName + "_common.h";

	int numSteps;
    get( NUM_INTEGRATOR_STEPS, numSteps );

	if ( numSteps <= 0 )
		return ACADOERROR( RET_INVALID_OPTION );

	int integratorType;
	get( INTEGRATOR_TYPE, integratorType );

	if ( integrator != NULL )
		delete integrator;

	integrator = IntegratorExportFactory::instance().createAlgorithm(this, commonHeaderName, static_cast<ExportIntegratorType>(integratorType));

	if ( integrator == NULL )
		return ACADOERROR( RET_INVALID_OPTION );

	Grid grid( 0.0, T, modelData.getN()+1 );
	modelData.setIntegrationGrid( grid, numSteps );
	integrator->setModelData( modelData );
	
	if( modelData.hasOutputs() ) {
		uint i;

		std::vector<Grid> newGrids_;
		if( !referenceProvided ) _refOutputFiles.clear();
		_outputFiles.clear();
		for( i = 0; i < modelData.getNumOutputs(); i++ ) {
			if( !referenceProvided ) _refOutputFiles.push_back( (std::string)"refOutput" + toString(i) +  ".txt" );
			_outputFiles.push_back( (std::string)"output" + toString(i) +  ".txt" );
		}
	}

	if( !integrator->equidistantControlGrid() ) return ACADOERROR( RET_INVALID_OPTION );
	
	setStatus( BS_READY );

	return SUCCESSFUL_RETURN;
}


returnValue SIMexport::checkConsistency( ) const
{
	// consistency checks:
	// only time-continuous DAEs without parameter and disturbances supported!
	DifferentialEquation f;
	modelData.getModel(f);
	if ( f.isDiscretized( ) == true )
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


returnValue SIMexport::exportTest(	const std::string& _dirName,
									const std::string& _fileName,
									const std::string& _resultsFile,
									const std::vector<std::string>& outputFiles,
									const bool& TIMING,
									const uint jumpReference
											) const
{
	int i;
	int sensGen;
	get( DYNAMIC_SENSITIVITY, sensGen );
	bool DERIVATIVES = ((ExportSensitivityType) sensGen != NO_SENSITIVITY);
	
	std::vector<Grid> outputGrids;
	std::vector<Expression> outputExpressions;
	std::vector<std::string> outputNames;
	modelData.getOutputGrids(outputGrids);
	modelData.getOutputExpressions(outputExpressions);
	modelData.getNameOutputs(outputNames);
	if( outputFiles.size() != outputGrids.size() || (outputFiles.size() != outputExpressions.size() && outputFiles.size() != outputNames.size()) ) {
		return ACADOERROR( RET_INVALID_OPTION );
	}

    std::string fileName( _dirName );
    fileName += "/" + _fileName;

	ExportFile main( fileName,commonHeaderName );

	main.addStatement( "#include <stdio.h>\n" );
	main << "#include \"acado_auxiliary_sim_functions.h\"\n";
	main.addLinebreak( 1 );
	main.addComment( "SOME CONVENIENT DEFINTIONS:" );
	main.addComment( "---------------------------------------------------------------" );
	main.addStatement( (std::string)"   #define JUMP           " + toString(jumpReference)  + "      /* jump for the output reference    */\n" );
	main.addStatement( (std::string)"   #define h           " + toString(T/modelData.getN())  + "      /* length of one simulation interval    */\n" );
	if( TIMING == true ) main.addStatement( (std::string)"   #define STEPS_TIMING   " + toString(timingSteps) + "      /* number of steps for the timing */\n" );
	if( TIMING == true ) main.addStatement( (std::string)"   #define CALLS_TIMING   " + toString(timingCalls) + "      /* number of calls for the timing */\n" );
	main.addStatement( (std::string)"   #define RESULTS_NAME	  \"" + _resultsFile + "\"\n" );
	for( i = 0; i < (int)outputGrids.size(); i++ ) {
		main.addStatement( (std::string)"   #define OUTPUT" + toString(i) +  "_NAME	  \"" + outputFiles[i] + "\"\n" );
	}
	main.addStatement( (std::string)"   #define CONTROLS_NAME  \"" + _controls + "\"\n" );
	main.addStatement( (std::string)"   #define INIT_NAME	  \"" + _initStates + "\"\n" );
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
		main.addStatement( (std::string)"      FILE *output" + toString(i) +  ";\n" );
	}
    main.addStatement( "      int i,j,k,nil,reset;\n" );
    for( i = 0; i < (int)outputGrids.size(); i++ ) {
    	if( !DERIVATIVES )  main.addStatement( (std::string)"      const int dimOut" + toString(i) +  " = ACADO_NOUT[" + toString(i) +  "];\n" );
    	else  main.addStatement( (std::string)"      const int dimOut" + toString(i) +  " = ACADO_NOUT[" + toString(i) +  "]*(1+ACADO_NX+ACADO_NU);\n" );
	}
    if( !DERIVATIVES )  main.addStatement( "      real_t x[ACADO_NX+ACADO_NXA+ACADO_NU];\n" );
    else  main.addStatement( "      real_t x[(ACADO_NX+ACADO_NXA)*(1+ACADO_NX+ACADO_NU)+ACADO_NU];\n" );

    for( i = 0; i < (int)outputGrids.size(); i++ ) {
		main.addStatement( (std::string)"      real_t out" + toString(i) +  "[ACADO_NMEAS[" + toString(i) +  "]*dimOut" + toString(i) +  "];\n" );
	}
    main.addStatement( "      real_t u[ACADO_NU];\n" );
    if( modelData.getNXA() > 0 ) main.addStatement( "      real_t norm;\n" );
    for( i = 0; i < (int)outputGrids.size(); i++ ) {
		main.addStatement( (std::string)"      real_t step" + toString(i) +  " = h/ACADO_NMEAS[" + toString(i) +  "];\n" );
	}
    if( TIMING == true ) {
		main.addStatement( "      struct timeval theclock;\n" );
		main.addStatement( "      real_t start, end, time;\n" );
		if( !DERIVATIVES )  main.addStatement( "      real_t xT[ACADO_NX+ACADO_NXA+ACADO_NU];\n" );
		else  main.addStatement( "      real_t xT[(ACADO_NX+ACADO_NXA)*(1+ACADO_NX+ACADO_NU)+ACADO_NU];\n" );
	}
    main.addStatement( "      const ACADOworkspace nullWork2 = {0};\n" );
    main.addStatement( " 	  acadoWorkspace = nullWork2;\n" );
    main.addLinebreak( 2 );

    main.addComment( 3,"INITIALIZATION:" );
    main.addComment( 3,"----------------------------------------" );
    main.addStatement( "      initStates = fopen( INIT_NAME,\"r\" );\n" );
    main.addStatement( "      for( j = 0; j < ACADO_NX+ACADO_NXA; j++) {\n" );
    main.addStatement( "      		nil = fscanf( initStates, \"%lf\", &x[j] );\n" );
    main.addStatement( "      }\n" );
    main.addStatement( "      fclose( initStates );\n" );
    main.addLinebreak( 1 );
    if( DERIVATIVES ) {
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
    }
    main.addLinebreak( 1 );
    main.addStatement( " 	  reset = 1;\n" );
    main.addLinebreak( 1 );
    main.addComment( 3,"RUN INTEGRATOR:" );
    main.addComment( 3,"----------------------------------------" );
    main.addStatement( "      file = fopen(RESULTS_NAME,\"w\");\n" );
    for( i = 0; i < (int)outputGrids.size(); i++ ) {
		main.addStatement( (std::string)"      output" + toString(i) +  " = fopen(OUTPUT" + toString(i) +  "_NAME,\"w\");\n" );
	}
    main.addStatement( "      controls = fopen(CONTROLS_NAME,\"r\");\n" );
    main.addStatement( "      for( i = 0; i < ACADO_N; i++ ) {\n" );
    main.addStatement( "      		fprintf(file, \"%.16f \", i*h);\n" );
    if( !DERIVATIVES )  main.addStatement( "      		for( j = 0; j < ACADO_NX+ACADO_NXA; j++) {\n" );
    else  main.addStatement( "      		for( j = 0; j < (ACADO_NX+ACADO_NXA)*(1+ACADO_NX+ACADO_NU); j++) {\n" );
    main.addStatement( "      			fprintf(file, \"%.16f \", x[j]);\n" );
    main.addStatement( "      		}\n" );
    main.addStatement( "      		fprintf(file, \"\\n\");\n" );
    main.addLinebreak( );
    if( !DERIVATIVES )  main.addStatement( "      		nil = fscanf( controls, \"%lf\", &x[ACADO_NX+ACADO_NXA] );\n" );
    else  main.addStatement( "      		nil = fscanf( controls, \"%lf\", &x[(ACADO_NX+ACADO_NXA)*(1+ACADO_NX+ACADO_NU)] );\n" );
    main.addStatement( "      		for( j = 0; j < ACADO_NU; j++) {\n" );
    if( !DERIVATIVES )  main.addStatement( "      			nil = fscanf( controls, \"%lf\", &x[ACADO_NX+ACADO_NXA+j] );\n" );
    else  main.addStatement( "      			nil = fscanf( controls, \"%lf\", &x[(ACADO_NX+ACADO_NXA)*(1+ACADO_NX+ACADO_NU)+j] );\n" );
    main.addStatement( "      		}\n" );
    main.addLinebreak( );
    if( TIMING == true ) {
		main.addStatement( "      		if( i == 0 ) {\n" );
		if( !DERIVATIVES )  main.addStatement( "      			for( j=0; j < ACADO_NX+ACADO_NXA+ACADO_NU; j++ ) {\n" );
		else  main.addStatement( "      			for( j=0; j < (ACADO_NX+ACADO_NXA)*(1+ACADO_NX+ACADO_NU)+ACADO_NU; j++ ) {\n" );
		main.addStatement( "      				xT[j] = x[j];\n" );
		main.addStatement( "     			}\n" );
		main.addStatement( "      		}\n" );
	}
    main.addLinebreak( );
    std::string integrate( "      		integrate( x" );
    for( i = 0; i < (int)outputGrids.size(); i++ ) {
		integrate += string(", out") + toString(i);
	}
    integrate += ", reset";
    main.addStatement( integrate + " );\n" );
    main.addStatement( "      		reset = 0;\n" );
    main.addLinebreak( );
    for( i = 0; i < (int)outputGrids.size(); i++ ) {
		main.addStatement( (std::string)"      		for( j = 0; j < ACADO_NMEAS[" + toString(i) +  "]; j=j+JUMP ) {\n" );
		main.addStatement( (std::string)"      			fprintf(output" + toString(i) +  ", \"%.16f \", i*h+(j+1)*step" + toString(i) +  ");\n" );
		main.addStatement( (std::string)"      			for( k = 0; k < dimOut" + toString(i) +  "; k++ ) {\n" );
		main.addStatement( (std::string)"      				fprintf(output" + toString(i) +  ", \"%.16f \", out" + toString(i) +  "[j*dimOut" + toString(i) +  "+k]);\n" );
		main.addStatement( "      			}\n" );
		main.addStatement( (std::string)"      			fprintf(output" + toString(i) +  ", \"%s\", \"\\n\");\n" );
		main.addStatement( "      		}\n" );
	}
    main.addStatement( "      }\n" );
    main.addStatement( "      fprintf(file, \"%.16f \", ACADO_N*h);\n" );
    if( !DERIVATIVES )  main.addStatement( "      for( j = 0; j < ACADO_NX+ACADO_NXA; j++) {\n" );
    else  main.addStatement( "      for( j = 0; j < (ACADO_NX+ACADO_NXA)*(1+ACADO_NX+ACADO_NU); j++) {\n" );
    main.addStatement( "      		fprintf(file, \"%.16f \", x[j]);\n" );
    main.addStatement( "      }\n" );
    main.addStatement( "      fprintf(file, \"\\n\");\n" );
    main.addLinebreak( );
    main.addStatement( "      fclose(file);\n" );
    for( i = 0; i < (int)outputGrids.size(); i++ ) {
		main.addStatement( (std::string)"      fclose(output" + toString(i) +  ");\n" );
	}
    main.addStatement( "      fclose(controls);\n" );
    if( TIMING == true ) {
		main.addStatement( "      gettimeofday( &theclock,0 );\n" );
		main.addStatement( "      start = 1.0*theclock.tv_sec + 1.0e-6*theclock.tv_usec;\n" );
	    main.addStatement( "      reset = 1;\n" );
		main.addStatement( "      for( i=0; i < CALLS_TIMING; i++ ) {\n" );
		main.addStatement( "      		for( j=0; j < (ACADO_NX+ACADO_NXA); j++ ) {\n" );
		main.addStatement( "      			x[j] = xT[j];\n" );
		main.addStatement( "      		}\n" );
		integrate = std::string( "      		integrate( x" );
		for( i = 0; i < (int)outputGrids.size(); i++ ) {
			integrate += string(", out") + toString(i);
		}
		integrate += ", reset";
		main.addStatement( integrate + " );\n" );
	    main.addStatement( "      		reset = 0;\n" );
		main.addStatement( "      }\n" );
		main.addStatement( "      gettimeofday( &theclock,0 );\n" );
		main.addStatement( "      end = 1.0*theclock.tv_sec + 1.0e-6*theclock.tv_usec;\n" );
		main.addStatement( "      time = (end-start);\n" );
		main.addLinebreak( );
		main.addStatement( "      printf( \"\\n\\n AVERAGE DURATION OF ONE INTEGRATION STEP:   %.3g μs\\n\\n\", 1e6*time/STEPS_TIMING );\n" );
	}
    main.addLinebreak( );
	main.addStatement( "      return 0;\n" );
	main.addStatement( "}\n" );
    
    
	return main.exportCode( );
}


returnValue SIMexport::exportEvaluation(	const std::string& _dirName,
											const std::string& _fileName
											) const
{
	int i;
	int sensGen;
	get( DYNAMIC_SENSITIVITY, sensGen );
	bool DERIVATIVES = ((ExportSensitivityType) sensGen != NO_SENSITIVITY);
	
	DVector nMeasV = modelData.getNumMeas();
	DVector nOutV = modelData.getDimOutputs();

	std::vector<Grid> outputGrids;
	modelData.getOutputGrids(outputGrids);

    std::string fileName =  _dirName;
    fileName += string("/") + _fileName;

	ExportFile main( fileName,commonHeaderName );

	main.addStatement( "#include <stdio.h>\n" );
	main.addStatement( "#include \"acado_auxiliary_sim_functions.h\"\n" );
	main.addLinebreak( 1 );
	main.addComment( "SOME CONVENIENT DEFINTIONS:" );
	main.addComment( "---------------------------------------------------------------" );
	main.addStatement( (std::string)"   #define h           " + toString(T/modelData.getN())  + "      /* length of one simulation interval   */\n" );
	main.addStatement( (std::string)"   #define RESULTS_NAME	  \"" + _results + "\"\n" );
	for( i = 0; i < (int)outputGrids.size(); i++ ) {
		main.addStatement( (std::string)"   #define OUTPUT" + toString(i) +  "_NAME	  \"" + _outputFiles[i] + "\"\n" );
		main.addStatement( (std::string)"   #define REF_OUTPUT" + toString(i) +  "_NAME	  \"" + _refOutputFiles[i] + "\"\n" );
	}
	main.addStatement( (std::string)"   #define REF_NAME  \"" + _ref + "\"\n" );
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
		main.addStatement( (std::string)"      FILE *output" + toString(i) +  ";\n" );
		main.addStatement( (std::string)"      FILE *refOutput" + toString(i) +  ";\n" );
	}
    main.addStatement( "      int i, j, nil;\n" );
    main.addStatement( "      real_t x[ACADO_NX+ACADO_NXA];\n" );
    main.addStatement( "      real_t xRef[ACADO_NX+ACADO_NXA];\n" );
    for( i = 0; i < (int)outputGrids.size(); i++ ) {
		main.addStatement( (std::string)"      real_t step" + toString(i) +  " = h/ACADO_NMEAS[" + toString(i) +  "];\n" );
		main.addStatement( (std::string)"      real_t out" + toString(i) +  "[ACADO_NMEAS[" + toString(i) +  "]*ACADO_NOUT[" + toString(i) +  "]];\n" );
		main.addStatement( (std::string)"      real_t refOut" + toString(i) +  "[ACADO_NMEAS[" + toString(i) +  "]*ACADO_NOUT[" + toString(i) +  "]];\n" );
	}
    main.addStatement( "      real_t maxErr, meanErr, maxErrX, meanErrX, maxErrXA, meanErrXA, temp;\n" );
    main.addStatement( "      const ACADOworkspace nullWork2 = {0};\n" );
    main.addStatement( " 	  acadoWorkspace = nullWork2;\n" );
    main.addLinebreak( 2 );

    main.addComment( 3,"START EVALUATION RESULTS:" );
    main.addComment( 3,"----------------------------------------" );
	main.addStatement( "      meanErrX = 0;\n" );
	main.addStatement( "      meanErrXA = 0;\n" );
    main.addStatement( "      file = fopen(RESULTS_NAME,\"r\");\n" );
    main.addStatement( "      ref = fopen(REF_NAME,\"r\");\n" );
    if( DERIVATIVES )  main.addStatement( "      for( i = 0; i < (ACADO_NX+ACADO_NXA)*(1+ACADO_NX+ACADO_NU)+1; i++ ) {\n" );
    else  main.addStatement( "      for( i = 0; i < ACADO_NX+ACADO_NXA+1; i++ ) {\n" );
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
    if( PRINT_DETAILS && modelData.getNXA() > 0 ) {
    	main.addStatement( "      		printf( \"MAX ERROR AT %.3f s:   %.4e   %.4e \\n\", i*h, maxErrX, maxErrXA );\n" );
    }
    else if( PRINT_DETAILS ) {
    	main.addStatement( "      		printf( \"MAX ERROR AT %.3f s:   %.4e \\n\", i*h, maxErrX );\n" );
    }
    main.addStatement( "			meanErrX += maxErrX;\n" );
    main.addStatement( "			meanErrXA += maxErrXA;\n" );
    main.addLinebreak( );
    if( DERIVATIVES ) {
    	main.addStatement( "      		for( j = 0; j < (ACADO_NX+ACADO_NXA)*(ACADO_NX+ACADO_NU); j++ ) {\n" );
    	main.addStatement( "      			nil = fscanf( file, \"%lf\", &temp );\n" );
    	main.addStatement( "      			nil = fscanf( ref, \"%lf\", &temp );\n" );
    	main.addStatement( "      		}\n" );
    }
    main.addStatement( "      }\n" );
    main.addStatement( "	  meanErrX = meanErrX/ACADO_N;\n" );
    main.addStatement( "	  meanErrXA = meanErrXA/ACADO_N;\n" );
    if( PRINT_DETAILS ) main.addStatement( "      printf( \"\\n\" );\n" );
    if( modelData.getNXA() > 0 ) {
    	main.addStatement( "      printf( \"TOTAL MEAN ERROR:   %.4e   %.4e \\n\", meanErrX, meanErrXA );\n" );
    }
    else {
    	main.addStatement( "      printf( \"TOTAL MEAN ERROR:   %.4e \\n\", meanErrX );\n" );
    }
    main.addStatement( "      printf( \"\\n\\n\" );\n" );
    for( i = 0; i < (int)outputGrids.size(); i++ ) {
		main.addLinebreak( );
		main.addStatement( (std::string)"      printf( \" OUTPUT FUNCTION " + toString(i+1) + ":\\n\" );\n" );
		main.addStatement( (std::string)"      meanErr = 0;\n" );
		main.addStatement( (std::string)"      output" + toString(i) +  " = fopen(OUTPUT" + toString(i) +  "_NAME,\"r\");\n" );
		main.addStatement( (std::string)"      refOutput" + toString(i) +  " = fopen(REF_OUTPUT" + toString(i) +  "_NAME,\"r\");\n" );
		main.addLinebreak( );
		main.addStatement( (std::string)"      for( i = 1; i <= ACADO_N*ACADO_NMEAS[" + toString(i) +  "]; i++ ) {\n" );
		main.addStatement( (std::string)"      		nil = fscanf( output" + toString(i) +  ", \"%lf\", &temp );\n" );
		main.addStatement( (std::string)"      		nil = fscanf( refOutput" + toString(i) +  ", \"%lf\", &temp );\n" );
		main.addLinebreak( );
		main.addStatement( "      		maxErr = 0;\n" );
		main.addStatement( (std::string)"      		for( j = 0; j < ACADO_NOUT[" + toString(i) +  "]; j++ ) {\n" );
		main.addStatement( (std::string)"      			nil = fscanf( output" + toString(i) +  ", \"%lf\", &out" + toString(i) +  "[j] );\n" );
		main.addStatement( (std::string)"      			nil = fscanf( refOutput" + toString(i) +  ", \"%lf\", &refOut" + toString(i) +  "[j] );\n" );
		main.addStatement( (std::string)"      			temp = fabs(out" + toString(i) +  "[j] - refOut" + toString(i) +  "[j])/fabs(refOut" + toString(i) +  "[j]);\n" );
		main.addStatement( "      			if( temp > maxErr ) maxErr = temp;\n" );
		main.addStatement( (std::string)"      			if( isnan(out" + toString(i) +  "[j]) ) maxErr = sqrt(-1);\n" );
		main.addStatement( "      		}\n" );
		main.addLinebreak( );
		if( PRINT_DETAILS ) main.addStatement( (std::string)"      		printf( \"MAX ERROR AT %.3f s:   %.4e \\n\", (i-1)*step" + toString(i) + ", maxErr );\n" );
		main.addStatement( "      		meanErr += maxErr;\n" );
		main.addLinebreak( );
		if( DERIVATIVES ) {
			main.addStatement( (std::string)"      		for( j = 0; j < ACADO_NOUT[" + toString(i) + "]*(ACADO_NX+ACADO_NU); j++ ) {\n" );
			main.addStatement( (std::string)"      			nil = fscanf( output" + toString(i) + ", \"%lf\", &temp );\n" );
			main.addStatement( (std::string)"      			nil = fscanf( refOutput" + toString(i) + ", \"%lf\", &temp );\n" );
			main.addStatement( "      		}\n" );
		}
		main.addStatement( "      }\n" );
		main.addStatement( (std::string)"	  meanErr = meanErr/(ACADO_N*ACADO_NMEAS[" + toString(i) + "]);\n" );
		if( PRINT_DETAILS ) main.addStatement( "      printf( \"\\n\" );\n" );
		main.addStatement( "      printf( \"TOTAL MEAN ERROR:   %.4e \\n\", meanErr );\n" );
		main.addStatement( "      printf( \"\\n\\n\" );\n" );
	}
    main.addLinebreak( );
    main.addStatement( "      return 0;\n" );
    main.addStatement( "}\n" );
    
	return main.exportCode( );
}



returnValue SIMexport::exportAndRun(	const std::string& dirName,
										const std::string& initStates,
										const std::string& controls,
										const std::string& results,
										const std::string& ref
										)
{
	std::string test( "acado_test.c" );
	set( GENERATE_TEST_FILE, 1 );

	Grid integrationGrid;
	modelData.getIntegrationGrid(integrationGrid);
	std::vector<Grid> outputGrids;
	modelData.getOutputGrids(outputGrids);

	int measGrid;
	get( MEASUREMENT_GRID, measGrid );
	if( (MeasurementGrid)measGrid == ONLINE_GRID ) return ACADOERROR( RET_INVALID_OPTION );

	_initStates = initStates;
	_controls = controls;
	_results = results;
	_ref = ref;

	int numSteps;
    get( NUM_INTEGRATOR_STEPS, numSteps );
	timingCalls = (uint) ceil((double)(timingSteps*modelData.getN())/((double) numSteps) - 10.0*EPS);
	timingSteps = (uint) ceil((double)timingCalls*((double) numSteps/((double) modelData.getN())) - 10.0*EPS);
    
    if( !referenceProvided ) {
	    // REFERENCE:
    	set( NUM_INTEGRATOR_STEPS,  (int)factorRef*numSteps );
    	exportCode(	dirName );
    	exportTest(	dirName, test, _ref, _refOutputFiles, false, 1 );
    	executeTest( dirName );
	}
    modelData.clearIntegrationGrid();
    
    // THE INTEGRATOR:
	set( NUM_INTEGRATOR_STEPS,  numSteps );
	exportCode(	dirName );
	if(timingSteps > 0 && timingCalls > 0) 	exportTest(	dirName, test, _results, _outputFiles, true, 1 );
	else 									exportTest(	dirName, test, _results, _outputFiles, false, 1 );
	executeTest( dirName );

	// THE EVALUATION:
	int nil;
	nil = system( (dirName + "/./acado_compare").c_str() );
	nil = nil+1;
	
	return SUCCESSFUL_RETURN;
}


returnValue SIMexport::exportAcadoHeader(	const std::string& _dirName,
											const std::string& _fileName,
											const std::string& _realString,
											const std::string& _intString,
											int _precision
											) const
{
	string moduleName;
	get(CG_MODULE_NAME, moduleName);

	int qpSolver;
	get(QP_SOLVER, qpSolver);

	int useSinglePrecision;
	get(USE_SINGLE_PRECISION, useSinglePrecision);

	string fileName;
	fileName = _dirName + "/" + _fileName;


	map<string, pair<string, string> > options;

	DVector nMeasV = getNumMeas();
	DVector nOutV = getDimOutputs();

	options[ "ACADO_N" ]   = make_pair(toString( getN() ),   "Number of control/estimation intervals.");
	options[ "ACADO_NX" ]  = make_pair(toString( getNX() ),  "Number of differential variables.");
	options[ "ACADO_NXD" ] = make_pair(toString( getNDX() ), "Number of differential derivative variables.");
	options[ "ACADO_NXA" ] = make_pair(toString( getNXA() ), "Number of algebraic variables.");
	options[ "ACADO_NU" ]  = make_pair(toString( getNU() ),  "Number of control variables.");
	options[ "ACADO_NOD" ]  = make_pair(toString( getNOD() ),  "Number of online data values.");
	options[ "ACADO_NUMOUT" ]  = make_pair(toString( nOutV.getDim() ),  "Number of output functions.");

	if( !nMeasV.isEmpty() && !nOutV.isEmpty() ) {
		std::ostringstream acado_nout;
		ExportVariable( "ACADO_NOUT",nOutV,STATIC_CONST_INT ).exportDataDeclaration(acado_nout);
		std::ostringstream acado_nmeas;
		ExportVariable( "ACADO_NMEAS",nMeasV,STATIC_CONST_INT ).exportDataDeclaration(acado_nmeas);
		options[ "ACADO_OUTPUTS_DEFINED" ]  = make_pair("\n" + acado_nout.str() + acado_nmeas.str(),  "Dimension and measurements of the output functions per shooting interval.");
	}

	//
	// ACADO variables and workspace
	//
	ExportStatementBlock variablesBlock;
	stringstream variables;

	if (collectDataDeclarations(variablesBlock, ACADO_VARIABLES) != SUCCESSFUL_RETURN)
		return ACADOERROR( RET_UNABLE_TO_EXPORT_CODE );
	variablesBlock.exportCode(variables, _realString, _intString, _precision);

	ExportStatementBlock workspaceBlock;
	stringstream workspace;

	if (collectDataDeclarations(workspaceBlock, ACADO_WORKSPACE) != SUCCESSFUL_RETURN)
		return ACADOERROR( RET_UNABLE_TO_EXPORT_CODE );
	workspaceBlock.exportCode(workspace, _realString, _intString, _precision);

	ExportStatementBlock functionsBlock;
	stringstream functions;

	if (collectFunctionDeclarations( functionsBlock ) != SUCCESSFUL_RETURN)
		return ACADOERROR( RET_UNABLE_TO_EXPORT_CODE );
	functionsBlock.exportCode(functions, _realString);

	ExportCommonHeader ech(fileName, "", _realString, _intString, _precision);
	ech.configure( moduleName, useSinglePrecision, (QPSolverName)qpSolver,
			options, variables.str(), workspace.str(), functions.str());

	return ech.exportCode();
}


returnValue SIMexport::exportMakefile(	const std::string& _dirName,
										const std::string& _fileName,
										const std::string& _realString,
										const std::string& _intString,
										int _precision
										) const
{
	std::string fileName( _dirName );
	fileName += "/" + _fileName;

	acadoCopyTemplateFile(MAKEFILE_INTEGRATOR, fileName, "#", true);

	return SUCCESSFUL_RETURN;
}


returnValue SIMexport::setReference( const std::string& reference, const std::vector<std::string>& outputReference ) {
	if( hasOutputs() && outputReference.size() == 0 ) {
		referenceProvided = false;
		return RET_UNABLE_TO_EXPORT_CODE;
	}
	referenceProvided = true;
	_ref = reference;
	if( outputReference.size() > 0 ) _refOutputFiles = outputReference;
	
	return SUCCESSFUL_RETURN;
}


returnValue SIMexport::setTimingSteps( uint _timingSteps ) {
	timingSteps = _timingSteps;
	
	return SUCCESSFUL_RETURN;
}


returnValue SIMexport::printDetails( bool details ) {
	PRINT_DETAILS = details;
	
	return SUCCESSFUL_RETURN;
}


returnValue SIMexport::executeTest( const std::string& _dirName ) {
	//sleep(2); does not compile on windows!!
	int nil;
	nil = system((string("make clean -s -C ") + _dirName).c_str());
	nil = system((string("make -s -C ") + _dirName).c_str());
	nil = system((_dirName + "/./acado_test").c_str());
	nil = nil+1;
	
	return SUCCESSFUL_RETURN;
}

returnValue SIMexport::setTimingCalls( uint _timingCalls ) {
	timingCalls = _timingCalls;

	return SUCCESSFUL_RETURN;
}


CLOSE_NAMESPACE_ACADO

// end of file.
