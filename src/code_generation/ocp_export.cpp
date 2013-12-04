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
 *    \file source/code_generation/ocp_export.cpp
 *    \authors Hans Joachim Ferreau, Boris Houska, Milan Vukov
 *    \date 2010 - 2013
 */

#include <acado/code_generation/ocp_export.hpp>
#include <acado/code_generation/export_simulink_interface.hpp>
#include <acado/code_generation/export_auxiliary_functions.hpp>

#include <acado/code_generation/templates/templates.hpp>

#include <sstream>
#include <string>

using namespace std;

BEGIN_NAMESPACE_ACADO

OCPexport::OCPexport( ) : ExportModule( )
{
	setStatus( BS_NOT_INITIALIZED );
}


OCPexport::OCPexport(	const OCP& _ocp
						) : ExportModule( )
{
	ocp = _ocp;

	setStatus( BS_NOT_INITIALIZED );
}

returnValue OCPexport::exportCode(	const String& dirName,
									const String& _realString,
									const String& _intString,
									int _precision
									)
{
	int qpSolver;
	get(QP_SOLVER, qpSolver);

	acadoPrintCopyrightNotice( "Code Generation Tool" );

	//
	// Create the export folders
	//

	setExportFolderName( dirName );

	returnValue dirStatus = acadoCreateFolder( dirName.getName() );
	if (dirStatus != SUCCESSFUL_RETURN)
		return dirStatus;

	//
	// Setup the export structures
	//
	returnValue setupStatus = setup( );
	if ( setupStatus != SUCCESSFUL_RETURN )
		return ACADOERRORTEXT(setupStatus, "Error in setting up the OCP export");

	//
	// Export common header
	//
	if (exportAcadoHeader(dirName, getCommonHeaderName(), _realString, _intString, _precision)
			!= SUCCESSFUL_RETURN )
		return ACADOERROR( RET_UNABLE_TO_EXPORT_CODE );

	//
	// Export integrator
	//
	if (integrator != 0)
	{
		ExportFile integratorFile(dirName + "/" + getName() + "_integrator.c",
				getCommonHeaderName(), _realString, _intString, _precision);

		integrator->getCode( integratorFile );

		if (integratorFile.exportCode( ) != SUCCESSFUL_RETURN)
			return ACADOERROR( RET_UNABLE_TO_EXPORT_CODE );
	}
	else
		return ACADOERROR( RET_INVALID_ARGUMENTS );

	//
	// Export solver
	//
	if( solver != 0 )
	{
		ExportFile solverFile(dirName + "/" + getName() + "_solver.c",
				getCommonHeaderName(), _realString, _intString, _precision);

		solver->getCode( solverFile );

		if ( solverFile.exportCode( ) != SUCCESSFUL_RETURN )
			return ACADOERROR( RET_UNABLE_TO_EXPORT_CODE );
	}
	else
		return ACADOERROR( RET_INVALID_ARGUMENTS );

	LOG( LVL_DEBUG ) << "Export templates" << endl;

	//
	// Export auxiliary functions, always
	//
	String str;

	ExportAuxiliaryFunctions eaf(
			dirName + String("/") + getName() + String("_auxiliary_functions.h"),
			dirName + String("/") + getName() + String("_auxiliary_functions.c"),
			getName()
			);
	eaf.configure();
	eaf.exportCode();

	//
	// Export Makefile
	//
	int generateMakeFile;
	get(GENERATE_MAKE_FILE, generateMakeFile);

	if ( (BooleanType)generateMakeFile == BT_TRUE )
	{
		str = dirName + String("/Makefile");

		switch ( (QPSolverName)qpSolver )
		{
			case QP_QPOASES:
				acadoCopyTempateFile(MAKEFILE_QPOASES, str.getName(), "#", BT_TRUE);
				break;

			case QP_FORCES:
				acadoCopyTempateFile(MAKEFILE_FORCES, str.getName(), "#", BT_TRUE);
				break;

			case QP_QPDUNES:
				ACADOWARNINGTEXT(RET_NOT_IMPLEMENTED_YET, "Makefile for qpDUNES based OCP solver is not yet available.");
				break;

			default:
				return ACADOERROR( RET_UNABLE_TO_EXPORT_CODE );
		}
	}

	//
	// Export a dummy test file
	//
	int generateTestFile;
	get(GENERATE_TEST_FILE, generateTestFile);
	String testFileName = dirName + "/test.c";
	if ((BooleanType) generateTestFile == BT_TRUE)
		acadoCopyTempateFile(DUMMY_TEST_FILE, testFileName.getName(), 0, BT_TRUE);

	//
	// Generate MATLAB MEX interface
	//
	int generateMexInterface;
	get(GENERATE_MATLAB_INTERFACE, generateMexInterface);
	if ( (BooleanType)generateMexInterface == BT_TRUE )
	{
		str = dirName + String("/") + getName() + String("_solver_mex.c");

		acadoCopyTempateFile(SOLVER_MEX, str.getName(), 0, BT_TRUE);

		str = dirName + String("/make_") + getName() + String("_solver.m");

		switch ( (QPSolverName)qpSolver )
		{
		case QP_QPOASES:
			acadoCopyTempateFile(MAKE_MEX_QPOASES, str.getName(), "%", BT_TRUE);
			break;

		case QP_FORCES:
			acadoCopyTempateFile(MAKE_MEX_FORCES, str.getName(), "%", BT_TRUE);
			break;

		case QP_QPDUNES:
			ACADOWARNINGTEXT(RET_NOT_IMPLEMENTED_YET, "MEX interface for qpDUNES based OCP solver is not yet available.");
			break;

		default:
			return ACADOERROR( RET_UNABLE_TO_EXPORT_CODE );
		}
	}

	//
	// Generate MATLAB Simulink interface
	//
	int generateSimulinkInterface;
	get(GENERATE_SIMULINK_INTERFACE, generateSimulinkInterface);
	if ((BooleanType) generateSimulinkInterface == BT_TRUE)
	{
		if ((QPSolverName)qpSolver != QP_QPOASES)
			return ACADOERRORTEXT(RET_INVALID_ARGUMENTS,
					"At the moment, Simulink interface is available only with qpOASES based OCP solver.");

		String makefileName = dirName + String("/make_") + getName() + "_solver_sfunction.m";
		String wrapperHeaderName = dirName + String("/") + getName() + "_solver_sfunction.h";
		String wrapperSourceName = dirName + String("/") + getName() + "_solver_sfunction.c";

		ExportSimulinkInterface esi(makefileName, wrapperHeaderName, wrapperSourceName, getName());

		// Get options
		int useSinglePrecision;
		get(USE_SINGLE_PRECISION, useSinglePrecision);

		int hardcodeConstraintValues;
		get(CG_HARDCODE_CONSTRAINT_VALUES, hardcodeConstraintValues);
		if ((BooleanType)hardcodeConstraintValues == BT_FALSE)
			return ACADOERROR( RET_NOT_IMPLEMENTED_YET );

		int fixInitialState;
		get(FIX_INITIAL_STATE, fixInitialState);
		int useAC;
		get(CG_USE_ARRIVAL_COST, useAC);
		int covCalc;
		get(CG_COMPUTE_COVARIANCE_MATRIX, covCalc);

		// Configure templates
		esi.configure(
				ocp.getN(), ocp.getNX(), ocp.getNDX(), ocp.getNXA(), ocp.getNU(), ocp.getNP(),
				solver->getNY(), solver->getNYN(),
				(BooleanType)fixInitialState,
				(unsigned)solver->weightingMatricesType(),
				(BooleanType)hardcodeConstraintValues,
				(BooleanType)useAC,
				(BooleanType)covCalc);

		esi.exportCode();
	}

    return SUCCESSFUL_RETURN;
}



returnValue OCPexport::printDimensionsQP( )
{
	if (getStatus() != BS_READY)
		return SUCCESSFUL_RETURN;

	LOG( LVL_INFO ) << "ACADO Code Generation Tool:" << endl
			<< "\t* Number of QP variables: " << solver->getNumQPvars( ) << endl
			<< "\t* Number of path and point constraints: " << solver->getNumComplexConstraints() << endl;

	return SUCCESSFUL_RETURN;
}

returnValue OCPexport::setup( )
{
	// Nothing to do as object is up-to-date
	if ( getStatus() == BS_READY )
		return SUCCESSFUL_RETURN;

	// Consistency check
	returnValue returnvalue = checkConsistency( );
 	if ( returnvalue != SUCCESSFUL_RETURN )
 		return ACADOERROR( returnvalue );

 	//
 	// Set common header name
 	//
 	commonHeaderName = getName() + "_common.h";

	//
	// Prepare integrator export
	//
	int numSteps;
	get(NUM_INTEGRATOR_STEPS, numSteps);

	int integratorType;
	get(INTEGRATOR_TYPE, integratorType);

	integrator = IntegratorExportPtr(
			IntegratorExportFactory::instance().createAlgorithm(this, getCommonHeaderName(), static_cast<ExportIntegratorType>(integratorType)));
	if (integrator == 0)
		return ACADOERROR( RET_INVALID_OPTION );

	ocp.setNumberIntegrationSteps( numSteps );
	// NOTE: This function internally calls setup() function
	integrator->setModelData( ocp.getModelData() );

	//
	// Prepare solver export
	//

	int qpSolver;
	get(QP_SOLVER, qpSolver);
	int qpSolution;
	get(SPARSE_QP_SOLUTION, qpSolution);

	// TODO Extend ExportNLPSolver ctor to accept OCP reference.

	switch ( (SparseQPsolutionMethods)qpSolution )
	{
	case FULL_CONDENSING:
	case CONDENSING:

		if ((QPSolverName)qpSolver != QP_QPOASES)
			return ACADOERRORTEXT(RET_INVALID_ARGUMENTS,
					"For condensed solution only qpOASES QP solver is supported");

		solver = ExportNLPSolverPtr(
				NLPSolverFactory::instance().createAlgorithm(this, getCommonHeaderName(), GAUSS_NEWTON_CONDENSED));

		break;

	case FULL_CONDENSING_N2:

		if ((QPSolverName)qpSolver != QP_QPOASES)
			return ACADOERRORTEXT(RET_INVALID_ARGUMENTS,
					"For condensed solution only qpOASES QP solver is supported");

		solver = ExportNLPSolverPtr(
				NLPSolverFactory::instance().createAlgorithm(this, getCommonHeaderName(), GAUSS_NEWTON_CN2));

		break;

	case SPARSE_SOLVER:
		if ((QPSolverName)qpSolver != QP_FORCES && (QPSolverName)qpSolver != QP_QPDUNES)
			return ACADOERRORTEXT(RET_INVALID_ARGUMENTS,
					"For sparse solution FORCES and qpDUNES QP solvers are supported");
		if ( (QPSolverName)qpSolver == QP_FORCES)
			solver = ExportNLPSolverPtr(
					NLPSolverFactory::instance().createAlgorithm(this, getCommonHeaderName(), GAUSS_NEWTON_FORCES));
		else if ((QPSolverName)qpSolver == QP_QPDUNES)
			solver = ExportNLPSolverPtr(
					NLPSolverFactory::instance().createAlgorithm(this, getCommonHeaderName(), GAUSS_NEWTON_QPDUNES));
		break;

	default:
		return ACADOERRORTEXT(RET_INVALID_ARGUMENTS, "QP solver option is invalid");
	}
	if (solver == 0)
		return ACADOERRORTEXT(RET_INVALID_OPTION, "Cannot allocate the solver object");

	solver->setDimensions(ocp.getNX(), ocp.getNDX(), ocp.getNXA(), ocp.getNU(), ocp.getNP(), ocp.getN());
	solver->setIntegratorExport( integrator );

	Objective objective;
	ocp.getObjective( objective );

	returnValue statusObjective;
	statusObjective = solver->setObjective( objective );
	if (statusObjective != SUCCESSFUL_RETURN)
		return ACADOERRORTEXT(status, "Error in retrieving the objective.");

	solver->setConstraints( ocp );

	// Get LM multiplier
	double levenbergMarquardt;
	get( LEVENBERG_MARQUARDT,levenbergMarquardt );

	solver->setLevenbergMarquardt( levenbergMarquardt );

	solver->setup( );

	setStatus( BS_READY );

	return SUCCESSFUL_RETURN;
}


returnValue OCPexport::checkConsistency( ) const
{
	//
	// Consistency checks:
	//

 	if ( ocp.hasObjective( ) == BT_TRUE )
 		return ACADOERROR( RET_INVALID_OBJECTIVE_FOR_CODE_EXPORT );

 	DifferentialEquation f;
 	ocp.getModel( f );

// 	if ( f.isDiscretized( ) == BT_TRUE )
// 		return ACADOERROR( RET_NO_DISCRETE_ODE_FOR_CODE_EXPORT );

 	if ( f.getNUI( ) > 0 )
 		return ACADOERROR( RET_INVALID_ARGUMENTS );

 	int hessianApproximation;
 	get( HESSIAN_APPROXIMATION, hessianApproximation );
 	if ( (HessianApproximationMode)hessianApproximation != GAUSS_NEWTON )
 		return ACADOERROR( RET_INVALID_OPTION );

 	int discretizationType;
 	get( DISCRETIZATION_TYPE,discretizationType );
 	if ( ( (StateDiscretizationType)discretizationType != SINGLE_SHOOTING ) &&
 			( (StateDiscretizationType)discretizationType != MULTIPLE_SHOOTING ) )
 		return ACADOERROR( RET_INVALID_OPTION );

	return SUCCESSFUL_RETURN;
}


returnValue OCPexport::collectDataDeclarations(	ExportStatementBlock& declarations,
												ExportStruct dataStruct
												) const
{
	if (integrator->getDataDeclarations(declarations, dataStruct) != SUCCESSFUL_RETURN)
		return RET_UNABLE_TO_EXPORT_CODE;

	if (solver->getDataDeclarations(declarations, dataStruct) != SUCCESSFUL_RETURN)
		return RET_UNABLE_TO_EXPORT_CODE;

	return SUCCESSFUL_RETURN;
}


returnValue OCPexport::collectFunctionDeclarations(	ExportStatementBlock& declarations
													) const
{
	if (integrator->getFunctionDeclarations( declarations ) != SUCCESSFUL_RETURN)
		return RET_UNABLE_TO_EXPORT_CODE;

	if (solver->getFunctionDeclarations( declarations ) != SUCCESSFUL_RETURN)
		return RET_UNABLE_TO_EXPORT_CODE;

	return SUCCESSFUL_RETURN;
}

returnValue OCPexport::exportAcadoHeader(	const String& _dirName,
											const String& _fileName,
											const String& _realString,
											const String& _intString,
											int _precision
											) const
{
	int qpSolver;
	get(QP_SOLVER, qpSolver);

	int useSinglePrecision;
	get(USE_SINGLE_PRECISION, useSinglePrecision);

	int hardcodeConstraintValues;
	get(CG_HARDCODE_CONSTRAINT_VALUES, hardcodeConstraintValues);

	String fileName( _dirName );
	fileName << "/" << _fileName;
	ExportFile acadoHeader( fileName,"", _realString,_intString,_precision );

	// TODO Here we might put it to uppercase...
	String moduleName = getName();

	acadoHeader
		<< String("#ifndef ") + moduleName + String("_COMMON_H\n")
		<< String("#define ") + moduleName + String("_COMMON_H\n\n")
		<< String("#include <math.h>\n");

	if ((QPSolverName)qpSolver == QP_FORCES)
		acadoHeader.addStatement( "#include <string.h>\n" );

	acadoHeader.addLinebreak( 2 );
	acadoHeader.addStatement(
			"#ifndef __MATLAB__\n"
			"#ifdef __cplusplus\n"
			"extern \"C\"\n"
			"{\n"
			"#endif /* __cplusplus */\n"
			"#endif /* __MATLAB__ */\n\n"
	);

	acadoHeader.addStatement(
			"/** \\defgroup acado_solver ACADO Optimal Control Problem (OCP) solver */\n"
			"/** @{ */\n\n"
	);

	switch ( (QPSolverName)qpSolver )
	{
	case QP_QPOASES:
		acadoHeader.addStatement(
				String("#include \"") + getName() + "_qpoases_interface.hpp\"\n"
		);

		break;

	case QP_FORCES:

		acadoHeader.addStatement( "/** Definition of the floating point data type. */\n" );
		if ( (BooleanType)useSinglePrecision == BT_TRUE )
			acadoHeader.addStatement( "typedef float real_t;\n" );
		else
			acadoHeader.addStatement( "typedef double real_t;\n" );

		break;

	case QP_QPDUNES:
		acadoHeader.addStatement(
				"#include \"qpDUNES.h\"\n"
		);

		break;

	default:
		return ACADOERROR( RET_INVALID_OPTION );

	}
	acadoHeader.addLinebreak( 2 );

	//
	// Some common defines
	//
	acadoHeader.addStatement(
			"/*\n"
			" * Common definitions\n"
			" */\n\n"
	);

	stringstream s;

	s	<< "/** Number of control/estimation intervals. */" << endl
		<< "#define ACADO_N   " << ocp.getN() << endl
		<< "/** Number of differential variables. */" << endl
		<< "#define ACADO_NX  " << ocp.getNX() << endl
		<< "/** Number of differential derivative variables. */" << endl
		<< "#define ACADO_NXD " << ocp.getNDX() << endl
		<< "/** Number of algebraic variables. */" << endl
		<< "#define ACADO_NXA " << ocp.getNXA() << endl
		<< "/** Number of control variables. */" << endl
		<< "#define ACADO_NU  " << ocp.getNU() << endl
		<< "/** Number of parameters (which are NOT optimization variables). */" << endl
		<< "#define ACADO_NP  " << ocp.getNP() << endl
		<< "/** Number of references/measurements per node on the first N nodes. */" << endl
		<< "#define ACADO_NY  " << solver->getNY() << endl
		<< "/** Number of references/measurements on the last (N + 1)st node. */" << endl
		<< "#define ACADO_NYN " << solver->getNYN() << endl;

	acadoHeader.addStatement( s.str().c_str() );
	acadoHeader.addLinebreak( 1 );

	s.str( string() );
	s 	<< "/** qpOASES QP solver indicator. */" << endl
		<< "#define ACADO_QPOASES 0" << endl
		<< "/** FORCES QP solver indicator.*/" << endl
		<< "#define ACADO_FORCES  1" << endl
		<< "/** qpDUNES QP solver indicator.*/" << endl
		<< "#define ACADO_QPDUNES 2" << endl
		<< "/** Indicator for determining the QP solver used by the ACADO solver code. */" << endl;

	acadoHeader.addStatement( s.str().c_str() );
	switch ( (QPSolverName)qpSolver )
	{
	case QP_QPOASES:
		acadoHeader.addStatement( (String)"#define ACADO_QP_SOLVER ACADO_QPOASES\n" );

		break;

	case QP_FORCES:
		acadoHeader.addStatement( (String)"#define ACADO_QP_SOLVER ACADO_FORCES\n" );

		break;

	case QP_QPDUNES:
		acadoHeader.addStatement( (String)"#define ACADO_QP_SOLVER ACADO_QPDUNES\n" );

		break;

	default:
		return ACADOERROR( RET_INVALID_OPTION );

	}

	int fixInitialState;
	get(FIX_INITIAL_STATE, fixInitialState);
	int useAC;
	get(CG_USE_ARRIVAL_COST, useAC);
	int covCalc;
	get(CG_COMPUTE_COVARIANCE_MATRIX, covCalc);

	s.str( string() );
	s	<< "/** Indicator for fixed initial state. */" << endl
		<< "#define ACADO_INITIAL_STATE_FIXED " << fixInitialState << endl
		<< "/** Indicator for type of fixed weighting matrices. */" << endl
		<< "#define ACADO_WEIGHTING_MATRICES_TYPE " << (unsigned)solver->weightingMatricesType() << endl
		<< "/** Flag indicating whether constraint values are hard-coded or not. */" << endl
		<< "#define ACADO_HARDCODED_CONSTRAINT_VALUES " << hardcodeConstraintValues << endl
		<< "/** Providing interface for arrival cost. */" << endl
		<< "#define ACADO_USE_ARRIVAL_COST " << useAC << endl
		<< "/** Compute covariance matrix of the last state estimate. */" << endl
		<< "#define ACADO_COMPUTE_COVARIANCE_MATRIX " << covCalc << endl;
	acadoHeader.addStatement( s.str().c_str() );

	acadoHeader.addLinebreak( 1 );

	//
	// ACADO variables and workspace
	//
	acadoHeader.addStatement(
			"/*\n"
			" * Globally used structure definitions\n"
			" */\n\n"
	);

	acadoHeader.addStatement(
			"/** The structure containing the user data.\n"
			" * \n"
			" *  Via this structure the user \"communicates\" with the solver code.\n"
			" */\n"
	);
	acadoHeader.addStatement( "typedef struct ACADOvariables_\n{\n" );

	if ( collectDataDeclarations( acadoHeader,ACADO_VARIABLES ) != SUCCESSFUL_RETURN )
		return ACADOERROR( RET_UNABLE_TO_EXPORT_CODE );

	acadoHeader.addStatement( "} ACADOvariables;\n" );
	acadoHeader.addLinebreak( 2 );

	acadoHeader.addStatement(
			"/** Private workspace used by the auto-generated code.\n"
			" * \n"
			" *  Data members of this structure are private to the solver.\n"
			" *  In other words, the user code should not modify values of this \n"
			" *  structure. \n"
			" */\n"
	);
	acadoHeader.addStatement( "typedef struct ACADOworkspace_\n{\n" );

	if (collectDataDeclarations(acadoHeader, ACADO_WORKSPACE) != SUCCESSFUL_RETURN )
		return ACADOERROR( RET_UNABLE_TO_EXPORT_CODE );

	acadoHeader.addStatement( "} ACADOworkspace;\n" );
	acadoHeader.addLinebreak( 2 );

	acadoHeader.addStatement(
			"/* \n"
			" * Forward function declarations. \n"
			" */\n\n"
	);

	if (collectFunctionDeclarations( acadoHeader ) != SUCCESSFUL_RETURN)
		return ACADOERROR( RET_UNABLE_TO_EXPORT_CODE );
	acadoHeader.addLinebreak( 2 );

	acadoHeader.addStatement(
			"/* \n"
			" * Extern declarations. \n"
			" */\n\n"
			"extern ACADOworkspace acadoWorkspace;\n"
			"extern ACADOvariables acadoVariables;\n"
	);
	acadoHeader.addLinebreak( 2 );

	acadoHeader.addStatement(
			"/** @} */\n\n"
	);

	acadoHeader.addStatement(
			"#ifndef __MATLAB__\n"
			"#ifdef __cplusplus\n"
			"} /* extern \"C\" */\n"
			"#endif /* __cplusplus */\n"
			"#endif /* __MATLAB__ */\n\n"
			"#endif /* Close the module */\n"
	);

	return acadoHeader.exportCode( );
}


CLOSE_NAMESPACE_ACADO
