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
#include <acado/code_generation/export_common_header.hpp>

#include <acado/code_generation/templates/templates.hpp>

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

returnValue OCPexport::exportCode(	const std::string& dirName,
									const std::string& _realString,
									const std::string& _intString,
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

	returnValue dirStatus = acadoCreateFolder( dirName );
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
	std::string str;

	ExportAuxiliaryFunctions eaf(
			dirName + string("/") + getName() + "_auxiliary_functions.h",
			dirName + string("/") + getName() + "_auxiliary_functions.c",
			getName()
			);
	eaf.configure();
	eaf.exportCode();

	//
	// Export Makefile
	//
	int generateMakeFile;
	get(GENERATE_MAKE_FILE, generateMakeFile);

	if ( (bool)generateMakeFile == true )
	{
		str = dirName + "/Makefile";

		switch ( (QPSolverName)qpSolver )
		{
			case QP_QPOASES:
				acadoCopyTempateFile(MAKEFILE_QPOASES, str, "#", true);
				break;

			case QP_FORCES:
				acadoCopyTempateFile(MAKEFILE_FORCES, str, "#", true);
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
	string testFileName = dirName + "/test.c";
	if ((bool) generateTestFile == true)
		acadoCopyTempateFile(DUMMY_TEST_FILE, testFileName, "", true);

	//
	// Generate MATLAB MEX interface
	//
	int generateMexInterface;
	get(GENERATE_MATLAB_INTERFACE, generateMexInterface);
	if ( (bool)generateMexInterface == true )
	{
		str = dirName + "/" + getName() + "_solver_mex.c";

		acadoCopyTempateFile(SOLVER_MEX, str, "", true);

		str = dirName + "/make_" + getName() + "_solver.m";

		switch ( (QPSolverName)qpSolver )
		{
		case QP_QPOASES:
			acadoCopyTempateFile(MAKE_MEX_QPOASES, str, "%", true);
			break;

		case QP_FORCES:
			acadoCopyTempateFile(MAKE_MEX_FORCES, str, "%", true);
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
	if ((bool) generateSimulinkInterface == true)
	{
		if ((QPSolverName)qpSolver != QP_QPOASES)
			return ACADOERRORTEXT(RET_INVALID_ARGUMENTS,
					"At the moment, Simulink interface is available only with qpOASES based OCP solver.");

		string makefileName = dirName + "/make_" + getName() + "_solver_sfunction.m";
		string wrapperHeaderName = dirName + "/" + getName() + "_solver_sfunction.h";
		string wrapperSourceName = dirName + "/" + getName() + "_solver_sfunction.c";

		ExportSimulinkInterface esi(makefileName, wrapperHeaderName, wrapperSourceName, getName());

		// Get options
		int useSinglePrecision;
		get(USE_SINGLE_PRECISION, useSinglePrecision);

		int hardcodeConstraintValues;
		get(CG_HARDCODE_CONSTRAINT_VALUES, hardcodeConstraintValues);
		if ((bool)hardcodeConstraintValues == false)
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
				(bool)fixInitialState,
				(unsigned)solver->weightingMatricesType(),
				(bool)hardcodeConstraintValues,
				(bool)useAC,
				(bool)covCalc);

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

 	if ( ocp.hasObjective( ) == true )
 		return ACADOERROR( RET_INVALID_OBJECTIVE_FOR_CODE_EXPORT );

 	DifferentialEquation f;
 	ocp.getModel( f );

 	if ( f.isDiscretized( ) == true )
 		return ACADOERROR( RET_NO_DISCRETE_ODE_FOR_CODE_EXPORT );

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

returnValue OCPexport::exportAcadoHeader(	const std::string& _dirName,
											const std::string& _fileName,
											const std::string& _realString,
											const std::string& _intString,
											int _precision
											) const
{
	int qpSolver;
	get(QP_SOLVER, qpSolver);

	int useSinglePrecision;
	get(USE_SINGLE_PRECISION, useSinglePrecision);

	int hardcodeConstraintValues;
	get(CG_HARDCODE_CONSTRAINT_VALUES, hardcodeConstraintValues);

	int fixInitialState;
	get(FIX_INITIAL_STATE, fixInitialState);
	int useAC;
	get(CG_USE_ARRIVAL_COST, useAC);
	int covCalc;
	get(CG_COMPUTE_COVARIANCE_MATRIX, covCalc);

	string fileName;
	fileName = _dirName + "/" + _fileName;

	map<string, pair<string, string> > options;

	options[ "ACADO_N" ]   = make_pair(toString( ocp.getN() ),   "Number of control/estimation intervals.");
	options[ "ACADO_NX" ]  = make_pair(toString( ocp.getNX() ),  "Number of differential variables.");
	options[ "ACADO_NXD" ] = make_pair(toString( ocp.getNDX() ), "Number of differential derivative variables.");
	options[ "ACADO_NXA" ] = make_pair(toString( ocp.getNXA() ), "Number of algebraic variables.");
	options[ "ACADO_NU" ]  = make_pair(toString( ocp.getNU() ),  "Number of control variables.");
	options[ "ACADO_NP" ]  = make_pair(toString( ocp.getNP() ),  "Number of parameters (which are NOT optimization variables).");
	options[ "ACADO_NY" ]  = make_pair(toString( solver->getNY() ),  "Number of references/measurements per node on the first N nodes.");
	options[ "ACADO_NYN" ] = make_pair(toString( solver->getNYN() ), "Number of references/measurements on the last (N + 1)st node.");

	options[ "ACADO_INITIAL_STATE_FIXED" ] =
			make_pair(toString( fixInitialState ), "Indicator for fixed initial state.");
	options[ "ACADO_WEIGHTING_MATRICES_TYPE" ] =
			make_pair(toString( (unsigned)solver->weightingMatricesType() ), "Indicator for type of fixed weighting matrices.");
	options[ "ACADO_HARDCODED_CONSTRAINT_VALUES" ] =
			make_pair(toString( hardcodeConstraintValues ), "Flag indicating whether constraint values are hard-coded or not.");
	options[ "ACADO_USE_ARRIVAL_COST" ] =
			make_pair(toString( useAC ), "Providing interface for arrival cost.");
	options[ "ACADO_COMPUTE_COVARIANCE_MATRIX" ] =
			make_pair(toString( covCalc ), "Compute covariance matrix of the last state estimate.");

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
	ech.configure( getName(), useSinglePrecision, (QPSolverName)qpSolver, options, variables.str(), workspace.str(), functions.str());

	return ech.exportCode();
}

CLOSE_NAMESPACE_ACADO
