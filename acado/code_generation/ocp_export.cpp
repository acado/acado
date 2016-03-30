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
 *    \file source/code_generation/ocp_export.cpp
 *    \authors Hans Joachim Ferreau, Boris Houska, Milan Vukov
 *    \date 2010 - 2014
 */

#include <acado/code_generation/ocp_export.hpp>
#include <acado/code_generation/export_nlp_solver.hpp>
#include <acado/code_generation/export_simulink_interface.hpp>
#include <acado/code_generation/export_auxiliary_functions.hpp>
#include <acado/code_generation/export_hessian_regularization.hpp>
#include <acado/code_generation/export_common_header.hpp>
#include <acado/code_generation/export_data_internal.hpp>

#include <acado/code_generation/export_gauss_newton_block_cn2.hpp>
#include <acado/code_generation/export_gauss_newton_forces.hpp>

#include <acado/code_generation/templates/templates.hpp>

#include <acado/code_generation/integrators/rk_export.hpp>

#include <acado/objective/objective.hpp>
#include <acado/ocp/ocp.hpp>

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
    
	string moduleName, modulePrefix;
	get(CG_MODULE_NAME, moduleName);
    get(CG_MODULE_PREFIX, modulePrefix);
    
    ExportDataInternal::fcnPrefix = moduleName;
    ExportStatement::fcnPrefix = moduleName;
    ExportStatement::varPrefix = modulePrefix;

	acadoPrintCopyrightNotice( "Code Generation Tool" );

	//
	// Create the export folders
	//

	set(CG_EXPORT_FOLDER_NAME, dirName);

	returnValue dirStatus = acadoCreateFolder( dirName );
	if (dirStatus != SUCCESSFUL_RETURN)
		return dirStatus;

	//
	// Setup the export structures
	//
	returnValue setupStatus = setup( );
	if ( setupStatus != SUCCESSFUL_RETURN )
		return setupStatus;

	//
	// Export common header
	//
	if (exportAcadoHeader(dirName, commonHeaderName, _realString, _intString, _precision)
			!= SUCCESSFUL_RETURN )
		return ACADOERROR( RET_UNABLE_TO_EXPORT_CODE );

	//
	// Export integrator
	//
	if (integrator != 0)
	{
		ExportFile integratorFile(dirName + "/" + moduleName + "_integrator.c",
				commonHeaderName, _realString, _intString, _precision);

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
		ExportFile solverFile(dirName + "/" + moduleName + "_solver.c",
				commonHeaderName, _realString, _intString, _precision);

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
			dirName + string("/") + moduleName + "_auxiliary_functions.h",
			dirName + string("/") + moduleName + "_auxiliary_functions.c",
			moduleName,
            modulePrefix
			);
	eaf.configure();
	eaf.exportCode();

	//
	// Export Makefile
	//
	int generateMakeFile;
	get(GENERATE_MAKE_FILE, generateMakeFile);
	int hessianApproximation;
	get( HESSIAN_APPROXIMATION, hessianApproximation );
	int hessianRegularization;
	get( HESSIAN_REGULARIZATION, hessianRegularization );

	if ( (bool)generateMakeFile == true )
	{
        ExportTemplatedFile makefile;
        makefile.dictionary[ "@MODULE_NAME@" ] = moduleName;
        makefile.dictionary[ "@MODULE_PREFIX@" ] = modulePrefix;
		str = dirName + "/Makefile";

		switch ( (QPSolverName)qpSolver )
		{
			case QP_QPOASES:
				if ( (HessianApproximationMode)hessianApproximation == EXACT_HESSIAN ) {
					acadoCopyTemplateFile(MAKEFILE_EH_QPOASES, str, "#", true);
				}
				else {
					acadoCopyTemplateFile(MAKEFILE_QPOASES, str, "#", true);
				}
				break;

			case QP_QPOASES3:
				if ( (HessianApproximationMode)hessianApproximation == EXACT_HESSIAN ) {
					acadoCopyTemplateFile(MAKEFILE_EH_QPOASES3, str, "#", true);
				}
				else {
					acadoCopyTemplateFile(MAKEFILE_QPOASES3, str, "#", true);
				}
				break;

			case QP_FORCES:
				//acadoCopyTemplateFile(MAKEFILE_FORCES, str, "#", true);
                makefile.setup( MAKEFILE_FORCES,str,"","real_t","int",16,"#" );
                makefile.configure();
                makefile.exportCode();
				break;

			case QP_QPDUNES:
				if ( (HessianApproximationMode)hessianApproximation == EXACT_HESSIAN ) {
					acadoCopyTemplateFile(MAKEFILE_EH_QPDUNES, str, "#", true);
				}
				else {
					acadoCopyTemplateFile(MAKEFILE_QPDUNES, str, "#", true);
				}
				break;

			case QP_HPMPC:
				//acadoCopyTemplateFile(MAKEFILE_HPMPC, str, "#", true);
                makefile.setup( MAKEFILE_HPMPC,str,"","real_t","int",16,"#" );
                makefile.configure();
                makefile.exportCode();
				break;

			default:
				ACADOWARNINGTEXT(RET_NOT_IMPLEMENTED_YET, "Makefile is not yet available.");
				break;
		}
	}

	//
	// Export a dummy test file
	//
	int generateTestFile;
	get(GENERATE_TEST_FILE, generateTestFile);
	string testFileName = dirName + "/test.c";
	if ((bool) generateTestFile == true)
    {
		//acadoCopyTemplateFile(DUMMY_TEST_FILE, testFileName, "", true);
        ExportTemplatedFile testFile;
        testFile.dictionary[ "@MODULE_NAME@" ] = moduleName;
        testFile.dictionary[ "@MODULE_PREFIX@" ] = modulePrefix;

        testFile.setup( DUMMY_TEST_FILE,testFileName );
        testFile.configure();
        testFile.exportCode();
    }

	//
	// Generate MATLAB MEX interface
	//
	int qpSolution;
	get(SPARSE_QP_SOLUTION, qpSolution);
	int generateMexInterface;
	get(GENERATE_MATLAB_INTERFACE, generateMexInterface);
	if ( (bool)generateMexInterface == true )
	{
        ExportTemplatedFile mexInterface;
        mexInterface.dictionary[ "@MODULE_NAME@" ] = moduleName;
        mexInterface.dictionary[ "@MODULE_PREFIX@" ] = modulePrefix;
		str = dirName + "/" + moduleName + "_solver_mex.c";

		if ( (HessianApproximationMode)hessianApproximation == EXACT_HESSIAN ) {
            //acadoCopyTemplateFile(EH_SOLVER_MEX, str, "", true);
            mexInterface.setup( EH_SOLVER_MEX,str  );
            mexInterface.configure();
            mexInterface.exportCode();
		}
		else {
			//acadoCopyTemplateFile(SOLVER_MEX, str, "", true);
            mexInterface.setup( SOLVER_MEX,str  );
            mexInterface.configure();
            mexInterface.exportCode();
		}

        ExportTemplatedFile mexInterfaceMake;
        mexInterfaceMake.dictionary[ "@MODULE_NAME@" ] = moduleName;
        mexInterfaceMake.dictionary[ "@MODULE_PREFIX@" ] = modulePrefix;
		str = dirName + "/make_" + moduleName + "_solver.m";

		switch ( (QPSolverName)qpSolver )
		{
		case QP_QPOASES:
			if ( (HessianApproximationMode)hessianApproximation == EXACT_HESSIAN ) {
				//acadoCopyTemplateFile(MAKE_MEX_EH_QPOASES, str, "%", true);
                mexInterfaceMake.setup( MAKE_MEX_EH_QPOASES,str, "","real_t","int",16,"%" );
                mexInterfaceMake.configure();
                mexInterfaceMake.exportCode();
			}
			else {
				//acadoCopyTemplateFile(MAKE_MEX_QPOASES, str, "%", true);
                mexInterfaceMake.setup( MAKE_MEX_QPOASES,str, "","real_t","int",16,"%" );
                mexInterfaceMake.configure();
                mexInterfaceMake.exportCode();
			}
			break;

		case QP_QPOASES3:
			if ( (HessianApproximationMode)hessianApproximation == EXACT_HESSIAN ) {
				//acadoCopyTemplateFile(MAKE_MEX_EH_QPOASES3, str, "%", true);
                mexInterfaceMake.setup( MAKE_MEX_EH_QPOASES3,str, "","real_t","int",16,"%" );
                mexInterfaceMake.configure();
                mexInterfaceMake.exportCode();
			}
			else {
				//acadoCopyTemplateFile(MAKE_MEX_QPOASES3, str, "%", true);
                mexInterfaceMake.setup( MAKE_MEX_QPOASES3,str, "","real_t","int",16,"%" );
                mexInterfaceMake.configure();
                mexInterfaceMake.exportCode();
			}
			break;

		case QP_FORCES:
			//acadoCopyTemplateFile(MAKE_MEX_FORCES, str, "%", true);
            mexInterfaceMake.setup( MAKE_MEX_FORCES,str, "","real_t","int",16,"%" );
            mexInterfaceMake.configure();
            mexInterfaceMake.exportCode();
			break;

		case QP_QPDUNES:
			if ( (HessianApproximationMode)hessianApproximation == EXACT_HESSIAN ) {
				//acadoCopyTemplateFile(MAKE_MEX_EH_QPDUNES, str, "%", true);
                mexInterfaceMake.setup( MAKE_MEX_EH_QPDUNES,str, "","real_t","int",16,"%" );
                mexInterfaceMake.configure();
                mexInterfaceMake.exportCode();
			}
			else if ( (SparseQPsolutionMethods)qpSolution == BLOCK_CONDENSING_N2 ) {
				acadoCopyTemplateFile(MAKE_MEX_BLOCK_QPDUNES, str, "%", true);
			}
			else {
				//acadoCopyTemplateFile(MAKE_MEX_QPDUNES, str, "%", true);
                mexInterfaceMake.setup( MAKE_MEX_QPDUNES,str, "","real_t","int",16,"%" );
                mexInterfaceMake.configure();
                mexInterfaceMake.exportCode();
			}
			break;

		default:
			ACADOWARNINGTEXT(RET_NOT_IMPLEMENTED_YET, "MEX interface is not yet available.");
			break;
		}
	}

	//
	// Generate MATLAB Simulink interface
	//
	int generateSimulinkInterface;
	get(GENERATE_SIMULINK_INTERFACE, generateSimulinkInterface);
	if ((bool) generateSimulinkInterface == true)
	{     
		if (!((QPSolverName)qpSolver == QP_QPOASES || (QPSolverName)qpSolver == QP_QPOASES3 || (QPSolverName)qpSolver == QP_QPDUNES))
			ACADOWARNINGTEXT(RET_NOT_IMPLEMENTED_YET,
					"At the moment, Simulink interface is available only with qpOASES, qpOASES3 and qpDUNES based OCP solvers.");
		else
		{
			string makefileName = dirName + "/make_" + moduleName + "_solver_sfunction.m";
			string wrapperHeaderName = dirName + "/" + moduleName + "_solver_sfunction.h";
			string wrapperSourceName = dirName + "/" + moduleName + "_solver_sfunction.c";
			string qpSolverString;

			if ((QPSolverName)qpSolver == QP_QPOASES)
				qpSolverString = "QPOASES";
			else if ((QPSolverName)qpSolver == QP_QPOASES3)
				qpSolverString = "QPOASES3";
			else
				qpSolverString = "QPDUNES";

			ExportSimulinkInterface esi(makefileName, wrapperHeaderName, wrapperSourceName, moduleName, modulePrefix);

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
					ocp.getN(), ocp.getNX(), ocp.getNDX(), ocp.getNXA(), ocp.getNU(), ocp.getNOD(),
					solver->getNY(), solver->getNYN(),
					(bool)fixInitialState,
					(unsigned)solver->weightingMatricesType(),
					(bool)hardcodeConstraintValues,
					(bool)useAC,
					(bool)covCalc,
					qpSolverString);

			esi.exportCode();
		}
	}

	//
	// Generate Symmetric EVD code
	//
	if ( (HessianApproximationMode)hessianApproximation == EXACT_HESSIAN && (HessianRegularizationMode)hessianRegularization == BLOCK_REG ) {
//		LOG( LVL_DEBUG ) << "Exporting Hessian regularization code... " << endl;
		ExportHessianRegularization evd(
				dirName + string("/") + moduleName + "_hessian_regularization.c",
				moduleName
		);
		evd.configure( ocp.getNX()+ocp.getNU(), 1e-12 );
		if ( evd.exportCode() != SUCCESSFUL_RETURN )
			return ACADOERROR( RET_UNABLE_TO_EXPORT_CODE );
	}
	else if( (HessianApproximationMode)hessianApproximation == EXACT_HESSIAN && (HessianRegularizationMode)hessianRegularization == CONDENSED_REG ) {
		//		LOG( LVL_DEBUG ) << "Exporting Hessian regularization code... " << endl;
		ExportHessianRegularization evd(
				dirName + string("/") + moduleName + "_hessian_regularization.c",
				moduleName
		);
		int fixInitialState;
		get(FIX_INITIAL_STATE, fixInitialState);
		if( (bool)fixInitialState == 1 ) {
			evd.configure( ocp.getN()*ocp.getNU(), 1e-12 );
		}
		else {
			evd.configure( ocp.getNX()+ocp.getN()*ocp.getNU(), 1e-12 );
		}
		if ( evd.exportCode() != SUCCESSFUL_RETURN )
			return ACADOERROR( RET_UNABLE_TO_EXPORT_CODE );
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
 		return returnvalue;

 	//
 	// Set common header name
 	//
	string moduleName;
	get(CG_MODULE_NAME, moduleName);
 	commonHeaderName = moduleName + "_common.h";

	//
	// Prepare integrator export
	//
	int numSteps;
	get(NUM_INTEGRATOR_STEPS, numSteps);

	int integratorType;
	get(INTEGRATOR_TYPE, integratorType);

	integrator = IntegratorExportPtr(
			IntegratorExportFactory::instance().createAlgorithm(this, commonHeaderName, static_cast<ExportIntegratorType>(integratorType)));
	if (integrator == 0)
		return ACADOERROR( RET_INVALID_OPTION );

	ocp.setNumberIntegrationSteps( numSteps );
	// NOTE: This function internally calls setup() function
	returnvalue = integrator->setModelData( ocp.getModelData() );
 	if ( returnvalue != SUCCESSFUL_RETURN )
 		return returnvalue;

	//
	// Prepare solver export
	//

	int qpSolver;
	get(QP_SOLVER, qpSolver);
	int qpSolution;
	get(SPARSE_QP_SOLUTION, qpSolution);
	int hessianApproximation;
 	get( HESSIAN_APPROXIMATION, hessianApproximation );

	// TODO Extend ExportNLPSolver ctor to accept OCP reference.

	switch ( (SparseQPsolutionMethods)qpSolution )
	{
	case FULL_CONDENSING:
	case CONDENSING:

		if ( ((QPSolverName)qpSolver != QP_QPOASES) && ((QPSolverName)qpSolver != QP_QPOASES3) )
			return ACADOERRORTEXT(RET_INVALID_ARGUMENTS,
					"For condensed solution only qpOASES and qpOASES3 QP solver are supported");

		solver = ExportNLPSolverPtr(
				NLPSolverFactory::instance().createAlgorithm(this, commonHeaderName, GAUSS_NEWTON_CONDENSED));

		break;

	case FULL_CONDENSING_N2:
	case CONDENSING_N2:

		if ( ((QPSolverName)qpSolver != QP_QPOASES) && ((QPSolverName)qpSolver != QP_QPOASES3) )
			return ACADOERRORTEXT(RET_INVALID_ARGUMENTS,
					"For condensed solution only qpOASES and qpOASES3 QP solver are supported");

		if ( (HessianApproximationMode)hessianApproximation == GAUSS_NEWTON ) {
			solver = ExportNLPSolverPtr(
					NLPSolverFactory::instance().createAlgorithm(this, commonHeaderName, GAUSS_NEWTON_CN2));
		}
		else if ( (HessianApproximationMode)hessianApproximation == EXACT_HESSIAN ) {
			solver = ExportNLPSolverPtr(
					NLPSolverFactory::instance().createAlgorithm(this, commonHeaderName, EXACT_HESSIAN_CN2));
		}
		else {
			return ACADOERRORTEXT(RET_INVALID_ARGUMENTS, "Only Gauss-Newton and Exact Hessian methods are currently supported");
		}

		break;

	case BLOCK_CONDENSING_N2:

		if ((QPSolverName)qpSolver != QP_QPDUNES && (QPSolverName)qpSolver != QP_FORCES)
			return ACADOERRORTEXT(RET_INVALID_ARGUMENTS,
					"For block condensed solution only qpDUNES QP solver is currently supported");

		if ( (HessianApproximationMode)hessianApproximation == GAUSS_NEWTON && (QPSolverName)qpSolver == QP_QPDUNES ) {
			solver = ExportNLPSolverPtr(
					NLPSolverFactory::instance().createAlgorithm(this, commonHeaderName, GAUSS_NEWTON_BLOCK_QPDUNES));
		}
		else if ( (HessianApproximationMode)hessianApproximation == GAUSS_NEWTON && (QPSolverName)qpSolver == QP_FORCES ) {
			solver = ExportNLPSolverPtr(
					NLPSolverFactory::instance().createAlgorithm(this, commonHeaderName, GAUSS_NEWTON_BLOCK_FORCES));
		}
		else {
			return ACADOERRORTEXT(RET_INVALID_ARGUMENTS, "Only Gauss-Newton methods are currently supported in combination with block condensing.");
		}

		break;

	case FULL_CONDENSING_N2_FACTORIZATION:

			if ( ((QPSolverName)qpSolver != QP_QPOASES) && ((QPSolverName)qpSolver != QP_QPOASES3) )
				return ACADOERRORTEXT(RET_INVALID_ARGUMENTS,
						"For condensed solution only qpOASES and qpOASES3 QP solver are supported");

			solver = ExportNLPSolverPtr(
					NLPSolverFactory::instance().createAlgorithm(this, commonHeaderName, GAUSS_NEWTON_CN2_FACTORIZATION));

			break;

	case SPARSE_SOLVER:
		if ((QPSolverName)qpSolver != QP_FORCES && (QPSolverName)qpSolver != QP_QPDUNES && (QPSolverName)qpSolver != QP_HPMPC)
			return ACADOERRORTEXT(RET_INVALID_ARGUMENTS,
					"For sparse solution FORCES, qpDUNES and HPMPC QP solvers are supported");
		if ( (QPSolverName)qpSolver == QP_FORCES)
			solver = ExportNLPSolverPtr(
					NLPSolverFactory::instance().createAlgorithm(this, commonHeaderName, GAUSS_NEWTON_FORCES));
		else if ((QPSolverName)qpSolver == QP_QPDUNES) {
			if ( (HessianApproximationMode)hessianApproximation == EXACT_HESSIAN ) {
				solver = ExportNLPSolverPtr(
									NLPSolverFactory::instance().createAlgorithm(this, commonHeaderName, EXACT_HESSIAN_QPDUNES));
			}
			else {
				solver = ExportNLPSolverPtr(
					NLPSolverFactory::instance().createAlgorithm(this, commonHeaderName, GAUSS_NEWTON_QPDUNES));
			}
		}
		else if ((QPSolverName)qpSolver == QP_HPMPC)
			solver = ExportNLPSolverPtr(
					NLPSolverFactory::instance().createAlgorithm(this, commonHeaderName, GAUSS_NEWTON_HPMPC));
		break;

	default:
		return ACADOERRORTEXT(RET_INVALID_ARGUMENTS, "QP solver option is invalid");
	}
	if (solver == 0)
		return ACADOERRORTEXT(RET_INVALID_OPTION, "Cannot allocate the solver object");

	solver->setDimensions(ocp.getNX(), ocp.getNDX(), ocp.getNXA(), ocp.getNU(), ocp.getNP(), ocp.getN(), ocp.getNOD());
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

	returnValue statusSetup;
	statusSetup = solver->setup( );
	if (statusSetup != SUCCESSFUL_RETURN)
		return ACADOERRORTEXT(status, "Error in setting up solver.");

	setStatus( BS_READY );

	return SUCCESSFUL_RETURN;
}


returnValue OCPexport::checkConsistency( ) const
{
	//
	// Consistency checks:
	//
	Objective objective;
	ocp.getObjective( objective );
	int hessianApproximation;
	get( HESSIAN_APPROXIMATION, hessianApproximation );

 	if ( ocp.hasObjective( ) == true && !((HessianApproximationMode)hessianApproximation == EXACT_HESSIAN &&
 			(objective.getNumMayerTerms() == 1 || objective.getNumLagrangeTerms() == 1)) ) { // for Exact Hessian RTI
 		return ACADOERROR( RET_INVALID_OBJECTIVE_FOR_CODE_EXPORT );
 	}


	int sensitivityProp;
	get(DYNAMIC_SENSITIVITY, sensitivityProp);

// 	if( (HessianApproximationMode)hessianApproximation == EXACT_HESSIAN && (ExportSensitivityType) sensitivityProp != THREE_SWEEPS ) {
// 		return ACADOERROR( RET_INVALID_OPTION );
// 	}

 	DifferentialEquation f;
 	ocp.getModel( f );

// 	if ( f.isDiscretized( ) == BT_TRUE )
// 		return ACADOERROR( RET_NO_DISCRETE_ODE_FOR_CODE_EXPORT );

 	if ( f.getNUI( ) > 0 )
 		return ACADOERROR( RET_INVALID_ARGUMENTS );

 	if ( f.getNP( ) > 0 )
 		return ACADOERRORTEXT(RET_INVALID_ARGUMENTS,
 				"Free parameters are not supported. For the old functionality use OnlineData class.");

 	if ( (HessianApproximationMode)hessianApproximation != GAUSS_NEWTON && (HessianApproximationMode)hessianApproximation != EXACT_HESSIAN )
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
	string moduleName, modulePrefix;
	get(CG_MODULE_NAME, moduleName);
    get(CG_MODULE_PREFIX, modulePrefix);

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

	int linSolver;
	get(LINEAR_ALGEBRA_SOLVER, linSolver);
	bool useComplexArithmetic = false;

	if( (LinearAlgebraSolver)linSolver == SIMPLIFIED_IRK_NEWTON ) useComplexArithmetic = true;

	string fileName;
	fileName = _dirName + "/" + _fileName;

	map<string, pair<string, string> > options;

	options[ modulePrefix + "_N" ]   = make_pair(toString( ocp.getN() ),   "Number of control/estimation intervals.");
	options[ modulePrefix + "_NX" ]  = make_pair(toString( ocp.getNX() ),  "Number of differential variables.");
	options[ modulePrefix + "_NXD" ] = make_pair(toString( ocp.getNDX() ), "Number of differential derivative variables.");
	options[ modulePrefix + "_NXA" ] = make_pair(toString( ocp.getNXA() ), "Number of algebraic variables.");
	options[ modulePrefix + "_NU" ]  = make_pair(toString( ocp.getNU() ),  "Number of control variables.");
	options[ modulePrefix + "_NOD" ]  = make_pair(toString( ocp.getNOD() ),  "Number of online data values.");
	options[ modulePrefix + "_NY" ]  = make_pair(toString( solver->getNY() ),  "Number of references/measurements per node on the first N nodes.");
	options[ modulePrefix + "_NYN" ] = make_pair(toString( solver->getNYN() ), "Number of references/measurements on the last (N + 1)st node.");

	Grid integrationGrid;
	ocp.getIntegrationGrid(integrationGrid);
	uint NIS = integrationGrid.getNumIntervals();
	if( ocp.hasEquidistantControlGrid() ) options[ modulePrefix + "_RK_NIS" ] = make_pair(toString( NIS ),   "Number of integration steps per shooting interval.");

	RungeKuttaExport *rk_integrator = static_cast<RungeKuttaExport *>(integrator.get());  // Note: As long as only Runge-Kutta type methods are exported.
	options[ modulePrefix + "_RK_NSTAGES" ] = make_pair(toString( rk_integrator->getNumStages() ),   "Number of Runge-Kutta stages per integration step.");

	options[ modulePrefix + "_INITIAL_STATE_FIXED" ] =
			make_pair(toString( fixInitialState ), "Indicator for fixed initial state.");
	options[ modulePrefix + "_WEIGHTING_MATRICES_TYPE" ] =
			make_pair(toString( (unsigned)solver->weightingMatricesType() ), "Indicator for type of fixed weighting matrices.");
	options[ modulePrefix + "_USE_LINEAR_TERMS" ] =
				make_pair(toString( (unsigned)solver->usingLinearTerms() ), "Indicator for usage of non-hard-coded linear terms in the objective.");
	options[ modulePrefix + "_HARDCODED_CONSTRAINT_VALUES" ] =
			make_pair(toString( hardcodeConstraintValues ), "Flag indicating whether constraint values are hard-coded or not.");
	options[ modulePrefix + "_USE_ARRIVAL_COST" ] =
			make_pair(toString( useAC ), "Providing interface for arrival cost.");
	options[ modulePrefix + "_COMPUTE_COVARIANCE_MATRIX" ] =
			make_pair(toString( covCalc ), "Compute covariance matrix of the last state estimate.");
	options[ modulePrefix + "_QP_NV" ] =
			make_pair(toString( solver->getNumQPvars() ), "Total number of QP optimization variables.");

	int qpSolution;
	get(SPARSE_QP_SOLUTION, qpSolution);
	if( (QPSolverName)qpSolver == QP_FORCES && (SparseQPsolutionMethods)qpSolution != BLOCK_CONDENSING_N2 ) {
		ExportGaussNewtonForces *blockSolver = static_cast<ExportGaussNewtonForces*>(solver.get());
		options[ modulePrefix + "_QP_NLB" ] =
				make_pair(toString( blockSolver->getNumLowerBounds() ), "Total number of QP lower bound values.");
		options[ modulePrefix + "_QP_NUB" ] =
				make_pair(toString( blockSolver->getNumUpperBounds() ), "Total number of QP upper bound values.");
	}

	// QPDunes block based condensing:
	if ( (SparseQPsolutionMethods)qpSolution == BLOCK_CONDENSING_N2 ) {
		ExportGaussNewtonBlockCN2 *blockSolver = static_cast<ExportGaussNewtonBlockCN2*>(solver.get());

		options[ modulePrefix + "_BLOCK_CONDENSING" ] =
				make_pair(toString( 1 ), "User defined block based condensing.");
		options[ modulePrefix + "_QP_NCA" ] =
				make_pair(toString( blockSolver->getNumStateBoundsPerBlock()*blockSolver->getNumberOfBlocks() ), "Total number of QP affine constraints.");
	}
	else {
		options[ modulePrefix + "_BLOCK_CONDENSING" ] =
						make_pair(toString( 0 ), "User defined block based condensing.");
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
	ech.configure( moduleName, modulePrefix, useSinglePrecision, useComplexArithmetic, (QPSolverName)qpSolver,
			options, variables.str(), workspace.str(), functions.str());

	return ech.exportCode();
}

CLOSE_NAMESPACE_ACADO
