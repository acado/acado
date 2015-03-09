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
 *    \file src/code_generation/export_gauss_newton_block_forces.cpp
 *    \author Rien Quirynen
 *    \date 2014
 */

#include <acado/code_generation/export_gauss_newton_block_forces.hpp>

#include <acado/code_generation/templates/templates.hpp>

using namespace std;

BEGIN_NAMESPACE_ACADO

ExportGaussNewtonBlockForces::ExportGaussNewtonBlockForces(	UserInteraction* _userInteraction,
											const std::string& _commonHeaderName
											) : ExportGaussNewtonBlockCN2( _userInteraction,_commonHeaderName )
{
	qpObjPrefix = "acadoForces";
	qpModuleName = "forces";
}

returnValue ExportGaussNewtonBlockForces::setup( )
{

	returnValue status = ExportGaussNewtonBlockCN2::setup();

	return ACADOERROR( RET_NOT_IMPLEMENTED_YET );

	LOG( LVL_DEBUG ) << "Solver: setup extra initialization... " << endl;
	// Add QP initialization call to the initialization
	ExportFunction initializeForces( "initializeForces" );
	initialize.addFunctionCall( initializeForces );
	LOG( LVL_DEBUG ) << "done!" << endl;

	return SUCCESSFUL_RETURN;
}

returnValue ExportGaussNewtonBlockForces::getCode(	ExportStatementBlock& code
											)
{
	setupQPInterface();
	code.addStatement( *qpInterface );

	code.addFunction( cleanup );
	code.addFunction( shiftQpData );

	code.addFunction( evaluateConstraints );

	return ExportGaussNewtonCN2::getCode( code );
}

returnValue ExportGaussNewtonBlockForces::setupEvaluation( )
{
	////////////////////////////////////////////////////////////////////////////
	//
	// Preparation phase
	//
	////////////////////////////////////////////////////////////////////////////

	preparation.setup( "preparationStep" );
	preparation.doc( "Preparation step of the RTI scheme." );
	ExportVariable retSim("ret", 1, 1, INT, ACADO_LOCAL, true);
	retSim.setDoc("Status of the integration module. =0: OK, otherwise the error code.");
	preparation.setReturnValue(retSim, false);
	ExportIndex index("index");
	preparation.addIndex( index );

	preparation	<< retSim.getFullName() << " = " << modelSimulation.getName() << "();\n";

	preparation.addFunctionCall( evaluateObjective );
	if( regularizeHessian.isDefined() ) preparation.addFunctionCall( regularizeHessian );
	preparation.addFunctionCall( evaluateConstraints );

	preparation.addLinebreak();
	preparation << (Dy -= y) << (DyN -= yN);
	preparation.addLinebreak();

	ExportForLoop condensePrepLoop( index, 0, getNumberOfBlocks() );
	condensePrepLoop.addFunctionCall( condensePrep, index );
	preparation.addStatement( condensePrepLoop );

	preparation.addStatement( qpH.getRows(getNumberOfBlocks()*getNumBlockVariables()*getNumBlockVariables(),getNumberOfBlocks()*getNumBlockVariables()*getNumBlockVariables()+NX*NX) == QN1.makeColVector() );
	if( levenbergMarquardt > ZERO_EPS ) {
		for( uint i = 0; i < NX; i++ ) {
			preparation.addStatement( qpH.getElement(getNumberOfBlocks()*getNumBlockVariables()*getNumBlockVariables()+i*NX+i,0) += levenbergMarquardt );
		}
	}
	preparation.addLinebreak();

	preparation.addStatement( g.getRows(getNumberOfBlocks()*getNumBlockVariables(), getNumQPvars()) == QN2 * DyN );
	int variableObjS;
	get(CG_USE_VARIABLE_WEIGHTING_MATRIX, variableObjS);
	ExportVariable SlxCall =
				objSlx.isGiven() == true || variableObjS == false ? objSlx : objSlx.getRows(N * NX, (N + 1) * NX);
	preparation.addStatement( g.getRows(getNumberOfBlocks()*getNumBlockVariables(), getNumQPvars()) += SlxCall );
	preparation.addLinebreak();

	stringstream prep;
	prep << retSim.getName() << " = prepareQpDunes( );" << endl;
	preparation << prep.str();
	preparation.addLinebreak();

	////////////////////////////////////////////////////////////////////////////
	//
	// Feedback phase
	//
	////////////////////////////////////////////////////////////////////////////

	ExportVariable tmp("tmp", 1, 1, INT, ACADO_LOCAL, true);
	tmp.setDoc( "Status code of the qpOASES QP solver." );

	feedback.setup("feedbackStep");
	feedback.doc( "Feedback/estimation step of the RTI scheme." );
	feedback.setReturnValue( tmp );
	feedback.addIndex( index );

	if (initialStateFixed() == true)
	{
		feedback.addStatement( qpLb0.getTranspose().getRows(0, NX) == x0 - x.getRow( 0 ).getTranspose() );
		feedback.addStatement( qpUb0.getCols(0, NX) == qpLb0.getCols(0, NX) );
	}
	else
	{
		feedback << (qpgN == g.getRows(getNumberOfBlocks()*getNumBlockVariables(), getNumQPvars()));
	}
	feedback.addLinebreak();

	stringstream s;
	s << tmp.getName() << " = solveQpDunes( );" << endl;
	feedback <<  s.str();
	feedback.addLinebreak();

	ExportForLoop expandLoop( index, 0, getNumberOfBlocks() );
	expandLoop.addFunctionCall( expand, index );
	feedback.addStatement( expandLoop );

	feedback.addStatement( (x.getRow(getNumberOfBlocks()*getBlockSize())).getTranspose() += xVars.getRows(getNumberOfBlocks()*getNumBlockVariables(), getNumberOfBlocks()*getNumBlockVariables()+NX) );

	////////////////////////////////////////////////////////////////////////////
	//
	// Setup evaluation of the KKT tolerance
	//
	////////////////////////////////////////////////////////////////////////////

	ExportVariable kkt("kkt", 1, 1, REAL, ACADO_LOCAL, true);

	getKKT.setup( "getKKT" );
	getKKT.doc( "Get the KKT tolerance of the current iterate. Under development." );
	//	kkt.setDoc( "The KKT tolerance value." );
	kkt.setDoc( "1e-15." );
	getKKT.setReturnValue( kkt );

	getKKT.addStatement( kkt == 1e-15 );

	return SUCCESSFUL_RETURN;
}

returnValue ExportGaussNewtonBlockForces::setupQPInterface( )
{
	//
	// Configure and export QP interface
	//

	qpInterface = std::tr1::shared_ptr< ExportForcesInterface >(new ExportForcesInterface(FORCES_TEMPLATE, "", commonHeaderName));

	ExportVariable tmp1("tmp", 1, 1, REAL, FORCES_PARAMS, false, qpObjPrefix);
	ExportVariable tmp2("tmp", 1, 1, REAL, FORCES_OUTPUT, false, qpObjPrefix);
	ExportVariable tmp3("tmp", 1, 1, REAL, FORCES_INFO, false, qpObjPrefix);

	string params = qpModuleName + "_params";

	string output = qpModuleName + "_output";

	string info = qpModuleName + "_info";

	string header = qpModuleName + ".h";

	qpInterface->configure(
			header,

			params,
			tmp1.getDataStructString(),

			output,
			tmp2.getDataStructString(),

			info,
			tmp3.getDataStructString()
	);

	//
	// Configure and export MATLAB QP generator
	//

	string folderName;
	get(CG_EXPORT_FOLDER_NAME, folderName);
	string outFile = folderName + "/acado_forces_generator.m";

	qpGenerator = std::tr1::shared_ptr< ExportForcesGenerator >(new ExportForcesGenerator(FORCES_GENERATOR, outFile, "", "real_t", "int", 16, "%"));

	int maxNumQPiterations;
	get( MAX_NUM_QP_ITERATIONS,maxNumQPiterations );

	int printLevel;
	get(PRINTLEVEL, printLevel);

	// if not specified, use default value
	if ( maxNumQPiterations <= 0 )
		maxNumQPiterations = 3 * getNumQPvars();

	int useOMP;
	get(CG_USE_OPENMP, useOMP);

	int hotstartQP;
	get(HOTSTART_QP, hotstartQP);

//	qpGenerator->configure(
//			NX,
//			NU,
//			N,
//			conLBIndices,
//			conUBIndices,
//			conABIndices,
//			(Q1.isGiven() == true && R1.isGiven() == true) ? 1 : 0,
//					diagonalH,
//					diagonalHN,
//					initialStateFixed(),
//					qpModuleName,
//					(PrintLevel)printLevel == HIGH ? 2 : 0,
//							maxNumQPiterations,
//							useOMP,
//							true,
//							hotstartQP
//	);

	qpGenerator->exportCode();

	//
	// Export Python generator
	//
	ACADOWARNINGTEXT(RET_NOT_IMPLEMENTED_YET,
			"A python code generator interface for block condensing with FORCES is under development.");

	return SUCCESSFUL_RETURN;
}


CLOSE_NAMESPACE_ACADO
