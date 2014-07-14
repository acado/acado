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
 *    \file src/code_generation/export_gauss_newton_hpmpc.cpp
 *    \author Milan Vukov
 *    \date 2014
 */

#include <acado/code_generation/export_gauss_newton_hpmpc.hpp>
#include <acado/code_generation/templates/templates.hpp>

BEGIN_NAMESPACE_ACADO

using namespace std;

ExportGaussNewtonHpmpc::ExportGaussNewtonHpmpc(	UserInteraction* _userInteraction,
													const std::string& _commonHeaderName
													) : ExportNLPSolver( _userInteraction,_commonHeaderName )
{}

returnValue ExportGaussNewtonHpmpc::setup( )
{
	setupInitialization();

	setupVariables();

	setupSimulation();

	setupObjectiveEvaluation();

	setupConstraintsEvaluation();

	setupEvaluation();

	setupAuxiliaryFunctions();

	return SUCCESSFUL_RETURN;
}

returnValue ExportGaussNewtonHpmpc::getDataDeclarations(	ExportStatementBlock& declarations,
															ExportStruct dataStruct
															) const
{
	returnValue status;
	status = ExportNLPSolver::getDataDeclarations(declarations, dataStruct);
	if (status != SUCCESSFUL_RETURN)
		return status;

	declarations.addDeclaration(x0, dataStruct);

	if (Q1.isGiven() == true)
		declarations.addDeclaration(qpQ, dataStruct);
	if (QN1.isGiven() == true)
		declarations.addDeclaration(qpQf, dataStruct);
	if (S1.isGiven() == true)
		declarations.addDeclaration(qpS, dataStruct);
	if (R1.isGiven() == true)
		declarations.addDeclaration(qpR, dataStruct);

	declarations.addDeclaration(qpq, dataStruct);
	declarations.addDeclaration(qpqf, dataStruct);
	declarations.addDeclaration(qpr, dataStruct);

	declarations.addDeclaration(qpx, dataStruct);
	declarations.addDeclaration(qpu, dataStruct);

	declarations.addDeclaration(qpLb, dataStruct);
	declarations.addDeclaration(qpUb, dataStruct);

	declarations.addDeclaration(nIt, dataStruct);

	return SUCCESSFUL_RETURN;
}

returnValue ExportGaussNewtonHpmpc::getFunctionDeclarations(	ExportStatementBlock& declarations
																) const
{
	declarations.addDeclaration( preparation );
	declarations.addDeclaration( feedback );

	declarations.addDeclaration( initialize );
	declarations.addDeclaration( initializeNodes );
	declarations.addDeclaration( shiftStates );
	declarations.addDeclaration( shiftControls );
	declarations.addDeclaration( getKKT );
	declarations.addDeclaration( getObjective );

	declarations.addDeclaration( evaluateLSQ );
	declarations.addDeclaration( evaluateLSQEndTerm );

	return SUCCESSFUL_RETURN;
}

returnValue ExportGaussNewtonHpmpc::getCode(	ExportStatementBlock& code
												)
{
	setupQPInterface();

	// Forward declaration, same as in the template file.
	code << "#ifdef __cplusplus\n";
	code << "extern \"C\"{\n";
	code << "int acado_hpmpc_ip_wrapper(unsigned N, unsigned nx, unsigned nu, double* A, double* B, double* d, double* Q, double* Qf, double* S, double* R, double* q, double* qf, double* r, double* lb, double* ub, double* x, double* u, int* nIt);\n";
	code << "}\n";
	code << "#endif\n";

	code.addLinebreak( 2 );
	code.addStatement( "/******************************************************************************/\n" );
	code.addStatement( "/*                                                                            */\n" );
	code.addStatement( "/* ACADO code generation                                                      */\n" );
	code.addStatement( "/*                                                                            */\n" );
	code.addStatement( "/******************************************************************************/\n" );
	code.addLinebreak( 2 );

	int useOMP;
	get(CG_USE_OPENMP, useOMP);
	if ( useOMP )
	{
		code.addDeclaration( state );
	}

	code.addFunction( modelSimulation );

	code.addFunction( evaluateLSQ );
	code.addFunction( evaluateLSQEndTerm );
	code.addFunction( setObjQ1Q2 );
	code.addFunction( setObjR1R2 );
	code.addFunction( setObjQN1QN2 );
	code.addFunction( setStagef );
	code.addFunction( evaluateObjective );

	code.addFunction( evaluateConstraints );

	code.addFunction( acc );

	code.addFunction( preparation );
	code.addFunction( feedback );

	code.addFunction( initialize );
	code.addFunction( initializeNodes );
	code.addFunction( shiftStates );
	code.addFunction( shiftControls );
	code.addFunction( getKKT );
	code.addFunction( getObjective );

	return SUCCESSFUL_RETURN;
}


unsigned ExportGaussNewtonHpmpc::getNumQPvars( ) const
{
	return (N + 1) * NX + N * NU;
}

//
// PROTECTED FUNCTIONS:
//

returnValue ExportGaussNewtonHpmpc::setupObjectiveEvaluation( void )
{
	evaluateObjective.setup("evaluateObjective");

	int variableObjS;
	get(CG_USE_VARIABLE_WEIGHTING_MATRIX, variableObjS);

	//
	// LM regularization preparation
	//
	if (levenbergMarquardt > 0.0)
	{
		ACADOWARNINGTEXT(RET_INVALID_ARGUMENTS, "LM regularization is under development.");
	}

	//
	// Main loop that calculates Hessian and gradients
	//

	ExportIndex runObj( "runObj" );
	ExportForLoop loopObjective( runObj, 0, N );

	evaluateObjective.addIndex( runObj );

	loopObjective.addStatement( objValueIn.getCols(0, getNX()) == x.getRow( runObj ) );
	loopObjective.addStatement( objValueIn.getCols(NX, NX + NU) == u.getRow( runObj ) );
	loopObjective.addStatement( objValueIn.getCols(NX + NU, NX + NU + NOD) == od.getRow( runObj ) );
	loopObjective.addLinebreak( );

	// Evaluate the objective function
	loopObjective.addFunctionCall(evaluateLSQ, objValueIn, objValueOut);

	// Stack the measurement function value
	loopObjective.addStatement(
			Dy.getRows(runObj * NY, (runObj + 1) * NY) ==  objValueOut.getTranspose().getRows(0, getNY())
	);
	loopObjective.addLinebreak( );

	// Optionally compute derivatives

	ExportVariable tmpObjS, tmpFx, tmpFu;
	ExportVariable tmpFxEnd, tmpObjSEndTerm;
	tmpObjS.setup("tmpObjS", NY, NY, REAL, ACADO_LOCAL);
	if (objS.isGiven() == true)
		tmpObjS = objS;
	tmpFx.setup("tmpFx", NY, NX, REAL, ACADO_LOCAL);
	if (objEvFx.isGiven() == true)
		tmpFx = objEvFx;
	tmpFu.setup("tmpFu", NY, NU, REAL, ACADO_LOCAL);
	if (objEvFu.isGiven() == true)
		tmpFu = objEvFu;
	tmpFxEnd.setup("tmpFx", NYN, NX, REAL, ACADO_LOCAL);
	if (objEvFxEnd.isGiven() == true)
		tmpFxEnd = objEvFxEnd;
	tmpObjSEndTerm.setup("tmpObjSEndTerm", NYN, NYN, REAL, ACADO_LOCAL);
	if (objSEndTerm.isGiven() == true)
		tmpObjSEndTerm = objSEndTerm;

	unsigned indexX = getNY();
	ExportArgument tmpFxCall = tmpFx;
	if (tmpFx.isGiven() == false)
	{
		tmpFxCall = objValueOut.getAddress(0, indexX);
		indexX += objEvFx.getDim();
	}

	ExportArgument tmpFuCall = tmpFu;
	if (tmpFu.isGiven() == false)
	{
		tmpFuCall = objValueOut.getAddress(0, indexX);
	}

	ExportArgument objSCall = variableObjS == true ? objS.getAddress(runObj * NY, 0) : objS;

	//
	// Optional computation of Q1, Q2
	//
	if (Q1.isGiven() == false)
	{
		ExportVariable tmpQ1, tmpQ2;
		tmpQ1.setup("tmpQ1", NX, NX, REAL, ACADO_LOCAL);
		tmpQ2.setup("tmpQ2", NX, NY, REAL, ACADO_LOCAL);

		setObjQ1Q2.setup("setObjQ1Q2", tmpFx, tmpObjS, tmpQ1, tmpQ2);
		setObjQ1Q2.addStatement( tmpQ2 == (tmpFx ^ tmpObjS) );
		setObjQ1Q2.addStatement( tmpQ1 == tmpQ2 * tmpFx );

		loopObjective.addFunctionCall(
				setObjQ1Q2,
				tmpFxCall, objSCall,
				Q1.getAddress(runObj * NX, 0), Q2.getAddress(runObj * NX, 0)
		);

		loopObjective.addLinebreak( );
	}

	if (R1.isGiven() == false)
	{
		ExportVariable tmpR1, tmpR2;
		tmpR1.setup("tmpR1", NU, NU, REAL, ACADO_LOCAL);
		tmpR2.setup("tmpR2", NU, NY, REAL, ACADO_LOCAL);

		setObjR1R2.setup("setObjR1R2", tmpFu, tmpObjS, tmpR1, tmpR2);
		setObjR1R2.addStatement( tmpR2 == (tmpFu ^ tmpObjS) );
		setObjR1R2.addStatement( tmpR1 == tmpR2 * tmpFu );

		loopObjective.addFunctionCall(
				setObjR1R2,
				tmpFuCall, objSCall,
				R1.getAddress(runObj * NU, 0), R2.getAddress(runObj * NU, 0)
		);

		loopObjective.addLinebreak( );
	}

	if (S1.isGiven() == false)
	{
		ExportVariable tmpS1;
		ExportVariable tmpS2;

		tmpS1.setup("tmpS1", NX, NU, REAL, ACADO_LOCAL);
		tmpS2.setup("tmpS2", NX, NY, REAL, ACADO_LOCAL);

		setObjS1.setup("setObjS1", tmpFx, tmpFu, tmpObjS, tmpS1);
		setObjS1.addVariable( tmpS2 );
		setObjS1.addStatement( tmpS2 == (tmpFx ^ tmpObjS) );
		setObjS1.addStatement( tmpS1 == tmpS2 * tmpFu );

		loopObjective.addFunctionCall(
				setObjS1,
				tmpFxCall, tmpFuCall, objSCall,
				S1.getAddress(runObj * NX, 0)
		);
	}

	evaluateObjective.addStatement( loopObjective );

	//
	// Evaluate the quadratic Mayer term
	//
	evaluateObjective.addStatement( objValueIn.getCols(0, NX) == x.getRow( N ) );
	evaluateObjective.addStatement( objValueIn.getCols(NX, NX + NOD) == od.getRow( N ) );

	// Evaluate the objective function, last node.
	evaluateObjective.addFunctionCall(evaluateLSQEndTerm, objValueIn, objValueOut);
	evaluateObjective.addLinebreak( );

	evaluateObjective.addStatement( DyN.getTranspose() == objValueOut.getCols(0, NYN) );
	evaluateObjective.addLinebreak();

	if (QN1.isGiven() == false)
	{
		ExportVariable tmpQN1, tmpQN2;
		tmpQN1.setup("tmpQN1", NX, NX, REAL, ACADO_LOCAL);
		tmpQN2.setup("tmpQN2", NX, NYN, REAL, ACADO_LOCAL);

		setObjQN1QN2.setup("setObjQN1QN2", tmpFxEnd, tmpObjSEndTerm, tmpQN1, tmpQN2);
		setObjQN1QN2.addStatement( tmpQN2 == (tmpFxEnd ^ tmpObjSEndTerm) );
		setObjQN1QN2.addStatement( tmpQN1 == tmpQN2 * tmpFxEnd );

		indexX = getNYN();
		ExportArgument tmpFxEndCall = tmpFxEnd.isGiven() == true ? tmpFxEnd  : objValueOut.getAddress(0, indexX);

		evaluateObjective.addFunctionCall(
				setObjQN1QN2,
				tmpFxEndCall, objSEndTerm,
				QN1.getAddress(0, 0), QN2.getAddress(0, 0)
		);

		evaluateObjective.addLinebreak( );
	}

	//
	// Hessian setup
	//

	ExportIndex index( "index" );

	//
	// Gradient setup
	//
	ExportVariable qq, rr;
	qq.setup("stageq", NX, 1, REAL, ACADO_LOCAL);
	rr.setup("stager", NU, 1, REAL, ACADO_LOCAL);
	setStagef.setup("setStagef", qq, rr, index);

	if (Q2.isGiven() == false)
		setStagef.addStatement(
				qq == Q2.getSubMatrix(index * NX, (index + 1) * NX, 0, NY) * Dy.getRows(index * NY, (index + 1) * NY)
		);
	else
	{
		setStagef.addStatement( index == index );
		setStagef.addStatement(
				qq == Q2 * Dy.getRows(index * NY, (index + 1) * NY)
		);
	}
	setStagef.addLinebreak();

	if (R2.isGiven() == false)
		setStagef.addStatement(
				rr == R2.getSubMatrix(index * NU, (index + 1) * NU, 0, NY) * Dy.getRows(index * NY, (index + 1) * NY)
		);
	else
	{
		setStagef.addStatement(
				rr == R2 * Dy.getRows(index * NY, (index + 1) * NY)
		);
	}

	//
	// Setup necessary QP variables
	//

	if (Q1.isGiven() == true)
	{
		qpQ.setup("qpQ", N * NX, NX, REAL, ACADO_WORKSPACE);
		for (unsigned blk = 0; blk < N; ++blk)
			initialize.addStatement( qpQ.getSubMatrix(blk * NX, (blk + 1) * NX, 0, NX) == Q1);
	}
	else
	{
		qpQ = Q1;
	}

	if (R1.isGiven() == true)
	{
		qpR.setup("qpR", N * NU, NU, REAL, ACADO_WORKSPACE);
		for (unsigned blk = 0; blk < N; ++blk)
			initialize.addStatement( qpR.getSubMatrix(blk * NU, (blk + 1) * NU, 0, NU) == R1);
	}
	else
	{
		qpR = R1;
	}

	if (S1.isGiven() == true)
	{
		qpS.setup("qpS", N * NX, NU, REAL, ACADO_WORKSPACE);
		if (S1.getGivenMatrix().isZero() == true)
			initialize.addStatement(qpS == zeros<double>(N * NX, NU));
		else
			for (unsigned blk = 0; blk < N; ++blk)
				initialize.addStatement( qpS.getSubMatrix(blk * NX, (blk + 1) * NX, 0, NU) == S1);
	}
	else
	{
		qpS = S1;
	}

	if (QN1.isGiven() == true)
	{
		qpQf.setup("qpQf", NX, NX, REAL, ACADO_WORKSPACE);
		initialize.addStatement( qpQf == QN1 );
	}
	else
	{
		qpQf = QN1;
	}

	return SUCCESSFUL_RETURN;
}

returnValue ExportGaussNewtonHpmpc::setupConstraintsEvaluation( void )
{
	////////////////////////////////////////////////////////////////////////////
	//
	// Setup evaluation of box constraints on states and controls
	//
	////////////////////////////////////////////////////////////////////////////

	evaluateConstraints.setup("evaluateConstraints");

	DVector lbValues, ubValues;
	for(unsigned node = 0; node < N; ++node)
	{
		lbValues.append( uBounds.getLowerBounds( node ) );
		ubValues.append( uBounds.getUpperBounds( node ) );
	}

	for(unsigned node = 1; node < N + 1; ++node)
	{
		lbValues.append( xBounds.getLowerBounds( node ) );
		ubValues.append( xBounds.getUpperBounds( node ) );
	}

	qpLb.setup("qpLb", N * NU + N * NX, 1, REAL, ACADO_WORKSPACE);
	qpUb.setup("qpUb", N * NU + N * NX, 1, REAL, ACADO_WORKSPACE);

	evLbValues.setup("evLbValues", lbValues, STATIC_CONST_REAL, ACADO_LOCAL);
	evUbValues.setup("evUbValues", ubValues, STATIC_CONST_REAL, ACADO_LOCAL);

	evaluateConstraints.addVariable( evLbValues );
	evaluateConstraints.addVariable( evUbValues );

	evaluateConstraints.addStatement( qpLb.getRows(0, N * NU) == evLbValues.getRows(0, N * NU) - u.makeColVector() );
	evaluateConstraints.addStatement( qpUb.getRows(0, N * NU) == evUbValues.getRows(0, N * NU) - u.makeColVector() );

	evaluateConstraints.addStatement( qpLb.getRows(N * NU, N * NU + N * NX) == evLbValues.getRows(N * NU, N * NU + N * NX) - x.makeColVector().getRows(NX, NX * (N + 1)) );
	evaluateConstraints.addStatement( qpUb.getRows(N * NU, N * NU + N * NX) == evUbValues.getRows(N * NU, N * NU + N * NX) - x.makeColVector().getRows(NX, NX * (N + 1)) );

	return SUCCESSFUL_RETURN;
}

returnValue ExportGaussNewtonHpmpc::setupVariables( )
{
	if (initialStateFixed() == true)
	{
		x0.setup("x0",  NX, 1, REAL, ACADO_VARIABLES);
		x0.setDoc( "Current state feedback vector." );
	}

	return SUCCESSFUL_RETURN;
}

returnValue ExportGaussNewtonHpmpc::setupMultiplicationRoutines( )
{
	return SUCCESSFUL_RETURN;
}

returnValue ExportGaussNewtonHpmpc::setupEvaluation( )
{
	////////////////////////////////////////////////////////////////////////////
	//
	// Setup preparation phase
	//
	////////////////////////////////////////////////////////////////////////////
	preparation.setup("preparationStep");
	preparation.doc( "Preparation step of the RTI scheme." );

	ExportVariable retSim("ret", 1, 1, INT, ACADO_LOCAL, true);
	retSim.setDoc("Status of the integration module. =0: OK, otherwise the error code.");
	preparation.setReturnValue(retSim, false);

	preparation	<< retSim.getFullName() << " = " << modelSimulation.getName() << "();\n";

	preparation.addFunctionCall( evaluateObjective );
	preparation.addFunctionCall( evaluateConstraints );

	////////////////////////////////////////////////////////////////////////////
	//
	// Setup feedback phase
	//
	////////////////////////////////////////////////////////////////////////////
	ExportVariable stateFeedback("stateFeedback", NX, 1, REAL, ACADO_LOCAL);
	ExportVariable returnValueFeedbackPhase("retVal", 1, 1, INT, ACADO_LOCAL, true);
	returnValueFeedbackPhase.setDoc( "Status code of the FORCES QP solver." );
	feedback.setup("feedbackStep" );
	feedback.doc( "Feedback/estimation step of the RTI scheme." );
	feedback.setReturnValue( returnValueFeedbackPhase );

	qpx.setup("qpx", NX * (N + 1), 1, REAL, ACADO_WORKSPACE);
	qpu.setup("qpu", NU * N,       1, REAL, ACADO_WORKSPACE);

	qpq.setup("qpq", NX * N, 1, REAL, ACADO_WORKSPACE);
	qpqf.setup("qpqf", NX, 1, REAL, ACADO_WORKSPACE);
	qpr.setup("qpr", NU * N, 1, REAL, ACADO_WORKSPACE);

	nIt.setup("nIt", 1, 1, INT, ACADO_WORKSPACE);

	// State feedback
	feedback.addStatement( qpx.getRows(0, NX) == x0 - x.getRow( 0 ).getTranspose() );

	//
	// Calculate objective residuals
	//
	feedback.addStatement( Dy -= y );
	feedback.addLinebreak();
	feedback.addStatement( DyN -= yN );
	feedback.addLinebreak();

	for (unsigned i = 0; i < N; ++i)
		feedback.addFunctionCall(setStagef, qpq.getAddress(i * NX), qpr.getAddress(i * NU), ExportIndex( i ));
	feedback.addLinebreak();
	feedback.addStatement( qpqf == QN2 * DyN );
	feedback.addLinebreak();

	//
	// Here we have to add the differences....
	//

	// Call the solver
	feedback
		<< returnValueFeedbackPhase.getFullName() << " = " << "acado_hpmpc_ip_wrapper("

		<< toString( N ) << ", " << toString( NX ) << ", " << toString( NU ) << ", "

		<< evGx.getAddressString( true ) << ", "
		<< evGu.getAddressString( true ) << ", "
		<< d.getAddressString( true ) << ", "

		<< qpQ.getAddressString( true ) << ", "
		<< qpQf.getAddressString( true ) << ", "
		<< qpS.getAddressString( true ) << ", "
		<< qpR.getAddressString( true ) << ", "

		<< qpq.getAddressString( true ) << ", "
		<< qpqf.getAddressString( true ) << ", "
		<< qpr.getAddressString( true ) << ", "

		<< qpLb.getAddressString( true ) << ", "
		<< qpUb.getAddressString( true ) << ", "

		<< qpx.getAddressString( true ) << ", "
		<< qpu.getAddressString( true ) << ", "

		<< nIt.getAddressString( true )
		<< ");\n";

	// Accumulate the solution, i.e. perform full Newton step
	feedback.addStatement( x.makeColVector() += qpx );
	feedback.addStatement( u.makeColVector() += qpu );

	////////////////////////////////////////////////////////////////////////////
	//
	// TODO Setup evaluation of KKT
	//
	////////////////////////////////////////////////////////////////////////////
	ExportVariable kkt("kkt", 1, 1, REAL, ACADO_LOCAL, true);

	getKKT.setup( "getKKT" );
	getKKT.doc( "Get the KKT tolerance of the current iterate. Under development." );
//	kkt.setDoc( "The KKT tolerance value." );
	kkt.setDoc( "0." );
	getKKT.setReturnValue( kkt );

	getKKT.addStatement( kkt == 0 );

	return SUCCESSFUL_RETURN;
}

returnValue ExportGaussNewtonHpmpc::setupQPInterface( )
{
	string folderName;
	get(CG_EXPORT_FOLDER_NAME, folderName);
	string outFile = folderName + "/acado_hpmpc_interface.c";

	acadoCopyTempateFile(HPMPC_INTERFACE, outFile, "", true);

	return SUCCESSFUL_RETURN;
}

CLOSE_NAMESPACE_ACADO
