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
#include <acado/code_generation/export_hpmpc_interface.hpp>

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

	setupQPInterface();

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

	declarations.addDeclaration(sigmaN, dataStruct);

	declarations.addDeclaration(qpLambda, dataStruct);
	declarations.addDeclaration(qpMu, dataStruct);
	declarations.addDeclaration(qpSlacks, dataStruct);

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

	declarations.addDeclaration( evaluateStageCost );
	declarations.addDeclaration( evaluateTerminalCost );

	return SUCCESSFUL_RETURN;
}

returnValue ExportGaussNewtonHpmpc::getCode(	ExportStatementBlock& code
												)
{
	qpInterface->exportCode();

	// Forward declaration, same as in the template file.
	code << "#ifdef __cplusplus\n";
	code << "extern \"C\"{\n";
	code << "#endif\n";
	code << "int acado_hpmpc_ip_wrapper(real_t* A, real_t* B, real_t* d, real_t* Q, real_t* Qf, real_t* S, real_t* R, real_t* q, real_t* qf, real_t* r, real_t* lb, real_t* ub, real_t* x, real_t* u, real_t* lambda, real_t* mu, real_t* slacks, int* nIt);\n";
	code << "#ifdef __cplusplus\n";
	code << "}\n";
	code << "#endif\n";

	if (initialStateFixed() == false)
	{
		// MHE case, we use a wrapper directly from the NMPC
		code << "#define NX " << toString( NX ) << "\n";
		code << "#define NU " << toString( NU ) << "\n";
		code << "#define NW " << toString( NU ) << "\n";
		code << "#define NY " << toString( NY ) << "\n";
		code << "#define NN " << toString( N  ) << "\n";

		code << "#include <hpmpc/c_interface.h>\n\n";
	}

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

	code.addFunction( evaluateStageCost );
	code.addFunction( evaluateTerminalCost );
	code.addFunction( setObjQ1Q2 );
	code.addFunction( setObjR1R2 );
	code.addFunction( setObjS1 );
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
	if (initialStateFixed() == true)
		return N * NX + N * NU;

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

	ExportVariable evLmX = zeros<double>(NX, NX);
	ExportVariable evLmU = zeros<double>(NU, NU);

	if (levenbergMarquardt > 0.0)
	{
		DMatrix lmX = eye<double>( NX );
		lmX *= levenbergMarquardt;

		DMatrix lmU = eye<double>( NU );
		lmU *= levenbergMarquardt;

		evLmX = lmX;
		evLmU = lmU;
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
	loopObjective.addFunctionCall(evaluateStageCost, objValueIn, objValueOut);

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
		setObjQ1Q2.addStatement( tmpQ1 += evLmX );

		loopObjective.addFunctionCall(
				setObjQ1Q2,
				tmpFxCall, objSCall,
				Q1.getAddress(runObj * NX, 0), Q2.getAddress(runObj * NX, 0)
		);

		loopObjective.addLinebreak( );
	}
	else if (levenbergMarquardt > 0.0)
		Q1 = Q1.getGivenMatrix() + evLmX.getGivenMatrix();

	if (R1.isGiven() == false)
	{
		ExportVariable tmpR1, tmpR2;
		tmpR1.setup("tmpR1", NU, NU, REAL, ACADO_LOCAL);
		tmpR2.setup("tmpR2", NU, NY, REAL, ACADO_LOCAL);

		setObjR1R2.setup("setObjR1R2", tmpFu, tmpObjS, tmpR1, tmpR2);
		setObjR1R2.addStatement( tmpR2 == (tmpFu ^ tmpObjS) );
		setObjR1R2.addStatement( tmpR1 == tmpR2 * tmpFu );
		setObjR1R2.addStatement( tmpR1 += evLmU );

		loopObjective.addFunctionCall(
				setObjR1R2,
				tmpFuCall, objSCall,
				R1.getAddress(runObj * NU, 0), R2.getAddress(runObj * NU, 0)
		);

		loopObjective.addLinebreak( );
	}
	else if (levenbergMarquardt > 0.0)
		R1 = R1.getGivenMatrix() + evLmU.getGivenMatrix();

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
	evaluateObjective.addFunctionCall(evaluateTerminalCost, objValueIn, objValueOut);
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
		setObjQN1QN2.addStatement( tmpQN1 += evLmX );

		indexX = getNYN();
		ExportArgument tmpFxEndCall = tmpFxEnd.isGiven() == true ? tmpFxEnd  : objValueOut.getAddress(0, indexX);

		evaluateObjective.addFunctionCall(
				setObjQN1QN2,
				tmpFxEndCall, objSEndTerm,
				QN1.getAddress(0, 0), QN2.getAddress(0, 0)
		);

		evaluateObjective.addLinebreak( );
	}
	else if (levenbergMarquardt > 0.0)
		QN1 = QN1.getGivenMatrix() + evLmX.getGivenMatrix();

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
		setStagef << "(void)" << index.getFullName() << ";\n";
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

	DVector lbTmp, ubTmp;

	DVector lbXInf( NX );
	lbXInf.setAll( -INFTY );

	DVector ubXInf( NX );
	ubXInf.setAll( INFTY );

	DVector lbUInf( NU );
	lbUInf.setAll( -INFTY );

	DVector ubUInf( NU );
	ubUInf.setAll( INFTY );

	DVector lbValues, ubValues;

	for (unsigned node = 0; node < N; ++node)
	{
		lbTmp = uBounds.getLowerBounds( node );
		if ( !lbTmp.getDim() )
			lbValues.append( lbUInf );
		else
			lbValues.append( lbTmp );

		ubTmp = uBounds.getUpperBounds( node );
		if ( !ubTmp.getDim() )
			ubValues.append( ubUInf );
		else
			ubValues.append( ubTmp );
	}

	for (unsigned node = 1; node < N + 1; ++node)
	{
		lbTmp = xBounds.getLowerBounds( node );
		if ( !lbTmp.getDim() )
			lbValues.append( lbXInf );
		else
			lbValues.append( lbTmp );

		ubTmp = xBounds.getUpperBounds( node );
		if ( !ubTmp.getDim() )
			ubValues.append( ubXInf );
		else
			ubValues.append( ubTmp );
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
	else
	{
		xAC.setup("xAC", NX, 1, REAL, ACADO_VARIABLES);
		DxAC.setup("DxAC", NX, 1, REAL, ACADO_WORKSPACE);
		SAC.setup("SAC", NX, NX, REAL, ACADO_VARIABLES);
		sigmaN.setup("sigmaN", NX, NX, REAL, ACADO_VARIABLES);
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

	qpLambda.setup("qpLambda", N * NX, 1, REAL, ACADO_WORKSPACE);
	qpMu.setup("qpMu", 2 * N * (NX + NU), 1, REAL, ACADO_WORKSPACE);
	qpSlacks.setup("qpSlacks", 2 * N * (NX + NU), 1, REAL, ACADO_WORKSPACE);

	nIt.setup("nIt", 1, 1, INT, ACADO_WORKSPACE);


	if (initialStateFixed() == false)
	{
		// Temporary hack for the workspace
		feedback << "static real_t qpWork[ HPMPC_RIC_MHE_IF_DP_WORK_SPACE ];\n";
	}
	else
	{
		// State feedback
		feedback.addStatement( qpx.getRows(0, NX) == x0 - x.getRow( 0 ).getTranspose() );
	}

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
	// Arrival cost in the MHE case
	//
	if (initialStateFixed() == false)
	{
		// It is assumed this is the shifted version from the previous time step!
		feedback.addStatement( DxAC == xAC - x.getRow( 0 ).getTranspose() );
	}

	//
	// Here we have to add the differences....
	//

	// Call the solver
	if (initialStateFixed() == true)
		feedback
			<< returnValueFeedbackPhase.getFullName() << " = " << "acado_hpmpc_ip_wrapper("

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

			<< qpLambda.getAddressString( true ) << ", "
			<< qpMu.getAddressString( true ) << ", "
			<< qpSlacks.getAddressString( true ) << ", "

			<< nIt.getAddressString( true )
			<< ");\n";

	else
		feedback
			<< returnValueFeedbackPhase.getFullName() << " = " << "c_order_riccati_mhe_if('d', 2, "
			<< toString( NX ) << ", "
			<< toString( NU ) << ", "
			<< toString( NY ) << ", "
			<< toString( N ) << ", "

			<< evGx.getAddressString( true ) << ", "
			<< evGu.getAddressString( true ) << ", "
			<< "0, " // C
			<< d.getAddressString( true ) << ", "

			<< qpR.getAddressString( true ) << ", "
			<< qpQ.getAddressString( true ) << ", "
			<< qpQf.getAddressString( true ) << ", "
//			<< qpS.getAddressString( true ) << ", "

			<< qpr.getAddressString( true ) << ", "
			<< qpq.getAddressString( true ) << ", "
			<< qpqf.getAddressString( true ) << ", "
			<< "0, " // y

			<< DxAC.getAddressString( true ) << ", "
			<< SAC.getAddressString( true ) << ", "

			<< qpx.getAddressString( true ) << ", "
			<< sigmaN.getAddressString( true ) << ", "
			<< qpu.getAddressString( true ) << ", "

			<< qpLambda.getAddressString( true ) << ", "

			<< "qpWork);\n";

	/*
	int c_order_riccati_mhe_if( char prec, int alg,
	int nx, int nw, int ny, int N,
	double *A, double *G, double *C, double *f,
	double *R, double *Q, double *Qf,
	double *r, double *q, double *qf, double *y,
	double *x0, double *L0,
	double *xe, double *Le, double *w,
	double *lam, double *work0 );
	*/

	// XXX Not 100% sure about this one

	// Accumulate the solution, i.e. perform full Newton step
	feedback.addStatement( x.makeColVector() += qpx );
	feedback.addStatement( u.makeColVector() += qpu );

	if (initialStateFixed() == false)
	{
		// This is the arrival cost for the next time step!
		feedback.addStatement( xAC == x.getRow( 1 ).getTranspose() + DxAC );
	}

	////////////////////////////////////////////////////////////////////////////
	//
	// Setup evaluation of the KKT tolerance
	//
	////////////////////////////////////////////////////////////////////////////

	ExportVariable kkt("kkt", 1, 1, REAL, ACADO_LOCAL, true);
	ExportVariable tmp("tmp", 1, 1, REAL, ACADO_LOCAL, true);
	ExportIndex index( "index" );

	getKKT.setup( "getKKT" );
	getKKT.doc( "Get the KKT tolerance of the current iterate." );
	kkt.setDoc( "The KKT tolerance value." );
	getKKT.setReturnValue( kkt );
	getKKT.addVariable( tmp );
	getKKT.addIndex( index );

	getKKT.addStatement( kkt == 0.0 );

	// XXX This is still probably relevant for the NMPC case

	getKKT.addStatement( tmp == (qpq ^ qpx.getRows(0, N * NX)) );
	getKKT << kkt.getFullName() << " += fabs( " << tmp.getFullName() << " );\n";
	getKKT.addStatement( tmp == (qpqf ^ qpx.getRows(N * NX, (N + 1) * NX)) );
	getKKT << kkt.getFullName() << " += fabs( " << tmp.getFullName() << " );\n";
	getKKT.addStatement( tmp == (qpr ^ qpu) );
	getKKT << kkt.getFullName() << " += fabs( " << tmp.getFullName() << " );\n";

	ExportForLoop lamLoop(index, 0, N * NX);
	lamLoop << kkt.getFullName() << "+= fabs( " << d.get(index, 0) << " * " << qpLambda.get(index, 0) << ");\n";
	getKKT.addStatement( lamLoop );

	if (initialStateFixed() == true)
	{
		// XXX This is because the MHE does not support inequality constraints at the moment

		ExportForLoop lbLoop(index, 0, N * NU + N * NX);
		lbLoop << kkt.getFullName() << "+= fabs( " << qpLb.get(index, 0) << " * " << qpMu.get(index, 0) << ");\n";
		ExportForLoop ubLoop(index, 0, N * NU + N * NX);
		ubLoop << kkt.getFullName() << "+= fabs( " << qpUb.get(index, 0) << " * " << qpMu.get(index + N * NU + N * NX, 0) << ");\n";

		getKKT.addStatement( lbLoop );
		getKKT.addStatement( ubLoop );
	}

	return SUCCESSFUL_RETURN;
}

returnValue ExportGaussNewtonHpmpc::setupQPInterface( )
{
	//
	// Configure and export QP interface
	//

	string folderName;
	get(CG_EXPORT_FOLDER_NAME, folderName);
	string outFile = folderName + "/acado_hpmpc_interface.c";

	qpInterface = std::shared_ptr< ExportHpmpcInterface >(new ExportHpmpcInterface(outFile, commonHeaderName));

	int maxNumQPiterations;
	get(MAX_NUM_QP_ITERATIONS, maxNumQPiterations);

	// XXX If not specified, use default value
	if ( maxNumQPiterations <= 0 )
		maxNumQPiterations = getNumQPvars();

	int printLevel;
	get(PRINTLEVEL, printLevel);

	if ( (PrintLevel)printLevel >= HIGH )
		printLevel = 2;
	else
		printLevel = 0;

	int useSinglePrecision;
	get(USE_SINGLE_PRECISION, useSinglePrecision);

	int hotstartQP;
	get(HOTSTART_QP, hotstartQP);

	qpInterface->configure(
			maxNumQPiterations,
			printLevel,
			useSinglePrecision,
			hotstartQP
	);

	return SUCCESSFUL_RETURN;
}

CLOSE_NAMESPACE_ACADO
