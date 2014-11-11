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
 *    \file src/code_generation/export_exact_hessian_cn2.cpp
 *    \author Rien Quirynen
 *    \date 2014
 */

#include <acado/code_generation/export_exact_hessian_cn2.hpp>
#include <acado/code_generation/export_qpoases_interface.hpp>

using namespace std;

BEGIN_NAMESPACE_ACADO

ExportExactHessianCN2::ExportExactHessianCN2(	UserInteraction* _userInteraction,
											const std::string& _commonHeaderName
											) : ExportGaussNewtonCN2New( _userInteraction,_commonHeaderName )
{}

returnValue ExportExactHessianCN2::setup( )
{
	std::cout << "NOTE: You are using the new (unstable) N2 condensing feature for exact Hessian based RTI..\n";

	if (performFullCondensing() == false || initialStateFixed() == false || getNumComplexConstraints() > 0)
		return ACADOERROR( RET_NOT_IMPLEMENTED_YET );
	if (performsSingleShooting() == true)
		return ACADOERROR( RET_NOT_IMPLEMENTED_YET );

	LOG( LVL_DEBUG ) << "Solver: setup initialization... " << endl;
	setupInitialization();
	LOG( LVL_DEBUG ) << "done!" << endl;

	LOG( LVL_DEBUG ) << "Solver: setup variables... " << endl;
	setupVariables();
	LOG( LVL_DEBUG ) << "done!" << endl;

	LOG( LVL_DEBUG ) << "Solver: setup multiplication routines... " << endl;
	setupMultiplicationRoutines();
	LOG( LVL_DEBUG ) << "done!" << endl;

	LOG( LVL_DEBUG ) << "Solver: setup model simulation... " << endl;
	setupSimulation();
	LOG( LVL_DEBUG ) << "done!" << endl;

	LOG( LVL_DEBUG ) << "Solver: setup objective evaluation... " << endl;
	setupObjectiveEvaluation();
	LOG( LVL_DEBUG ) << "done!" << endl;

	LOG( LVL_DEBUG ) << "Solver: setup condensing... " << endl;
	setupCondensing();
	LOG( LVL_DEBUG ) << "done!" << endl;

	LOG( LVL_DEBUG ) << "Solver: setup constraints... " << endl;
	setupConstraintsEvaluation();
	LOG( LVL_DEBUG ) << "done!" << endl;

	LOG( LVL_DEBUG ) << "Solver: setup hessian regularization... " << endl;
	setupHessianRegularization();
	LOG( LVL_DEBUG ) << "done!" << endl;

	LOG( LVL_DEBUG ) << "Solver: setup evaluation... " << endl;
	setupEvaluation();
	LOG( LVL_DEBUG ) << "done!" << endl;

	LOG( LVL_DEBUG ) << "Solver: setup auxiliary functions... " << endl;
	setupAuxiliaryFunctions();
	LOG( LVL_DEBUG ) << "done!" << endl;

	return SUCCESSFUL_RETURN;
}

returnValue ExportExactHessianCN2::getFunctionDeclarations(	ExportStatementBlock& declarations
															) const
{
	ExportGaussNewtonCN2New::getFunctionDeclarations( declarations );

	declarations.addDeclaration( regularization );

	return SUCCESSFUL_RETURN;
}

returnValue ExportExactHessianCN2::getCode(	ExportStatementBlock& code
											)
{
	LOG( LVL_DEBUG ) << "Solver: getting code... " << endl;
	setupQPInterface();

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
	code.addFunction( evaluateObjective );

	code.addFunction( regularizeHessian );

//	code.addFunction( multGxd );
//	code.addFunction( moveGxT );
//	code.addFunction( multGxGx );
	code.addFunction( multGxGu );
	code.addFunction( moveGuE );

	code.addFunction( multBTW1 );
	code.addFunction( macBTW1_R1 );
	code.addFunction( multGxTGu );
	code.addFunction( macQEW2 );

//	ExportFunction multATw1Q, macBTw1, macQSbarW2, macASbarD;

	code.addFunction( macATw1QDy );
	code.addFunction( macBTw1 );
	code.addFunction( macQSbarW2 );
	code.addFunction( macASbar );
//	code.addFunction( macASbarD2 );
	code.addFunction( expansionStep );

	code.addFunction( expansionStep2 );

//	code.addFunction( setBlockH11 );
//	code.addFunction( zeroBlockH11 );
	code.addFunction( copyHTH );
//	code.addFunction( multQ1d );
//	code.addFunction( multQN1d );
//	code.addFunction( multRDy );
//	code.addFunction( multQDy );
//	code.addFunction( multEQDy );
//	code.addFunction( multQETGx );
//	code.addFunction( zeroBlockH10 );
//	code.addFunction( multEDu );
//	code.addFunction( multQ1Gx );
//	code.addFunction( multQN1Gx );
//	code.addFunction( multQ1Gu );
	code.addFunction( multQN1Gu );
//	code.addFunction( zeroBlockH00 );
//	code.addFunction( multCTQC );

	code.addFunction( multHxC );
	code.addFunction( multHxE );
	code.addFunction( macHxd );

	code.addFunction( evaluatePathConstraints );

	for (unsigned i = 0; i < evaluatePointConstraints.size(); ++i)
	{
		if (evaluatePointConstraints[ i ] == 0)
			continue;
		code.addFunction( *evaluatePointConstraints[ i ] );
	}

	code.addFunction( macCTSlx );
	code.addFunction( macETSlu );

	code.addFunction( condensePrep );
	code.addFunction( condenseFdb );
	code.addFunction( expand );

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

//
// PROTECTED FUNCTIONS:
//

returnValue ExportExactHessianCN2::setupObjectiveEvaluation( void )
{
	evaluateObjective.setup("evaluateObjective");

	//
	// A loop the evaluates objective and corresponding gradients
	//
	ExportIndex runObj( "runObj" );
	ExportForLoop loopObjective( runObj, 0, N );

	evaluateObjective.addIndex( runObj );

	if( evaluateLSQ.getFunctionDim() > 0 ) {
		loopObjective.addStatement( objValueIn.getCols(0, getNX()) == x.getRow( runObj ) );
		loopObjective.addStatement( objValueIn.getCols(NX, NX + NU) == u.getRow( runObj ) );
		loopObjective.addStatement( objValueIn.getCols(NX + NU, NX + NU + NOD) == od );
		loopObjective.addLinebreak( );

		// Evaluate the objective function
		loopObjective.addFunctionCall(evaluateLSQ, objValueIn, objValueOut);
		loopObjective.addLinebreak( );

		ExportVariable tmpFxx, tmpFxu, tmpFuu;
		tmpFxx.setup("tmpFxx", NX, NX, REAL, ACADO_LOCAL);
		tmpFxu.setup("tmpFxu", NX, NU, REAL, ACADO_LOCAL);
		tmpFuu.setup("tmpFuu", NU, NU, REAL, ACADO_LOCAL);

		//
		// Optional computation of Q1, Q2
		//
		ExportVariable tmpEH;
		tmpEH.setup("tmpEH", NX+NU, NX+NU, REAL, ACADO_LOCAL);

		setObjQ1Q2.setup("addObjTerm", tmpFxx, tmpFxu, tmpFuu, tmpEH);
		setObjQ1Q2.addStatement( tmpEH.getSubMatrix(0,NX,0,NX) += tmpFxx );
		setObjQ1Q2.addStatement( tmpEH.getSubMatrix(0,NX,NX,NX+NU) += tmpFxu );
		setObjQ1Q2.addStatement( tmpEH.getSubMatrix(NX,NX+NU,0,NX) += tmpFxu.getTranspose() );
		setObjQ1Q2.addStatement( tmpEH.getSubMatrix(NX,NX+NU,NX,NX+NU) += tmpFuu );

		loopObjective.addFunctionCall(
				setObjQ1Q2, objValueOut.getAddress(0, 1+NX+NU), objValueOut.getAddress(0, 1+NX+NU+NX*NX),
				objValueOut.getAddress(0, 1+NX+NU+NX*(NX+NU)), objS.getAddress(runObj*(NX+NU), 0) );

		ExportVariable tmpDx, tmpDu, tmpDF;
		tmpDx.setup("tmpDx", NX, 1, REAL, ACADO_LOCAL);
		tmpDu.setup("tmpDu", NU, 1, REAL, ACADO_LOCAL);
		tmpDF.setup("tmpDF", NX+NU, 1, REAL, ACADO_LOCAL);
		setObjR1R2.setup("addObjLinearTerm", tmpDx, tmpDu, tmpDF);
		setObjR1R2.addStatement( tmpDx == tmpDF.getRows(0,NX) );
		setObjR1R2.addStatement( tmpDu == tmpDF.getRows(NX,NX+NU) );

		loopObjective.addFunctionCall(
				setObjR1R2, QDy.getAddress(runObj * NX), g.getAddress(runObj * NU, 0), objValueOut.getAddress(0, 1) );

		loopObjective.addLinebreak( );
	}
	else {
		DMatrix Du(NU,1); Du.setAll(0);
		DMatrix Dx(NX,1); Dx.setAll(0);
		loopObjective.addStatement( g.getRows(runObj*NU, runObj*NU+NU) == Du );
		loopObjective.addStatement( QDy.getRows(runObj*NX, runObj*NX+NX) == Dx );
	}

	evaluateObjective.addStatement( loopObjective );

	//
	// Evaluate the quadratic Mayer term
	//
	if( evaluateLSQEndTerm.getFunctionDim() > 0 ) {
		evaluateObjective.addStatement( objValueIn.getCols(0, NX) == x.getRow( N ) );
		evaluateObjective.addStatement( objValueIn.getCols(NX, NX + NOD) == od );

		// Evaluate the objective function, last node.
		evaluateObjective.addFunctionCall(evaluateLSQEndTerm, objValueIn, objValueOut);
		evaluateObjective.addLinebreak( );

		ExportVariable tmpFxxEnd;
		tmpFxxEnd.setup("tmpFxxEnd", NX, NX, REAL, ACADO_LOCAL);

		//
		// Optional computation of QN1
		//
		ExportVariable tmpEH_N;
		tmpEH_N.setup("tmpEH_N", NX, NX, REAL, ACADO_LOCAL);

		setObjQN1QN2.setup("addObjEndTerm", tmpFxxEnd, tmpEH_N);
		setObjQN1QN2.addStatement( tmpEH_N == tmpFxxEnd );

		evaluateObjective.addFunctionCall(
				setObjQN1QN2, objValueOut.getAddress(0, 1+NX), objSEndTerm );

		evaluateObjective.addStatement( QDy.getRows(N * NX, (N + 1) * NX) == objValueOut.getCols(1,1+NX).getTranspose() );

		evaluateObjective.addLinebreak( );
	}
	else {
		DMatrix hess(NX,NX); hess.setAll(0);
		loopObjective.addStatement(objSEndTerm == hess);

		DMatrix Dx(NX,1); Dx.setAll(0);
		evaluateObjective.addStatement( QDy.getRows(N * NX, (N + 1) * NX) == Dx );
	}

	return SUCCESSFUL_RETURN;
}

returnValue ExportExactHessianCN2::setupGetObjective() {
	////////////////////////////////////////////////////////////////////////////
	//
	// Objective value calculation
	//
	////////////////////////////////////////////////////////////////////////////

	getObjective.setup( "getObjective" );
	getObjective.doc( "Calculate the objective value." );
	ExportVariable objVal("objVal", 1, 1, REAL, ACADO_LOCAL, true);
	objVal.setDoc( "Value of the objective function." );
	getObjective.setReturnValue( objVal );

	ExportIndex oInd;
	getObjective.acquire( oInd );

	// Recalculate objective
	getObjective.addStatement( objVal == 0 );

	if( evaluateLSQ.getFunctionDim() > 0 ) {
		ExportForLoop loopObjective(oInd, 0, N);

		loopObjective.addStatement( objValueIn.getCols(0, getNX()) == x.getRow( oInd ) );
		loopObjective.addStatement( objValueIn.getCols(NX, NX + NU) == u.getRow( oInd ) );
		loopObjective.addStatement( objValueIn.getCols(NX + NU, NX + NU + NOD) == od.getRow( oInd ) );
		loopObjective.addLinebreak( );

		// Evaluate the objective function
		loopObjective.addFunctionCall(evaluateLSQ, objValueIn, objValueOut);

		// Stack the measurement function value
		loopObjective.addStatement( objVal += objValueOut.getCol(0) );

		getObjective.addStatement( loopObjective );
	}
	if( evaluateLSQEndTerm.getFunctionDim() > 0 ) {
		getObjective.addStatement( objValueIn.getCols(0, NX) == x.getRow( N ) );
		getObjective.addStatement( objValueIn.getCols(NX, NX + NOD) == od.getRow( N ) );

		// Evaluate the objective function
		getObjective.addFunctionCall(evaluateLSQEndTerm, objValueIn, objValueOut);

		getObjective.addStatement( objVal += objValueOut.getCol(0) );
	}

	return SUCCESSFUL_RETURN;
}

returnValue ExportExactHessianCN2::setupHessianRegularization( )
{
	ExportVariable block( "hessian_block", NX+NU, NX+NU );
	regularization = ExportFunction( "acado_regularize", block );
	regularization.doc( "EVD-based regularization of a Hessian block." );
	regularization.addLinebreak();

	regularizeHessian.setup( "regularizeHessian" );
	regularizeHessian.doc( "Regularization procedure of the computed exact Hessian." );

	ExportIndex oInd;
	regularizeHessian.acquire( oInd );

	ExportForLoop loopObjective(oInd, 0, N);
	loopObjective.addFunctionCall( regularization, objS.getAddress(oInd*(NX+NU),0) );
	loopObjective.addStatement( Q1.getRows(oInd*NX, oInd*NX+NX) == objS.getSubMatrix(oInd*(NX+NU), oInd*(NX+NU)+NX, 0, NX) );
	loopObjective.addStatement( S1.getRows(oInd*NX, oInd*NX+NX) == objS.getSubMatrix(oInd*(NX+NU), oInd*(NX+NU)+NX, NX, NX+NU) );
	loopObjective.addStatement( R1.getRows(oInd*NU, oInd*NU+NU) == objS.getSubMatrix(oInd*(NX+NU)+NX, oInd*(NX+NU)+NX+NU, NX, NX+NU) );
	regularizeHessian.addStatement( loopObjective );

	regularizeHessian.addStatement( QN1 == objSEndTerm );

	return SUCCESSFUL_RETURN;
}

returnValue ExportExactHessianCN2::setupEvaluation( )
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

	preparation	<< retSim.getFullName() << " = " << modelSimulation.getName() << "();\n";

	preparation.addFunctionCall( evaluateObjective );
	preparation.addFunctionCall( regularizeHessian );
	preparation.addFunctionCall( condensePrep );

	////////////////////////////////////////////////////////////////////////////
	//
	// Feedback phase
	//
	////////////////////////////////////////////////////////////////////////////

	ExportVariable tmp("tmp", 1, 1, INT, ACADO_LOCAL, true);
	tmp.setDoc( "Status code of the qpOASES QP solver." );

	ExportFunction solve("solve");
	solve.setReturnValue( tmp );

	feedback.setup("feedbackStep");
	feedback.doc( "Feedback/estimation step of the RTI scheme." );
	feedback.setReturnValue( tmp );

	feedback.addFunctionCall( condenseFdb );
	feedback.addLinebreak();

	stringstream s;
	s << tmp.getName() << " = " << solve.getName() << "( );" << endl;
	feedback <<  s.str();
	feedback.addLinebreak();

	feedback.addFunctionCall( expand );

	////////////////////////////////////////////////////////////////////////////
	//
	// Setup evaluation of the KKT tolerance
	//
	////////////////////////////////////////////////////////////////////////////

	ExportVariable kkt("kkt", 1, 1, REAL, ACADO_LOCAL, true);
	ExportVariable prd("prd", 1, 1, REAL, ACADO_LOCAL, true);
	ExportIndex index( "index" );

	getKKT.setup( "getKKT" );
	getKKT.doc( "Get the KKT tolerance of the current iterate." );
	kkt.setDoc( "The KKT tolerance value." );
	getKKT.setReturnValue( kkt );
	getKKT.addVariable( prd );
	getKKT.addIndex( index );

	// ACC = |\nabla F^T * xVars|
	getKKT.addStatement( kkt == (g ^ xVars) );
	getKKT << kkt.getFullName() << " = fabs( " << kkt.getFullName() << " );\n";

	ExportForLoop bLoop(index, 0, getNumQPvars());

	bLoop.addStatement( prd == yVars.getRow( index ) );
	bLoop << "if (" << prd.getFullName() << " > " << toString(1.0 / INFTY) << ")\n";
	bLoop << kkt.getFullName() << " += fabs(" << lb.get(index, 0) << " * " << prd.getFullName() << ");\n";
	bLoop << "else if (" << prd.getFullName() << " < " << toString(-1.0 / INFTY) << ")\n";
	bLoop << kkt.getFullName() << " += fabs(" << ub.get(index, 0) << " * " << prd.getFullName() << ");\n";
	getKKT.addStatement( bLoop );

	if ((getNumStateBounds() + getNumComplexConstraints())> 0)
	{
		ExportForLoop cLoop(index, 0, getNumStateBounds() + getNumComplexConstraints());

		cLoop.addStatement( prd == yVars.getRow( getNumQPvars() + index ));
		cLoop << "if (" << prd.getFullName() << " > " << toString(1.0 / INFTY) << ")\n";
		cLoop << kkt.getFullName() << " += fabs(" << lbA.get(index, 0) << " * " << prd.getFullName() << ");\n";
		cLoop << "else if (" << prd.getFullName() << " < " << toString(-1.0 / INFTY) << ")\n";
		cLoop << kkt.getFullName() << " += fabs(" << ubA.get(index, 0) << " * " << prd.getFullName() << ");\n";

		getKKT.addStatement( cLoop );
	}

	return SUCCESSFUL_RETURN;
}

CLOSE_NAMESPACE_ACADO
