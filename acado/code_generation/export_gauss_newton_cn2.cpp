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
 *    \file src/code_generation/export_gauss_newton_cn2.cpp
 *    \author Milan Vukov, Joel Andersson, Rien Quirynen
 *    \date 2013 - 2014
 */

#include <acado/code_generation/export_gauss_newton_cn2.hpp>
#include <acado/code_generation/export_qpoases_interface.hpp>

using namespace std;

BEGIN_NAMESPACE_ACADO

ExportGaussNewtonCN2::ExportGaussNewtonCN2(	UserInteraction* _userInteraction,
											const std::string& _commonHeaderName
											) : ExportNLPSolver( _userInteraction,_commonHeaderName )
{}

returnValue ExportGaussNewtonCN2::setup( )
{
	if (performFullCondensing() == false && initialStateFixed() == true)
		return ACADOERROR( RET_NOT_IMPLEMENTED_YET );
	if (getNumComplexConstraints() > 0)
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

	LOG( LVL_DEBUG ) << "Solver: setup evaluation... " << endl;
	setupEvaluation();
	LOG( LVL_DEBUG ) << "done!" << endl;

	LOG( LVL_DEBUG ) << "Solver: setup auxiliary functions... " << endl;
	setupAuxiliaryFunctions();
	LOG( LVL_DEBUG ) << "done!" << endl;

	return SUCCESSFUL_RETURN;
}

returnValue ExportGaussNewtonCN2::getDataDeclarations(	ExportStatementBlock& declarations,
															ExportStruct dataStruct
															) const
{
	ExportNLPSolver::getDataDeclarations(declarations, dataStruct);

	declarations.addDeclaration(sbar, dataStruct);
	declarations.addDeclaration(x0, dataStruct);
	declarations.addDeclaration(Dx0, dataStruct);

	declarations.addDeclaration(T1, dataStruct);
	declarations.addDeclaration(T2, dataStruct);
	declarations.addDeclaration(C, dataStruct);

	declarations.addDeclaration(W1, dataStruct);
	declarations.addDeclaration(W2, dataStruct);
	declarations.addDeclaration(E, dataStruct);

	declarations.addDeclaration(QDy, dataStruct);
	declarations.addDeclaration(w1, dataStruct);
	declarations.addDeclaration(w2, dataStruct);

	declarations.addDeclaration(lbValues, dataStruct);
	declarations.addDeclaration(ubValues, dataStruct);
	declarations.addDeclaration(lbAValues, dataStruct);
	declarations.addDeclaration(ubAValues, dataStruct);

	declarations.addDeclaration(H, dataStruct);
	declarations.addDeclaration(A, dataStruct);
	declarations.addDeclaration(g, dataStruct);
	declarations.addDeclaration(lb, dataStruct);
	declarations.addDeclaration(ub, dataStruct);
	declarations.addDeclaration(lbA, dataStruct);
	declarations.addDeclaration(ubA, dataStruct);
	declarations.addDeclaration(xVars, dataStruct);
	declarations.addDeclaration(yVars, dataStruct);

	// lagrange multipliers
	declarations.addDeclaration(mu, dataStruct);

	return SUCCESSFUL_RETURN;
}

returnValue ExportGaussNewtonCN2::getFunctionDeclarations(	ExportStatementBlock& declarations
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

returnValue ExportGaussNewtonCN2::getCode(	ExportStatementBlock& code
											)
{
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

	code.addFunction( evaluateStageCost );
	code.addFunction( evaluateTerminalCost );

	code.addFunction( setObjQ1Q2 );
	code.addFunction( setObjR1R2 );
	code.addFunction( setObjS1 );
	code.addFunction( setObjQN1QN2 );
	code.addFunction( evaluateObjective );

	code.addFunction( regularizeHessian );

	code.addFunction( moveGxT );
	code.addFunction( multGxGx );
	code.addFunction( multGxGu );
	code.addFunction( moveGuE );

	code.addFunction( multBTW1 );
	code.addFunction( mac_S1T_E );
	code.addFunction( macBTW1_R1 );
	code.addFunction( multGxTGu );
	code.addFunction( macQEW2 );

	code.addFunction( macATw1QDy );
	code.addFunction( macBTw1 );
	code.addFunction( macS1TSbar );
	code.addFunction( macQSbarW2 );
	code.addFunction( macASbar );
	code.addFunction( expansionStep );

	code.addFunction( expansionStep2 );

	code.addFunction( mult_BT_T1 );
	code.addFunction( mac_ST_C );
	code.addFunction( multGxTGx );
	code.addFunction( macGxTGx );

	code.addFunction( copyHTH );
	code.addFunction( copyHTH1 );

	code.addFunction( multRDy );
	code.addFunction( multQDy );

	code.addFunction( multQN1Gx );
	code.addFunction( multQN1Gu );

	code.addFunction( evaluatePathConstraints );

	for (unsigned i = 0; i < evaluatePointConstraints.size(); ++i)
	{
		if (evaluatePointConstraints[ i ] == 0)
			continue;
		code.addFunction( *evaluatePointConstraints[ i ] );
	}

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


unsigned ExportGaussNewtonCN2::getNumQPvars( ) const
{
	if (performFullCondensing() == true)
		return (N * NU);

	return (NX + N * NU);
}

unsigned ExportGaussNewtonCN2::getNumStateBounds() const
{
	return xBoundsIdx.size();
}

//
// PROTECTED FUNCTIONS:
//

returnValue ExportGaussNewtonCN2::setupObjectiveEvaluation( void )
{
	evaluateObjective.setup("evaluateObjective");

	int variableObjS;
	get(CG_USE_VARIABLE_WEIGHTING_MATRIX, variableObjS);

	//
	// A loop the evaluates objective and corresponding gradients
	//
	ExportIndex runObj( "runObj" );
	ExportForLoop loopObjective(runObj, 0, N);

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

		indexX = getNYN();
		ExportArgument tmpFxEndCall = tmpFxEnd.isGiven() == true ? tmpFxEnd  : objValueOut.getAddress(0, indexX);

		evaluateObjective.addFunctionCall(
				setObjQN1QN2,
				tmpFxEndCall, objSEndTerm,
				QN1.getAddress(0, 0), QN2.getAddress(0, 0)
		);

		evaluateObjective.addLinebreak( );
	}

	return SUCCESSFUL_RETURN;
}

returnValue ExportGaussNewtonCN2::setupConstraintsEvaluation( void )
{
	ExportVariable tmp("tmp", 1, 1, REAL, ACADO_LOCAL, true);

	int hardcodeConstraintValues;
	get(CG_HARDCODE_CONSTRAINT_VALUES, hardcodeConstraintValues);

	////////////////////////////////////////////////////////////////////////////
	//
	// Setup the bounds on control variables
	//
	////////////////////////////////////////////////////////////////////////////

	unsigned numBounds = initialStateFixed( ) == true ? N * NU : NX + N * NU;
	unsigned offsetBounds = initialStateFixed( ) == true ? 0 : NX;

	DVector lbValuesMatrix( numBounds );
	DVector ubValuesMatrix( numBounds );

	if (initialStateFixed( ) == false)
		for(unsigned run1 = 0; run1 < NX; ++run1)
		{
			lbValuesMatrix( run1 )= xBounds.getLowerBound(0, run1);
			ubValuesMatrix( run1) = xBounds.getUpperBound(0, run1);
		}

	for(unsigned run1 = 0; run1 < N; ++run1)
		for(unsigned run2 = 0; run2 < NU; ++run2)
		{
			lbValuesMatrix(offsetBounds + run1 * getNU() + run2) = uBounds.getLowerBound(run1, run2);
			ubValuesMatrix(offsetBounds + run1 * getNU() + run2) = uBounds.getUpperBound(run1, run2);
		}


	if (hardcodeConstraintValues == YES)
	{
		lbValues.setup("lbValues", lbValuesMatrix, REAL, ACADO_VARIABLES);
		ubValues.setup("ubValues", ubValuesMatrix, REAL, ACADO_VARIABLES);
	}
	else if (isFinite( lbValuesMatrix ) || isFinite( ubValuesMatrix ))
	{
		lbValues.setup("lbValues", numBounds, 1, REAL, ACADO_VARIABLES);
		lbValues.setDoc( "Lower bounds values." );
		ubValues.setup("ubValues", numBounds, 1, REAL, ACADO_VARIABLES);
		ubValues.setDoc( "Upper bounds values." );

		initialize.addStatement(lbValues == lbValuesMatrix);
		initialize.addStatement(ubValues == ubValuesMatrix);
	}

	ExportFunction* boundSetFcn = hardcodeConstraintValues == YES ? &condensePrep : &condenseFdb;

	if (performFullCondensing() == true)
	{
		boundSetFcn->addStatement( lb.getRows(0, getNumQPvars()) == lbValues - u.makeColVector() );
		boundSetFcn->addStatement( ub.getRows(0, getNumQPvars()) == ubValues - u.makeColVector() );
	}
	else
	{
		if ( initialStateFixed( ) == true )
		{
			condenseFdb.addStatement( lb.getRows(0, NX) == Dx0 );
			condenseFdb.addStatement( ub.getRows(0, NX) == Dx0 );

			boundSetFcn->addStatement( lb.getRows(NX, getNumQPvars()) == lbValues - u.makeColVector() );
			boundSetFcn->addStatement( ub.getRows(NX, getNumQPvars()) == ubValues - u.makeColVector() );
		}
		else
		{
			boundSetFcn->addStatement( lb.getRows(0, NX) == lbValues.getRows(0, NX) - x.getRow( 0 ).getTranspose() );
			boundSetFcn->addStatement( lb.getRows(NX, getNumQPvars()) == lbValues.getRows(NX, getNumQPvars()) - u.makeColVector() );

			boundSetFcn->addStatement( ub.getRows(0, NX) == ubValues.getRows(0, NX) - x.getRow( 0 ).getTranspose() );
			boundSetFcn->addStatement( ub.getRows(NX, getNumQPvars()) == ubValues.getRows(NX, getNumQPvars()) - u.makeColVector() );
		}
	}
	condensePrep.addLinebreak( );
	condenseFdb.addLinebreak( );

	////////////////////////////////////////////////////////////////////////////
	//
	// Setup the bounds on state variables
	//
	////////////////////////////////////////////////////////////////////////////

	if( getNumStateBounds() )
	{
		condenseFdb.addVariable( tmp );

		DVector xLowerBounds( getNumStateBounds( )), xUpperBounds( getNumStateBounds( ) );
		for(unsigned i = 0; i < xBoundsIdx.size(); ++i)
		{
			xLowerBounds( i ) = xBounds.getLowerBound( xBoundsIdx[ i ] / NX, xBoundsIdx[ i ] % NX );
			xUpperBounds( i ) = xBounds.getUpperBound( xBoundsIdx[ i ] / NX, xBoundsIdx[ i ] % NX );
		}

		unsigned numOps = getNumStateBounds() * N * (N + 1) / 2 * NU;

		if (numOps < 1024)
		{
			for(unsigned row = 0; row < getNumStateBounds( ); ++row)
			{
				unsigned conIdx = xBoundsIdx[ row ] - NX;
				if( initialStateFixed( ) == false ) {
					condensePrep.addStatement( A.getSubMatrix(row, row + 1, 0, NX ) == C.getRow( conIdx ) );
				}

				unsigned blk = conIdx / NX + 1;
				for (unsigned col = 0; col < blk; ++col)
				{
					unsigned blkRow = (col * (2 * N - col - 1) / 2 + blk - 1) * NX + conIdx % NX;

					condensePrep.addStatement(
							A.getSubMatrix(row, row + 1, offsetBounds + col * NU, offsetBounds + (col + 1) * NU ) == E.getRow( blkRow ) );
				}

				condensePrep.addLinebreak();
			}
		}
		else
		{
			unsigned nXBounds = getNumStateBounds( );

			DMatrix vXBoundIndices(1, nXBounds);
			for (unsigned i = 0; i < nXBounds; ++i)
				vXBoundIndices(0, i) = xBoundsIdx[ i ];
			ExportVariable evXBounds("xBoundIndices", vXBoundIndices, STATIC_CONST_INT, ACADO_LOCAL, false);

			condensePrep.addVariable( evXBounds );

			ExportIndex row, col, conIdx, blk, blkRow;

			condensePrep.acquire( row ).acquire( col ).acquire( conIdx ).acquire( blk ).acquire( blkRow );

			ExportForLoop lRow(row, 0, nXBounds);

			lRow << conIdx.getFullName() << " = " << evXBounds.getFullName() << "[ " << row.getFullName() << " ] - " << toString(NX) << ";\n";

			if( initialStateFixed( ) == false ) {
				lRow.addStatement( A.getSubMatrix(row, row + 1, 0, NX ) == C.getRow( conIdx ) );
			}

			lRow.addStatement( blk == conIdx / NX + 1 );

			ExportForLoop lCol(col, 0, blk);

			lCol.addStatement( blkRow == (col * (2 * N - col - 1) / 2 + blk - 1) * NX + conIdx % NX );
			lCol.addStatement(
					A.getSubMatrix(row, row + 1, offsetBounds + col * NU, offsetBounds + (col + 1) * NU ) == E.getRow( blkRow ) );

			lRow.addStatement( lCol );
			condensePrep.addStatement( lRow );

			condensePrep.release( row ).release( col ).release( conIdx ).release( blk ).release( blkRow );
		}
		condensePrep.addLinebreak( );


		unsigned nXBounds = getNumStateBounds( );
		if (hardcodeConstraintValues == YES)
		{
			lbAValues.setup("lbAValues", xLowerBounds, REAL, ACADO_VARIABLES);
			ubAValues.setup("ubAValues", xUpperBounds, REAL, ACADO_VARIABLES);
		}
		else
		{
			lbAValues.setup("lbAValues", nXBounds, 1, REAL, ACADO_VARIABLES);
			lbAValues.setDoc( "Lower bounds values for affine constraints." );
			ubAValues.setup("ubAValues", nXBounds, 1, REAL, ACADO_VARIABLES);
			ubAValues.setDoc( "Upper bounds values for affine constraints." );

			initialize.addStatement(lbAValues == xLowerBounds);
			initialize.addStatement(ubAValues == xUpperBounds);
		}

		// Shift constraint bounds by first interval
		for(unsigned boundIndex = 0; boundIndex < getNumStateBounds( ); ++boundIndex)
		{
			unsigned row = xBoundsIdx[boundIndex];

			condenseFdb.addStatement( tmp == sbar.getRow( row ) + x.makeRowVector().getCol( row ) );
			condenseFdb.addStatement( lbA.getRow( boundIndex ) == lbAValues.getRow( boundIndex ) - tmp );
			condenseFdb.addStatement( ubA.getRow( boundIndex ) == ubAValues.getRow( boundIndex ) - tmp );
		}
		condenseFdb.addLinebreak( );
	}

	return SUCCESSFUL_RETURN;
}

returnValue ExportGaussNewtonCN2::setupCondensing( void )
{
	condensePrep.setup("condensePrep");
	condenseFdb.setup( "condenseFdb" );

	////////////////////////////////////////////////////////////////////////////
	//
	// Setup local memory for preparation phase: T1, T2, W1, W2
	//
	////////////////////////////////////////////////////////////////////////////

	// TODO

	////////////////////////////////////////////////////////////////////////////
	//
	// Compute Hessian block H00, H10 and C
	//
	////////////////////////////////////////////////////////////////////////////

	if (performFullCondensing() == false)
	{
		LOG( LVL_DEBUG ) << "Setup condensing: H00, H10 and C" << endl;

		T1.setup("T1", NX, NX, REAL, ACADO_WORKSPACE);
		T2.setup("T2", NX, NX, REAL, ACADO_WORKSPACE);

		condensePrep.addFunctionCall(moveGxT, evGx.getAddress(0, 0), C.getAddress(0, 0));
		for (unsigned row = 1; row < N; ++row)
			condensePrep.addFunctionCall(
					multGxGx, evGx.getAddress(row * NX), C.getAddress((row - 1) * NX), C.getAddress(row * NX));

		/* Algorithm for computation of H10 and H00

		T1 = Q_N * C_{N - 1}
		for i = N - 1: 1
		 	H_{i + 1, 0} =  B_i^T * T1
		 	H_{i + 1, 0} += S_i^T * C_{i - 1}

			T2 = A_i^T * T1
			T1 = T2 + Q_i * C_{i - 1}

		H_{1, 0} =  B_0^T * T1
		H_{1, 0} += S_0^T
		H_{0, 0} = Q_0 + A_0^T * T1

		 */

		// H10 Block
		ExportFunction* QN1Call = QN1.isGiven() == true ? &multQN1Gx : &multGxGx;
		condensePrep.addFunctionCall(*QN1Call, QN1, C.getAddress((N - 1) * NX), T1);
		for (unsigned row = N - 1; row > 0; --row)
		{
			condensePrep.addFunctionCall(mult_BT_T1, evGu.getAddress(row * NX), T1, ExportIndex( row ));

			if ((S1.isGiven() && S1.getGivenMatrix().isZero() == false) || S1.isGiven() == false)
			{
				ExportArgument S1Call = S1.isGiven() == false ? S1.getAddress(row * NX) : S1;
				condensePrep.addFunctionCall(mac_ST_C, S1Call, C.getAddress((row - 1) * NX), ExportIndex( row ));
			}

			condensePrep.addFunctionCall(multGxTGx, evGx.getAddress(row * NX), T1, T2);

			ExportArgument Q1Call = Q1.isGiven() == true ? Q1 : Q1.getAddress(row * NX);
			condensePrep.addFunctionCall(macGxTGx, T2, Q1Call, C.getAddress((row - 1) * NX), T1);
		}

		condensePrep.addFunctionCall(mult_BT_T1, evGu.getAddress( 0 ), T1, ExportIndex( 0 ));
		if ((S1.isGiven() && S1.getGivenMatrix().isZero() == false) || S1.isGiven() == false)
		{
			condensePrep.addStatement(
					H.getSubMatrix(NX, NX + NU, 0, NX) += S1.getSubMatrix(0, NX, 0, NU).getTranspose()
			);
		}

		// H00 Block
		DMatrix mRegH00 = eye<double>( getNX() );
		mRegH00 *= levenbergMarquardt;
		condensePrep.addStatement(
				H.getSubMatrix(0, NX, 0, NX) == Q1.getSubMatrix(0, NX, 0, NX) + (evGx.getSubMatrix(0, NX, 0, NX).getTranspose() * T1)
		);
		condensePrep.addStatement( H.getSubMatrix(0, NX, 0, NX) += mRegH00 );
	}

	////////////////////////////////////////////////////////////////////////////
	//
	// Compute Hessian block H11
	//
	////////////////////////////////////////////////////////////////////////////

	LOG( LVL_DEBUG ) << "Setup condensing: H11 and E" << endl;

	/*

	This one is only for the case where we have input constraints only

	// Create E and H

	for i = 0: N - 1
	{
		// Storage for E: (N x nx) x nu

		E_0 = B_i;
		for k = 1: N - i - 1
			E_k = A_{i + k} * E_{k - 1};

		// Two temporary matrices W1, W2 of size nx x nu

		W1 = Q_N^T * E_{N - i - 1};

		for k = N - 1: i + 1
		{
			H_{k, i} = B_k^T * W1;

			W2 = A_k^T * W1;
			W1 = Q_k^T * E_{k - i - 1} + W2;
		}
		H_{i, i} = B_i^T * W1 + R_i^T
	}

	Else, in general case:

	for i = 0: N - 1
	{
		// Storage for E: (N * (N + 1) / 2 x nx) x nu

		j = 1 / 2 * i * (2 * N - i + 1);

		E_j = B_i;
		for k = 1: N - i - 1
			E_{j + k} = A_{i + k} * E_{j + k - 1};

		// Two temporary matrices W1, W2 of size nx x nu

		W1 = Q_N^T * E_{j + N - i - 1};

		for k = N - 1: i + 1
		{
			H_{k, i}  = B_k^T * W1;
			H_{k, i} += S_k^T * E_{j + k - i - 1};

			W2 = A_k^T * W1;
			W1 = Q_k^T * E_{j + k - i - 1} + W2;
		}
		H_{i, i} = B_i^T * W1 + R_i^T
	}

	 */

	W1.setup("W1", NX, NU, REAL, ACADO_WORKSPACE);
	W2.setup("W2", NX, NU, REAL, ACADO_WORKSPACE);

	if (N <= 15)
	{
		for (unsigned col = 0; col < N; ++col)
		{
			int offset = col * (2 * N - col + 1) / 2;

			condensePrep.addComment( "Column: " + toString( col ) );
			condensePrep.addFunctionCall(
					moveGuE, evGu.getAddress(col * NX), E.getAddress(offset * NX)
			);
			for (unsigned row = 1; row < N - col; ++row)
				condensePrep.addFunctionCall(
						multGxGu,
						evGx.getAddress((col + row) * NX),
						E.getAddress((offset + row - 1) * NX), E.getAddress((offset + row) * NX)
				);
			condensePrep.addLinebreak();

			ExportFunction* QN1Call = QN1.isGiven() == true ? &multQN1Gu : &multGxGu;
			condensePrep.addFunctionCall(
					*QN1Call, QN1, E.getAddress((offset + N - col - 1) * NX), W1
			);

			for (unsigned row = N - 1; col < row; --row)
			{
				condensePrep.addFunctionCall(
						multBTW1, evGu.getAddress(row * NX), W1,
						ExportIndex( row ), ExportIndex( col )
				);

				if ((S1.isGiven() && S1.getGivenMatrix().isZero() == false) || S1.isGiven() == false)
				{
					ExportArgument S1Call = S1.isGiven() == false ? S1.getAddress(row * NX) : S1;

					condensePrep.addFunctionCall(
							mac_S1T_E,
							S1Call, E.getAddress((offset + row - col - 1) * NX),
							ExportIndex( row ), ExportIndex( col )
					);
				}

				condensePrep.addFunctionCall(
						multGxTGu, evGx.getAddress(row * NX), W1, W2
				);

				ExportArgument Q1Call = Q1.isGiven() == true ? Q1 : Q1.getAddress(row * NX);
				condensePrep.addFunctionCall(
						macQEW2, Q1Call, E.getAddress((offset + row - col - 1) * NX), W2, W1
				);
			}

			ExportArgument R1Call = R1.isGiven() == true ? R1 : R1.getAddress(col * NU);
			condensePrep.addFunctionCall(
					macBTW1_R1, R1Call, evGu.getAddress(col * NX), W1,
					ExportIndex( col )
			);

			condensePrep.addLinebreak();
		}
	}
	else
	{
		// Long horizons

		ExportIndex row, col, offset;
		condensePrep.acquire( row ).acquire( col ).acquire( offset );

		ExportForLoop cLoop(col, 0, N);
		ExportForLoop fwdLoop(row, 1, N - col);
		ExportForLoop adjLoop(row, N - 1, col, -1);

		cLoop.addStatement( offset == col * (2 * N + 1 - col) / 2 );
		cLoop.addFunctionCall(
				moveGuE, evGu.getAddress(col * NX), E.getAddress(offset * NX)
		);

		fwdLoop.addFunctionCall(
				multGxGu,
				evGx.getAddress((col + row) * NX),
				E.getAddress((offset + row - 1) * NX), E.getAddress((offset + row) * NX)
		);
		cLoop.addStatement( fwdLoop );
		cLoop.addLinebreak();

		ExportFunction* QN1Call = QN1.isGiven() == true ? &multQN1Gu : &multGxGu;
		cLoop.addFunctionCall(
				*QN1Call, QN1, E.getAddress((offset - col + N - 1) * NX), W1
		);

		adjLoop.addFunctionCall(
				multBTW1, evGu.getAddress(row * NX), W1,
				row, col
		);

		if ((S1.isGiven() && S1.getGivenMatrix().isZero() == false) || S1.isGiven() == false)
		{
			ExportArgument S1Call = S1.isGiven() == false ? S1.getAddress(row * NX) : S1;

			adjLoop.addFunctionCall(
					mac_S1T_E,
					S1Call, E.getAddress((offset + row - col - 1) * NX),
					row, col
			);
		}

		adjLoop.addFunctionCall(
				multGxTGu, evGx.getAddress(row * NX), W1, W2
		);

		ExportArgument Q1Call = Q1.isGiven() == true ? Q1 : Q1.getAddress(row * NX);
		adjLoop.addFunctionCall(
				macQEW2, Q1Call, E.getAddress((offset + row - col - 1) * NX), W2, W1
		);

		cLoop.addStatement( adjLoop );

		ExportArgument R1Call = R1.isGiven() == true ? R1 : R1.getAddress(col * NU);
		cLoop.addFunctionCall(
				macBTW1_R1, R1Call, evGu.getAddress(col * NX), W1,
				ExportIndex( col )
		);

		condensePrep.addStatement( cLoop );
		condensePrep.addLinebreak();

		condensePrep.release( row ).release( col ).release( offset );
	}

	/// NEW CODE END

	LOG( LVL_DEBUG ) << "---> Copy H11 lower part" << endl;

	// Copy to H11 upper lower part to upper triangular part
	if (N <= 20)
	{
		for (unsigned ii = 0; ii < N; ++ii)
			for(unsigned jj = 0; jj < ii; ++jj)
				condensePrep.addFunctionCall(
						copyHTH, ExportIndex( jj ), ExportIndex( ii ));

		// Copy H10 to H01
		if( performFullCondensing() == false) {
			for (unsigned ii = 0; ii < N; ++ii)
				condensePrep.addFunctionCall( copyHTH1, ExportIndex( ii ) );
		}
	}
	else
	{
		ExportIndex ii, jj;

		condensePrep.acquire( ii );
		condensePrep.acquire( jj );

		ExportForLoop eLoopI(ii, 0, N);
		ExportForLoop eLoopJ(jj, 0, ii);

		eLoopJ.addFunctionCall(copyHTH, jj, ii);
		eLoopI.addStatement( eLoopJ );
		condensePrep.addStatement( eLoopI );

		// Copy H10 to H01
		if( performFullCondensing() == false) {
			ExportForLoop eLoopK(ii, 0, N);
			eLoopK.addFunctionCall(copyHTH1, ii);
			condensePrep.addStatement( eLoopK );
		}

		condensePrep.release( ii );
		condensePrep.release( jj );
	}
	condensePrep.addLinebreak();

	////////////////////////////////////////////////////////////////////////////
	//
	// Compute gradient components g0 and g1
	//
	////////////////////////////////////////////////////////////////////////////

	LOG( LVL_DEBUG ) << "Setup condensing: create Dx0, Dy and DyN" << endl;

	unsigned offset = performFullCondensing() == true ? 0 : NX;

	if (initialStateFixed() == true)
	{
		condenseFdb.addStatement( Dx0 == x0 - x.getRow( 0 ).getTranspose() );
	}
	condenseFdb << (Dy -= y) << (DyN -= yN);
	condenseFdb.addLinebreak();

	int variableObjS;
	get(CG_USE_VARIABLE_WEIGHTING_MATRIX, variableObjS);

	// Compute RDy
	if( getNY() > 0 || getNYN() ) {
	for(unsigned run1 = 0; run1 < N; ++run1)
	{
		ExportArgument R2Call = R2.isGiven() == true ? R2 : R2.getAddress(run1 * NU, 0);
		ExportArgument SluCall =
				objSlu.isGiven() == true || variableObjS == false ? objSlu : objSlu.getAddress(run1 * NU, 0);
		condenseFdb.addFunctionCall(
				multRDy, R2Call, Dy.getAddress(run1 * NY, 0), SluCall, g.getAddress(offset + run1 * NU, 0)
		);
	}
	condenseFdb.addLinebreak();

	// Compute QDy
	for(unsigned run1 = 0; run1 < N; run1++ )
	{
		ExportArgument Q2Call = Q2.isGiven() == true ? Q2 : Q2.getAddress(run1 * NX);
		ExportArgument SlxCall =
				objSlx.isGiven() == true || variableObjS == false ? objSlx : objSlx.getAddress(run1 * NX, 0);
		condenseFdb.addFunctionCall(
				multQDy, Q2Call, Dy.getAddress(run1 * NY), SlxCall, QDy.getAddress(run1 * NX) );
	}
	condenseFdb.addLinebreak();
	ExportVariable SlxCall =
				objSlx.isGiven() == true || variableObjS == false ? objSlx : objSlx.getRows(N * NX, (N + 1) * NX);
	condenseFdb.addStatement( QDy.getRows(N * NX, (N + 1) * NX) == SlxCall + QN2 * DyN );
	condenseFdb.addLinebreak();
	}


	if (performFullCondensing() == false)
		condenseFdb.addStatement(g.getRows(0, NX) == QDy.getRows(0, NX));

	/*
	if partial condensing:
		g0 = q_0

	for k = 0: N - 1
		g1_k = r_k

	if partial condensing:
		sbar_0 = 0;
	else:
		sbar_0 = Dx_0;

	sbar(1: N) = d;

	for k = 0: N - 1
		sbar_{k + 1} += A_k sbar_k;

	w1 = Q_N^T * sbar_N + q_N;
	for k = N - 1: 1
	{
		g1_k += B_k^T * w1;
		g1_k += S_k^T * sbar_k;
		w2 = A_k^T * w1 + q_k;
		w1 = Q_k^T * sbar_k + w2;
	}

	g1_0 += B_0^T * w1;
	if partial condensing:
		g_0 += A_0^T * w1
	else:
		g1_0 += S^0^T * x0;

	*/

	w1.setup("w1", NX, 1, REAL, ACADO_WORKSPACE);
	w2.setup("w2", NX, 1, REAL, ACADO_WORKSPACE);
	sbar.setup("sbar", (N + 1) * NX, 1, REAL, ACADO_WORKSPACE);

	if( performFullCondensing() == true ) {
		condenseFdb.addStatement( sbar.getRows(0, NX) == Dx0 );
	}
	else {
		condenseFdb.addStatement( sbar.getRows(0, NX) == zeros<double>(NX,1) );  // Dx0 is now a variable as well !!
	}
	condensePrep.addStatement( sbar.getRows(NX, (N + 1) * NX) == d );

	for (unsigned i = 0; i < N; ++i)
		condenseFdb.addFunctionCall(
				macASbar, evGx.getAddress(i * NX), sbar.getAddress(i * NX), sbar.getAddress((i + 1) * NX)
		);
	condenseFdb.addLinebreak();

	condenseFdb.addStatement(
			w1 == QDy.getRows(N * NX, (N + 1) * NX) + QN1 * sbar.getRows(N * NX, (N + 1) * NX)
	);
	for (unsigned i = N - 1; 0 < i; --i)
	{
		condenseFdb.addFunctionCall(
				macBTw1, evGu.getAddress(i * NX), w1, g.getAddress(offset + i * NU)
		);

		if ((S1.isGiven() == true && S1.getGivenMatrix().isZero() == false) || S1.isGiven() == false)
		{
			ExportArgument S1Call = S1.isGiven() == false ? S1.getAddress(i * NX) : S1;
			condenseFdb.addFunctionCall(macS1TSbar, S1Call, sbar.getAddress(i * NX), g.getAddress(offset + i * NU));
		}

		condenseFdb.addFunctionCall(
				// TODO Check indexing for QDy
				macATw1QDy, evGx.getAddress(i * NX), w1, QDy.getAddress(i * NX), w2
		);

		ExportArgument Q1Call = Q1.isGiven() == true ? Q1 : Q1.getAddress(i * NX);
		condenseFdb.addFunctionCall(
				macQSbarW2, Q1Call, sbar.getAddress(i * NX), w2, w1);
	}
	condenseFdb.addFunctionCall(
			macBTw1, evGu.getAddress( 0 ), w1, g.getAddress( offset + 0 )
	);
	if( performFullCondensing() == true ) {
		if ((S1.isGiven() == true && S1.getGivenMatrix().isZero() == false) || S1.isGiven() == false)
		{
			condenseFdb.addFunctionCall(macS1TSbar, S1, sbar.getAddress(0), g);
		}
	}
	else {
		condenseFdb.addStatement(g.getRows(0, NX) += evGx.getRows(0, NX).getTranspose() * w1);
	}
	condenseFdb.addLinebreak();


	////////////////////////////////////////////////////////////////////////////
	//
	// Expansion routine
	//
	////////////////////////////////////////////////////////////////////////////

	/*

	// Step expansion, assuming that u_k is already updated

	Ds_0 = Dx_0;
	Ds_{1: N} = d;

	for i = 0: N - 1
	{
		Ds_{k + 1} += A_k * Ds_k;	// Reuse multABarD
		Ds_{k + 1} += B_k * Du_k;
		s_{k + 1}  += Ds_{k + 1};
	}

	 */

	LOG( LVL_DEBUG ) << "Setup condensing: create expand routine" << endl;

	expand.setup( "expand" );

	if (performFullCondensing() == true)
	{
		expand.addStatement( u.makeRowVector() += xVars.getTranspose() );
	}
	else
	{
		expand.addStatement( u.makeColVector() += xVars.getRows(NX, getNumQPvars()) );
	}

	if( performFullCondensing() == true ) {
		expand.addStatement( sbar.getRows(0, NX) == Dx0 );
	}
	else {
		expand.addStatement( sbar.getRows(0, NX) == xVars.getRows(0, NX) );
	}
	expand.addStatement( sbar.getRows(NX, (N + 1) * NX) == d );

	for (unsigned row = 0; row < N; ++row )
		expand.addFunctionCall(
				expansionStep, evGx.getAddress(row * NX), evGu.getAddress(row * NX),
				xVars.getAddress(offset + row * NU), sbar.getAddress(row * NX),
				sbar.getAddress((row + 1) * NX)
		);

	expand.addStatement( x.makeColVector() += sbar );

	// !! Calculation of multipliers: !!
	int hessianApproximation;
	get( HESSIAN_APPROXIMATION, hessianApproximation );
	bool secondOrder = ((HessianApproximationMode)hessianApproximation == EXACT_HESSIAN);
	if( secondOrder ) {
		//	mu_N = lambda_N + q_N + Q_N^T * Ds_N  --> wrong in Joel's paper !!
		//		for i = N - 1: 1
		//			mu_k = Q_k^T * Ds_k + A_k^T * mu_{k + 1} + S_k * Du_k + q_k

		for (uint j = 0; j < NX; j++ ) {
			uint item = N*NX+j;
			uint IdxF = std::find(xBoundsIdx.begin(), xBoundsIdx.end(), item) - xBoundsIdx.begin();
			if( IdxF != xBoundsIdx.size() ) { // INDEX FOUND
				expand.addStatement( mu.getSubMatrix(N-1,N,j,j+1) == yVars.getRow(getNumQPvars()+IdxF) );
			}
			else { // INDEX NOT FOUND
				expand.addStatement( mu.getSubMatrix(N-1,N,j,j+1) == 0.0 );
			}
		}
		expand.addStatement( mu.getRow(N-1) += sbar.getRows(N*NX,(N+1)*NX).getTranspose()*QN1 );
		expand.addStatement( mu.getRow(N-1) += QDy.getRows(N*NX,(N+1)*NX).getTranspose() );
		for (int i = N - 1; i >= 1; i--) {
			for (uint j = 0; j < NX; j++ ) {
				uint item = i*NX+j;
				uint IdxF = std::find(xBoundsIdx.begin(), xBoundsIdx.end(), item) - xBoundsIdx.begin();
				if( IdxF != xBoundsIdx.size() ) { // INDEX FOUND
					expand.addStatement( mu.getSubMatrix(i-1,i,j,j+1) == yVars.getRow(getNumQPvars()+IdxF) );
				}
				else { // INDEX NOT FOUND
					expand.addStatement( mu.getSubMatrix(i-1,i,j,j+1) == 0.0 );
				}
			}
			expand.addFunctionCall(
					expansionStep2, QDy.getAddress(i*NX), Q1.getAddress(i * NX), sbar.getAddress(i*NX),
					S1.getAddress(i * NX), xVars.getAddress(offset + i * NU), evGx.getAddress(i * NX),
					mu.getAddress(i-1), mu.getAddress(i) );
		}
	}

	return SUCCESSFUL_RETURN;
}

returnValue ExportGaussNewtonCN2::setupVariables( )
{
	////////////////////////////////////////////////////////////////////////////
	//
	// Make index vector for state constraints
	//
	////////////////////////////////////////////////////////////////////////////

	bool boxConIsFinite = false;
	xBoundsIdx.clear();

	DVector lbBox, ubBox;
	for (unsigned i = 0; i < xBounds.getNumPoints(); ++i)
	{
		lbBox = xBounds.getLowerBounds( i );
		ubBox = xBounds.getUpperBounds( i );

		if (isFinite( lbBox ) || isFinite( ubBox ))
			boxConIsFinite = true;

		// This is maybe not necessary
		if (boxConIsFinite == false || i == 0)
			continue;

		for (unsigned j = 0; j < lbBox.getDim(); ++j)
		{
			if ( ( acadoIsFinite( ubBox( j ) ) == true ) || ( acadoIsFinite( lbBox( j ) ) == true ) )
			{
				xBoundsIdx.push_back(i * lbBox.getDim() + j);
			}
		}
	}

	if (initialStateFixed() == true)
	{
		x0.setup("x0",  NX, 1, REAL, ACADO_VARIABLES);
		x0.setDoc( (std::string)"Current state feedback vector." );
		Dx0.setup("Dx0", NX, 1, REAL, ACADO_WORKSPACE);
	}

	if (performFullCondensing() == false)
	{
		C.setup("C", N*NX, NX, REAL, ACADO_WORKSPACE);
	}
	E.setup("E", N * (N + 1) / 2 * NX, NU, REAL, ACADO_WORKSPACE);

	QDy.setup ("QDy", (N + 1) * NX, 1, REAL, ACADO_WORKSPACE);

	H.setup("H", getNumQPvars(), getNumQPvars(), REAL, ACADO_WORKSPACE);
	g.setup("g",  getNumQPvars(), 1, REAL, ACADO_WORKSPACE);
	A.setup("A", getNumStateBounds( ) + getNumComplexConstraints(), getNumQPvars(), REAL, ACADO_WORKSPACE);

	lb.setup("lb", getNumQPvars(), 1, REAL, ACADO_WORKSPACE);
	ub.setup("ub", getNumQPvars(), 1, REAL, ACADO_WORKSPACE);

	lbA.setup("lbA", getNumStateBounds() + getNumComplexConstraints(), 1, REAL, ACADO_WORKSPACE);
	ubA.setup("ubA", getNumStateBounds() + getNumComplexConstraints(), 1, REAL, ACADO_WORKSPACE);

	xVars.setup("x", getNumQPvars(), 1, REAL, ACADO_WORKSPACE);
	yVars.setup("y", getNumQPvars() + getNumStateBounds() + getNumComplexConstraints(), 1, REAL, ACADO_WORKSPACE);

	return SUCCESSFUL_RETURN;
}

returnValue ExportGaussNewtonCN2::setupMultiplicationRoutines( )
{
	ExportIndex iCol( "iCol" );
	ExportIndex iRow( "iRow" );

	ExportVariable dp, dn, Gx1, Gx2, Gx3, Gx4, Gu1, Gu2, Gu3;
	ExportVariable R22, Dy1, RDy1, Q22, QDy1, E1, U1, U2, H101, w11, w12, w13;
	dp.setup("dOld", NX, 1, REAL, ACADO_LOCAL);
	dn.setup("dNew", NX, 1, REAL, ACADO_LOCAL);
	Gx1.setup("Gx1", NX, NX, REAL, ACADO_LOCAL);
	Gx2.setup("Gx2", NX, NX, REAL, ACADO_LOCAL);
	Gx3.setup("Gx3", NX, NX, REAL, ACADO_LOCAL);
	Gx4.setup("Gx4", NX, NX, REAL, ACADO_LOCAL);
	Gu1.setup("Gu1", NX, NU, REAL, ACADO_LOCAL);
	Gu2.setup("Gu2", NX, NU, REAL, ACADO_LOCAL);
	Gu3.setup("Gu3", NX, NU, REAL, ACADO_LOCAL);
	R22.setup("R2", NU, NY, REAL, ACADO_LOCAL);
	Dy1.setup("Dy1", NY, 1, REAL, ACADO_LOCAL);
	RDy1.setup("RDy1", NU, 1, REAL, ACADO_LOCAL);
	Q22.setup("Q2", NX, NY, REAL, ACADO_LOCAL);
	QDy1.setup("QDy1", NX, 1, REAL, ACADO_LOCAL);
	E1.setup("E1", NX, NU, REAL, ACADO_LOCAL);
	U1.setup("U1", NU, 1, REAL, ACADO_LOCAL);
	U2.setup("U2", NU, 1, REAL, ACADO_LOCAL);
	H101.setup("H101", NU, NX, REAL, ACADO_LOCAL);

	w11.setup("w11", NX, 1, REAL, ACADO_LOCAL);
	w12.setup("w12", NX, 1, REAL, ACADO_LOCAL);
	w13.setup("w13", NX, 1, REAL, ACADO_LOCAL);

	if ( Q2.isGiven() )
		Q22 = Q2;
	if ( R2.isGiven() )
		R22 = R2;

	// multGxd; // d_k += Gx_k * d_{k-1}
	multGxd.setup("multGxd", dp, Gx1, dn);
	multGxd.addStatement( dn += Gx1 * dp );

	if( performFullCondensing() == false ) {
		// moveGxT
		moveGxT.setup("moveGxT", Gx1, Gx2);
		moveGxT.addStatement( Gx2 == Gx1 );
		// multGxGx
		multGxGx.setup("multGxGx", Gx1, Gx2, Gx3);
		multGxGx.addStatement( Gx3 == Gx1 * Gx2 );
	}

	// multGxGu
	multGxGu.setup("multGxGu", Gx1, Gu1, Gu2);
	multGxGu.addStatement( Gu2 == Gx1 * Gu1 );
	// moveGuE
	moveGuE.setup("moveGuE", Gu1, Gu2);
	moveGuE.addStatement( Gu2 == Gu1 );

	unsigned offset = (performFullCondensing() == true) ? 0 : NX;

	// copyHTH
	copyHTH.setup("copyHTH", iRow, iCol);
	copyHTH.addStatement(
			H.getSubMatrix(offset + iRow * NU, offset + (iRow + 1) * NU, offset + iCol * NU, offset + (iCol + 1) * NU) ==
					H.getSubMatrix(offset + iCol * NU, offset + (iCol + 1) * NU, offset + iRow * NU, offset + (iRow + 1) * NU).getTranspose()
	);

	if( performFullCondensing() == false ) {
		// copyHTH1
		copyHTH1.setup("copyHTH1", iCol);
		copyHTH1.addStatement(
				H.getSubMatrix(0, NX, offset + iCol * NU, offset + (iCol + 1) * NU) ==
						H.getSubMatrix(offset + iCol * NU, offset + (iCol + 1) * NU, 0, NX).getTranspose()
		);
	}
	// multRDy
	ExportVariable SluCall = objSlu.isGiven() == true ? objSlu : ExportVariable("Slu", NU, 1, REAL, ACADO_LOCAL);
	multRDy.setup("multRDy", R22, Dy1, SluCall, RDy1);
	multRDy.addStatement( RDy1 ==  R22 * Dy1 );
	multRDy.addStatement( RDy1 +=  SluCall );

	// mult QDy1
	ExportVariable SlxCall = objSlx.isGiven() == true ? objSlx : ExportVariable("Slx", NX, 1, REAL, ACADO_LOCAL);
	multQDy.setup("multQDy", Q22, Dy1, SlxCall, QDy1);
	multQDy.addStatement( QDy1 == Q22 * Dy1 );
	multQDy.addStatement( QDy1 += SlxCall );
	// multEQDy;
	multEQDy.setup("multEQDy", E1, QDy1, U1);
	multEQDy.addStatement( U1 += (E1 ^ QDy1) );
	// multQETGx
	multQETGx.setup("multQETGx", E1, Gx1, H101);
	multQETGx.addStatement( H101 += (E1 ^ Gx1) );

	if (performsSingleShooting() == false)
	{
		// multEDu
		multEDu.setup("multEDu", E1, U1, dn);
		multEDu.addStatement( dn += E1 * U1 );
	}

	if (Q1.isGiven() == true)
	{
		// multQ1Gx
		multQ1Gx.setup("multQ1Gx", Gx1, Gx2);
		multQ1Gx.addStatement( Gx2 == Q1 * Gx1 );

		// multQ1Gu
		multQ1Gu.setup("multQ1Gu", Gu1, Gu2);
		multQ1Gu.addStatement( Gu2 == Q1 * Gu1 );

		// multQ1d
		multQ1d.setup("multQ1d", Q1, dp, dn);
		multQ1d.addStatement( dn == Q1 * dp );
	}
	else
	{
		// multQ1d
		multQ1d.setup("multQ1d", Gx1, dp, dn);
		multQ1d.addStatement( dn == Gx1 * dp );
	}

	if (QN1.isGiven() == true)
	{
		// multQN1Gx
		multQN1Gx.setup("multQN1Gx", QN1, Gx1, Gx2);
		multQN1Gx.addStatement( Gx2 == QN1 * Gx1 );

		// multQN1Gu
		multQN1Gu.setup("multQN1Gu", QN1, Gu1, Gu2);
		multQN1Gu.addStatement( Gu2 == QN1 * Gu1 );

		// multQN1d
		multQN1d.setup("multQN1d", QN1, dp, dn);
		multQN1d.addStatement( dn == QN1 * dp );
	}

	//
	// N2 condensing related
	//

	multBTW1.setup("multBTW1", Gu1, Gu2, iRow, iCol);
	multBTW1.addStatement(
			H.getSubMatrix(offset + iRow * NU, offset + (iRow + 1) * NU, offset + iCol * NU, offset + (iCol + 1) * NU) ==
					(Gu1 ^ Gu2)
	);

	multGxTGu.setup("multGxTGu", Gx1, Gu1, Gu2);
	multGxTGu.addStatement( Gu2 == (Gx1 ^ Gu1) );

	ExportVariable Q11 = Q1.isGiven() ? Q1 : ExportVariable("Q11", NX, NX, REAL, ACADO_LOCAL);

	macQEW2.setup("multQEW2", Q11, Gu1, Gu2, Gu3);
	macQEW2.addStatement( Gu3 == Gu2 + Q11 * Gu1 );

	macASbar.setup("macASbar", Gx1, w11, w12);
	macASbar.addStatement( w12 += Gx1 * w11 );

	macBTw1.setup("macBTw1", Gu1, w11, 	U1);
	macBTw1.addStatement( U1 += Gu1 ^ w11 );

	macATw1QDy.setup("macATw1QDy", Gx1, w11, w12, w13);
	macATw1QDy.addStatement( w13 == w12 + (Gx1 ^ w11) );

	macQSbarW2.setup("macQSbarW2", Q11, w11, w12, w13);
	macQSbarW2.addStatement( w13 == w12 + Q11 * w11 );

	expansionStep.setup("expansionStep", Gx1, Gu1, U1, w11, w12);
	expansionStep.addStatement( w12 += Gx1 * w11 );
	expansionStep.addStatement( w12 += Gu1 * U1 );

	//
	// Define LM regularization terms
	//
	DMatrix mRegH11 = eye<double>( getNU() );
	mRegH11 *= levenbergMarquardt;

	ExportVariable R11 = R1.isGiven() == true ? R1 : ExportVariable("R11", NU, NU, REAL, ACADO_LOCAL);

	macBTW1_R1.setup("multBTW1_R1", R11, Gu1, Gu2, iRow);
	macBTW1_R1.addStatement(
			H.getSubMatrix(offset + iRow * NU, offset + (iRow + 1) * NU, offset + iRow * NU, offset + (iRow + 1) * NU) ==
					R11 + (Gu1 ^ Gu2)
	);
	macBTW1_R1.addStatement(
			H.getSubMatrix(offset + iRow * NU, offset + (iRow + 1) * NU, offset + iRow * NU, offset + (iRow + 1) * NU) += mRegH11
	);

	ExportVariable S11 = zeros<double>(NX, NU);
	if (S1.isGiven() == true && S1.getGivenMatrix().isZero() == false)
		S11 = S1;
	else if (S1.isGiven() == false)
		S11 = Gu1;

	if ((S11.isGiven() == true && S11.getGivenMatrix().isZero() == false) || S11.isGiven() == false)
	{
		mac_S1T_E.setup("mac_S1T_E", S11, Gu2, iRow, iCol);
		mac_S1T_E.addStatement(
				H.getSubMatrix(offset + iRow * NU, offset + (iRow + 1) * NU, offset + iCol * NU, offset + (iCol + 1) * NU) +=
						(S11 ^ Gu2)
		);

		macS1TSbar.setup("macS1TSbar", S11, w11, U1);
		macS1TSbar.addStatement( U1 += (S11 ^ w11) );
	}

	int hessianApproximation;
	get( HESSIAN_APPROXIMATION, hessianApproximation );
	bool secondOrder = ((HessianApproximationMode)hessianApproximation == EXACT_HESSIAN);
	if( secondOrder ) {
		ExportVariable mu1; mu1.setup("mu1", 1, NX, REAL, ACADO_LOCAL);
		ExportVariable mu2; mu2.setup("mu2", 1, NX, REAL, ACADO_LOCAL);

		expansionStep2.setup("expansionStep2", QDy1, Q11, w11, S11, U1, Gx1, mu1, mu2);
		expansionStep2.addStatement( mu1 += QDy1.getTranspose() );
		expansionStep2.addStatement( mu1 += w11 ^ Q11.getTranspose() );
		expansionStep2.addStatement( mu1 += U1 ^ S11.getTranspose() );
		expansionStep2.addStatement( mu1 += mu2*Gx1 );
	}

	if (performFullCondensing() == false)
	{
		// ExportFunction mult_BT_T1, mac_ST_C, multGxTGx, macGxTGx;

		mult_BT_T1.setup("mult_BT_T1", Gu1, Gx1, iRow);
		mult_BT_T1.addStatement( H.getSubMatrix(offset + iRow * NU, offset + (iRow + 1) * NU, 0, NX) == (Gu1 ^ Gx1) );

		if ((S11.isGiven() == true && S11.getGivenMatrix().isZero() == false) || S11.isGiven() == false)
		{
			mac_ST_C.setup("mac_ST_C", S11, Gx1, iRow);
			mac_ST_C.addStatement( H.getSubMatrix(offset + iRow * NU, offset + (iRow + 1) * NU, 0, NX) += (S11 ^ Gx1) );
		}

		multGxTGx.setup("multGxTGx", Gx1, Gx2, Gx3);
		multGxTGx.addStatement( Gx3 == (Gx1 ^ Gx2) );

		macGxTGx.setup("macGxTGx", Gx1, Gx2, Gx3, Gx4);
		macGxTGx.addStatement( Gx4 == Gx1 + (Gx2 * Gx3) );
	}

	return SUCCESSFUL_RETURN;
}

returnValue ExportGaussNewtonCN2::setupEvaluation( )
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
	if( regularizeHessian.isDefined() ) preparation.addFunctionCall( regularizeHessian );
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

returnValue ExportGaussNewtonCN2::setupQPInterface( )
{
	string folderName;
	get(CG_EXPORT_FOLDER_NAME, folderName);
	string moduleName;
	get(CG_MODULE_NAME, moduleName);
	std::string sourceFile = folderName + "/" + moduleName + "_qpoases_interface.cpp";
	std::string headerFile = folderName + "/" + moduleName + "_qpoases_interface.hpp";

	int useSinglePrecision;
	get(USE_SINGLE_PRECISION, useSinglePrecision);

	int hotstartQP;
	get(HOTSTART_QP, hotstartQP);

	int covCalc;
	get(CG_COMPUTE_COVARIANCE_MATRIX, covCalc);

	int maxNumQPiterations;
	get(MAX_NUM_QP_ITERATIONS, maxNumQPiterations);

	//
	// Set up export of the source file
	//

	ExportQpOasesInterface qpInterface = ExportQpOasesInterface(headerFile, sourceFile, "");

	qpInterface.configure(
			"",
			"QPOASES_HEADER",
			getNumQPvars(),
			getNumStateBounds() + getNumComplexConstraints(),
			maxNumQPiterations,
			"PL_NONE",
			useSinglePrecision,

			commonHeaderName,
			"",
			xVars.getFullName(),
			yVars.getFullName(),
			"", // TODO
//			sigma.getFullName(),
			hotstartQP,
			true,
			H.getFullName(),
			string(),
			g.getFullName(),
			A.getFullName(),
			lb.getFullName(),
			ub.getFullName(),
			lbA.getFullName(),
			ubA.getFullName()
	);

	return qpInterface.exportCode();
}

bool ExportGaussNewtonCN2::performFullCondensing() const
{
	int sparseQPsolution;
	get(SPARSE_QP_SOLUTION, sparseQPsolution);

	if ((SparseQPsolutionMethods)sparseQPsolution == CONDENSING || (SparseQPsolutionMethods)sparseQPsolution == CONDENSING_N2)
		return false;

	return true;
}

CLOSE_NAMESPACE_ACADO
