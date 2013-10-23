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
 *    \file src/code_generation/export_gauss_newton_cn2.cpp
 *    \author Milan Vukov, Joel Andersson
 *    \date 2013
 */

#include <acado/code_generation/export_gauss_newton_cn2.hpp>
#include <acado/code_generation/export_qpoases_interface.hpp>
#include <acado/code_generation/export_module.hpp>

#include <sstream>
#include <string>

using namespace std;

BEGIN_NAMESPACE_ACADO

ExportGaussNewtonCN2::ExportGaussNewtonCN2(	UserInteraction* _userInteraction,
														const String& _commonHeaderName
														) : ExportNLPSolver( _userInteraction,_commonHeaderName )
{}

returnValue ExportGaussNewtonCN2::setup( )
{
	if (performFullCondensing() == BT_FALSE || initialStateFixed() == BT_FALSE || getNumComplexConstraints() > 0)
		return ACADOERROR( RET_NOT_IMPLEMENTED_YET );
	if (performsSingleShooting() == BT_TRUE)
		return ACADOERROR( RET_NOT_IMPLEMENTED_YET );

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
	returnValue status;
	status = ExportNLPSolver::getDataDeclarations(declarations, dataStruct);
	if (status != SUCCESSFUL_RETURN)
		return status;

	declarations.addDeclaration(sbar, dataStruct);
	declarations.addDeclaration(x0, dataStruct);
	declarations.addDeclaration(Dx0, dataStruct);

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

	if (performFullCondensing() == BT_TRUE)
		declarations.addDeclaration(A10, dataStruct);
	declarations.addDeclaration(A20, dataStruct);

	declarations.addDeclaration(pacA01Dx0, dataStruct);
	declarations.addDeclaration(pocA02Dx0, dataStruct);

	declarations.addDeclaration(H, dataStruct);
	declarations.addDeclaration(A, dataStruct);
	declarations.addDeclaration(g, dataStruct);
	declarations.addDeclaration(lb, dataStruct);
	declarations.addDeclaration(ub, dataStruct);
	declarations.addDeclaration(lbA, dataStruct);
	declarations.addDeclaration(ubA, dataStruct);
	declarations.addDeclaration(xVars, dataStruct);
	declarations.addDeclaration(yVars, dataStruct);

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

	code.addFunction( evaluateLSQ );
	code.addFunction( evaluateLSQEndTerm );
	code.addFunction( setObjQ1Q2 );
	code.addFunction( setObjR1R2 );
	code.addFunction( setObjQN1QN2 );
	code.addFunction( evaluateObjective );

//	code.addFunction( multGxd );
//	code.addFunction( moveGxT );
//	code.addFunction( multGxGx );
	code.addFunction( multGxGu );
	code.addFunction( moveGuE );

	code.addFunction( multBTW1 );
	code.addFunction( multGxTGu );
	code.addFunction( macQEW2 );

//	ExportFunction multATw1Q, macBTw1, macQSbarW2, macASbarD;

	code.addFunction( macATw1QDy );
	code.addFunction( macBTw1 );
	code.addFunction( macQSbarW2 );
	code.addFunction( macASbar );
//	code.addFunction( macASbarD2 );
	code.addFunction( expansionStep );

//	code.addFunction( setBlockH11 );
//	code.addFunction( zeroBlockH11 );
	code.addFunction( copyHTH );
//	code.addFunction( multQ1d );
//	code.addFunction( multQN1d );
	code.addFunction( multRDy );
	code.addFunction( multQDy );
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


unsigned ExportGaussNewtonCN2::getNumQPvars( ) const
{
	if (performFullCondensing() == BT_TRUE)
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
	ExportForLoop loopObjective( runObj, 0, N );

	evaluateObjective.addIndex( runObj );

	loopObjective.addStatement( objValueIn.getCols(0, getNX()) == x.getRow( runObj ) );
	loopObjective.addStatement( objValueIn.getCols(NX, NX + NU) == u.getRow( runObj ) );
	loopObjective.addStatement( objValueIn.getCols(NX + NU, NX + NU + NP) == p );
	loopObjective.addLinebreak( );

	// Evaluate the objective function
	if (externObjective == BT_FALSE)
		loopObjective.addFunctionCall( "evaluateLSQ", objValueIn, objValueOut );
	else
		loopObjective.addFunctionCall( evaluateExternLSQ, objValueIn, objValueOut );

	// Stack the measurement function value
	loopObjective.addStatement(
			Dy.getRows(runObj * NY, (runObj + 1) * NY) ==  objValueOut.getTranspose().getRows(0, getNY())
	);
	loopObjective.addLinebreak( );

	// Optionally compute derivatives
	unsigned indexX = getNY();
//	unsigned indexG = indexX;

	ExportVariable tmpObjS, tmpFx, tmpFu;
	ExportVariable tmpFxEnd, tmpObjSEndTerm;
	tmpObjS.setup("tmpObjS", NY, NY, REAL, ACADO_LOCAL);
	if (objS.isGiven() == BT_TRUE)
		tmpObjS = objS;
	tmpFx.setup("tmpFx", NY, NX, REAL, ACADO_LOCAL);
	if (objEvFx.isGiven() == BT_TRUE)
		tmpFx = objEvFx;
	tmpFu.setup("tmpFu", NY, NU, REAL, ACADO_LOCAL);
	if (objEvFu.isGiven() == BT_TRUE)
		tmpFu = objEvFu;
	tmpFxEnd.setup("tmpFx", NYN, NX, REAL, ACADO_LOCAL);
	if (objEvFxEnd.isGiven() == BT_TRUE)
		tmpFxEnd = objEvFxEnd;
	tmpObjSEndTerm.setup("tmpObjSEndTerm", NYN, NYN, REAL, ACADO_LOCAL);
	if (objSEndTerm.isGiven() == BT_TRUE)
		tmpObjSEndTerm = objSEndTerm;

	//
	// Optional computation of Q1, Q2
	//
	if (Q1.isGiven() == BT_FALSE)
	{
		ExportVariable tmpQ1, tmpQ2;
		tmpQ1.setup("tmpQ1", NX, NX, REAL, ACADO_LOCAL);
		tmpQ2.setup("tmpQ2", NX, NY, REAL, ACADO_LOCAL);

		setObjQ1Q2.setup("setObjQ1Q2", tmpFx, tmpObjS, tmpQ1, tmpQ2);
		setObjQ1Q2.addStatement( tmpQ2 == (tmpFx ^ tmpObjS) );
		setObjQ1Q2.addStatement( tmpQ1 == tmpQ2 * tmpFx );

		if (tmpFx.isGiven() == BT_TRUE)
		{
			if (variableObjS == YES)
			{
				loopObjective.addFunctionCall(
						setObjQ1Q2,
						tmpFx, objS.getAddress(runObj * NY, 0),
						Q1.getAddress(runObj * NX, 0), Q2.getAddress(runObj * NX, 0)
				);
			}
			else
			{
				loopObjective.addFunctionCall(
						setObjQ1Q2,
						tmpFx, objS,
						Q1.getAddress(runObj * NX, 0), Q2.getAddress(runObj * NX, 0)
				);
			}
		}
		else
		{
			if (variableObjS == YES)
			{
				if (objEvFx.isGiven() == BT_TRUE)

					loopObjective.addFunctionCall(
							setObjQ1Q2,
							objValueOut.getAddress(0, indexX), objS.getAddress(runObj * NY, 0),
							Q1.getAddress(runObj * NX, 0), Q2.getAddress(runObj * NX, 0)
					);
			}
			else
			{
				loopObjective.addFunctionCall(
						setObjQ1Q2,
						objValueOut.getAddress(0, indexX), objS,
						Q1.getAddress(runObj * NX, 0), Q2.getAddress(runObj * NX, 0)
				);
			}
			indexX += objEvFx.getDim();
		}

		loopObjective.addLinebreak( );
	}

	if (R1.isGiven() == BT_FALSE)
	{
		ExportVariable tmpR1, tmpR2;
		tmpR1.setup("tmpR1", NU, NU, REAL, ACADO_LOCAL);
		tmpR2.setup("tmpR2", NU, NY, REAL, ACADO_LOCAL);

		setObjR1R2.setup("setObjR1R2", tmpFu, tmpObjS, tmpR1, tmpR2);
		setObjR1R2.addStatement( tmpR2 == (tmpFu ^ tmpObjS) );
		setObjR1R2.addStatement( tmpR1 == tmpR2 * tmpFu );

		if (tmpFu.isGiven() == BT_TRUE)
		{
			if (variableObjS == YES)
			{
				loopObjective.addFunctionCall(
						setObjR1R2,
						tmpFu, objS.getAddress(runObj * NY, 0),
						R1.getAddress(runObj * NU, 0), R2.getAddress(runObj * NU, 0)
				);
			}
			else
			{
				loopObjective.addFunctionCall(
						setObjR1R2,
						tmpFu, objS,
						R1.getAddress(runObj * NU, 0), R2.getAddress(runObj * NU, 0)
				);
			}
		}
		else
		{
			if (variableObjS == YES)
			{
				loopObjective.addFunctionCall(
						setObjR1R2,
						objValueOut.getAddress(0, indexX), objS.getAddress(runObj * NY, 0),
						R1.getAddress(runObj * NU, 0), R2.getAddress(runObj * NU, 0)
				);
			}
			else
			{
				loopObjective.addFunctionCall(
						setObjR1R2,
						objValueOut.getAddress(0, indexX), objS,
						R1.getAddress(runObj * NU, 0), R2.getAddress(runObj * NU, 0)
				);
			}
		}

		loopObjective.addLinebreak( );
	}

	evaluateObjective.addStatement( loopObjective );

	//
	// Evaluate the quadratic Mayer term
	//
	evaluateObjective.addStatement( objValueIn.getCols(0, NX) == x.getRow( N ) );
	evaluateObjective.addStatement( objValueIn.getCols(NX, NX + NP) == p );

	// Evaluate the objective function
	if (externObjective == BT_FALSE)
		evaluateObjective.addFunctionCall( "evaluateLSQEndTerm", objValueIn, objValueOut );
	else
		evaluateObjective.addFunctionCall( evaluateExternLSQEndTerm, objValueIn, objValueOut );
	evaluateObjective.addLinebreak( );

	evaluateObjective.addStatement( DyN.getTranspose() == objValueOut.getCols(0, NYN) );
	evaluateObjective.addLinebreak();

	if (QN1.isGiven() == BT_FALSE)
	{
		indexX = getNYN();

		ExportVariable tmpQN1, tmpQN2;
		tmpQN1.setup("tmpQN1", NX, NX, REAL, ACADO_LOCAL);
		tmpQN2.setup("tmpQN2", NX, NYN, REAL, ACADO_LOCAL);

		setObjQN1QN2.setup("setObjQN1QN2", tmpFxEnd, tmpObjSEndTerm, tmpQN1, tmpQN2);
		setObjQN1QN2.addStatement( tmpQN2 == (tmpFxEnd ^ tmpObjSEndTerm) );
		setObjQN1QN2.addStatement( tmpQN1 == tmpQN2 * tmpFxEnd );

		if (tmpFxEnd.isGiven() == BT_TRUE)
			evaluateObjective.addFunctionCall(
					setObjQN1QN2,
					tmpFxEnd, objSEndTerm,
					QN1.getAddress(0, 0), QN2.getAddress(0, 0)
			);
		else
			evaluateObjective.addFunctionCall(
					setObjQN1QN2,
					objValueOut.getAddress(0, indexX), objSEndTerm,
					QN1.getAddress(0, 0), QN2.getAddress(0, 0)
			);

		evaluateObjective.addLinebreak( );
	}

	return SUCCESSFUL_RETURN;
}

returnValue ExportGaussNewtonCN2::setupConstraintsEvaluation( void )
{
	ExportVariable tmp("tmp", 1, 1, REAL, ACADO_LOCAL, BT_TRUE);

	int hardcodeConstraintValues;
	get(CG_HARDCODE_CONSTRAINT_VALUES, hardcodeConstraintValues);

	////////////////////////////////////////////////////////////////////////////
	//
	// Setup the bounds on control variables
	//
	////////////////////////////////////////////////////////////////////////////

	unsigned numBounds = initialStateFixed( ) == BT_TRUE ? N * NU : NX + N * NU;
	unsigned offsetBounds = initialStateFixed( ) == BT_TRUE ? 0 : NX;

	Vector lbValuesMatrix( numBounds );
	Vector ubValuesMatrix( numBounds );

	if (initialStateFixed( ) == BT_FALSE)
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

	// TODO This might be set with an option to be variable!!!
	if (hardcodeConstraintValues == YES)
	{
		lbValues.setup("lbValues", lbValuesMatrix, REAL, ACADO_VARIABLES);
		ubValues.setup("ubValues", ubValuesMatrix, REAL, ACADO_VARIABLES);
	}
	else
	{
		lbValues.setup("lbValues", numBounds, 1, REAL, ACADO_VARIABLES);
		lbValues.setDoc( "Lower bounds values." );
		ubValues.setup("ubValues", numBounds, 1, REAL, ACADO_VARIABLES);
		ubValues.setDoc( "Upper bounds values." );
	}

	ExportFunction* boundSetFcn = hardcodeConstraintValues == YES ? &condensePrep : &condenseFdb;

	if (performFullCondensing() == BT_TRUE)
	{
		boundSetFcn->addStatement( lb.getRows(0, getNumQPvars()) == lbValues - u.makeColVector() );
		boundSetFcn->addStatement( ub.getRows(0, getNumQPvars()) == ubValues - u.makeColVector() );
	}
	else
	{
		if ( initialStateFixed( ) == BT_TRUE )
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

		Vector xLowerBounds( getNumStateBounds( )), xUpperBounds( getNumStateBounds( ) );
		for(unsigned i = 0; i < xBoundsIdx.size(); ++i)
		{
			xLowerBounds( i ) = xBounds.getLowerBound( xBoundsIdx[ i ] / NX, xBoundsIdx[ i ] % NX );
			xUpperBounds( i ) = xBounds.getUpperBound( xBoundsIdx[ i ] / NX, xBoundsIdx[ i ] % NX );
		}

		unsigned numOps = getNumStateBounds() * N * (N + 1) / 2 * NU;

		if (numOps < 1024)
		{
			for(unsigned boundIndex = 0; boundIndex < getNumStateBounds( ); ++boundIndex)
			{
				unsigned row = xBoundsIdx[ boundIndex ] - NX;

				unsigned blkCol;
				unsigned blkRow = row / NX;
				for (blkCol = 0; blkCol <= blkRow; ++blkCol)
				{
					unsigned blkIdx = blkCol * (2 * N - blkCol - 1) / 2 + blkRow;
					unsigned ind = blkIdx * NX + (row % NX);

					condensePrep.addStatement(
							A.getSubMatrix(boundIndex, boundIndex + 1, blkCol * NU, (blkCol + 1) * NU ) == E.getRow( ind ) );
				}

				condensePrep.addLinebreak();
			}
		}
		else
		{
			unsigned nXBounds = getNumStateBounds( );

			Matrix vXBoundIndices(1, nXBounds);
			for (unsigned i = 0; i < nXBounds; ++i)
				vXBoundIndices(0, i) = xBoundsIdx[ i ];
			ExportVariable evXBounds("xBoundIndices", vXBoundIndices, STATIC_CONST_INT, ACADO_LOCAL, BT_FALSE);

			condensePrep.addVariable( evXBounds );

			ExportIndex boundIndex, blkCol, row, blkRow, ind;

			condensePrep.acquire( boundIndex );
			condensePrep.acquire( blkCol );
			condensePrep.acquire( row );
			condensePrep.acquire( blkRow );
			condensePrep.acquire( ind );

			ExportForLoop eLoopI(boundIndex, 0, nXBounds);

			eLoopI.addStatement( row.getFullName() << " = " << evXBounds.getFullName() << "[ " << boundIndex.getFullName() << " ] - " << NX << ";\n" );
			eLoopI.addStatement( blkRow == row / NX + 1 );

			ExportForLoop eLoopJ(blkCol, 0, blkRow);

			eLoopJ.addStatement( ind == (blkCol * (2 * N - blkCol - 1) / 2 + blkRow - 1) * NX + row % NX );
			eLoopJ.addStatement(
					A.getSubMatrix(boundIndex, boundIndex + 1, blkCol * NU, (blkCol + 1) * NU ) == E.getRow( ind ) );

			eLoopI.addStatement( eLoopJ );
			condensePrep.addStatement( eLoopI );

			condensePrep.release( boundIndex );
			condensePrep.release( blkCol );
			condensePrep.release( row );
			condensePrep.release( blkRow );
			condensePrep.release( ind );
		}
		condensePrep.addLinebreak( );

		// Shift constraint bounds by first interval
		for(unsigned boundIndex = 0; boundIndex < getNumStateBounds( ); ++boundIndex)
		{
			unsigned row = xBoundsIdx[boundIndex];
//			LOG( LVL_DEBUG ) << row << endl;

			condenseFdb.addStatement( tmp == sbar.getRow( row ) + x.makeRowVector().getCol( row ) );
			condenseFdb.addStatement( lbA.getRow( boundIndex ) == xLowerBounds( boundIndex ) - tmp );
			condenseFdb.addStatement( ubA.getRow( boundIndex ) == xUpperBounds( boundIndex ) - tmp );
		}
		condenseFdb.addLinebreak( );
	}

	return SUCCESSFUL_RETURN;
}

returnValue ExportGaussNewtonCN2::setupCondensing( void )
{
	//
	// Define LM regularization terms
	//
	Matrix mRegH00 = eye( getNX() );
	Matrix mRegH11 = eye( getNU() );

	mRegH00 *= levenbergMarquardt;
	mRegH11 *= levenbergMarquardt;

	condensePrep.setup("condensePrep");
	condenseFdb.setup( "condenseFdb" );

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
			H_{k, i} = B_k^T * W1;

			W2 = A_k^T * W1;
			W1 = Q_k^T * E_{j + k - i - 1} + W2;
		}
		H_{i, i} = B_i^T * W1 + R_i^T
	}

	 */


	/// NEW CODE START

	W1.setup("W1", NX, NU, REAL, ACADO_WORKSPACE);
	W2.setup("W2", NX, NU, REAL, ACADO_WORKSPACE);

	if (N <= 15)
	{
		for (unsigned col = 0; col < N; ++col)
		{
			int offset = col * (2 * N - col + 1) / 2;

			condensePrep.addComment( (String)"Column: " << col );
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

			if (QN1.isGiven() == BT_TRUE)
				condensePrep.addFunctionCall(
						multQN1Gu, E.getAddress((offset + N - col - 1) * NX), W1
				);
			else
				condensePrep.addFunctionCall(
						multGxGu, QN1, E.getAddress((offset + N - col - 1) * NX), W1
				);

			for (unsigned row = N - 1; col < row; --row)
			{
				condensePrep.addFunctionCall(
						multBTW1, evGu.getAddress(row * NX), W1,
						ExportIndex( row ).makeArgument(), ExportIndex( col ).makeArgument()
				);

				condensePrep.addFunctionCall(
						multGxTGu, evGx.getAddress(row * NX), W1, W2
				);

				if (Q1.isGiven() == BT_TRUE)
					condensePrep.addFunctionCall(
							macQEW2, Q1, E.getAddress((offset + row - col - 1) * NX), W2, W1
					);
				else
					condensePrep.addFunctionCall(
							macQEW2,
							Q1.getAddress(row * NX), E.getAddress((offset + row - col - 1) * NX), W2, W1
					);
			}

			condensePrep.addFunctionCall(
					multBTW1, evGu.getAddress(col * NX), W1,
					ExportIndex( col ).makeArgument(), ExportIndex( col ).makeArgument()
			);

			// TODO move this addition to multBTW1
			if (R1.isGiven() == BT_TRUE)
				condensePrep.addStatement(
						H.getSubMatrix(col * NU, (col + 1) * NU, col * NU, (col + 1) * NU)
						+= R1 + mRegH11
				);
			else
				condensePrep.addStatement(
						H.getSubMatrix(col * NU, (col + 1) * NU, col * NU, (col + 1) * NU)
						+= R1.getSubMatrix(col * NU, (col + 1) * NU, 0, NU) + mRegH11
				);

			condensePrep.addLinebreak();
		}
	}
	else
	{
		// Long horizons

		ExportIndex row, col, offset;
		condensePrep.acquire( row );
		condensePrep.acquire( col );
		condensePrep.acquire( offset );

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

		if (QN1.isGiven() == BT_TRUE)
			cLoop.addFunctionCall(
					multQN1Gu, E.getAddress((offset - col + N - 1) * NX), W1
			);
		else
			cLoop.addFunctionCall(
					multGxGu, QN1, E.getAddress((offset - col + N - 1) * NX), W1
			);

		adjLoop.addFunctionCall(
				multBTW1, evGu.getAddress(row * NX), W1,
				row.makeArgument(), col.makeArgument()
//				col.makeArgument(), row.makeArgument()
		);

		adjLoop.addFunctionCall(
				multGxTGu, evGx.getAddress(row * NX), W1, W2
		);

		if (Q1.isGiven() == BT_TRUE)
			adjLoop.addFunctionCall(
					macQEW2, Q1, E.getAddress((offset + row - col - 1) * NX), W2, W1
			);
		else
			adjLoop.addFunctionCall(
					macQEW2,
					Q1.getAddress(row * NX), E.getAddress((offset + row - col - 1) * NX), W2, W1
			);

		cLoop.addStatement( adjLoop );

		cLoop.addFunctionCall(
				multBTW1, evGu.getAddress(col * NX), W1,
				col.makeArgument(), col.makeArgument()
		);

		if (R1.isGiven() == BT_TRUE)
			cLoop.addStatement(
					H.getSubMatrix(col * NU, (col + 1) * NU, col * NU, (col + 1) * NU)
					+= R1 + mRegH11
			);
		else
			cLoop.addStatement(
					H.getSubMatrix(col * NU, (col + 1) * NU, col * NU, (col + 1) * NU)
					+= R1.getSubMatrix(col * NU, (col + 1) * NU, 0, NU) + mRegH11
			);

		condensePrep.addStatement( cLoop );
		condensePrep.addLinebreak();

		condensePrep.release( row );
		condensePrep.release( col );
		condensePrep.release( offset );
	}

	/// NEW CODE END

	LOG( LVL_DEBUG ) << "---> Copy H11 lower part" << endl;

	// Copy to H11 upper lower part to upper triangular part
	if (N <= 20)
	{
		for (unsigned ii = 0; ii < N; ++ii)
			for(unsigned jj = 0; jj < ii; ++jj)
			{
				stringstream s;
				s << copyHTH.getName().getName() << "(" << jj <<  ", " << ii << ");" << endl;

				condensePrep.addStatement( s.str().c_str() );
			}
	}
	else
	{
		ExportIndex ii, jj;

		condensePrep.acquire( ii );
		condensePrep.acquire( jj );

		ExportForLoop eLoopI(ii, 0, N);
		ExportForLoop eLoopJ(jj, 0, ii);

		eLoopJ.addFunctionCall(copyHTH, jj.makeArgument(), ii.makeArgument());
//		eLoopJ.addFunctionCall(copyHTH, ii.makeArgument(), jj.makeArgument());
		eLoopI.addStatement( eLoopJ );
		condensePrep.addStatement( eLoopI );

		condensePrep.release( ii );
		condensePrep.release( jj );
	}
	condensePrep.addLinebreak();

	////////////////////////////////////////////////////////////////////////////
	//
	// Compute gradient components g0 and g1
	//
	////////////////////////////////////////////////////////////////////////////

	//// NEW CODE START

	if (initialStateFixed() == BT_FALSE)
		return ACADOERROR( RET_NOT_IMPLEMENTED_YET );

	LOG( LVL_DEBUG ) << "Setup condensing: create Dx0, Dy and DyN" << endl;

	{
		condenseFdb.addStatement( Dx0 == x0 - x.getRow( 0 ).getTranspose() );
		condenseFdb.addLinebreak();
	}

	condenseFdb.addStatement( Dy -=  y );
	condenseFdb.addStatement( DyN -=  yN );
	condenseFdb.addLinebreak();

	// Compute RDy
	for(unsigned run1 = 0; run1 < N; ++run1)
	{
		if (R2.isGiven() == BT_TRUE)
			condenseFdb.addFunctionCall(
					multRDy, R2,
					Dy.getAddress(run1 * NY, 0),
					g.getAddress(run1 * NU, 0) );
		else
			condenseFdb.addFunctionCall(
					multRDy, R2.getAddress(run1 * NU, 0),
					Dy.getAddress(run1 * NY, 0),
					g.getAddress(run1 * NU, 0) );
	}
	condenseFdb.addLinebreak();

	// Compute QDy
	// NOTE: This is just for the MHE case :: run1 starts from 0; in MPC :: from 1 ;)
	for(unsigned run1 = 0; run1 < N; run1++ )
	{
		if (Q2.isGiven() == BT_TRUE)
			condenseFdb.addFunctionCall(
					multQDy, Q2,
					Dy.getAddress(run1 * NY),
					QDy.getAddress(run1 * NX) );
		else
			condenseFdb.addFunctionCall(
					multQDy, Q2.getAddress(run1 * NX),
					Dy.getAddress(run1 * NY),
					QDy.getAddress(run1 * NX) );
	}
	condenseFdb.addLinebreak();
	condenseFdb.addStatement( QDy.getRows(N * NX, (N + 1) * NX) == QN2 * DyN );
	condenseFdb.addLinebreak();

	/*
	for k = 0: N - 1
		g1_k = r_k;

	sbar_0 = Dx_0;
	sbar(1: N) = d;

	for k = 0: N - 1
		sbar_{k + 1} += A_k sbar_k;

	w1 = Q_N^T * sbar_N + q_N;
	for k = N - 1: 1
	{
		g1_k += B_k^T * w1;
		w2 = A_k^T * w1 + q_k;
		w1 = Q_k^T * sbar_k + w2;
	}

	g1_0 += B_0^T * w1;

	*/

	sbar.setup("sbar", (N + 1) * NX, 1, REAL, ACADO_WORKSPACE);
	w1.setup("w1", NX, 1, REAL, ACADO_WORKSPACE);
	w2.setup("w2", NX, 1, REAL, ACADO_WORKSPACE);

	condenseFdb.addStatement( sbar.getRows(0, NX) == Dx0 );

	if( performsSingleShooting() == BT_FALSE )
		condensePrep.addStatement( sbar.getRows(NX, (N + 1) * NX) == d );
	else
		condensePrep.addStatement( sbar.getRows(NX, (N + 1) * NX) == zeros(N, 1) );

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
				macBTw1, evGu.getAddress(i * NX), w1, g.getAddress(i * NU)
		);
		condenseFdb.addFunctionCall(
				macATw1QDy, evGx.getAddress(i * NX), w1, QDy.getAddress(i * NX), w2 // Proveri indexiranje za QDy
		);
		if (Q1.isGiven() == BT_TRUE)
			condenseFdb.addFunctionCall(
					macQSbarW2, Q1, sbar.getAddress(i * NX), w2, w1
			);
		else
			condenseFdb.addFunctionCall(
					macQSbarW2, Q1.getAddress(i * NX), sbar.getAddress(i * NX), w2, w1
			);
	}
	condenseFdb.addFunctionCall(
			macBTw1, evGu.getAddress( 0 ), w1, g.getAddress( 0 )
	);
	condenseFdb.addLinebreak();

	//// NEW CODE END

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

	// TODO Calculation of multipliers

	mu_N = lambda_N + Q_N^T * s_N
	for i = N - 1: 1
		mu_k = lambda_k + Q_k^T * s_k + A_k^T * mu_{k + 1} + q_k

	mu_0 = Q_0^T s_0 + A_0^T * mu_1 + q_0

	 */

	LOG( LVL_DEBUG ) << "Setup condensing: create expand routine" << endl;

	expand.setup( "expand" );

	if (performFullCondensing() == BT_TRUE)
	{
		expand.addStatement( u.makeRowVector() += xVars.getTranspose() );
	}
	else
	{
		// TODO
		expand.addStatement( x.makeColVector().getRows(0, NX) += xVars.getRows(0, NX) );
		expand.addLinebreak();
		expand.addStatement( u.makeRowVector() += xVars.getTranspose().getCols(NX, getNumQPvars() ) );
	}
	expand.addLinebreak();

	expand.addStatement( sbar.getRows(0, NX) == Dx0 );
	if( performsSingleShooting() == BT_FALSE )
		expand.addStatement( sbar.getRows(NX, (N + 1) * NX) == d );
	else
		expand.addStatement( sbar.getRows(NX, (N + 1) * NX) == zeros(N, 1) );


	for (unsigned row = 0; row < N; ++row )
		expand.addFunctionCall(
				expansionStep, evGx.getAddress(row * NX), evGu.getAddress(row * NX),
				xVars.getAddress(row * NU), sbar.getAddress(row * NX),
				sbar.getAddress((row + 1) * NX)
		);

	expand.addStatement( x.makeColVector() += sbar );

	return SUCCESSFUL_RETURN;
}

returnValue ExportGaussNewtonCN2::setupVariables( )
{
	if (initialStateFixed() == BT_TRUE)
	{
		x0.setup("x0",  NX, 1, REAL, ACADO_VARIABLES);
		x0.setDoc( (String)"Current state feedback vector." );
		Dx0.setup("Dx0", NX, 1, REAL, ACADO_WORKSPACE);
	}

	T.setup("T", NX, NX, REAL, ACADO_WORKSPACE);
	E.setup("E", N * (N + 1) / 2 * NX, NU, REAL, ACADO_WORKSPACE);
	QE.setup("QE", N * (N + 1) / 2 * NX, NU, REAL, ACADO_WORKSPACE);
	QGx.setup("QGx", N * NX, NX, REAL, ACADO_WORKSPACE);

	QDy.setup ("QDy", (N + 1) * NX, 1, REAL, ACADO_WORKSPACE);

	// Setup all QP stuff

	H.setup("H", getNumQPvars(), getNumQPvars(), REAL, ACADO_WORKSPACE);

	// Stupid aliasing to avoid copying of data
	if (performFullCondensing() == BT_TRUE)
	{
		H11 = H;
	}
	else
	{
		H00 = H.getSubMatrix(0, NX, 0, NX);
		H11 = H.getSubMatrix(NX, getNumQPvars(), NX, getNumQPvars());
	}

	H10.setup("H10", N * NU, NX, REAL, ACADO_WORKSPACE);

	A.setup("A", getNumStateBounds( ) + getNumComplexConstraints(), getNumQPvars(), REAL, ACADO_WORKSPACE);

	g.setup("g",  getNumQPvars(), 1, REAL, ACADO_WORKSPACE);

	if (performFullCondensing() == BT_TRUE)
	{
		g1 = g;
	}
	else
	{
		g0 = g.getRows(0, NX);
		g1 = g.getRows(NX, getNumQPvars());
	}

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

	ExportVariable dp, dn, Gx1, Gx2, Gx3, Gu1, Gu2, Gu3;
	ExportVariable R22, Dy1, RDy1, Q22, QDy1, E1, U1, U2, H101, w11, w12, w13;
	dp.setup("dOld", NX, 1, REAL, ACADO_LOCAL);
	dn.setup("dNew", NX, 1, REAL, ACADO_LOCAL);
	Gx1.setup("Gx1", NX, NX, REAL, ACADO_LOCAL);
	Gx2.setup("Gx2", NX, NX, REAL, ACADO_LOCAL);
	Gx3.setup("Gx3", NX, NX, REAL, ACADO_LOCAL);
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
	// moveGxT
	moveGxT.setup("moveGxT", Gx1, Gx2);
	moveGxT.addStatement( Gx2 == Gx1 );
	// multGxGx
	multGxGx.setup("multGxGx", Gx1, Gx2, Gx3);
	multGxGx.addStatement( Gx3 == Gx1 * Gx2 );
	// multGxGu
	multGxGu.setup("multGxGu", Gx1, Gu1, Gu2);
	multGxGu.addStatement( Gu2 == Gx1 * Gu1 );
	// moveGuE
	moveGuE.setup("moveGuE", Gu1, Gu2);
	moveGuE.addStatement( Gu2 == Gu1 );

	unsigned offset = (performFullCondensing() == BT_TRUE) ? 0 : NX;

	// setBlockH11
	setBlockH11.setup("setBlockH11", iRow.makeArgument(), iCol.makeArgument(), Gu1, Gu2);
	setBlockH11.addStatement( H.getSubMatrix(offset + iRow * NU, offset + (iRow + 1) * NU, offset + iCol * NU, offset + (iCol + 1) * NU) += (Gu1 ^ Gu2) );
	// zeroBlockH11
	zeroBlockH11.setup("zeroBlockH11", iRow.makeArgument(), iCol.makeArgument());
	zeroBlockH11.addStatement( H.getSubMatrix(offset + iRow * NU, offset + (iRow + 1) * NU, offset + iCol * NU, offset + (iCol + 1) * NU) == zeros(NU, NU) );
	// copyHTH
	copyHTH.setup("copyHTH", iRow.makeArgument(), iCol.makeArgument());
	copyHTH.addStatement(
			H.getSubMatrix(offset + iRow * NU, offset + (iRow + 1) * NU, offset + iCol * NU, offset + (iCol + 1) * NU) ==
					H.getSubMatrix(offset + iCol * NU, offset + (iCol + 1) * NU, offset + iRow * NU, offset + (iRow + 1) * NU).getTranspose()
	);
	// multRDy
	multRDy.setup("multRDy", R22, Dy1, RDy1);
	multRDy.addStatement( RDy1 == R22 * Dy1 );
	// mult QDy1
	multQDy.setup("multQDy", Q22, Dy1, QDy1);
	multQDy.addStatement( QDy1 == Q22 * Dy1 );
	// multEQDy;
	multEQDy.setup("multEQDy", E1, QDy1, U1);
	multEQDy.addStatement( U1 += (E1 ^ QDy1) );
	// multQETGx
	multQETGx.setup("multQETGx", E1, Gx1, H101);
	multQETGx.addStatement( H101 += (E1 ^ Gx1) );
	// zerBlockH10
	zeroBlockH10.setup("zeroBlockH10", H101);
	zeroBlockH10.addStatement( H101 == zeros(NU, NX) );

	if (performsSingleShooting() == BT_FALSE)
	{
		// multEDu
		multEDu.setup("multEDu", E1, U1, dn);
		multEDu.addStatement( dn += E1 * U1 );
	}

	if (Q1.isGiven() == BT_TRUE)
	{
		// multQ1Gx
		multQ1Gx.setup("multQ1Gx", Gx1, Gx2);
		multQ1Gx.addStatement( Gx2 == Q1 * Gx1 );

		// multQN1Gx
		multQN1Gx.setup("multQN1Gx", Gx1, Gx2);
		multQN1Gx.addStatement( Gx2 == QN1 * Gx1 );

		// multQ1Gu
		multQ1Gu.setup("multQ1Gu", Gu1, Gu2);
		multQ1Gu.addStatement( Gu2 == Q1 * Gu1 );

		// multQN1Gu
		multQN1Gu.setup("multQN1Gu", Gu1, Gu2);
		multQN1Gu.addStatement( Gu2 == QN1 * Gu1 );

		// multQ1d
		multQ1d.setup("multQ1d", Q1, dp, dn);
		multQ1d.addStatement( dn == Q1 * dp );

		// multQN1d
		multQN1d.setup("multQN1d", QN1, dp, dn);
		multQN1d.addStatement( dn == QN1 * dp );
	}
	else
	{
		// multQ1d
		multQ1d.setup("multQ1d", Gx1, dp, dn);
		multQ1d.addStatement( dn == Gx1 * dp );
	}

	if (performFullCondensing() == BT_FALSE)
	{
		// zeroBlockH00
		zeroBlockH00.setup( "zeroBlockH00" );
		zeroBlockH00.addStatement( H00 == zeros(NX, NX) );

		// multCTQC
		multCTQC.setup("multCTQC", Gx1, Gx2);
		multCTQC.addStatement( H00 += (Gx1 ^ Gx2) );
	}

	// N2 condensing related

	//ExportFunction multBTW1, multGxTGu, macQEW2;

	// multBTW1, evGu.getAddress(row * NX), W1, row, col
	multBTW1.setup("multBTW1", Gu1, Gu2, iRow.makeArgument(), iCol.makeArgument());
	multBTW1.addStatement(
			H.getSubMatrix(iRow * NU, (iRow + 1) * NU, iCol * NU, (iCol + 1) * NU) ==
					(Gu1 ^ Gu2)
//					(Gu2 ^ Gu1)
	);

	multGxTGu.setup("multGxTGu", Gx1, Gu1, Gu2);
	multGxTGu.addStatement( Gu2 == (Gx1 ^ Gu1) );

	// macQEW2, Q1, E.getAddress((offest + row - col - 1) * NX), W2, W1
	ExportVariable Q11;
	if (Q1.isGiven())
		Q11 = Q1;
	else
		Q11.setup("Q11", NX, NX, REAL, ACADO_LOCAL);


	macQEW2.setup("multQEW2", Q11, Gu1, Gu2, Gu3);
	macQEW2.addStatement( Gu3 == Gu2 + Q11 * Gu1 );

	// ExportFunction macATw1Q, macBTw1, macQSbarW2, macASbarD;
	macASbar.setup("macASbar", Gx1, w11, w12);
	macASbar.addStatement( w12 += Gx1 * w11 );

//	macASbarD2.setup("macASbarD2", Gx1, w11, w12, w13);
//	macASbarD2.addStatement( w13 == Gx1 * w11 );
//	macASbarD2.addStatement( w13 -= w12 );

	macBTw1.setup("macBTw1", Gu1, w11, 	U1);
	macBTw1.addStatement( U1 += Gu1 ^ w11 );

	// macATw1QDy, A.getAddress(i * NX), w1, QDy.getAddress(i * NX), w2
	macATw1QDy.setup("macATw1QDy", Gx1, w11, w12, w13);
	macATw1QDy.addStatement( w13 == w12 + (Gx1 ^ w11) );

	// macQSbarW2, Q1, sbar.getAddress(i * NX), w2, w1
	macQSbarW2.setup("macQSbarW2", Q11, w11, w12, w13);
	macQSbarW2.addStatement( w13 == w12 + Q11 * w11 );

	/*
	expansionStep, evGx.getAddress(row * NX), evGu.getAddress(row * NX),
				xVars.getAddress(row * NU), sbar.getAddress(row * NX),
				sbar.getAddress((row + 1) * NX), x.getAddress(row + 1)
	 * */
	expansionStep.setup("expansionStep", Gx1, Gu1, U1, w11, w12);
	expansionStep.addStatement( w12 += Gx1 * w11 );
	expansionStep.addStatement( w12 += Gu1 * U1 );

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

	preparation.addFunctionCall( modelSimulation );
	preparation.addFunctionCall( evaluateObjective );
	preparation.addFunctionCall( condensePrep );

	////////////////////////////////////////////////////////////////////////////
	//
	// Feedback phase
	//
	////////////////////////////////////////////////////////////////////////////

	ExportVariable tmp("tmp", 1, 1, INT, ACADO_LOCAL, BT_TRUE);
	tmp.setDoc( "Status code of the qpOASES QP solver." );

	ExportFunction solve("solve");
	solve.setReturnValue( tmp );

	feedback.setup("feedbackStep");
	feedback.doc( "Feedback/estimation step of the RTI scheme." );
	feedback.setReturnValue( tmp );

	feedback.addFunctionCall( condenseFdb );
	feedback.addLinebreak();

	stringstream s;
	s << tmp.getName().getName() << " = " << solve.getName().getName() << "( );" << endl;
	feedback.addStatement( s.str().c_str() );
	feedback.addLinebreak();

	feedback.addFunctionCall( expand );

	////////////////////////////////////////////////////////////////////////////
	//
	// Setup evaluation of the KKT tolerance
	//
	////////////////////////////////////////////////////////////////////////////

	ExportVariable kkt("kkt", 1, 1, REAL, ACADO_LOCAL, BT_TRUE);
	ExportVariable prd("prd", 1, 1, REAL, ACADO_LOCAL, BT_TRUE);
	ExportIndex index( "index" );

	getKKT.setup( "getKKT" );
	getKKT.doc( "Get the KKT tolerance of the current iterate." );
	kkt.setDoc( "The KKT tolerance value." );
	getKKT.setReturnValue( kkt );
	getKKT.addVariable( prd );
	getKKT.addIndex( index );

	// ACC = |\nabla F^T * xVars|
	getKKT.addStatement( kkt == (g ^ xVars) );
	getKKT.addStatement( kkt.getFullName() << " = fabs( " << kkt.getFullName() << " );\n");

	ExportForLoop bLoop(index, 0, getNumQPvars());

	bLoop.addStatement( prd == yVars.getRow( index ) );
	bLoop.addStatement( (String)"if (" << prd.getFullName() << " > " << 1.0 / INFTY << ")\n" );
	bLoop.addStatement( kkt.getFullName() << " += fabs(" << lb.get(index, 0) << " * " << prd.getFullName() << ");\n" );
	bLoop.addStatement( (String)"else if (" << prd.getFullName() << " < " << -1.0 / INFTY << ")\n" );
	bLoop.addStatement( kkt.getFullName() << " += fabs(" << ub.get(index, 0) << " * " << prd.getFullName() << ");\n" );
	getKKT.addStatement( bLoop );

	if ((getNumStateBounds() + getNumComplexConstraints())> 0)
	{
		ExportForLoop cLoop(index, 0, getNumStateBounds() + getNumComplexConstraints());

		cLoop.addStatement( prd == yVars.getRow( getNumQPvars() + index ) );
		cLoop.addStatement( (String)"if (" << prd.getFullName() << " > " << 1.0 / INFTY << ")\n" );
		cLoop.addStatement( kkt.getFullName() << " += fabs(" << lbA.get(index, 0) << " * " << prd.getFullName() << ");\n" );
		cLoop.addStatement( (String)"else if (" << prd.getFullName() << " < " << -1.0 / INFTY << ")\n" );
		cLoop.addStatement( kkt.getFullName() << " += fabs(" << ubA.get(index, 0) << " * " << prd.getFullName() << ");\n" );

		getKKT.addStatement( cLoop );
	}

	return SUCCESSFUL_RETURN;
}

returnValue ExportGaussNewtonCN2::setupQPInterface( )
{
	ExportQpOasesInterface* qpInterface;

	String folderName = dynamic_cast< ExportModule* >( userInteraction )->getExportFolderName();
	String moduleName = dynamic_cast< ExportModule* >( userInteraction )->getName();
	String sourceFile = folderName + "/" + moduleName + "_qpoases_interface.cpp";
	String headerFile = folderName + "/" + moduleName + "_qpoases_interface.hpp";

	qpInterface = new ExportQpOasesInterface(headerFile, sourceFile, "");

	int useSinglePrecision;
	get(USE_SINGLE_PRECISION, useSinglePrecision);

	int hotstartQP;
	get(HOTSTART_QP, hotstartQP);

	int covCalc;
	get(CG_COMPUTE_COVARIANCE_MATRIX, covCalc);

	double eps;
	string realT;

	if ( useSinglePrecision )
	{
		eps = 1.193e-07;
		realT = "float";
	}
	else
	{
		eps = 2.221e-16;
		realT = "double";
	}

	int maxNumQPiterations;
	get( MAX_NUM_QP_ITERATIONS,maxNumQPiterations );

	// if not specified, use default value
	if ( maxNumQPiterations <= 0 )
		maxNumQPiterations = 3 * (getNumQPvars() + getNumStateBounds() + getNumComplexConstraints());

	//
	// Set up export of the source file
	//

	string solverName;
	stringstream s;
	stringstream ctor;

	if ((getNumStateBounds() + getNumComplexConstraints()) > 0)
	{
		solverName = "QProblem";

		s << H.getFullName().getName() << ", "
				<< g.getFullName().getName() << ", "
				<< A.getFullName().getName() << ", "
				<< lb.getFullName().getName() << ", "
				<< ub.getFullName().getName() << ", "
				<< lbA.getFullName().getName() << ", "
				<< ubA.getFullName().getName() << ", "
				<< "nWSR";

		if ( (BooleanType)hotstartQP == BT_TRUE )
			s << ", " << yVars.getFullName().getName();

		s << "";

		ctor << solverName << " qp(" << getNumQPvars() << ", "
				<< getNumStateBounds() + getNumComplexConstraints()
				<< ")";
	}
	else
	{
		solverName = "QProblemB";

		s << H.getFullName().getName() << ", "
				<< g.getFullName().getName() << ", "
				<< lb.getFullName().getName() << ", "
				<< ub.getFullName().getName() << ", "
				<< "nWSR";

		if ( (BooleanType)hotstartQP == BT_TRUE )
			s << ", " << yVars.getFullName().getName();

		s << "";

		ctor << solverName << " qp( " << getNumQPvars() << " )";
	}

	string primal( xVars.getFullName().getName() );
	string dual( yVars.getFullName().getName() );
	string commonHeader( commonHeaderName.getName() );

	string strSigma;
//	if (covCalc)
//		strSigma = string( sigma.getFullName().getName() );
//	else
		strSigma = "";

	qpInterface->configure(
			"",
			"QPOASES_HEADER",
			getNumQPvars(),
			getNumStateBounds() + getNumComplexConstraints(),
			maxNumQPiterations,
			"PL_NONE",
			eps,
			realT,

			commonHeader,
			solverName,
			"",
			s.str(),
			primal,
			dual,
			ctor.str(),
			strSigma
	);

	returnValue qpStatus = qpInterface->exportCode();

	delete qpInterface;

	if (qpStatus != SUCCESSFUL_RETURN)
		return qpStatus;

	return SUCCESSFUL_RETURN;
}

BooleanType ExportGaussNewtonCN2::performFullCondensing() const
{
	int sparseQPsolution;
	get(SPARSE_QP_SOLUTION, sparseQPsolution);

	if ((SparseQPsolutionMethods)sparseQPsolution == CONDENSING)
		return BT_FALSE;

	return BT_TRUE;
}

//
// Solver registration
//

ExportNLPSolver* createGaussNewtonCN2(	UserInteraction* _userInteraction,
										const String& _commonHeaderName
										)
{
	return new ExportGaussNewtonCN2(_userInteraction, _commonHeaderName);
}

RegisterGaussNewtonCN2::RegisterGaussNewtonCN2()
{
	NLPSolverFactory::instance().registerAlgorithm(GAUSS_NEWTON_CN2, createGaussNewtonCN2);
}

CLOSE_NAMESPACE_ACADO
