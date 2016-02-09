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
 *    \file src/code_generation/export_gauss_newton_cn2_factorization.cpp
 *    \author Milan Vukov
 *    \date 2013
 */

#include <acado/code_generation/export_gauss_newton_cn2_factorization.hpp>
#include <acado/code_generation/export_qpoases_interface.hpp>
#include <acado/code_generation/export_qpoases3_interface.hpp>

using namespace std;

BEGIN_NAMESPACE_ACADO

ExportGaussNewtonCn2Factorization::ExportGaussNewtonCn2Factorization(	UserInteraction* _userInteraction,
																		const std::string& _commonHeaderName
																		) : ExportNLPSolver( _userInteraction,_commonHeaderName )
{}

returnValue ExportGaussNewtonCn2Factorization::setup( )
{
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

	LOG( LVL_DEBUG ) << "Solver: setup evaluation... " << endl;
	setupEvaluation();
	LOG( LVL_DEBUG ) << "done!" << endl;

	LOG( LVL_DEBUG ) << "Solver: setup auxiliary functions... " << endl;
	setupAuxiliaryFunctions();
	LOG( LVL_DEBUG ) << "done!" << endl;

	return SUCCESSFUL_RETURN;
}

returnValue ExportGaussNewtonCn2Factorization::getDataDeclarations(	ExportStatementBlock& declarations,
																	ExportStruct dataStruct
																	) const
{
	ExportNLPSolver::getDataDeclarations(declarations, dataStruct);

	declarations.addDeclaration(sbar, dataStruct);
	declarations.addDeclaration(x0, dataStruct);
	declarations.addDeclaration(Dx0, dataStruct);

	declarations.addDeclaration(W1, dataStruct);
	declarations.addDeclaration(W2, dataStruct);

	declarations.addDeclaration(D, dataStruct);
	declarations.addDeclaration(L, dataStruct);

	declarations.addDeclaration(T1, dataStruct);
	declarations.addDeclaration(T2, dataStruct);
	declarations.addDeclaration(T3, dataStruct);

	declarations.addDeclaration(E, dataStruct);
	declarations.addDeclaration(F, dataStruct);

	declarations.addDeclaration(QDy, dataStruct);
	declarations.addDeclaration(w1, dataStruct);
	declarations.addDeclaration(w2, dataStruct);

	declarations.addDeclaration(lbValues, dataStruct);
	declarations.addDeclaration(ubValues, dataStruct);
	declarations.addDeclaration(lbAValues, dataStruct);
	declarations.addDeclaration(ubAValues, dataStruct);

	if (performFullCondensing() == true)
		declarations.addDeclaration(A10, dataStruct);
	declarations.addDeclaration(A20, dataStruct);

	declarations.addDeclaration(pacA01Dx0, dataStruct);
	declarations.addDeclaration(pocA02Dx0, dataStruct);

	declarations.addDeclaration(H, dataStruct);
	declarations.addDeclaration(U, dataStruct);
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

returnValue ExportGaussNewtonCn2Factorization::getFunctionDeclarations(	ExportStatementBlock& declarations
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

returnValue ExportGaussNewtonCn2Factorization::getCode(	ExportStatementBlock& code
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
	code.addFunction( setObjQN1QN2 );
	code.addFunction( evaluateObjective );

//	code.addFunction( multGxd );
	code.addFunction( moveGxT );
//	code.addFunction( multGxGx );
	code.addFunction( multGxGu );
	code.addFunction( moveGuE );

	code.addFunction( multBTW1 );
	code.addFunction( macBTW1_R1 );
	code.addFunction( multGxTGu );
	code.addFunction( macQEW2 );

	code.addFunction( mult_H_W2T_W3 );
	code.addFunction( mac_H_W2T_W3_R );
	code.addFunction( mac_W3_G_W1T_G );

	code.addFunction( mac_R_T2_B_D );
	code.addFunction( move_D_U );
	code.addFunction( mult_L_E_U );
	code.addFunction( updateQ );
	code.addFunction( mul_T2_A_L );
	code.addFunction( mult_BT_T1_T2 );

//	ExportFunction mac_R_BT_F_D, mult_FT_A_L;
//		ExportFunction updateQ2;
//		ExportFunction mac_W1_T1_E_F;

	code.addFunction( mac_R_BT_F_D );
	code.addFunction( mult_FT_A_L );
	code.addFunction( updateQ2 );
	code.addFunction( mac_W1_T1_E_F );
	code.addFunction( move_GxT_T3 );

	cholSolver.getCode( code );

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
	code.addFunction( multQ1Gu );
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


unsigned ExportGaussNewtonCn2Factorization::getNumQPvars( ) const
{
	if (performFullCondensing() == true)
		return (N * NU);

	return (NX + N * NU);
}

unsigned ExportGaussNewtonCn2Factorization::getNumStateBounds() const
{
	return xBoundsIdxRev.size();
}

//
// PROTECTED FUNCTIONS:
//

returnValue ExportGaussNewtonCn2Factorization::setupObjectiveEvaluation( void )
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
	unsigned indexX = getNY();
//	unsigned indexG = indexX;

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

		if (tmpFx.isGiven() == true)
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
				if (objEvFx.isGiven() == true)

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

	if (R1.isGiven() == false)
	{
		ExportVariable tmpR1, tmpR2;
		tmpR1.setup("tmpR1", NU, NU, REAL, ACADO_LOCAL);
		tmpR2.setup("tmpR2", NU, NY, REAL, ACADO_LOCAL);

		setObjR1R2.setup("setObjR1R2", tmpFu, tmpObjS, tmpR1, tmpR2);
		setObjR1R2.addStatement( tmpR2 == (tmpFu ^ tmpObjS) );
		setObjR1R2.addStatement( tmpR1 == tmpR2 * tmpFu );

		if (tmpFu.isGiven() == true)
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
	evaluateObjective.addStatement( objValueIn.getCols(NX, NX + NOD) == od.getRow( N ) );

	// Evaluate the objective function
	evaluateObjective.addFunctionCall(evaluateTerminalCost, objValueIn, objValueOut);
	evaluateObjective.addLinebreak( );

	evaluateObjective.addStatement( DyN.getTranspose() == objValueOut.getCols(0, NYN) );
	evaluateObjective.addLinebreak();

	if (QN1.isGiven() == false)
	{
		indexX = getNYN();

		ExportVariable tmpQN1, tmpQN2;
		tmpQN1.setup("tmpQN1", NX, NX, REAL, ACADO_LOCAL);
		tmpQN2.setup("tmpQN2", NX, NYN, REAL, ACADO_LOCAL);

		setObjQN1QN2.setup("setObjQN1QN2", tmpFxEnd, tmpObjSEndTerm, tmpQN1, tmpQN2);
		setObjQN1QN2.addStatement( tmpQN2 == (tmpFxEnd ^ tmpObjSEndTerm) );
		setObjQN1QN2.addStatement( tmpQN1 == tmpQN2 * tmpFxEnd );

		if (tmpFxEnd.isGiven() == true)
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

returnValue ExportGaussNewtonCn2Factorization::setupConstraintsEvaluation( void )
{
	ExportVariable tmp("tmp", 1, 1, REAL, ACADO_LOCAL, true);

	int hardcodeConstraintValues;
	get(CG_HARDCODE_CONSTRAINT_VALUES, hardcodeConstraintValues);

	////////////////////////////////////////////////////////////////////////////
	//
	// Determine dimensions of constraints
	//
	////////////////////////////////////////////////////////////////////////////

	unsigned numBounds = initialStateFixed( ) == true ? N * NU : NX + N * NU;
	unsigned offsetBounds = initialStateFixed( ) == true ? 0 : NX;
	unsigned numStateBounds = getNumStateBounds();
//	unsigned numPathCon = N * dimPacH;
//	unsigned numPointCon = dimPocH;

	////////////////////////////////////////////////////////////////////////////
	//
	// Setup the bounds on independent variables
	//
	////////////////////////////////////////////////////////////////////////////

	DVector lbBoundValues( numBounds );
	DVector ubBoundValues( numBounds );

	if (initialStateFixed( ) == false)
		for(unsigned el = 0; el < NX; ++el)
		{
			lbBoundValues( el )= xBounds.getLowerBound(0, el);
			ubBoundValues( el ) = xBounds.getUpperBound(0, el);
		}

	// UPDATED
	for(unsigned node = 0; node < N; ++node)
		for(unsigned el = 0; el < NU; ++el)
		{
			lbBoundValues(offsetBounds + (N - 1 - node) * NU + el) = uBounds.getLowerBound(node, el);
			ubBoundValues(offsetBounds + (N - 1 - node) * NU + el) = uBounds.getUpperBound(node, el);
		}

	if (hardcodeConstraintValues == YES)
	{
		lbValues.setup("lbValues", lbBoundValues, REAL, ACADO_VARIABLES);
		ubValues.setup("ubValues", ubBoundValues, REAL, ACADO_VARIABLES);
	}
	else
	{
		lbValues.setup("lbValues", numBounds, 1, REAL, ACADO_VARIABLES);
		lbValues.setDoc( "Lower bounds values." );
		ubValues.setup("ubValues", numBounds, 1, REAL, ACADO_VARIABLES);
		ubValues.setDoc( "Upper bounds values." );
	}

	ExportFunction* boundSetFcn = hardcodeConstraintValues == YES ? &condensePrep : &condenseFdb;

	if (performFullCondensing() == true)
	{
		ExportVariable uCol = u.makeColVector();
		// Full condensing case
		for (unsigned blk = 0; blk < N; ++blk)
		{
			boundSetFcn->addStatement( lb.getRows(blk * NU, (blk + 1) * NU) ==
					lbValues.getRows(blk * NU, (blk + 1) * NU) - uCol.getRows((N - 1 - blk) * NU, (N - blk) * NU));
			boundSetFcn->addStatement( ub.getRows(blk * NU, (blk + 1) * NU) ==
					ubValues.getRows(blk * NU, (blk + 1) * NU) - uCol.getRows((N - 1 - blk) * NU, (N - blk) * NU));
		}
	}
	else
	{
		// Partial condensing case
		if (initialStateFixed() == true)
		{
			// MPC case
			condenseFdb.addStatement( lb.getRows(0, NX) == Dx0 );
			condenseFdb.addStatement( ub.getRows(0, NX) == Dx0 );

			boundSetFcn->addStatement( lb.getRows(NX, getNumQPvars()) == lbValues - u.makeColVector() );
			boundSetFcn->addStatement( ub.getRows(NX, getNumQPvars()) == ubValues - u.makeColVector() );
		}
		else
		{
			// MHE case
			boundSetFcn->addStatement( lb.getRows(0, NX) == lbValues.getRows(0, NX) - x.getRow( 0 ).getTranspose() );
			boundSetFcn->addStatement( lb.getRows(NX, getNumQPvars()) == lbValues.getRows(NX, getNumQPvars()) - u.makeColVector() );
			boundSetFcn->addStatement( ub.getRows(0, NX) == ubValues.getRows(0, NX) - x.getRow( 0 ).getTranspose() );
			boundSetFcn->addStatement( ub.getRows(NX, getNumQPvars()) == ubValues.getRows(NX, getNumQPvars()) - u.makeColVector() );
		}
	}
	boundSetFcn->addLinebreak( );

	////////////////////////////////////////////////////////////////////////////
	//
	// Determine number of affine constraints and set up structures that
	// holds constraint values
	//
	////////////////////////////////////////////////////////////////////////////

	unsigned sizeA = numStateBounds + getNumComplexConstraints();

	if ( sizeA )
	{
		if (hardcodeConstraintValues == true)
		{
			DVector lbTmp, ubTmp;

			if ( numStateBounds )
			{
				DVector lbStateBoundValues( numStateBounds );
				DVector ubStateBoundValues( numStateBounds );
				for (unsigned i = 0; i < numStateBounds; ++i)
				{
					lbStateBoundValues( i ) = xBounds.getLowerBound( xBoundsIdxRev[ i ] / NX, xBoundsIdxRev[ i ] % NX );
					ubStateBoundValues( i ) = xBounds.getUpperBound( xBoundsIdxRev[ i ] / NX, xBoundsIdxRev[ i ] % NX );
				}

				lbTmp.append( lbStateBoundValues );
				ubTmp.append( ubStateBoundValues );
			}

			lbTmp.append( lbPathConValues );
			ubTmp.append( ubPathConValues );

			lbTmp.append( lbPointConValues );
			ubTmp.append( ubPointConValues );


			lbAValues.setup("lbAValues", lbTmp, REAL, ACADO_VARIABLES);
			ubAValues.setup("ubAValues", ubTmp, REAL, ACADO_VARIABLES);
		}
		else
		{
			lbAValues.setup("lbAValues", sizeA, 1, REAL, ACADO_VARIABLES);
			lbAValues.setDoc( "Lower bounds values for affine constraints." );
			ubAValues.setup("ubAValues", sizeA, 1, REAL, ACADO_VARIABLES);
			ubAValues.setDoc( "Lower bounds values for affine constraints." );
		}
	}

	////////////////////////////////////////////////////////////////////////////
	//
	// Setup the bounds on state variables
	//
	////////////////////////////////////////////////////////////////////////////

	if ( numStateBounds )
	{
		condenseFdb.addVariable( tmp );

		unsigned offset = (performFullCondensing() == true) ? 0 : NX;
		unsigned numOps = getNumStateBounds() * N * (N + 1) / 2 * NU;

		if (numOps < 1024)
		{
			for(unsigned row = 0; row < numStateBounds; ++row)
			{
				unsigned conIdx = xBoundsIdx[ row ];
				unsigned blk = conIdx / NX;

				// TODO
//				if (performFullCondensing() == false)
//					condensePrep.addStatement( A.getSubMatrix(ii, ii + 1, 0, NX) == evGx.getRow( conIdx ) );

				for (unsigned col = blk; col < N; ++col)
				{
					// blk = (N - row) * (N - 1 - row) / 2 + (N - 1 - col)
					unsigned blkRow = ((N - blk) * (N - 1 - blk) / 2 + (N - 1 - col)) * NX + conIdx % NX;

					condensePrep.addStatement(
							A.getSubMatrix(row, row + 1, offset + col * NU, offset + (col + 1) * NU ) == E.getRow( blkRow ) );
				}

				condensePrep.addLinebreak();
			}
		}
		else
		{
			DMatrix vXBoundIndices(1, numStateBounds);
			for (unsigned i = 0; i < numStateBounds; ++i)
				vXBoundIndices(0, i) = xBoundsIdx[ i ];
			ExportVariable evXBounds("xBoundIndices", vXBoundIndices, STATIC_CONST_INT, ACADO_LOCAL, false);

			condensePrep.addVariable( evXBounds );

			ExportIndex row, col, conIdx, blk, blkRow, blkIdx;

			condensePrep.acquire( row ).acquire( col ).acquire( conIdx ).acquire( blk ).acquire( blkRow ).acquire( blkIdx );

			ExportForLoop lRow(row, 0, numStateBounds);

			lRow << conIdx.getFullName() << " = " << evXBounds.getFullName() << "[ " << row.getFullName() << " ];\n";
			lRow.addStatement( blk == conIdx / NX );

			// TODO
//			if (performFullCondensing() == false)
//				eLoopI.addStatement( A.getSubMatrix(ii, ii + 1, 0, NX) == evGx.getRow( conIdx ) );

			ExportForLoop lCol(col, blk, N);

			lCol.addStatement( blkIdx == (N - blk) * (N - 1 - blk) / 2 + (N - 1 - col) );
			lCol.addStatement( blkRow == blkIdx * NX + conIdx % NX );
			lCol.addStatement(
					A.getSubMatrix(row, row + 1, offset + col * NU, offset + (col + 1) * NU ) == E.getRow( blkRow ) );

			lRow.addStatement( lCol );
			condensePrep.addStatement( lRow );

			condensePrep.release( row ).release( col ).release( conIdx ).release( blk ).release( blkRow ).release( blkIdx );
		}
		condensePrep.addLinebreak( );

		// Shift constraint bounds by first interval
		// MPC case, only
		ExportVariable xVec = x.makeRowVector();
		for(unsigned row = 0; row < getNumStateBounds( ); ++row)
		{
			unsigned conIdx = xBoundsIdxRev[ row ];

			condenseFdb.addStatement( tmp == sbar.getRow( conIdx ) + xVec.getCol( conIdx ) );
			condenseFdb.addStatement( lbA.getRow( row ) == lbAValues( row ) - tmp );
			condenseFdb.addStatement( ubA.getRow( row ) == ubAValues( row ) - tmp );
		}
		condenseFdb.addLinebreak( );
	}

	return SUCCESSFUL_RETURN;
}

returnValue ExportGaussNewtonCn2Factorization::setupCondensing( void )
{
	condensePrep.setup("condensePrep");
	condenseFdb.setup( "condenseFdb" );

	////////////////////////////////////////////////////////////////////////////
	//
	// Compute Hessian block H11
	//
	////////////////////////////////////////////////////////////////////////////

	unsigned prepCacheSize =
			2 * NX * NU +
			NU * NU + NU * NX +
			2 * NX * NX + NU * NX;

	int useSinglePrecision;
	get(USE_SINGLE_PRECISION, useSinglePrecision);
	prepCacheSize = prepCacheSize * (useSinglePrecision ? 4 : 8);
	LOG( LVL_DEBUG ) << "---> Condensing prep. part, cache size: " << prepCacheSize << " bytes" << endl;

	ExportStruct prepCache = prepCacheSize < 16384 ? ACADO_LOCAL : ACADO_WORKSPACE;

	W1.setup("W1", NX, NU, REAL, prepCache);
	W2.setup("W2", NX, NU, REAL, prepCache);

	D.setup("D", NU, NU, REAL, prepCache);
	L.setup("L", NU, NX, REAL, prepCache);

	T1.setup("T1", NX, NX, REAL, prepCache);
	T2.setup("T2", NU, NX, REAL, prepCache);
	T3.setup("T3", NX, NX, REAL, prepCache);

	condensePrep
			.addVariable( W1 ).addVariable( W2 )
			.addVariable( D ).addVariable( L )
			.addVariable( T1 ).addVariable( T2 ).addVariable( T3 );

	LOG( LVL_DEBUG ) << "---> Setup condensing: E" << endl;
	/// Setup E matrix as in the N^3 implementation

	// Special case, row = col = 0
	condensePrep.addFunctionCall(moveGuE, evGu.getAddress(0, 0), E.getAddress(0, 0) );

	if (N <= 15)
	{
		unsigned row, col, prev, curr;
		for (row = 1; row < N; ++row)
		{
			condensePrep.addFunctionCall(moveGxT, evGx.getAddress(row* NX, 0), T1);

			for(col = 0; col < row; ++col)
			{
				prev = row * (row - 1) / 2 + col;
				curr = (row + 1) * row / 2 + col;

				condensePrep.addFunctionCall(multGxGu, T1, E.getAddress(prev * NX, 0), E.getAddress(curr * NX, 0));
			}

			curr = (row + 1) * row / 2 + col;
			condensePrep.addFunctionCall(moveGuE, evGu.getAddress(row * NX, 0), E.getAddress(curr * NX, 0) );

			condensePrep.addLinebreak();
		}
	}
	else
	{
		ExportIndex row, col, curr, prev;

		condensePrep.acquire( row ).acquire( col ).acquire( curr ).acquire( prev );

		ExportForLoop lRow(row, 1, N), lCol(col, 0, row);

		lRow.addFunctionCall(moveGxT, evGx.getAddress(row* NX, 0), T1);

		lCol.addStatement( prev == row * (row - 1) / 2 + col );
		lCol.addStatement( curr == (row + 1) * row / 2 + col );
		lCol.addFunctionCall( multGxGu, T1, E.getAddress(prev * NX, 0), E.getAddress(curr * NX, 0) );

		lRow.addStatement( lCol );
		lRow.addStatement( curr == (row + 1) * row / 2 + col );
		lRow.addFunctionCall(moveGuE, evGu.getAddress(row * NX, 0), E.getAddress(curr * NX, 0) );

		condensePrep.addStatement( lRow );

		condensePrep.release( row ).release( col ).release( curr ).release( prev );
	}
	condensePrep.addLinebreak( 2 );

	LOG( LVL_DEBUG ) << "---> Setup condensing: H11" << endl;

	/*

	Using some heuristics, W1, W2, W3 can be allocated on stack.

	The algorithm to create the Hessian becomes:

	for col = 0: N - 1
		row = 0
		W1 = Q(N - row) * E(N - 1 - row, N - 1 - col)

		for row = 0: col - 1
			H(row, col) = B(N - 1 - row)^T * W1
			W2 = A(N - (row + 1))^T * W1
			W1 = W2 + Q(N - (row + 1)) * E(N - 1 - (row + 1), N - 1 - col)

		H(col, col) += R(N - 1 - col) + B(N - 1 - col)^T * W1

	Indexing of G matrix:
		QE(row, col) -> block index is calculated as: blk = (row + 1) * row / 2 + col
		G is with reversed rows and columns, say: G(row, col) = QE(N - 1 - row, N - 1 - col)
		In more details, G(row, col) block index is calculated as:
			blk = (N - row) * (N - 1 - row) / 2 + (N - 1 - col)

	 */

	if (N <= 15)
	{
		for (unsigned col = 0; col < N; ++col)
		{
			// row = 0
			unsigned curr = (N) * (N - 1) / 2 + (N - 1 - col);
			if (QN1.isGiven() == true)
				condensePrep.addFunctionCall(
						multQN1Gu, E.getAddress(curr * NX), W1
				);
			else
				condensePrep.addFunctionCall(
						multGxGu, QN1, E.getAddress(curr * NX), W1
				);

			for (unsigned row = 0; row < col; ++row)
			{
				condensePrep.addFunctionCall(
						multBTW1, evGu.getAddress((N - 1 - row) * NX), W1, ExportIndex( row ), ExportIndex( col )
				);

				condensePrep.addFunctionCall(
						multGxTGu, evGx.getAddress((N - (row + 1)) * NX), W1, W2
				);

				unsigned next = (N - (row + 1)) * (N - 1 - (row + 1)) / 2 + (N - 1 - col);
				if (Q1.isGiven() == true)
					condensePrep.addFunctionCall(
							macQEW2, Q1, E.getAddress(next * NX), W2, W1
					);
				else
					condensePrep.addFunctionCall(
							macQEW2,
							Q1.getAddress((N - (row + 1)) * NX), E.getAddress(next * NX), W2, W1
					);
			}

			if (R1.isGiven() == true)
				condensePrep.addFunctionCall(
						macBTW1_R1, R1, evGu.getAddress((N - 1 - col) * NX), W1, ExportIndex( col )
				);
			else
				condensePrep.addFunctionCall(
						macBTW1_R1, R1.getAddress((N - 1 - col) * NU), evGu.getAddress((N - 1 - col) * NX), W1, ExportIndex( col )
				);

			condensePrep.addLinebreak();
		}
	}
	else
	{
		// No loop unrolling...
		ExportIndex row, col, curr, next;
		condensePrep.acquire( row ).acquire( col ).acquire( curr ).acquire( next );

		ExportForLoop colLoop(col, 0, N);
		colLoop << (curr == (N) * (N - 1) / 2 + (N - 1 - col));

		if (QN1.isGiven() == true)
			colLoop.addFunctionCall(multQN1Gu, E.getAddress(curr * NX), W1);
		else
			colLoop.addFunctionCall(multGxGu, QN1, E.getAddress(curr * NX), W1);

		ExportForLoop rowLoop(row, 0, col);
		rowLoop.addFunctionCall(
				multBTW1, evGu.getAddress((N - 1 - row) * NX), W1, ExportIndex( row ), ExportIndex( col )
		);

		rowLoop.addFunctionCall(
				multGxTGu, evGx.getAddress((N - (row + 1)) * NX), W1, W2
		);

		rowLoop << (next == (N - (row + 1)) * (N - 1 - (row + 1)) / 2 + (N - 1 - col));
		if (Q1.isGiven() == true)
			rowLoop.addFunctionCall(macQEW2, Q1, E.getAddress(next * NX), W2, W1);
		else
			rowLoop.addFunctionCall(macQEW2, Q1.getAddress((N - (row + 1)) * NX), E.getAddress(next * NX), W2, W1);

		colLoop << rowLoop;

		if (R1.isGiven() == true)
			colLoop.addFunctionCall(
					macBTW1_R1, R1, evGu.getAddress((N - 1 - col) * NX), W1, ExportIndex( col )
			);
		else
			colLoop.addFunctionCall(
					macBTW1_R1, R1.getAddress((N - 1 - col) * NU), evGu.getAddress((N - 1 - col) * NX), W1, ExportIndex( col )
			);

		condensePrep << colLoop;
		condensePrep.release( row ).release( col ).release( curr ).release( next );
	}
	condensePrep.addLinebreak( 2 );

	LOG( LVL_DEBUG ) << "---> Copy H11 upper to lower triangular part" << endl;

	// Copy to H11 upper triangular part to lower triangular part
	if (N <= 15)
	{
		for (unsigned row = 0; row < N; ++row)
			for(unsigned col = 0; col < row; ++col)
				condensePrep.addFunctionCall( copyHTH, ExportIndex( row ), ExportIndex( col ) );
	}
	else
	{
		ExportIndex row, col;

		condensePrep.acquire( row ).acquire( col );

		ExportForLoop lRow(row, 0, N), lCol(col, 0, row);

		lCol.addFunctionCall(copyHTH, row, col);
		lRow.addStatement( lCol );
		condensePrep.addStatement( lRow );

		condensePrep.release( row ).release( col );
	}
	condensePrep.addLinebreak( 2 );

	LOG( LVL_DEBUG ) << "---> Factorization of the condensed Hessian" << endl;

	/*

	T1 = Q( N )
	for blk = N - 1: 1
		T2 = B( blk )^T * T1

		D = R( blk ) + T2 * B( blk )
		L = T2 * A( blk )

		chol( D )
		L = chol_solve(D, L) // L <- D^(-T) * L

		T3 = T1 * A( blk )
		T1 = Q( blk ) + A( blk )^T * T3
		T1 -= L^T * L

		row = N - 1 - blk

		U(row, row) = D
		for col = row + 1: N - 1
			U(row, col) = L * E(row + 1, col)

	T2 = B( 0 )^T * T1
	D = R( 0 ) + T2 * B( 0 )
	chol( D )
	U(N - 1, N - 1) = D

	************************************************
	************************************************
	OR, an alternative, n_x^3 free version would be:

	T1 = Q( N )
	for col = 0: N - 1
		F( col ) = T1 * E(0, col)

	for blk = N - 1: 1
		row = N - 1 - blk

		D = R( blk ) + B( blk )^T * F( row )
		L = F( row )^T * A( blk )

		chol( D )
		L = chol_solve(D, L) // L <- D^(-T) * L

		T1 = Q( blk ) - L^T * L

		T3 = A( blk )^T

		for col = row + 1: N - 1
			W1 = T3 * F( col )
			F( col ) = T1 * E(row + 1, col) + W1

		U(row, row) = D
		for col = row + 1: N - 1
			U(row, col) = L * E(row + 1, col)

	D = R( 0 ) + B( 0 )^T * F(N - 1)
	chol( D )
	U(N - 1, N - 1) = D

	 */

	// The matrix where we store the factorization
	U.setup("U", getNumQPvars(), getNumQPvars(), REAL, ACADO_WORKSPACE);
	// A helper matrix
	F.setup("F", N * NX, NU, REAL, ACADO_WORKSPACE);

	cholSolver.init(NU, NX, "condensing");
	cholSolver.setup();

	// Just for testing...
//	condensePrep.addStatement( U == H );
//	condensePrep.addFunctionCall(cholSolver.getCholeskyFunction(), U);
//	condensePrep.addFunctionCall(cholSolver.getSolveFunction(), U, Id);

#if 0
	// N^2 factorization with n_x^3 terms

	condensePrep.addStatement( T1 == QN1 );
	for (unsigned blk = N - 1; blk > 0; --blk)
	{
		condensePrep.addFunctionCall(mult_BT_T1_T2, evGu.getAddress(blk * NX), T1, T2);
		if (R1.isGiven() == true)
			condensePrep.addFunctionCall(mac_R_T2_B_D, R1, T2, evGu.getAddress(blk * NX), D);
		else
			condensePrep.addFunctionCall(mac_R_T2_B_D, R1.getAddress(blk * NX), T2, evGu.getAddress(blk * NX), D);
		condensePrep.addFunctionCall(mul_T2_A_L, T2, evGx.getAddress(blk * NX), L);

		condensePrep.addFunctionCall(cholSolver.getCholeskyFunction(), D);
		condensePrep.addFunctionCall(cholSolver.getSolveFunction(), D, L);

		if (Q1.isGiven() == true)
			condensePrep.addFunctionCall(updateQ, Q1, T3, evGx.getAddress(blk * NX), L, T1);
		else
			condensePrep.addFunctionCall(updateQ, Q1.getAddress(blk * NX), T3, evGx.getAddress(blk * NX), L, T1);

		unsigned row = N - 1 - blk;

		condensePrep.addFunctionCall(move_D_U, D, U, ExportIndex( row ));
		for (unsigned col = row + 1; col < N; ++col)
		{
			// U(row, col) = L * E(row + 1, col)

			// blk = (N - row) * (N - 1 - row) / 2 + (N - 1 - col)

			unsigned blkE = (N - (row + 1)) * (N - 1 - (row + 1)) / 2 + (N - 1 - col);
			cout << "blkE " << blkE << endl;

			condensePrep.addFunctionCall(mult_L_E_U, L, E.getAddress(blkE * NX), U, ExportIndex( row ), ExportIndex( col ));
		}
	}
	condensePrep.addFunctionCall(mult_BT_T1_T2, evGu.getAddress( 0 ), T1, T2);
	condensePrep.addFunctionCall(mac_R_T2_B_D, R1.getAddress( 0 ), T2, evGu.getAddress( 0 ), D);
	condensePrep.addFunctionCall(cholSolver.getCholeskyFunction(), D);
	condensePrep.addFunctionCall(move_D_U, D, U, ExportIndex( N - 1 ));
#else
	// N^2 factorization, n_x^3 free version

	// Initial value for the "updated" Q^* matrix
	condensePrep.addStatement( T1 == QN1 );

	if (N <= 15)
	{
		for (unsigned col = 0; col < N; ++col)
		{
			unsigned blkE = (N - 0) * (N - 1 - 0) / 2 + (N - 1 - col);
			condensePrep.addFunctionCall(
					multGxGu, T1, E.getAddress(blkE * NX), F.getAddress(col * NX));
		}

		for (unsigned blk = N - 1; blk > 0; --blk)
		{
			unsigned row = N - 1 - blk;

			// D = R( blk ) + B( blk )^T * F( row )
			// L = F( row )^T * A( blk )
			if (R1.isGiven() == true)
				condensePrep.addFunctionCall(
						mac_R_BT_F_D, R1, evGu.getAddress(blk * NX), F.getAddress(row * NX), D);
			else
				condensePrep.addFunctionCall(
						mac_R_BT_F_D, R1.getAddress(blk * NX), evGu.getAddress(blk * NX), F.getAddress(row * NX), D);
			condensePrep.addFunctionCall(mult_FT_A_L, F.getAddress(row * NX), evGx.getAddress(blk * NX), L);

			// Chol and system solve
			condensePrep.addFunctionCall(cholSolver.getCholeskyFunction(), D);
			condensePrep.addFunctionCall(cholSolver.getSolveFunction(), D, L);

			// Update Q
			if (Q1.isGiven() == true)
				condensePrep.addFunctionCall(updateQ2, Q1, L, T1);
			else
				condensePrep.addFunctionCall(updateQ2, Q1.getAddress(blk * NX), L, T1);

			// for col = row + 1: N - 1
			//	 W1 = A( blk )^T * F( col )
			//	 F( col ) = W1 + T1 * E(row + 1, col)

			condensePrep.addFunctionCall(move_GxT_T3, evGx.getAddress(blk * NX), T3);

			for (unsigned col = row + 1; col < N; ++col)
			{
				condensePrep.addFunctionCall(multGxGu, T3, F.getAddress(col * NX), W1);

				unsigned blkE = (N - (row + 1)) * (N - 1 - (row + 1)) / 2 + (N - 1 - col);
				condensePrep.addFunctionCall(mac_W1_T1_E_F, W1, T1, E.getAddress(blkE * NX), F.getAddress(col * NX));
			}

			// Calculate one block-row of the factorized matrix
			condensePrep.addFunctionCall(move_D_U, D, U, ExportIndex( row ));
			for (unsigned col = row + 1; col < N; ++col)
			{
				// U(row, col) = L * E(row + 1, col)

				unsigned blkE = (N - (row + 1)) * (N - 1 - (row + 1)) / 2 + (N - 1 - col);
				condensePrep.addFunctionCall(mult_L_E_U, L, E.getAddress(blkE * NX), U, ExportIndex( row ), ExportIndex( col ));
			}
		}
	}
	else
	{
		// Do not unroll the for-loops...

		ExportIndex col, blkE, blk, row;

		condensePrep.acquire( col ).acquire( blkE ).acquire( blk ).acquire( row );

		ExportForLoop colLoop(col, 0, N);
		colLoop << (blkE == (N - 0) * (N - 1 - 0) / 2 + (N - 1 - col));
		colLoop.addFunctionCall(
				multGxGu, T1, E.getAddress(blkE * NX), F.getAddress(col * NX));
		condensePrep << colLoop;

		ExportForLoop blkLoop(blk, N - 1, 0, -1);
		blkLoop << ( row == N - 1 - blk );

		if (R1.isGiven() == true)
			blkLoop.addFunctionCall(
					mac_R_BT_F_D, R1, evGu.getAddress(blk * NX), F.getAddress(row * NX), D);
		else
			blkLoop.addFunctionCall(
					mac_R_BT_F_D, R1.getAddress(blk * NX), evGu.getAddress(blk * NX), F.getAddress(row * NX), D);
		blkLoop.addFunctionCall(mult_FT_A_L, F.getAddress(row * NX), evGx.getAddress(blk * NX), L);

		// Chol and system solve
		blkLoop.addFunctionCall(cholSolver.getCholeskyFunction(), D);
		blkLoop.addFunctionCall(cholSolver.getSolveFunction(), D, L);

		// Update Q
		if (Q1.isGiven() == true)
			blkLoop.addFunctionCall(updateQ2, Q1, L, T1);
		else
			blkLoop.addFunctionCall(updateQ2, Q1.getAddress(blk * NX), L, T1);

		blkLoop.addFunctionCall(move_GxT_T3, evGx.getAddress(blk * NX), T3);

		ExportForLoop colLoop2(col, row + 1, N);
		colLoop2.addFunctionCall(multGxGu, T3, F.getAddress(col * NX), W1);
		colLoop2 << (blkE == (N - (row + 1)) * (N - 1 - (row + 1)) / 2 + (N - 1 - col));
		colLoop2.addFunctionCall(mac_W1_T1_E_F, W1, T1, E.getAddress(blkE * NX), F.getAddress(col * NX));
		blkLoop << colLoop2;

		// Calculate one block-row of the factorized matrix
		blkLoop.addFunctionCall(move_D_U, D, U, ExportIndex( row ));
		ExportForLoop colLoop3(col, row + 1, N);
		colLoop3 << (blkE == (N - (row + 1)) * (N - 1 - (row + 1)) / 2 + (N - 1 - col));
		colLoop3.addFunctionCall(mult_L_E_U, L, E.getAddress(blkE * NX), U, row, col);
		blkLoop << colLoop3;

		condensePrep << blkLoop;
		condensePrep.release( col ).release( blkE ).release( blk ).release( row );
	}

	// Calculate the bottom-right block of the factorized Hessian
	condensePrep.addFunctionCall(
			mac_R_BT_F_D, R1.getAddress(0 * NX), evGu.getAddress(0 * NX), F.getAddress((N - 1) * NX), D);
	condensePrep.addFunctionCall(cholSolver.getCholeskyFunction(), D);
	condensePrep.addFunctionCall(move_D_U, D, U, ExportIndex( N - 1 ));
	condensePrep.addLinebreak( 2 );

#endif

	////////////////////////////////////////////////////////////////////////////
	//
	// Compute gradient components g0 and g1
	//
	////////////////////////////////////////////////////////////////////////////

	//// NEW CODE START

	LOG( LVL_DEBUG ) << "Setup condensing: create Dx0, Dy and DyN" << endl;

	{
		condenseFdb.addStatement( Dx0 == x0 - x.getRow( 0 ).getTranspose() );
		condenseFdb.addLinebreak();
	}

	condenseFdb.addStatement( Dy -= y );
	condenseFdb.addStatement( DyN -= yN );
	condenseFdb.addLinebreak();

	// Compute RDy, UPDATED
	for(unsigned blk = 0; blk < N; ++blk)
	{
		if (R2.isGiven() == true)
			condenseFdb.addFunctionCall(
					multRDy, R2,
					Dy.getAddress(blk * NY, 0),
					g.getAddress((N - 1 - blk) * NU, 0) );
		else
			condenseFdb.addFunctionCall(
					multRDy, R2.getAddress(blk * NU, 0),
					Dy.getAddress(blk * NY, 0),
					g.getAddress((N - 1 - blk) * NU, 0) );
	}
	condenseFdb.addLinebreak();

	// Compute QDy
	// NOTE: This is just for the MHE case :: run1 starts from 0; in MPC :: from 1 ;)
	for(unsigned blk = 0; blk < N; blk++ )
	{
		if (Q2.isGiven() == true)
			condenseFdb.addFunctionCall(
					multQDy, Q2,
					Dy.getAddress(blk * NY),
					QDy.getAddress(blk * NX) );
		else
			condenseFdb.addFunctionCall(
					multQDy, Q2.getAddress(blk * NX),
					Dy.getAddress(blk * NY),
					QDy.getAddress(blk * NX) );
	}
	condenseFdb.addLinebreak();
	condenseFdb.addStatement( QDy.getRows(N * NX, (N + 1) * NX) == QN2 * DyN );
	condenseFdb.addLinebreak();

	/*

	OK, we need to adapt the old N^2 algorithm: u vector is in reverse order

	for k = 0: N - 1
		g1_k = r_{N - 1 - k}; // OK, updated (look up)

	sbar_0 = Dx_0;
	sbar(1: N) = d;

	for k = 0: N - 1
		sbar_{k + 1} += A_k sbar_k; // can stay the same

	w1 = Q_N^T * sbar_N + q_N;
	for k = N - 1: 1
	{
		g1_{N - 1 - k} += B_k^T * w1;
		w2 = A_k^T * w1 + q_k;
		w1 = Q_k^T * sbar_k + w2;
	}

	g1_{N - 1} += B_0^T * w1;

	*/

	sbar.setup("sbar", (N + 1) * NX, 1, REAL, ACADO_WORKSPACE);
	w1.setup("w1", NX, 1, REAL, ACADO_WORKSPACE);
	w2.setup("w2", NX, 1, REAL, ACADO_WORKSPACE);

	condenseFdb.addStatement( sbar.getRows(0, NX) == Dx0 );

	if( performsSingleShooting() == false )
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
				macBTw1, evGu.getAddress(i * NX), w1, g.getAddress((N - 1 - i) * NU) // UPDATED
		);
		condenseFdb.addFunctionCall(
				macATw1QDy, evGx.getAddress(i * NX), w1, QDy.getAddress(i * NX), w2 // Proveri indexiranje za QDy
		);
		if (Q1.isGiven() == true)
			condenseFdb.addFunctionCall(
					macQSbarW2, Q1, sbar.getAddress(i * NX), w2, w1
			);
		else
			condenseFdb.addFunctionCall(
					macQSbarW2, Q1.getAddress(i * NX), sbar.getAddress(i * NX), w2, w1
			);
	}
	condenseFdb.addFunctionCall(
			macBTw1, evGu.getAddress( 0 ), w1, g.getAddress((N - 1) * NU) // UPDATED
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
		Ds_{k + 1} += B_k * Du_{N - 1 - k};
		s_{k + 1}  += Ds_{k + 1};
	}

	// TODO Calculation of multipliers, modify the order of Ds and Du

	mu_N = lambda_N + Q_N^T * s_N
	for i = N - 1: 1
		mu_k = lambda_k + Q_k^T * s_k + A_k^T * mu_{k + 1} + q_k

	mu_0 = Q_0^T s_0 + A_0^T * mu_1 + q_0

	 */

	LOG( LVL_DEBUG ) << "Setup condensing: create expand routine" << endl;

	expand.setup( "expand" );

	if (performFullCondensing() == true)
	{
		ExportVariable xVarsTranspose = xVars.getTranspose();
		for (unsigned row = 0; row < N; ++row)
			expand.addStatement( u.getRow( row ) += xVarsTranspose.getCols((N - 1 - row) * NU, (N - row) * NU) );
	}
	else
	{
		// TODO, does not work yet.
		expand.addStatement( x.makeColVector().getRows(0, NX) += xVars.getRows(0, NX) );
		expand.addLinebreak();
		expand.addStatement( u.makeColVector() += xVars.getRows(NX, getNumQPvars()) );
	}

	expand.addStatement( sbar.getRows(0, NX) == Dx0 );
	expand.addStatement( sbar.getRows(NX, (N + 1) * NX) == d );

	for (unsigned row = 0; row < N; ++row )
		expand.addFunctionCall(
				expansionStep, evGx.getAddress(row * NX), evGu.getAddress(row * NX),
				xVars.getAddress((N - 1 - row) * NU), sbar.getAddress(row * NX), // UPDATED
				sbar.getAddress((row + 1) * NX)
		);

	expand.addStatement( x.makeColVector() += sbar );

	return SUCCESSFUL_RETURN;
}

returnValue ExportGaussNewtonCn2Factorization::setupVariables( )
{
	////////////////////////////////////////////////////////////////////////////
	//
	// Make index vector for state constraints, reverse order
	//
	////////////////////////////////////////////////////////////////////////////

	bool boxConIsFinite = false;
	xBoundsIdxRev.clear();
	xBoundsIdx.clear();

	DVector lbBox, ubBox;
	unsigned dimBox = xBounds.getNumPoints();
	for (int i = dimBox - 1; i >= 0; --i)
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
				xBoundsIdxRev.push_back(i * lbBox.getDim() + j);
				xBoundsIdx.push_back((dimBox - 1 - i) * lbBox.getDim() + j);
			}
		}
	}

	if (initialStateFixed() == true)
	{
		x0.setup("x0",  NX, 1, REAL, ACADO_VARIABLES);
		x0.setDoc( (std::string)"Current state feedback vector." );
		Dx0.setup("Dx0", NX, 1, REAL, ACADO_WORKSPACE);
	}

	E.setup("E", N * (N + 1) / 2 * NX, NU, REAL, ACADO_WORKSPACE);
	QE.setup("QE", N * (N + 1) / 2 * NX, NU, REAL, ACADO_WORKSPACE);
	QGx.setup("QGx", N * NX, NX, REAL, ACADO_WORKSPACE);

	QDy.setup ("QDy", (N + 1) * NX, 1, REAL, ACADO_WORKSPACE);

	// Setup all QP stuff

	H.setup("H", getNumQPvars(), getNumQPvars(), REAL, ACADO_WORKSPACE);

	// Stupid aliasing to avoid copying of data
	if (performFullCondensing() == true)
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

	if (performFullCondensing() == true)
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

returnValue ExportGaussNewtonCn2Factorization::setupMultiplicationRoutines( )
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

	unsigned offset = (performFullCondensing() == true) ? 0 : NX;

	// setBlockH11
	setBlockH11.setup("setBlockH11", iRow, iCol, Gu1, Gu2);
	setBlockH11.addStatement( H.getSubMatrix(offset + iRow * NU, offset + (iRow + 1) * NU, offset + iCol * NU, offset + (iCol + 1) * NU) += (Gu1 ^ Gu2) );
	// zeroBlockH11
	zeroBlockH11.setup("zeroBlockH11", iRow, iCol);
	zeroBlockH11.addStatement( H.getSubMatrix(offset + iRow * NU, offset + (iRow + 1) * NU, offset + iCol * NU, offset + (iCol + 1) * NU) == zeros<double>(NU, NU) );
	// copyHTH
	copyHTH.setup("copyHTH", iRow, iCol);
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
	zeroBlockH10.addStatement( H101 == zeros<double>(NU, NX) );

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

	if (QN1.isGiven() == BT_TRUE)
	{
		// multQN1Gu
		multQN1Gu.setup("multQN1Gu", Gu1, Gu2);
		multQN1Gu.addStatement( Gu2 == QN1 * Gu1 );

		// multQN1Gx
		multQN1Gx.setup("multQN1Gx", Gx1, Gx2);
		multQN1Gx.addStatement( Gx2 == QN1 * Gx1 );
	}

	if (performsSingleShooting() == BT_FALSE)
	{
		// multQN1d
		multQN1d.setup("multQN1d", QN1, dp, dn);
		multQN1d.addStatement( dn == QN1 * dp );
	}

	if (performFullCondensing() == false)
	{
		// zeroBlockH00
		zeroBlockH00.setup( "zeroBlockH00" );
		zeroBlockH00.addStatement( H00 == zeros<double>(NX, NX) );

		// multCTQC
		multCTQC.setup("multCTQC", Gx1, Gx2);
		multCTQC.addStatement( H00 += (Gx1 ^ Gx2) );
	}

	// N2 condensing related

	//ExportFunction multBTW1, multGxTGu, macQEW2;

	// multBTW1, evGu.getAddress(row * NX), W1, row, col
	multBTW1.setup("multBTW1", Gu1, Gu2, iRow, iCol);
	multBTW1.addStatement(
			H.getSubMatrix(iRow * NU, (iRow + 1) * NU, iCol * NU, (iCol + 1) * NU) ==
					(Gu1 ^ Gu2)
//					(Gu2 ^ Gu1)
	);

	//
	// Define LM regularization terms
	//
	DMatrix mRegH00 = eye<double>( getNX() );
	DMatrix mRegH11 = eye<double>( getNU() );

	mRegH00 *= levenbergMarquardt;
	mRegH11 *= levenbergMarquardt;

	ExportVariable R11;
	if (R1.isGiven() == true)
		R11 = R1;
	else
		R11.setup("R11", NU, NU, REAL, ACADO_LOCAL);

	macBTW1_R1.setup("multBTW1_R1", R11, Gu1, Gu2, iRow);
	macBTW1_R1.addStatement(
			H.getSubMatrix(iRow * NU, (iRow + 1) * NU, iRow * NU, (iRow + 1) * NU) ==
					R11 + (Gu1 ^ Gu2)
			);
	macBTW1_R1.addStatement(
			H.getSubMatrix(iRow * NU, (iRow + 1) * NU, iRow * NU, (iRow + 1) * NU) += mRegH11
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

	expansionStep.setup("expansionStep", Gx1, Gu1, U1, w11, w12);
	expansionStep.addStatement( w12 += Gx1 * w11 );
	expansionStep.addStatement( w12 += Gu1 * U1 );

	//
	// Hessian factorization helper routines
	//

	ExportVariable D1("D", NU, NU, REAL, ACADO_LOCAL);
	ExportVariable L1("L", NU, NX, REAL, ACADO_LOCAL);
	ExportVariable H1("H", getNumQPvars(), getNumQPvars(), REAL, ACADO_LOCAL);

	ExportVariable T11("T11", NX, NX, REAL, ACADO_LOCAL);
	ExportVariable T22("T22", NU, NX, REAL, ACADO_LOCAL);
	ExportVariable T33("T33", NX, NX, REAL, ACADO_LOCAL);

//	ExportFunction mult_BT_T1_T2;
	mult_BT_T1_T2.setup("mult_BT_T1_T2", Gu1, T11, T22);
	mult_BT_T1_T2.addStatement( T22 == (Gu1 ^ T11) );

//	ExportFunction mul_T2_A_L;
	mul_T2_A_L.setup("mul_T2_A_L", T22, Gx1, L1);
	mul_T2_A_L.addStatement( L1 == T22 * Gx1 );

//	ExportFunction mac_R_T2_B_D;
	mac_R_T2_B_D.setup("mac_R_T2_B_D", R11, T22, Gu1, D1);
	mac_R_T2_B_D.addStatement( D1 == R11 + T22 * Gu1 );

//	ExportFunction move_D_H;
	move_D_U.setup("move_D_H", D1, H1, iRow);
	move_D_U.addStatement( H1.getSubMatrix(iRow * NU, (iRow + 1) * NU, iRow * NU, (iRow + 1) * NU) == D1 );

//	ExportFunction mult_L_E_H;
	mult_L_E_U.setup("mult_L_E_H", L1, Gu1, H1, iRow, iCol);
	mult_L_E_U.addStatement(
			H1.getSubMatrix(iRow * NU, (iRow + 1) * NU, iCol * NU, (iCol + 1) * NU) == L1 * Gu1
	);

//	ExportFunction updateQ; T1 = Q( blk ) + A^T * T1 * A - L^T * L
	// Q1, T3, evGx.getAddress(blk * NX), L, T1);
	updateQ.setup("updateQ", Q11, T33, Gx1, L1, T11);
	updateQ.addStatement( T33 == (Gx1 ^ T11) );
	updateQ.addStatement( T11 == Q11 + T33 * Gx1 );
	updateQ.addStatement( T11 -= (L1 ^ L1) );

//	ExportFunction mac_R_BT_F_D, mult_FT_A_L;
//	ExportFunction updateQ2;
//	ExportFunction mac_W1_T1_E_F;

	mac_R_BT_F_D.setup("mac_R_BT_F_D", R11, Gu1, Gu2, D1);
	mac_R_BT_F_D.addStatement( D1 == R11 + (Gu1 ^ Gu2) );

	mult_FT_A_L.setup("mult_FT_A_L", Gu1, Gx1, L1);
	mult_FT_A_L.addStatement( L1 == (Gu1 ^ Gx1) );

	updateQ2.setup("updateQ2", Q11, L1, T11);
	// TODO T11 == Q11 - (L1^L1) does not work properly
	//      Revisit this...
	updateQ2.addStatement( T11 == Q11 );
	updateQ2.addStatement( T11 -= (L1 ^ L1) );

	mac_W1_T1_E_F.setup("mac_W1_T1_E_F", Gu1, Gx1, Gu2, Gu3);
	mac_W1_T1_E_F.addStatement( Gu3 == Gu1 + Gx1 * Gu2 );

	move_GxT_T3.setup("move_GxT_T3", Gx1, Gx2);
	move_GxT_T3.addStatement( Gx2 == Gx1.getTranspose() );

	return SUCCESSFUL_RETURN;
}

returnValue ExportGaussNewtonCn2Factorization::setupEvaluation( )
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

returnValue ExportGaussNewtonCn2Factorization::setupQPInterface( )
{
	string folderName;
	get(CG_EXPORT_FOLDER_NAME, folderName);
    
	string moduleName;
	get(CG_MODULE_NAME, moduleName);
	
    int qpSolver;
	get(QP_SOLVER, qpSolver);
	
	std::string sourceFile, headerFile, solverDefine;
	ExportQpOasesInterface* qpInterface = 0;

	switch ( (QPSolverName)qpSolver )
	{
		case QP_QPOASES:
			sourceFile = folderName + "/" + moduleName + "_qpoases_interface.cpp";
			headerFile = folderName + "/" + moduleName + "_qpoases_interface.hpp";
			solverDefine = "QPOASES_HEADER";
			qpInterface = new ExportQpOasesInterface(headerFile, sourceFile, "");
			break;

		case QP_QPOASES3:
			sourceFile = folderName + "/" + moduleName + "_qpoases3_interface.c";
			headerFile = folderName + "/" + moduleName + "_qpoases3_interface.h";
			solverDefine = "QPOASES3_HEADER";
			qpInterface = new ExportQpOases3Interface(headerFile, sourceFile, "");
			break;

		default:
			return ACADOERRORTEXT(RET_INVALID_ARGUMENTS,
					"For condensed solution only qpOASES and qpOASES3 QP solver are supported");
	}

	int useSinglePrecision;
	get(USE_SINGLE_PRECISION, useSinglePrecision);

	int hotstartQP;
	get(HOTSTART_QP, hotstartQP);

	int covCalc;
	get(CG_COMPUTE_COVARIANCE_MATRIX, covCalc);

	int maxNumQPiterations;
	get(MAX_NUM_QP_ITERATIONS, maxNumQPiterations);

	int externalCholesky;
	get(CG_CONDENSED_HESSIAN_CHOLESKY, externalCholesky);

	//
	// Set up export of the source file
	//

	if ( qpInterface == 0 )
		return RET_UNKNOWN_BUG;

	qpInterface->configure(
			"",
			solverDefine,
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
			(CondensedHessianCholeskyDecomposition)externalCholesky == EXTERNAL,
			H.getFullName(),
			U.getFullName(),
			g.getFullName(),
			A.getFullName(),
			lb.getFullName(),
			ub.getFullName(),
			lbA.getFullName(),
			ubA.getFullName()
	);

	returnValue returnvalue = qpInterface->exportCode();

	if ( qpInterface != 0 )
		delete qpInterface;

	return returnvalue;
}

bool ExportGaussNewtonCn2Factorization::performFullCondensing() const
{
	int sparseQPsolution;
	get(SPARSE_QP_SOLUTION, sparseQPsolution);

	if ((SparseQPsolutionMethods)sparseQPsolution == CONDENSING)
		return false;

	return true;
}

CLOSE_NAMESPACE_ACADO
