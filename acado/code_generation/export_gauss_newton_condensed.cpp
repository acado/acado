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
 *    \file src/code_generation/export_gauss_newton_condensed.cpp
 *    \author Boris Houska, Hans Joachim Ferreau, Milan Vukov
 *    \date 2012 - 2015
 */

#include <acado/code_generation/export_gauss_newton_condensed.hpp>
#include <acado/code_generation/export_qpoases_interface.hpp>
#include <acado/code_generation/export_qpoases3_interface.hpp>
#include <acado/code_generation/export_module.hpp>

using namespace std;

BEGIN_NAMESPACE_ACADO

ExportGaussNewtonCondensed::ExportGaussNewtonCondensed(	UserInteraction* _userInteraction,
														const std::string& _commonHeaderName
														) : ExportNLPSolver( _userInteraction,_commonHeaderName )
{}

returnValue ExportGaussNewtonCondensed::setup( )
{
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

	LOG( LVL_DEBUG ) << "Solver: setup arrival cost update... " << endl;
	setupArrivalCostCalculation();
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

returnValue ExportGaussNewtonCondensed::getDataDeclarations(	ExportStatementBlock& declarations,
																ExportStruct dataStruct
																) const
{
	returnValue status;
	status = ExportNLPSolver::getDataDeclarations(declarations, dataStruct);
	if (status != SUCCESSFUL_RETURN)
		return status;

	declarations.addDeclaration(x0, dataStruct);
	declarations.addDeclaration(Dx0, dataStruct);

	declarations.addDeclaration(sigmaN, dataStruct);

	declarations.addDeclaration(T, dataStruct);
	declarations.addDeclaration(E, dataStruct);
	declarations.addDeclaration(QE, dataStruct);
	declarations.addDeclaration(QGx, dataStruct);
	declarations.addDeclaration(Qd, dataStruct);
	declarations.addDeclaration(QDy, dataStruct);
	declarations.addDeclaration(H10, dataStruct);

	declarations.addDeclaration(lbValues, dataStruct);
	declarations.addDeclaration(ubValues, dataStruct);
	declarations.addDeclaration(lbAValues, dataStruct);
	declarations.addDeclaration(ubAValues, dataStruct);

	if (performFullCondensing() == true)
		declarations.addDeclaration(A10, dataStruct);
	declarations.addDeclaration(A20, dataStruct);

	declarations.addDeclaration(pacA01Dx0, dataStruct);
	declarations.addDeclaration(pocA02Dx0, dataStruct);

	declarations.addDeclaration(CEN, dataStruct);
	declarations.addDeclaration(sigmaTmp, dataStruct);
	declarations.addDeclaration(sigma, dataStruct);

	declarations.addDeclaration(H, dataStruct);
	declarations.addDeclaration(R, dataStruct);
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

returnValue ExportGaussNewtonCondensed::getFunctionDeclarations(	ExportStatementBlock& declarations
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

	declarations.addDeclaration( updateArrivalCost );

	declarations.addDeclaration( evaluateStageCost );
	declarations.addDeclaration( evaluateTerminalCost );

	return SUCCESSFUL_RETURN;
}

returnValue ExportGaussNewtonCondensed::getCode(	ExportStatementBlock& code
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

	code.addFunction( multGxd );
	code.addFunction( moveGxT );
	code.addFunction( multGxGx );
	code.addFunction( multGxGu );
	code.addFunction( moveGuE );
	code.addFunction( setBlockH11 );
	code.addFunction( setBlockH11_R1 );
	code.addFunction( zeroBlockH11 );
	code.addFunction( copyHTH );
	code.addFunction( multQ1d );
	code.addFunction( multQN1d );
	code.addFunction( multRDy );
	code.addFunction( multQDy );
	code.addFunction( multEQDy );
	code.addFunction( multQETGx );
	code.addFunction( zeroBlockH10 );
	code.addFunction( multEDu );
	code.addFunction( multQ1Gx );
	code.addFunction( multQN1Gx );
	code.addFunction( multQ1Gu );
	code.addFunction( multQN1Gu );
	code.addFunction( zeroBlockH00 );
	code.addFunction( multCTQC );

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

	cholSolver.getCode( code );

	code.addFunction( condensePrep );
	code.addFunction( condenseFdb );
	code.addFunction( expand );
	code.addFunction( calculateCovariance );

	code.addFunction( preparation );
	code.addFunction( feedback );

	code.addFunction( initialize );
	code.addFunction( initializeNodes );
	code.addFunction( shiftStates );
	code.addFunction( shiftControls );
	code.addFunction( getKKT );
	code.addFunction( getObjective );

	int useArrivalCost;
	get(CG_USE_ARRIVAL_COST, useArrivalCost);
	if (useArrivalCost == YES)
	{
		acSolver.getCode( code );
		cholObjS.getCode( code );
		cholSAC.getCode( code );
		code.addFunction( updateArrivalCost );
	}

	return SUCCESSFUL_RETURN;
}


unsigned ExportGaussNewtonCondensed::getNumQPvars( ) const
{
	if (performFullCondensing() == true)
		return (N * NU);

	return (NX + N * NU);
}

unsigned ExportGaussNewtonCondensed::getNumStateBounds() const
{
	return xBoundsIdx.size();
}

//
// PROTECTED FUNCTIONS:
//

returnValue ExportGaussNewtonCondensed::setupObjectiveEvaluation( void )
{
	evaluateObjective.setup("evaluateObjective");

	int variableObjS;
	get(CG_USE_VARIABLE_WEIGHTING_MATRIX, variableObjS);

	if (S1.getGivenMatrix().isZero() == false)
		ACADOWARNINGTEXT(RET_INVALID_ARGUMENTS,
				"Mixed control-state terms in the objective function are not supported at the moment.");

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

	// Evaluate the objective function, last node.
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

returnValue ExportGaussNewtonCondensed::setupConstraintsEvaluation( void )
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
	unsigned numPathCon = N * dimPacH;
	unsigned numPointCon = dimPocH;

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

	for(unsigned node = 0; node < N; ++node)
		for(unsigned el = 0; el < NU; ++el)
		{
			lbBoundValues(offsetBounds + node * NU + el) = uBounds.getLowerBound(node, el);
			ubBoundValues(offsetBounds + node * NU + el) = uBounds.getUpperBound(node, el);
		}

	if (hardcodeConstraintValues == YES || !(isFinite( lbBoundValues ) || isFinite( ubBoundValues )))
	{
		lbValues.setup("lbValues", lbBoundValues, REAL, ACADO_VARIABLES);
		ubValues.setup("ubValues", ubBoundValues, REAL, ACADO_VARIABLES);
	}
	else if (isFinite( lbBoundValues ) || isFinite( ubBoundValues ))
	{
		lbValues.setup("lbValues", numBounds, 1, REAL, ACADO_VARIABLES);
		lbValues.setDoc( "Lower bounds values." );
		ubValues.setup("ubValues", numBounds, 1, REAL, ACADO_VARIABLES);
		ubValues.setDoc( "Upper bounds values." );

		initialize.addStatement( lbValues == lbBoundValues );
		initialize.addStatement( ubValues == ubBoundValues );
	}

	ExportFunction* boundSetFcn = hardcodeConstraintValues == YES ? &condensePrep : &condenseFdb;

	if (performFullCondensing() == true)
	{
		// Full condensing case
		boundSetFcn->addStatement( lb.getRows(0, getNumQPvars()) == lbValues - u.makeColVector() );
		boundSetFcn->addStatement( ub.getRows(0, getNumQPvars()) == ubValues - u.makeColVector() );
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
		DVector lbTmp, ubTmp;

		if ( numStateBounds )
		{
			DVector lbStateBoundValues( numStateBounds );
			DVector ubStateBoundValues( numStateBounds );
			for (unsigned i = 0; i < numStateBounds; ++i)
			{
				lbStateBoundValues( i ) = xBounds.getLowerBound( xBoundsIdx[ i ] / NX, xBoundsIdx[ i ] % NX );
				ubStateBoundValues( i ) = xBounds.getUpperBound( xBoundsIdx[ i ] / NX, xBoundsIdx[ i ] % NX );
			}

			lbTmp.append( lbStateBoundValues );
			ubTmp.append( ubStateBoundValues );
		}

		lbTmp.append( lbPathConValues );
		ubTmp.append( ubPathConValues );

		lbTmp.append( lbPointConValues );
		ubTmp.append( ubPointConValues );

		if (hardcodeConstraintValues == true)
		{
			lbAValues.setup("lbAValues", lbTmp, REAL, ACADO_VARIABLES);
			ubAValues.setup("ubAValues", ubTmp, REAL, ACADO_VARIABLES);
		}
		else
		{
			lbAValues.setup("lbAValues", sizeA, 1, REAL, ACADO_VARIABLES);
			lbAValues.setDoc( "Lower bounds values for affine constraints." );
			ubAValues.setup("ubAValues", sizeA, 1, REAL, ACADO_VARIABLES);
			ubAValues.setDoc( "Upper bounds values for affine constraints." );

			initialize.addStatement( lbAValues == lbTmp );
			initialize.addStatement( ubAValues == ubTmp );
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
				unsigned conIdx = xBoundsIdx[ row ] - NX;

				if (performFullCondensing() == false)
					condensePrep.addStatement( A.getSubMatrix(row, row + 1, 0, NX) == evGx.getRow( conIdx ) );

				unsigned blk = conIdx / NX + 1;
				for (unsigned col = 0; col < blk; ++col)
				{
					unsigned blkRow = (blk * (blk - 1) / 2 + col) * NX + conIdx % NX;

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

			ExportIndex row, col, conIdx, blk, blkRow;

			condensePrep.acquire( row ).acquire( col ).acquire( conIdx ).acquire( blk ).acquire( blkRow );

			ExportForLoop lRow(row, 0, numStateBounds);

			lRow << conIdx.getFullName() << " = " << evXBounds.getFullName() << "[ " << row.getFullName() << " ] - " << toString(NX) << ";\n";
			lRow.addStatement( blk == conIdx / NX + 1);

			if (performFullCondensing() == false)
				lRow.addStatement( A.getSubMatrix(row, row + 1, 0, NX) == evGx.getRow( conIdx ) );

			ExportForLoop lCol(col, 0, blk);

			lCol.addStatement( blkRow == (blk * (blk - 1) / 2 + col ) * NX + conIdx % NX );
			lCol.addStatement(
					A.getSubMatrix(row, row + 1, offset + col * NU, offset + (col + 1) * NU ) == E.getRow( blkRow ) );

			lRow.addStatement( lCol );
			condensePrep.addStatement( lRow );

			condensePrep.release( row ).release( col ).release( conIdx ).release( blk ).release( blkRow );
		}
		condensePrep.addLinebreak( );

		// shift constraint bounds by first interval
		for(unsigned run1 = 0; run1 < numStateBounds; ++run1)
		{
			unsigned row = xBoundsIdx[ run1 ];

			if (performFullCondensing() == true)
			{
				if (performsSingleShooting() == true)
				{
					condenseFdb.addStatement( tmp == x.makeRowVector().getCol( row ) + evGx.getRow(row - NX) * Dx0 );
				}
				else
				{
					condenseFdb.addStatement( tmp == x.makeRowVector().getCol( row ) + evGx.getRow(row - NX) * Dx0 );
					condenseFdb.addStatement( tmp += d.getRow(row - NX) );
				}
				condenseFdb.addStatement( lbA.getRow( run1 ) == lbAValues.getRow( run1 ) - tmp );
				condenseFdb.addStatement( ubA.getRow( run1 ) == ubAValues.getRow( run1 ) - tmp );
			}
			else
			{
				if (performsSingleShooting() == true)
					condenseFdb.addStatement( tmp == x.makeRowVector().getCol( row ) );
				else
					condenseFdb.addStatement( tmp == x.makeRowVector().getCol( row ) + d.getRow(row - NX) );

				condenseFdb.addStatement( lbA.getRow( run1 ) == lbAValues.getRow( run1 ) - tmp );
				condenseFdb.addStatement( ubA.getRow( run1 ) == ubAValues.getRow( run1 ) - tmp );
			}
		}
		condenseFdb.addLinebreak( );
	}

	////////////////////////////////////////////////////////////////////////////
	//
	// Setup the evaluation of the path constraints
	//
	////////////////////////////////////////////////////////////////////////////

	if (getNumComplexConstraints() == 0)
		return SUCCESSFUL_RETURN;

	if ( numPathCon )
	{
		unsigned rowOffset = numStateBounds;
		unsigned colOffset = performFullCondensing() == true ? 0 : NX;

		//
		// Setup evaluation
		//
		ExportIndex runPac;
		condensePrep.acquire( runPac );
		ExportForLoop loopPac(runPac, 0, N);

		loopPac.addStatement( conValueIn.getCols(0, NX) == x.getRow( runPac ) );
		loopPac.addStatement( conValueIn.getCols(NX, NX + NU) == u.getRow( runPac ) );
		loopPac.addStatement( conValueIn.getCols(NX + NU, NX + NU + NOD) == od.getRow( runPac ) );
		loopPac.addFunctionCall( evaluatePathConstraints.getName(), conValueIn, conValueOut );

		loopPac.addStatement( pacEvH.getRows( runPac * dimPacH, (runPac + 1) * dimPacH) ==
				conValueOut.getTranspose().getRows(0, dimPacH) );
		loopPac.addLinebreak( );

		unsigned derOffset = dimPacH;

		// Optionally store derivatives
		if ( pacEvHx.isGiven() == false )
		{
			loopPac.addStatement(
					pacEvHx.makeRowVector().
					getCols(runPac * dimPacH * NX, (runPac + 1) * dimPacH * NX)
					== conValueOut.getCols(derOffset, derOffset + dimPacH * NX )
			);

			derOffset = derOffset + dimPacH * NX;
		}
		if (pacEvHu.isGiven() == false )
		{
			loopPac.addStatement(
					pacEvHu.makeRowVector().
					getCols(runPac * dimPacH * NU, (runPac + 1) * dimPacH * NU)
					== conValueOut.getCols(derOffset, derOffset + dimPacH * NU )
			);
		}

		// Add loop to the function.
		condensePrep.addStatement( loopPac );
		condensePrep.addLinebreak( );

		// Define the multHxC multiplication routine
		ExportVariable tmpA01, tmpHx, tmpGx;

		if (pacEvHx.isGiven() == true)
			tmpHx = pacEvHx;
		else
			tmpHx.setup("Hx", dimPacH, NX, REAL, ACADO_LOCAL);

		tmpGx.setup("Gx", NX, NX, REAL, ACADO_LOCAL);

		if (performFullCondensing() == true)
		{
			tmpA01.setup("A01", dimPacH, NX, REAL, ACADO_LOCAL);

			multHxC.setup("multHxC", tmpHx, tmpGx, tmpA01);
			multHxC.addStatement( tmpA01 == tmpHx * tmpGx );

			A10.setup("A01", numPathCon, NX, REAL, ACADO_WORKSPACE);
		}
		else
		{
			tmpA01.setup("A01", dimPacH, getNumQPvars(), REAL, ACADO_LOCAL);
			multHxC.setup("multHxC", tmpHx, tmpGx, tmpA01);
			multHxC.addStatement( tmpA01.getSubMatrix(0, dimPacH, 0, NX) == tmpHx * tmpGx );

			A10 = A;
		}

		unsigned offsetA01 = (performFullCondensing() == true) ? 0 : rowOffset;

		// Define the block A_{10}(0: dimPacH, 0: NX) = H_{x}(0: dimPacH, 0: NX)
		if (pacEvHx.isGiven() == true)
		{
			condensePrep.addStatement(
					A10.getSubMatrix(offsetA01, offsetA01 + dimPacH, 0, NX)
							== pacEvHx);
		}
		else
		{
			condensePrep.addStatement(
					A10.getSubMatrix(offsetA01, offsetA01 + dimPacH, 0, NX)
							== pacEvHx.getSubMatrix(0, dimPacH, 0, NX));
		}
		condensePrep.addLinebreak();

		// Evaluate Hx * C
		for (unsigned row = 0; row < N - 1; ++row)
		{
			if (pacEvHx.isGiven() == true)
			{
				condensePrep.addFunctionCall(
						multHxC,
						pacEvHx,
						evGx.getAddress(row * NX, 0),
						A10.getAddress(offsetA01 + (row + 1) * dimPacH, 0) );
			}
			else
			{
				condensePrep.addFunctionCall(
						multHxC,
						pacEvHx.getAddress((row + 1) * dimPacH, 0),
						evGx.getAddress(row * NX, 0),
						A10.getAddress(offsetA01 + (row + 1) * dimPacH, 0) );
			}
		}
		condensePrep.addLinebreak();

		//
		// Evaluate Hx * E
		//
		ExportVariable tmpE;
		tmpE.setup("E", NX, NU, REAL, ACADO_LOCAL);
		ExportIndex iRow( "row" ), iCol( "col" );

		multHxE.setup("multHxE", tmpHx, tmpE, iRow, iCol);
		multHxE.addStatement(
				A.getSubMatrix(	rowOffset + iRow * dimPacH,
								rowOffset + (iRow + 1) * dimPacH,
								colOffset + iCol * NU,
								colOffset + (iCol + 1) * NU)
				== tmpHx * tmpE );

		if ( N <= 20 )
		{
			for (unsigned row = 0; row < N - 1; ++row)
			{
				for (unsigned col = 0; col <= row; ++col)
				{
					unsigned blk = (row + 1) * row / 2 + col;
					unsigned row2 = row + 1;

					if (pacEvHx.isGiven() == true)
						condensePrep.addFunctionCall(
								multHxE,
								pacEvHx,
								E.getAddress(blk * NX, 0),
								ExportIndex( row2 ),
								ExportIndex( col )
						);
					else
						condensePrep.addFunctionCall(
								multHxE,
								pacEvHx.getAddress((row + 1) * dimPacH, 0),
								E.getAddress(blk * NX, 0),
								ExportIndex( row2 ),
								ExportIndex( col )
						);
				}
			}
		}
		else
		{
			ExportIndex row, col, blk, row2;
			condensePrep.acquire( row );
			condensePrep.acquire( col );
			condensePrep.acquire( blk );
			condensePrep.acquire( row2 );

			ExportForLoop eLoopI(row, 0, N - 1);
			ExportForLoop eLoopJ(col, 0, row + 1);

			eLoopJ.addStatement( blk == (row + 1) * row / 2 + col );
			eLoopJ.addStatement( row2 == row + 1 );

			if (pacEvHx.isGiven() == true)
			{
				eLoopJ.addFunctionCall(
						multHxE,
						pacEvHx,
						E.getAddress(blk * NX, 0),
						row2,
						col
				);
			}
			else
			{
				eLoopJ.addFunctionCall(
						multHxE,
						pacEvHx.getAddress((row + 1) * dimPacH, 0),
						E.getAddress(blk * NX, 0),
						row2,
						col
				);
			}

			eLoopI.addStatement( eLoopJ );
			condensePrep.addStatement( eLoopI );

			condensePrep.release( row );
			condensePrep.release( col );
			condensePrep.release( blk );
			condensePrep.release( row2 );
		}
		condensePrep.addLinebreak();

		if (pacEvHu.getDim() > 0)
		{
			for (unsigned i = 0; i < N; ++i)
			{
				if (pacEvHu.isGiven() == true)
					initialize.addStatement(
							A.getSubMatrix(
									rowOffset + i * dimPacH,
									rowOffset + (i + 1) * dimPacH,
									colOffset + i * NU,
									colOffset + (i + 1) * NU)
							==	pacEvHu
					);
				else
					condensePrep.addStatement(
							A.getSubMatrix(
									rowOffset + i * dimPacH,
									rowOffset + (i + 1) * dimPacH,
									colOffset + i * NU,
									colOffset + (i + 1) * NU)
							==	pacEvHu.getSubMatrix(
									i * dimPacH,(i + 1) * dimPacH, 0, NU)
					);
			}
		}

		//
		// Set upper and lower bounds
		//
		condensePrep.addStatement(lbA.getRows(rowOffset, rowOffset + numPathCon) ==
				lbAValues.getRows(rowOffset, rowOffset + numPathCon) - pacEvH);
		condensePrep.addLinebreak();
		condensePrep.addStatement(ubA.getRows(rowOffset, rowOffset + numPathCon) ==
				ubAValues.getRows(rowOffset, rowOffset + numPathCon) - pacEvH);
		condensePrep.addLinebreak();

		if (performFullCondensing() == true)
		{
			pacA01Dx0.setup("pacA01Dx0", numPathCon, 1, REAL, ACADO_WORKSPACE);

			condenseFdb.addStatement( pacA01Dx0 == A10 * Dx0 );
			condenseFdb.addStatement(lbA.getRows(rowOffset, rowOffset + numPathCon) -= pacA01Dx0);
			condenseFdb.addLinebreak();
			condenseFdb.addStatement(ubA.getRows(rowOffset, rowOffset + numPathCon) -= pacA01Dx0);
			condenseFdb.addLinebreak();
		}

		// Evaluate Hx * d
		if ( performsSingleShooting() == false )
		{
			ExportVariable tmpd("tmpd", NX, 1, REAL, ACADO_LOCAL);
			ExportVariable tmpLb("lbA", dimPacH, 1, REAL, ACADO_LOCAL);
			ExportVariable tmpUb("ubA", dimPacH, 1, REAL, ACADO_LOCAL);

			macHxd.setup("macHxd", tmpHx, tmpd, tmpLb, tmpUb);
			macHxd.addStatement( pacEvHxd == tmpHx * tmpd );
			macHxd.addStatement( tmpLb -= pacEvHxd );
			macHxd.addStatement( tmpUb -= pacEvHxd );

			for (unsigned i = 0; i < N - 1; ++i)
			{
				if (pacEvHx.isGiven() == true)
				{
					condensePrep.addFunctionCall(
							macHxd,
							pacEvHx,
							d.getAddress(i * NX),
							lbA.getAddress(rowOffset + (i + 1) * dimPacH),
							ubA.getAddress(rowOffset + (i + 1) * dimPacH)
					);
				}
				else
				{
					condensePrep.addFunctionCall(
							macHxd,
							pacEvHx.getAddress((i + 1) * dimPacH, 0),
							d.getAddress(i * NX),
							lbA.getAddress(rowOffset + (i + 1) * dimPacH),
							ubA.getAddress(rowOffset + (i + 1) * dimPacH)
					);
				}
			}
			condensePrep.addLinebreak();
		}
	}

	////////////////////////////////////////////////////////////////////////////
	//
	// Setup the evaluation of the point constraints
	//
	////////////////////////////////////////////////////////////////////////////

	if ( numPointCon )
	{
		unsigned rowOffset = getNumStateBounds() + N * dimPacH;
		unsigned colOffset = performFullCondensing() == true ? 0 : NX;
		unsigned dim;

		if (performFullCondensing() == true)
			A20.setup("A02", dimPocH, NX, REAL, ACADO_WORKSPACE);

		//
		// Evaluate the point constraints
		//
		for (unsigned i = 0, intRowOffset = 0; i < N + 1; ++i)
		{
			if (evaluatePointConstraints[ i ] == 0)
				continue;

			condensePrep.addComment( string( "Evaluating constraint on node: #" ) + toString( i ) );
			condensePrep.addLinebreak();

			condensePrep.addStatement(conValueIn.getCols(0, getNX()) == x.getRow( i ) );
			if (i < N)
			{
				condensePrep.addStatement( conValueIn.getCols(NX, NX + NU) == u.getRow( i ) );
				condensePrep.addStatement( conValueIn.getCols(NX + NU, NX + NU + NOD) == od.getRow( i ) );
			}
			else
				condensePrep.addStatement( conValueIn.getCols(NX, NX + NOD) == od.getRow( i ) );

			condensePrep.addFunctionCall( evaluatePointConstraints[ i ]->getName(), conValueIn, conValueOut );
			condensePrep.addLinebreak();

			if (i < N)
				dim = evaluatePointConstraints[ i ]->getFunctionDim() / (1 + NX + NU);
			else
				dim = evaluatePointConstraints[ i ]->getFunctionDim() / (1 + NX);

			// Fill pocEvH, pocEvHx, pocEvHu
			condensePrep.addStatement(
					pocEvH.getRows(intRowOffset, intRowOffset + dim)
							== conValueOut.getTranspose().getRows(0, dim));
			condensePrep.addLinebreak();

			condensePrep.addStatement(
					pocEvHx.makeRowVector().getCols(intRowOffset * NX,
							(intRowOffset + dim) * NX)
							== conValueOut.getCols(dim, dim + dim * NX));
			condensePrep.addLinebreak();

			if (i < N)
			{
				condensePrep.addStatement(
						pocEvHu.makeRowVector().getCols(intRowOffset * NU,
								(intRowOffset + dim) * NU)
								== conValueOut.getCols(dim + dim * NX,
										dim + dim * NX + dim * NU));
				condensePrep.addLinebreak();
			}

			intRowOffset += dim;
		}

		//
		// Include point constraint data in the QP problem
		//
		for (unsigned row = 0, intRowOffset = 0; row < N + 1; ++row)
		{
			if (evaluatePointConstraints[ row ] == 0)
				continue;

			condensePrep.addComment(
					string( "Evaluating multiplications of constraint functions on node: #" ) + toString( row ) );
			condensePrep.addLinebreak();

			if (row < N)
				dim = evaluatePointConstraints[ row ]->getFunctionDim() / (1 + NX + NU);
			else
				dim = evaluatePointConstraints[ row ]->getFunctionDim() / (1 + NX);

			if (row == 0)
			{
				if (performFullCondensing() == true)
					condensePrep.addStatement(
							A20.getSubMatrix(0, dim, 0, NX)
									== pocEvHx.getSubMatrix(0, dim, 0, NX));
				else
					condensePrep.addStatement(
							A.getSubMatrix(rowOffset, rowOffset + dim, 0, NX)
									== pocEvHx.getSubMatrix(0, dim, 0, NX));
				condensePrep.addLinebreak();

				condensePrep.addStatement(
						A.getSubMatrix(rowOffset, rowOffset + dim, colOffset,
								colOffset + NU)
								== pocEvHu.getSubMatrix(0, dim, 0, NU));
				condensePrep.addLinebreak();
			}
			else
			{
				// Hx * C
				if (performFullCondensing() == true)
					condensePrep.addStatement(
							A20.getSubMatrix(intRowOffset, intRowOffset + dim, 0, NX) ==
									pocEvHx.getSubMatrix(intRowOffset, intRowOffset + dim, 0, NX) *
									evGx.getSubMatrix((row - 1) * NX, row * NX, 0, NX) );
				else
					condensePrep.addStatement(
							A.getSubMatrix(rowOffset + intRowOffset, rowOffset + intRowOffset + dim, 0, NX) ==
									pocEvHx.getSubMatrix(intRowOffset, intRowOffset + dim, 0, NX) *
									evGx.getSubMatrix((row - 1) * NX, row * NX, 0, NX) );
				condensePrep.addLinebreak();

				// Hx * E
				ExportIndex iCol, iBlk;
				condensePrep.acquire( iCol );
				condensePrep.acquire( iBlk );
				ExportForLoop eLoop(iCol, 0, row);

				// row - 1, col -> blk
				eLoop.addStatement( iBlk == row * (row - 1) / 2 + iCol );
				eLoop.addStatement(
						A.getSubMatrix(rowOffset + intRowOffset, rowOffset + intRowOffset + dim,
								colOffset + iCol * NU, colOffset + (iCol + 1) * NU) ==
										pocEvHx.getSubMatrix(intRowOffset, intRowOffset + dim, 0 , NX) *
										E.getSubMatrix(iBlk * NX, (iBlk + 1) * NX, 0, NU)
				);

				condensePrep.addStatement( eLoop );
				condensePrep.release( iCol );
				condensePrep.release( iBlk );

				// Hx * d, MS only
				if (performsSingleShooting() == false)
				{
					condensePrep.addStatement(
							pocEvHxd.getRows(intRowOffset, intRowOffset + dim) ==
									pocEvHx.getSubMatrix(intRowOffset, intRowOffset + dim, 0 , NX) *
									d.getRows((row - 1) * NX, row * NX) );
					condensePrep.addLinebreak();
				}

				// Add Hu block to the A21 block
				if (row < N)
				{
					condensePrep.addStatement(
							A.getSubMatrix(rowOffset + intRowOffset, rowOffset + intRowOffset + dim,
									colOffset + row * NU, colOffset + (row + 1) * NU) ==
									pocEvHu.getSubMatrix(intRowOffset, intRowOffset + dim, 0, NU));
					condensePrep.addLinebreak();
				}
			}

			intRowOffset += dim;
		}

		//
		// And now setup the lbA and ubA
		//
		condensePrep.addStatement( lbA.getRows(rowOffset, rowOffset + dimPocH) ==
				lbAValues.getRows(rowOffset, rowOffset + dimPocH) - pocEvH);
		condensePrep.addLinebreak();
		condensePrep.addStatement( ubA.getRows(rowOffset, rowOffset + dimPocH) ==
				ubAValues.getRows(rowOffset, rowOffset + dimPocH) - pocEvH);
		condensePrep.addLinebreak();

		if (performFullCondensing() == true)
		{
			pocA02Dx0.setup("pacA02Dx0", dimPocH, 1, REAL, ACADO_WORKSPACE);

			condenseFdb.addStatement( pocA02Dx0 == A20 * Dx0 );
			condenseFdb.addLinebreak();
			condenseFdb.addStatement(lbA.getRows(rowOffset, rowOffset + dimPocH) -= pocA02Dx0);
			condenseFdb.addLinebreak();
			condenseFdb.addStatement(ubA.getRows(rowOffset, rowOffset + dimPocH) -= pocA02Dx0);
			condenseFdb.addLinebreak();
		}

		if (performsSingleShooting() == false)
		{
			condensePrep.addStatement( lbA.getRows(rowOffset, rowOffset + dimPocH) -= pocEvHxd );
			condensePrep.addLinebreak();
			condensePrep.addStatement( ubA.getRows(rowOffset, rowOffset + dimPocH) -= pocEvHxd );
			condensePrep.addLinebreak();
		}
	}

	return SUCCESSFUL_RETURN;
}

returnValue ExportGaussNewtonCondensed::setupCondensing( void )
{
	//
	// Define LM regularization terms
	//
	DMatrix mRegH00 = eye<double>( getNX() );
	mRegH00 *= levenbergMarquardt;

	condensePrep.setup("condensePrep");
	condenseFdb.setup( "condenseFdb" );

	////////////////////////////////////////////////////////////////////////////
	//
	// Create block matrices C (alias evGx) and E
	//
	////////////////////////////////////////////////////////////////////////////

	LOG( LVL_DEBUG ) << "Setup condensing: create C & E matrices" << endl;

	// Special case, row = col = 0
	condensePrep.addFunctionCall(moveGuE, evGu.getAddress(0, 0), E.getAddress(0, 0) );

	if (N <= 20)
	{
		unsigned row, col, prev, curr;
		for (row = 1; row < N; ++row)
		{
			condensePrep.addFunctionCall(moveGxT, evGx.getAddress(row* NX, 0), T);

			if (performsSingleShooting() == false)
				condensePrep.addFunctionCall(multGxd, d.getAddress((row - 1) * NX), evGx.getAddress(row * NX), d.getAddress(row * NX));

			condensePrep.addFunctionCall(multGxGx, T, evGx.getAddress((row - 1) * NX, 0), evGx.getAddress(row * NX, 0));
			condensePrep.addLinebreak();

			for(col = 0; col < row; ++col)
			{
				prev = row * (row - 1) / 2 + col;
				curr = (row + 1) * row / 2 + col;

				condensePrep.addFunctionCall(multGxGu, T, E.getAddress(prev * NX, 0), E.getAddress(curr * NX, 0));
			}
			condensePrep.addLinebreak();

			curr = (row + 1) * row / 2 + col;
			condensePrep.addFunctionCall(moveGuE, evGu.getAddress(row * NX, 0), E.getAddress(curr * NX, 0) );

			condensePrep.addLinebreak();
		}
	}
	else
	{
		ExportIndex row, col, curr, prev;

		condensePrep.acquire( row );
		condensePrep.acquire( col );
		condensePrep.acquire( curr );
		condensePrep.acquire( prev );

		ExportForLoop eLoopI(row, 1, N);
		ExportForLoop eLoopJ(col, 0, row);

		eLoopI.addFunctionCall(moveGxT, evGx.getAddress(row* NX, 0), T);

		if (performsSingleShooting() == false)
			eLoopI.addFunctionCall(multGxd, d.getAddress((row - 1) * NX), evGx.getAddress(row * NX), d.getAddress(row * NX));

		eLoopI.addFunctionCall(multGxGx, T, evGx.getAddress((row - 1) * NX, 0), evGx.getAddress(row * NX, 0));

		eLoopJ.addStatement( prev == row * (row - 1) / 2 + col );
		eLoopJ.addStatement( curr == (row + 1) * row / 2 + col );
		eLoopJ.addFunctionCall( multGxGu, T, E.getAddress(prev * NX, 0), E.getAddress(curr * NX, 0) );

		eLoopI.addStatement( eLoopJ );
		eLoopI.addStatement( curr == (row + 1) * row / 2 + col );
		eLoopI.addFunctionCall(moveGuE, evGu.getAddress(row * NX, 0), E.getAddress(curr * NX, 0) );

		condensePrep.addStatement( eLoopI );
		condensePrep.addLinebreak();

		condensePrep.release( row );
		condensePrep.release( col );
		condensePrep.release( curr );
		condensePrep.release( prev );
	}

	//
	// Multiply Gx and E blocks with Q1.
	//
	LOG( LVL_DEBUG ) << "Setup condensing: multiply with Q1" << endl;

	if (performFullCondensing() == false)
	{
		for (unsigned i = 0; i < N - 1; ++i)
			if (Q1.isGiven() == true)
				condensePrep.addFunctionCall(multQ1Gx, evGx.getAddress(i * NX, 0), QGx.getAddress(i * NX, 0));
			else
				condensePrep.addFunctionCall(multGxGx, Q1.getAddress((i + 1) * NX, 0), evGx.getAddress(i * NX, 0), QGx.getAddress(i * NX, 0));

		if (QN1.isGiven() == true)
			condensePrep.addFunctionCall(multQN1Gx, evGx.getAddress((N - 1) * NX, 0), QGx.getAddress((N - 1) * NX, 0));
		else
			condensePrep.addFunctionCall(multGxGx, QN1, evGx.getAddress((N - 1) * NX, 0), QGx.getAddress((N - 1) * NX, 0));
		condensePrep.addLinebreak();
	}

	if (N <= 20)
	{
		for (unsigned i = 0; i < N; ++i)
			for (unsigned j = 0; j <= i; ++j)
			{
				unsigned k = (i + 1) * i / 2 + j;

				if (i < N - 1)
				{
					if (Q1.isGiven() == true)
						condensePrep.addFunctionCall(multQ1Gu, E.getAddress(k * NX, 0), QE.getAddress(k * NX, 0));
					else
						condensePrep.addFunctionCall(multGxGu, Q1.getAddress((i + 1) * NX, 0), E.getAddress(k * NX, 0), QE.getAddress(k * NX, 0));
				}
				else
				{
					if (QN1.isGiven() == true)
						condensePrep.addFunctionCall(multQN1Gu, E.getAddress(k * NX, 0), QE.getAddress(k * NX, 0));
					else
						condensePrep.addFunctionCall(multGxGu, QN1, E.getAddress(k * NX, 0), QE.getAddress(k * NX, 0));
				}
			}
		condensePrep.addLinebreak();
	}
	else
	{
		ExportIndex i, j, k;

		condensePrep.acquire( i );
		condensePrep.acquire( j );
		condensePrep.acquire( k );

		ExportForLoop eLoopI(i, 0, N - 1);
		ExportForLoop eLoopJ1(j, 0, i + 1);
		ExportForLoop eLoopJ2(j, 0, i + 1);

		eLoopJ1.addStatement( k == (i + 1) * i / 2 + j );

		if (Q1.isGiven() == true)
			eLoopJ1.addFunctionCall(multQ1Gu, E.getAddress(k * NX, 0), QE.getAddress(k * NX, 0));
		else
			eLoopJ1.addFunctionCall(multGxGu, Q1.getAddress((i + 1) * NX, 0), E.getAddress(k * NX, 0), QE.getAddress(k * NX, 0));

		eLoopI.addStatement( eLoopJ1 );

		eLoopJ2.addStatement( k == (i + 1) * i / 2 + j );

		if (QN1.isGiven() == true)
			eLoopJ2.addFunctionCall(multQN1Gu, E.getAddress(k * NX, 0), QE.getAddress(k * NX, 0));
		else
			eLoopJ2.addFunctionCall(multGxGu, QN1, E.getAddress(k * NX, 0), QE.getAddress(k * NX, 0));

		condensePrep.addStatement( eLoopI );
		condensePrep.addLinebreak();
		condensePrep.addStatement( eLoopJ2 );
		condensePrep.addLinebreak();

		condensePrep.release( i );
		condensePrep.release( j );
		condensePrep.release( k );
	}

	////////////////////////////////////////////////////////////////////////////
	//
	// Compute Hessian blocks H00, H10, H11
	//
	////////////////////////////////////////////////////////////////////////////

	LOG( LVL_DEBUG ) << "Setup condensing: create H00, H10 & H11" << endl;

	LOG( LVL_DEBUG ) << "---> Create H00" << endl;

	//
	// Create H00, in case of partial condensing only
	//
	if (performFullCondensing() == false)
	{
		condensePrep.addFunctionCall( zeroBlockH00 );

		for (unsigned i = 0; i < N; ++i)
			condensePrep.addFunctionCall(multCTQC, evGx.getAddress(i * NX, 0), QGx.getAddress(i * NX, 0));

		condensePrep.addStatement( H00 += mRegH00 );
		condensePrep.addLinebreak();

		// TODO: to be checked
		if (initialStateFixed() == false)
		{
			if (Q1.isGiven() == true)
			{
				condensePrep.addStatement( H00 += Q1 );
			}
			else
			{
				condensePrep.addStatement( H00 += Q1.getSubMatrix(0, NX, 0, NX) );
			}
		}

		if (SAC.getDim() > 0)
			condensePrep.addStatement( H00 += SAC );
	}

	LOG( LVL_DEBUG ) << "---> Create H10" << endl;

	//
	// Create H10 block
	//
	if (N <= 20)
	{
		for (unsigned i = 0; i < N; ++i)
		{
			condensePrep.addFunctionCall(zeroBlockH10, H10.getAddress(i * NU));

			for (unsigned j = i; j < N; ++j)
			{
				unsigned k = (j + 1) * j / 2 + i;

				condensePrep.addFunctionCall(multQETGx, QE.getAddress(k * NX), evGx.getAddress(j * NX), H10.getAddress(i * NU));
			}
		}
	}
	else
	{
		ExportIndex ii, jj, kk;
		condensePrep.acquire( ii );
		condensePrep.acquire( jj );
		condensePrep.acquire( kk );

		ExportForLoop eLoopI(ii, 0, N);
		ExportForLoop eLoopJ(jj, ii, N);

		eLoopI.addFunctionCall(zeroBlockH10, H10.getAddress(ii * NU));

		eLoopJ.addStatement( kk == (jj + 1) * jj / 2 + ii );
		eLoopJ.addFunctionCall( multQETGx, QE.getAddress(kk * NX), evGx.getAddress(jj * NX), H10.getAddress(ii * NU) );

		eLoopI.addStatement( eLoopJ );
		condensePrep.addStatement( eLoopI );

		condensePrep.release( ii );
		condensePrep.release( jj );
		condensePrep.release( kk );
	}
	condensePrep.addLinebreak();

	LOG( LVL_DEBUG ) << "---> Setup H01" << endl;

	//
	// Copy H01 = H10^T in case of partial condensing
	//
	if (performFullCondensing() == false)
	{
		condensePrep.addStatement( H.getSubMatrix(0, NX, NX, getNumQPvars()) == H10.getTranspose() );
		condensePrep.addLinebreak();
	}

	LOG( LVL_DEBUG ) << "---> Create H11" << endl;

	//
	// Create H11 block
	//
	if (N <= 20)
	{
		unsigned row, col;

		for (row = 0; row < N; ++row)
		{
			// Diagonal block
			if (R1.isGiven() == true)
				condensePrep.addFunctionCall(setBlockH11_R1, ExportIndex(row), ExportIndex(row), R1);
			else
				condensePrep.addFunctionCall(setBlockH11_R1, ExportIndex(row), ExportIndex(row), R1.getAddress(row * NU));

			col = row;
			for(unsigned blk = col; blk < N; ++blk)
			{
				unsigned indl = (blk + 1) * blk / 2 + row;
				unsigned indr = (blk + 1) * blk / 2 + col;

				condensePrep.addFunctionCall(
						setBlockH11, ExportIndex(row), ExportIndex(col),
						E.getAddress(indl * NX), QE.getAddress(indr * NX) );
			}
			condensePrep.addLinebreak();

			// The rest of the blocks in the row
			for(col = row + 1; col < N; ++col)
			{
				condensePrep.addFunctionCall(
						zeroBlockH11, ExportIndex(row), ExportIndex(col)
				);

				for(unsigned blk = col; blk < N; ++blk)
				{
					unsigned indl = (blk + 1) * blk / 2 + row;
					unsigned indr = (blk + 1) * blk / 2 + col;

					condensePrep.addFunctionCall(
							setBlockH11, ExportIndex(row), ExportIndex(col),
							E.getAddress(indl * NX), QE.getAddress(indr * NX) );
				}
				condensePrep.addLinebreak();
			}
		}
	}
	else
	{
		ExportIndex row, col, blk, indl, indr;
		condensePrep.acquire( row ).acquire( col ).acquire( blk ).acquire( indl ).acquire( indr );

		ExportForLoop eLoopI(row, 0, N);
		ExportForLoop eLoopK(blk, row, N);
		ExportForLoop eLoopJ(col, row + 1, N);
		ExportForLoop eLoopK2(blk, col, N);

		// The diagonal block
		// Diagonal block
		if (R1.isGiven() == true)
			eLoopI.addFunctionCall(setBlockH11_R1, row, row, R1);
		else
			eLoopI.addFunctionCall(setBlockH11_R1, row, row, R1.getAddress(row * NU));

		eLoopI.addStatement( col == row );
		eLoopK.addStatement( indl == (blk + 1) * blk / 2 + row );
		eLoopK.addStatement( indr == (blk + 1) * blk / 2 + col );
		eLoopK.addFunctionCall( setBlockH11, row, col, E.getAddress(indl * NX), QE.getAddress(indr * NX) );
		eLoopI.addStatement( eLoopK );

		// The rest of the blocks in the row
		eLoopJ.addFunctionCall( zeroBlockH11, row, col );

		eLoopK2.addStatement( indl == (blk + 1) * blk / 2 + row );
		eLoopK2.addStatement( indr == (blk + 1) * blk / 2 + col );
		eLoopK2.addFunctionCall( setBlockH11, row, col, E.getAddress(indl * NX), QE.getAddress(indr * NX) );
		eLoopJ.addStatement( eLoopK2 );

		eLoopI.addStatement( eLoopJ );
		condensePrep.addStatement( eLoopI );

		condensePrep.release( row ).release( col ).release( blk ).release( indl ).release( indr );
	}
	condensePrep.addLinebreak();

	LOG( LVL_DEBUG ) << "---> Copy H11 lower part" << endl;

	unsigned offset = (performFullCondensing() == true) ? 0 : NX;

	// Copy to H11 upper triangular part to lower triangular part
	if (N <= 20)
	{
		for (unsigned ii = 0; ii < N; ++ii)
			for(unsigned jj = 0; jj < ii; ++jj)
				condensePrep.addFunctionCall(
						copyHTH, ExportIndex( ii ), ExportIndex( jj )
				);
	}
	else
	{
		ExportIndex ii, jj;

		condensePrep.acquire( ii );
		condensePrep.acquire( jj );

		ExportForLoop eLoopI(ii, 0, N);
		ExportForLoop eLoopJ(jj, 0, ii);

		eLoopJ.addFunctionCall(copyHTH, ii, jj);
		eLoopI.addStatement( eLoopJ );
		condensePrep.addStatement( eLoopI );

		condensePrep.release( ii );
		condensePrep.release( jj );
	}
	condensePrep.addLinebreak();

	LOG( LVL_DEBUG ) << "---> Copy H10" << endl;

	//
	// Set block H10 in case of partial condensing
	//
	if (performFullCondensing() == false)
	{
		condensePrep.addStatement( H.getSubMatrix(NX, getNumQPvars(), 0, NX) == H10 );
		condensePrep.addLinebreak();
	}

	int externalCholesky;
	get(CG_CONDENSED_HESSIAN_CHOLESKY, externalCholesky);
	ASSERT((CondensedHessianCholeskyDecomposition)externalCholesky == INTERNAL_N3 ||
			(CondensedHessianCholeskyDecomposition)externalCholesky == EXTERNAL)

	if ((CondensedHessianCholeskyDecomposition)externalCholesky == INTERNAL_N3)
	{
		R.setup("R", getNumQPvars(), getNumQPvars(), REAL, ACADO_WORKSPACE);

		cholSolver.init(getNumQPvars(), NX, "condensing");
		cholSolver.setup();

		condensePrep.addStatement( R == H );
		condensePrep.addFunctionCall(cholSolver.getCholeskyFunction(), R);
	}

	////////////////////////////////////////////////////////////////////////////
	//
	// Compute gradient components g0 and g1
	//
	////////////////////////////////////////////////////////////////////////////

	//
	// Compute d and Qd = Q1 * d
	//
	LOG( LVL_DEBUG ) << "Setup condensing: compute Qd" << endl;

	if (performsSingleShooting() == false)
	{
		Qd.setup("Qd", N * NX, 1, REAL, ACADO_WORKSPACE);

		for(unsigned i = 0; i < N - 1; ++i)
		{
			if (Q1.isGiven() == true)
				condensePrep.addFunctionCall(
						multQ1d, Q1, d.getAddress(i * NX), Qd.getAddress(i * NX) );
			else
				condensePrep.addFunctionCall(
						multQ1d, Q1.getAddress((i + 1) * NX, 0), d.getAddress(i * NX), Qd.getAddress(i * NX) );
		}

		condensePrep.addFunctionCall(
				multQN1d, QN1, d.getAddress((N - 1) * NX), Qd.getAddress((N - 1) * NX) );

		condensePrep.addLinebreak();
	}

	LOG( LVL_DEBUG ) << "Setup condensing: create Dx0, Dy and DyN" << endl;

	if (initialStateFixed() == true)
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
		if (R2.isGiven() == true)
			condenseFdb.addFunctionCall(
					multRDy, R2,
					Dy.getAddress(run1 * NY, 0),
					g.getAddress(offset + run1 * NU, 0) );
		else
			condenseFdb.addFunctionCall(
					multRDy, R2.getAddress(run1 * NU, 0),
					Dy.getAddress(run1 * NY, 0),
					g.getAddress(offset + run1 * NU, 0) );
	}
	condenseFdb.addLinebreak();

	// Compute QDy
	// NOTE: This is just for the MHE case :: run1 starts from 0; in MPC :: from 1 ;)
	for(unsigned run1 = 0; run1 < N; run1++ )
	{
		if (Q2.isGiven() == true)
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

	LOG( LVL_DEBUG ) << "Setup condensing: QDy.getRows(NX, (N + 1) * NX) += Qd" << endl;

	if (performsSingleShooting() == false)
	{
		condenseFdb.addStatement( QDy.getRows(NX, (N + 1) * NX) += Qd );
		condenseFdb.addLinebreak();
	}

	if (performFullCondensing() == false)
	{
		// g0 == C^T * QDy(1: N + 1, :)
		condenseFdb.addStatement( g0 == (evGx ^ QDy.getRows(NX, (N + 1) * NX)) );
		condenseFdb.addLinebreak();

		if (initialStateFixed() == false)
			condenseFdb.addStatement( g0 += QDy.getRows(0, NX) );

		if (SAC.getDim() > 0)
		{
			// Include arrival cost
			condenseFdb.addStatement( DxAC == x.getRow( 0 ).getTranspose() - xAC );
			condenseFdb.addStatement( g0 +=  SAC * DxAC );
		}

		condenseFdb.addLinebreak();
	}

	if (N <= 20)
	{
		for (unsigned i = 0; i < N; ++i)
			for (unsigned j = i; j < N; ++j)
			{
				unsigned k = (j + 1) * j / 2 + i;

				condenseFdb.addFunctionCall(
						multEQDy, E.getAddress(k * NX, 0), QDy.getAddress((j + 1) * NX), g.getAddress(offset + i * NU) );
			}
	}
	else
	{
		ExportIndex i, j, k;
		condenseFdb.acquire( i );
		condenseFdb.acquire( j );
		condenseFdb.acquire( k );

		ExportForLoop eLoopI(i, 0, N);
		ExportForLoop eLoopJ(j, i, N);

		eLoopJ.addStatement( k == (j + 1) * j / 2 + i );
		eLoopJ.addFunctionCall(
				multEQDy, E.getAddress(k * NX, 0), QDy.getAddress((j + 1) * NX), g.getAddress(offset + i * NU) );

		eLoopI.addStatement( eLoopJ );
		condenseFdb.addStatement( eLoopI );

		condenseFdb.release( i );
		condenseFdb.release( j );
		condenseFdb.acquire( k );
	}
	condenseFdb.addLinebreak();

	if (performFullCondensing() == true)
	{
		condenseFdb.addStatement( g1 += H10 * Dx0 );
		condenseFdb.addLinebreak();
	}

	//
	// Add condensed linear terms from the objective to the gradient
	//
	if (performFullCondensing() == false && objSlx.getDim() > 0)
	{
		condensePrep.addStatement( g0 += objSlx );

		ExportVariable g00, C0, Slx0;
		g00.setup("g0", NX, 1, REAL, ACADO_LOCAL);
		C0.setup("C0", NX, NX, REAL, ACADO_LOCAL);

		if (objSlx.isGiven() == false)
			Slx0.setup("Slx0", NX, 1, REAL, ACADO_LOCAL);
		else
			Slx0 = objSlx;

		macCTSlx.setup("macCTSlx", C0, Slx0, g00);
		macCTSlx.addStatement( g00 += C0.getTranspose() * Slx0);

		for (unsigned i = 0; i < N; ++i)
			condensePrep.addFunctionCall(macCTSlx, evGx.getAddress(i * NX, 0), objSlx, g);
	}

	if (objSlx.getDim() > 0 || objSlu.getDim() > 0)
	{
		offset = performFullCondensing() == true ? 0 : NX;

		if (objSlu.getDim() > 0 && objSlx.getDim() == 0)
		{
			for (unsigned i = 0; i < N; ++i)
			{
				condensePrep.addStatement(
						g.getRows(offset + i * NU, offset + (i + 1) * NU) += objSlu
				);
			}
		}
		else
		{
			ExportVariable g10, E0, Slx0, Slu0;

			g10.setup("g1", NU, 1, REAL, ACADO_LOCAL);
			E0.setup("E0", NX, NU, REAL, ACADO_LOCAL);

			if (objSlx.isGiven() == false && objSlx.getDim() > 0)
				Slx0.setup("Slx0", NX, 1, REAL, ACADO_LOCAL);
			else
				Slx0 = objSlx;

			macETSlu.setup("macETSlu", E0, Slx0, g10);
			macETSlu.addStatement( g10 += E0.getTranspose() * Slx0 );

			if (N <= 20)
			{
				for (unsigned i = 0; i < N; ++i)
					for (unsigned j = i; j < N; ++j)
					{
						unsigned k = (j + 1) * j / 2 + i;

						condensePrep.addFunctionCall(macETSlu, QE.getAddress(k * NX), objSlx, g.getAddress(offset + i * NU));
					}
			}
			else
			{
				ExportIndex ii, jj, kk;
				condensePrep.acquire( ii );
				condensePrep.acquire( jj );
				condensePrep.acquire( kk );

				ExportForLoop iLoop(ii, 0, N);
				ExportForLoop jLoop(jj, ii, N);

				jLoop.addStatement( kk == (jj + 1) * jj / 2 + ii );
				jLoop.addFunctionCall(macETSlu, QE.getAddress(kk * NX), objSlx, g.getAddress(offset + ii * NU));

				iLoop.addStatement( jLoop );
				condensePrep.addStatement( iLoop );

				condensePrep.release( ii );
				condensePrep.release( jj );
				condensePrep.release( kk );
			}

			if (objSlu.getDim() > 0)
			{
				for (unsigned i = 0; i < N; ++i)
					condensePrep.addStatement(
							g.getRows(offset + i * NU, offset + (i + 1) * NU) += objSlu);
			}
		}
	}

	////////////////////////////////////////////////////////////////////////////
	//
	// Expansion routine
	//
	////////////////////////////////////////////////////////////////////////////

	LOG( LVL_DEBUG ) << "Setup condensing: create expand routine" << endl;

	expand.setup( "expand" );

	if (performFullCondensing() == true)
	{
		expand.addStatement( u.makeRowVector() += xVars.getTranspose() );
		expand.addLinebreak();
		expand.addStatement( x.getRow( 0 ) += Dx0.getTranspose() );
	}
	else
	{
		expand.addStatement( x.makeColVector().getRows(0, NX) += xVars.getRows(0, NX) );
		expand.addLinebreak();
		expand.addStatement( u.makeRowVector() += xVars.getTranspose().getCols(NX, getNumQPvars() ) );
	}
	expand.addLinebreak();

	// x += C * deltaX0

	if (performsSingleShooting() == false)
	{

		if (performFullCondensing() == true)
			expand.addStatement( x.makeColVector().getRows(NX, (N + 1) * NX) += d + evGx * Dx0);
		else
			expand.addStatement( x.makeColVector().getRows(NX, (N + 1) * NX) += d + evGx * xVars.getRows(0, NX) );
	}
	else
	{
		if (performFullCondensing() == true)
			expand.addStatement( x.makeColVector().getRows(NX, (N + 1) * NX) += evGx * Dx0);
		else
			expand.addStatement( x.makeColVector().getRows(NX, (N + 1) * NX) += evGx * xVars.getRows(0, NX) );
	}

	expand.addLinebreak();

	// x += E * deltaU

	offset = (performFullCondensing() == true) ? 0 : NX;

	if (N <= 20)
	{
		for (unsigned i = 0; i < N; ++i)
			for (unsigned j = 0; j <= i; ++j)
			{
				unsigned k = (i + 1) * i / 2 + j;

				expand.addFunctionCall(
						multEDu, E.getAddress(k * NX, 0), xVars.getAddress(offset + j * NU), x.getAddress(i + 1) );
			}
	}
	else
	{
		ExportIndex ii, jj, kk;
		expand.acquire( ii );
		expand.acquire( jj );
		expand.acquire( kk );

		ExportForLoop eLoopI(ii, 0, N);
		ExportForLoop eLoopJ(jj, 0, ii + 1);

		eLoopJ.addStatement( kk == (ii + 1) * ii / 2 + jj );
		eLoopJ.addFunctionCall(
				multEDu, E.getAddress(kk * NX, 0), xVars.getAddress(offset + jj * NU), x.getAddress(ii + 1) );

		eLoopI.addStatement( eLoopJ );
		expand.addStatement( eLoopI );

		expand.release( ii );
		expand.release( jj );
		expand.release( kk );
	}

	////////////////////////////////////////////////////////////////////////////
	//
	// Calculation of the covariance matrix for the last state estimate
	//
	////////////////////////////////////////////////////////////////////////////

	int covCalc;
	get(CG_COMPUTE_COVARIANCE_MATRIX, covCalc);

	if (covCalc == NO)
		return SUCCESSFUL_RETURN;

	if (performFullCondensing() == true)
		return ACADOERRORTEXT(RET_INVALID_ARGUMENTS, \
				"Calculation of covariance matrix works for partial condensing only atm.");

	LOG( LVL_DEBUG ) << "Setup condensing: covariance matrix calculation" << endl;

	CEN.setup("CEN", NX, getNumQPvars(), REAL, ACADO_WORKSPACE);
	sigmaTmp.setup("sigmaTemp", getNumQPvars(), NX, REAL, ACADO_WORKSPACE);
	sigma.setup("sigma", getNumQPvars(), getNumQPvars(), REAL, ACADO_WORKSPACE);

	sigmaN.setup("sigmaN", NX, NX, REAL, ACADO_VARIABLES);
	sigmaN.setDoc("Covariance matrix of the last state estimate.");

	calculateCovariance.setup("calculateCovariance");

	calculateCovariance.addStatement(
			CEN.getSubMatrix(0, NX, 0, NX) == evGx.getSubMatrix((N - 1) * NX, N * NX, 0, NX) );
	calculateCovariance.addLinebreak();

	ExportIndex cIndex("cIndex");
	ExportForLoop cLoop(cIndex, 0, N);
	calculateCovariance.addIndex( cIndex );

	unsigned blk = (N - 1 + 1) * (N - 1) / 2;
	cLoop.addStatement(
			CEN.getSubMatrix(0, NX, NX + cIndex * NU, NX + NU + cIndex * NU) ==
					E.getSubMatrix(blk * NX + cIndex * NX, blk * NX + NX + cIndex * NX, 0, NU)
	);
	calculateCovariance.addStatement( cLoop );
	calculateCovariance.addLinebreak();

	// XXX Optimize for long horizons.
	calculateCovariance.addStatement(
			sigmaTmp == sigma * CEN.getTranspose()
	);
	calculateCovariance.addLinebreak();

	calculateCovariance.addStatement(
			sigmaN == CEN * sigmaTmp
	);

	return SUCCESSFUL_RETURN;
}

returnValue ExportGaussNewtonCondensed::setupVariables( )
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

	T.setup("T", NX, NX, REAL, ACADO_WORKSPACE);
	E.setup("E", N * (N + 1) / 2 * NX, NU, REAL, ACADO_WORKSPACE);
	QE.setup("QE", N * (N + 1) / 2 * NX, NU, REAL, ACADO_WORKSPACE);

	if (performFullCondensing() == false)
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

returnValue ExportGaussNewtonCondensed::setupMultiplicationRoutines( )
{
	ExportIndex iCol( "iCol" );
	ExportIndex iRow( "iRow" );

	ExportVariable dp, dn, Gx1, Gx2, Gx3, Gu1, Gu2;
	ExportVariable R22, R11, Dy1, RDy1, Q22, QDy1, E1, U1, H101;
	dp.setup("dOld", NX, 1, REAL, ACADO_LOCAL);
	dn.setup("dNew", NX, 1, REAL, ACADO_LOCAL);
	Gx1.setup("Gx1", NX, NX, REAL, ACADO_LOCAL);
	Gx2.setup("Gx2", NX, NX, REAL, ACADO_LOCAL);
	Gx3.setup("Gx3", NX, NX, REAL, ACADO_LOCAL);
	Gu1.setup("Gu1", NX, NU, REAL, ACADO_LOCAL);
	Gu2.setup("Gu2", NX, NU, REAL, ACADO_LOCAL);
	R22.setup("R2", NU, NY, REAL, ACADO_LOCAL);
	R11.setup("R11", NU, NU, REAL, ACADO_LOCAL);
	Dy1.setup("Dy1", NY, 1, REAL, ACADO_LOCAL);
	RDy1.setup("RDy1", NU, 1, REAL, ACADO_LOCAL);
	Q22.setup("Q2", NX, NY, REAL, ACADO_LOCAL);
	QDy1.setup("QDy1", NX, 1, REAL, ACADO_LOCAL);
	E1.setup("E1", NX, NU, REAL, ACADO_LOCAL);
	U1.setup("U1", NU, 1, REAL, ACADO_LOCAL);
	H101.setup("H101", NU, NX, REAL, ACADO_LOCAL);

	if ( Q2.isGiven() )
		Q22 = Q2;
	if ( R2.isGiven() )
		R22 = R2;
	if ( R1.isGiven() )
		R11 = R1;

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
	// setBlockH11_R1
	DMatrix mRegH11 = eye<double>( getNU() );
	mRegH11 *= levenbergMarquardt;

	setBlockH11_R1.setup("setBlockH11_R1", iRow, iCol, R11);
	setBlockH11_R1.addStatement( H.getSubMatrix(offset + iRow * NU, offset + (iRow + 1) * NU, offset + iCol * NU, offset + (iCol + 1) * NU) == R11 + mRegH11 );
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

//	if (performsSingleShooting() == false)
//	{
		// multEDu
		multEDu.setup("multEDu", E1, U1, dn);
		multEDu.addStatement( dn += E1 * U1 );
//	}

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

	if (performFullCondensing() == BT_FALSE)
	{
		// zeroBlockH00
		zeroBlockH00.setup( "zeroBlockH00" );
		zeroBlockH00.addStatement( H00 == zeros<double>(NX, NX) );

		// multCTQC
		multCTQC.setup("multCTQC", Gx1, Gx2);
		multCTQC.addStatement( H00 += (Gx1 ^ Gx2) );
	}

	return SUCCESSFUL_RETURN;
}

returnValue ExportGaussNewtonCondensed::setupEvaluation( )
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

	feedback << tmp.getName() << " = " << solve.getName() << "( );\n";
	feedback.addLinebreak();

	feedback.addFunctionCall( expand );

	int covCalc;
	get(CG_COMPUTE_COVARIANCE_MATRIX, covCalc);
	if (covCalc)
		feedback.addFunctionCall( calculateCovariance );

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

		cLoop.addStatement( prd == yVars.getRow( getNumQPvars() + index ) );
		cLoop << "if (" << prd.getFullName() << " > " << toString(1.0 / INFTY) << ")\n";
		cLoop << kkt.getFullName() << " += fabs(" << lbA.get(index, 0) << " * " << prd.getFullName() << ");\n";
		cLoop << "else if (" << prd.getFullName() << " < " << toString(-1.0 / INFTY) << ")\n";
		cLoop << kkt.getFullName() << " += fabs(" << ubA.get(index, 0) << " * " << prd.getFullName() << ");\n";

		getKKT.addStatement( cLoop );
	}

	return SUCCESSFUL_RETURN;
}

returnValue ExportGaussNewtonCondensed::setupQPInterface( )
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
			sigma.getFullName(),
			hotstartQP,
			(CondensedHessianCholeskyDecomposition)externalCholesky == EXTERNAL,
			H.getFullName(),
			R.getFullName(),
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

bool ExportGaussNewtonCondensed::performFullCondensing() const
{
	int sparseQPsolution;
	get(SPARSE_QP_SOLUTION, sparseQPsolution);

	return (SparseQPsolutionMethods)sparseQPsolution == CONDENSING ? false : true;
}

CLOSE_NAMESPACE_ACADO
