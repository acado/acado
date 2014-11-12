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
 *    \file src/code_generation/export_gauss_newton_qpdunes.cpp
 *    \author Milan Vukov
 *    \date 2013
 */

#include <acado/code_generation/export_gauss_newton_qpdunes2.hpp>

#include <sstream>

BEGIN_NAMESPACE_ACADO

using namespace std;

ExportGaussNewtonQpDunes2::ExportGaussNewtonQpDunes2(	UserInteraction* _userInteraction,
													const std::string& _commonHeaderName
													) : ExportNLPSolver( _userInteraction,_commonHeaderName )
{}

returnValue ExportGaussNewtonQpDunes2::setup( )
{
	LOG( LVL_DEBUG ) << "Solver: setup initialization... " << endl;
	setupInitialization();

	//
	// Add QP initialization call to the initialization
	//
	ExportFunction initializeQpDunes( "initializeQpDunes" );
	initialize
		<< "ret = (int)initializeQpDunes();\n"
		<< "if ((return_t)ret != QPDUNES_OK) return ret;\n";

	cleanup.setup( "cleanupSolver" );
	ExportFunction cleanupQpDunes( "cleanupQpDunes" );
	cleanup.addFunctionCall( cleanupQpDunes );

	LOG( LVL_DEBUG ) << "done!" << endl;

	setupVariables();

	setupSimulation();

	setupObjectiveEvaluation();

	setupConstraintsEvaluation();

	setupEvaluation();

	setupAuxiliaryFunctions();

	return SUCCESSFUL_RETURN;
}

returnValue ExportGaussNewtonQpDunes2::getDataDeclarations(	ExportStatementBlock& declarations,
															ExportStruct dataStruct
															) const
{
	returnValue status;
	status = ExportNLPSolver::getDataDeclarations(declarations, dataStruct);
	if (status != SUCCESSFUL_RETURN)
		return status;

	declarations.addDeclaration(x0, dataStruct);

	declarations.addDeclaration(qpH, dataStruct);
	declarations.addDeclaration(qpg, dataStruct);
	declarations.addDeclaration(qpLb0, dataStruct);
	declarations.addDeclaration(qpUb0, dataStruct);
	declarations.addDeclaration(qpLb, dataStruct);
	declarations.addDeclaration(qpUb, dataStruct);
	declarations.addDeclaration(qpC, dataStruct);
	declarations.addDeclaration(qpc, dataStruct);

	declarations.addDeclaration(qpA, dataStruct);
	declarations.addDeclaration(qpLbA, dataStruct);
	declarations.addDeclaration(qpUbA, dataStruct);

	declarations.addDeclaration(qpPrimal, dataStruct);
	declarations.addDeclaration(qpLambda, dataStruct);
	declarations.addDeclaration(qpMu, dataStruct);

	return SUCCESSFUL_RETURN;
}

returnValue ExportGaussNewtonQpDunes2::getFunctionDeclarations(	ExportStatementBlock& declarations
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

	declarations.addDeclaration( cleanup );
	declarations.addDeclaration( shiftQpData );

	declarations.addDeclaration( evaluateStageCost );
	declarations.addDeclaration( evaluateTerminalCost );

	return SUCCESSFUL_RETURN;
}

returnValue ExportGaussNewtonQpDunes2::getCode(	ExportStatementBlock& code
														)
{
	setupQPInterface();
	code.addStatement( *qpInterface );

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
	code.addFunction( setStageH );
	code.addFunction( setStagef );
	code.addFunction( evaluateObjective );

	code.addFunction( evaluatePathConstraints );

	for (unsigned i = 0; i < evaluatePointConstraints.size(); ++i)
	{
		if (evaluatePointConstraints[ i ] == 0)
			continue;
		code.addFunction( *evaluatePointConstraints[ i ] );
	}

	code.addFunction( setStagePac );
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

	code.addFunction( cleanup );
	code.addFunction( shiftQpData );

	return SUCCESSFUL_RETURN;
}


unsigned ExportGaussNewtonQpDunes2::getNumQPvars( ) const
{
	return (N + 1) * NX + N * NU;
}

//
// PROTECTED FUNCTIONS:
//

returnValue ExportGaussNewtonQpDunes2::setupObjectiveEvaluation( void )
{
	evaluateObjective.setup("evaluateObjective");

	int variableObjS;
	get(CG_USE_VARIABLE_WEIGHTING_MATRIX, variableObjS);

//	if (S1.isGiven() == false or S1.getGivenMatrix().isZero() == false)
//		return ACADOFATALTEXT(RET_INVALID_ARGUMENTS,
//				"Mixed control-state terms in the objective function are not supported at the moment.");

	qpH.setup("qpH", N * (NX + NU) * (NX + NU) + NX * NX, 1, REAL, ACADO_WORKSPACE);
	qpg.setup("qpG", N * (NX + NU) + NX, 1, REAL, ACADO_WORKSPACE);

	//
	// LM regularization preparation
	//

	ExportVariable evLmX = zeros<double>(NX, NX);
	ExportVariable evLmU = zeros<double>(NU, NU);

	if  (levenbergMarquardt > 0.0)
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
		ExportVariable tmpQ1, tmpS1, tmpQ2;
		tmpQ1.setup("tmpQ1", NX, NX, REAL, ACADO_LOCAL);
		tmpS1.setup("tmpS1", NX, NU, REAL, ACADO_LOCAL);
		tmpQ2.setup("tmpQ2", NX, NY, REAL, ACADO_LOCAL);

		setObjQ1Q2.setup("setObjQ1S1Q2", tmpFx, tmpObjS, tmpQ1, tmpS1, tmpQ2);
		setObjQ1Q2.addStatement( tmpQ2 == (tmpFx ^ tmpObjS) );
		setObjQ1Q2.addStatement( tmpQ1 == tmpQ2 * tmpFx );
		setObjQ1Q2.addStatement( tmpS1 == tmpQ2 * tmpFu );

		if (tmpFx.isGiven() == true)
		{
			if (variableObjS == YES)
			{
				loopObjective.addFunctionCall(
						setObjQ1Q2,
						tmpFx, objS.getAddress(runObj * NY, 0),
						Q1.getAddress(runObj * NX, 0), S1.getAddress(runObj * NX, 0), Q2.getAddress(runObj * NX, 0)
				);
			}
			else
			{
				loopObjective.addFunctionCall(
						setObjQ1Q2,
						tmpFx, objS,
						Q1.getAddress(runObj * NX, 0), S1.getAddress(runObj * NX, 0), Q2.getAddress(runObj * NX, 0)
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
							Q1.getAddress(runObj * NX, 0), S1.getAddress(runObj * NX, 0), Q2.getAddress(runObj * NX, 0)
					);
			}
			else
			{
				loopObjective.addFunctionCall(
						setObjQ1Q2,
						objValueOut.getAddress(0, indexX), objS,
						Q1.getAddress(runObj * NX, 0), S1.getAddress(runObj * NX, 0), Q2.getAddress(runObj * NX, 0)
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

	//
	// Hessian setup
	//

	ExportVariable stageH;
	ExportIndex index( "index" );
	stageH.setup("stageH", NX + NU, NX + NU, REAL, ACADO_LOCAL);
	setStageH.setup("setStageH", stageH, index);

	if (Q1.isGiven() == false)
		setStageH.addStatement(
				stageH.getSubMatrix(0, NX, 0, NX) == Q1.getSubMatrix(index * NX, (index + 1) * NX, 0, NX) + evLmX
		);
	else
	{
		setStageH.addStatement( index == index );
		setStageH.addStatement(
				stageH.getSubMatrix(0, NX, 0, NX) == Q1 + evLmX
		);
	}
	setStageH.addLinebreak();

	if (S1.isGiven() == false) {
		setStageH.addStatement(
				stageH.getSubMatrix(0, NX, NX, NX+NU) == S1.getSubMatrix(index * NX, (index + 1) * NX, 0, NU)
		);
		setStageH.addStatement(
				stageH.getSubMatrix(NX, NX+NU, 0, NX) == S1.getSubMatrix(index * NX, (index + 1) * NX, 0, NU).getTranspose()
		);
	}
	else
	{
		setStageH.addStatement( index == index );
		setStageH.addStatement(
				stageH.getSubMatrix(0, NX, NX, NX+NU) == S1
		);
		setStageH.addStatement(
				stageH.getSubMatrix(NX, NX+NU, 0, NX) == S1.getTranspose()
		);
	}
	setStageH.addLinebreak();

	if (R1.isGiven() == false)
		setStageH.addStatement(
				stageH.getSubMatrix(NX, NX + NU, NX, NX + NU) == R1.getSubMatrix(index * NU, (index + 1) * NU, 0, NU) + evLmU
		);
	else
		setStageH.addStatement(
				stageH.getSubMatrix(NX, NX + NU, NX, NX + NU) == R1 + evLmU
		);

	if (Q1.isGiven() == true && R1.isGiven() == true)
	{
		for (unsigned i = 0; i < N; ++i)
		{
			initialize.addFunctionCall(
					setStageH, qpH.getAddress(i * (NX + NU) * (NX + NU)), ExportIndex( i ));
		}
		initialize.addLinebreak();
		initialize.addStatement(
				qpH.getTranspose().getCols(N * (NX + NU) * (NX + NU), N * (NX + NU) * (NX + NU) + NX * NX) == QN1.makeRowVector() + evLmX.makeRowVector()
		);
	}
	else
	{
		ACADOWARNINGTEXT(RET_NOT_YET_IMPLEMENTED, "Under dev, has to be tested!");

		for (unsigned i = 0; i < N; ++i)
		{
			evaluateObjective.addFunctionCall(
					setStageH, qpH.getAddress(i * (NX + NU) * (NX + NU)), ExportIndex( i ));
		}
		evaluateObjective.addLinebreak();
		evaluateObjective.addStatement(
				qpH.getTranspose().getCols(N * (NX + NU) * (NX + NU), N * (NX + NU) * (NX + NU) + NX * NX) == QN1.makeRowVector() + evLmX.makeRowVector()
		);
	}

	//
	// Gradient setup
	//

	ExportVariable stagef;
	stagef.setup("stagef", NX + NU, 1, REAL, ACADO_LOCAL);
	setStagef.setup("setStagef", stagef, index);

	if (Q2.isGiven() == false)
		setStagef.addStatement(
				stagef.getRows(0, NX) == Q2.getSubMatrix(index * NX, (index + 1) * NX, 0, NY) *
				Dy.getRows(index * NY, (index + 1) * NY)
		);
	else
	{
		setStagef.addStatement( index == index );
		setStagef.addStatement(
				stagef.getRows(0, NX) == Q2 *
				Dy.getRows(index * NY, (index + 1) * NY)
		);
	}
	setStagef.addLinebreak();

	if (R2.isGiven() == false)
		setStagef.addStatement(
				stagef.getRows(NX, NX + NU) == R2.getSubMatrix(index * NU, (index + 1) * NU, 0, NY) *
				Dy.getRows(index * NY, (index + 1) * NY)
		);
	else
	{
		setStagef.addStatement(
				stagef.getRows(NX, NX + NU) == R2 *
				Dy.getRows(index * NY, (index + 1) * NY)
		);
	}

	return SUCCESSFUL_RETURN;
}

returnValue ExportGaussNewtonQpDunes2::setupConstraintsEvaluation( void )
{
	////////////////////////////////////////////////////////////////////////////
	//
	// Setup evaluation of box constraints on states and controls
	//
	////////////////////////////////////////////////////////////////////////////

	// XXX This is for the NMPC case! TODO MHE!
	qpLb0.setup("qpLb0", 1, NX + NU, REAL, ACADO_WORKSPACE);
	qpUb0.setup("qpUb0", 1, NX + NU, REAL, ACADO_WORKSPACE);
	qpLb.setup("qpLb", 1, N * (NX + NU) + NX, REAL, ACADO_WORKSPACE);
	qpUb.setup("qpUb", 1, N * (NX + NU) + NX, REAL, ACADO_WORKSPACE);

	DVector lbTmp, ubTmp;
	DVector lbXValues, ubXValues;
	DVector lbUValues, ubUValues;

	DVector lbXInf( NX );
	lbXInf.setAll( -INFTY );

	DVector ubXInf( NX );
	ubXInf.setAll( INFTY );

	//
	// Stack state bounds
	//
	for (unsigned i = 0; i < N + 1; ++i)
	{
		lbTmp = xBounds.getLowerBounds( i );
		if ( !lbTmp.getDim() )
			lbXValues.append( lbXInf );
		else
			lbXValues.append( lbTmp );

		ubTmp = xBounds.getUpperBounds( i );
		if ( !ubTmp.getDim() )
			ubXValues.append( ubXInf );
		else
			ubXValues.append( ubTmp );
	}

	ExportVariable evLbXValues("lbXValues", lbXValues, STATIC_CONST_REAL, ACADO_LOCAL);
	ExportVariable evUbXValues("ubXValues", ubXValues, STATIC_CONST_REAL, ACADO_LOCAL);

	//
	// Stack control constraints
	//
	for (unsigned i = 0; i < N; ++i)
	{
		lbTmp = uBounds.getLowerBounds( i );
		lbUValues.append( lbTmp );

		ubTmp = uBounds.getUpperBounds( i );
		ubUValues.append( ubTmp );
	}

	ExportVariable evLbUValues("lbUValues", lbUValues, STATIC_CONST_REAL, ACADO_LOCAL);
	ExportVariable evUbUValues("ubUValues", ubUValues, STATIC_CONST_REAL, ACADO_LOCAL);

	//
	// Export evaluation of simple box constraints
	//
	evaluateConstraints.setup("evaluateConstraints");
	evaluateConstraints.addVariable( evLbXValues );
	evaluateConstraints.addVariable( evUbXValues );
	evaluateConstraints.addVariable( evLbUValues );
	evaluateConstraints.addVariable( evUbUValues );

	evaluateConstraints.addStatement(
			qpLb0.getCols(NX, NX + NU) == evLbUValues.getTranspose().getCols(0, NU) - u.getRow( 0 )
	);
	evaluateConstraints.addStatement(
			qpUb0.getCols(NX, NX + NU) == evUbUValues.getTranspose().getCols(0, NU) - u.getRow( 0 )
	);

	ExportIndex ind( "ind" );
	evaluateConstraints.addIndex( ind );
	ExportForLoop lbLoop(ind, 0, N);
	ExportForLoop ubLoop(ind, 0, N);

	lbLoop.addStatement(
			qpLb.getCols(ind * (NX + NU), ind * (NX + NU) + NX) ==
					evLbXValues.getTranspose().getCols(ind * NX, (ind + 1) * NX) - x.getRow( ind )
	);
	lbLoop.addStatement(
			qpLb.getCols(ind * (NX + NU) + NX, (ind + 1) * (NX + NU)) ==
					evLbUValues.getTranspose().getCols(ind * NU, (ind + 1) * NU) - u.getRow( ind )
	);

	ubLoop.addStatement(
			qpUb.getCols(ind * (NX + NU), ind * (NX + NU) + NX) ==
					evUbXValues.getTranspose().getCols(ind * NX, (ind + 1) * NX) - x.getRow( ind )
	);
	ubLoop.addStatement(
			qpUb.getCols(ind * (NX + NU) + NX, (ind + 1) * (NX + NU)) ==
					evUbUValues.getTranspose().getCols(ind * NU, (ind + 1) * NU) - u.getRow( ind )
	);

	evaluateConstraints.addStatement( lbLoop );
	evaluateConstraints.addStatement( ubLoop );
	evaluateConstraints.addLinebreak();

	evaluateConstraints.addStatement(
			qpLb.getCols(N * (NX + NU), N * (NX + NU) + NX) ==
					evLbXValues.getTranspose().getCols(N * NX, (N + 1) * NX) - x.getRow( N )
	);
	evaluateConstraints.addStatement(
			qpUb.getCols(N * (NX + NU), N * (NX + NU) + NX) ==
					evUbXValues.getTranspose().getCols(N * NX, (N + 1) * NX) - x.getRow( N )
	);
	evaluateConstraints.addLinebreak();

	////////////////////////////////////////////////////////////////////////////
	//
	// Evaluation of the equality constraints
	//  - system dynamics only
	//
	////////////////////////////////////////////////////////////////////////////

	//
	// Set QP C matrix
	//

	qpC.setup("qpC", N * NX, NX + NU, REAL, ACADO_WORKSPACE);
	qpc.setup("qpc", N * NX, 1, REAL, ACADO_WORKSPACE);

	ExportForLoop cLoop(ind, 0, N);

	cLoop.addStatement(
			qpC.getSubMatrix(ind * NX, (ind + 1) * NX, 0, NX) ==
					evGx.getSubMatrix(ind * NX, (ind + 1) * NX, 0, NX)
	);
	cLoop.addStatement(
			qpC.getSubMatrix(ind * NX, (ind + 1) * NX, NX, NX + NU) ==
					evGu.getSubMatrix(ind * NX, (ind + 1) * NX, 0, NU)
	);

	evaluateConstraints.addStatement( cLoop );
	evaluateConstraints.addLinebreak();

	// TODO MHE case; consult Janick

	//
	// Set QP c vector, ONLY MS is supported for now, so we do not perform checks atm
	//

	evaluateConstraints.addStatement(
			qpc == d
	);
	evaluateConstraints.addLinebreak();

	////////////////////////////////////////////////////////////////////////////
	//
	// Setup evaluation of path and point constraints
	//
	////////////////////////////////////////////////////////////////////////////

	if (getNumComplexConstraints() == 0)
		return SUCCESSFUL_RETURN;

	unsigned dimLbA  = N * dimPacH;
	unsigned dimConA = dimLbA * (NX + NU);

	qpConDim.resize(N + 1, 0);
	for (unsigned i = 0; i < N; ++i)
		qpConDim[ i ] += dimPacH;

	for (unsigned i = 0; i < N; ++i)
		if (evaluatePointConstraints[ i ])
		{
			unsigned dim = evaluatePointConstraints[ i ]->getFunctionDim() / (1 + NX + NU);

			dimLbA  += dim;
			dimConA += dim * (NX + NU);

			qpConDim[ i ] += dim;
		}

	if (evaluatePointConstraints[ N ])
	{
		unsigned dim = evaluatePointConstraints[ N ]->getFunctionDim() / (1 + NX);
		dimLbA  += dim;
		dimConA += dim * NX;

		qpConDim[ N ] += dim;
	}

	qpA.setup("qpA", dimConA, 1, REAL, ACADO_WORKSPACE);
	qpLbA.setup("qpLbA", dimLbA, 1, REAL, ACADO_WORKSPACE);
	qpUbA.setup("qpUbA", dimLbA, 1, REAL, ACADO_WORKSPACE);

	//
	// Setup constraint values for the whole horizon.
	//
	DVector lbAValues;
	DVector ubAValues;

	for (unsigned i = 0; i < N; ++i)
	{
		if ( dimPacH )
		{
			lbAValues.append( lbPathConValues.block(i * NX, 0, NX, 1) );
			ubAValues.append( ubPathConValues.block(i * NX, 0, NX, 1) );
		}
		lbAValues.append( pocLbStack[ i ] );
		ubAValues.append( pocUbStack[ i ] );
	}
	lbAValues.append( pocLbStack[ N ] );
	ubAValues.append( pocUbStack[ N ] );

	ExportVariable evLbAValues("lbAValues", lbAValues, STATIC_CONST_REAL, ACADO_LOCAL);
	ExportVariable evUbAValues("ubAValues", ubAValues, STATIC_CONST_REAL, ACADO_LOCAL);

	evaluateConstraints.addVariable( evLbAValues );
	evaluateConstraints.addVariable( evUbAValues );

	//
	// Evaluate path constraints
	//

	if ( dimPacH )
	{
		ExportIndex runPac;
		evaluateConstraints.acquire( runPac );
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
		if (pacEvHx.isGiven() == false)
		{
			loopPac.addStatement(
					pacEvHx.makeRowVector().getCols(runPac * dimPacH * NX, (runPac + 1) * dimPacH * NX) ==
							conValueOut.getCols(derOffset, derOffset + dimPacH * NX )
			);

			derOffset = derOffset + dimPacH * NX;
		}
		if (pacEvHu.isGiven() == false )
		{
			loopPac.addStatement(
					pacEvHu.makeRowVector().getCols(runPac * dimPacH * NU, (runPac + 1) * dimPacH * NU) ==
							conValueOut.getCols(derOffset, derOffset + dimPacH * NU )
			);
		}

		// Add loop to the function.
		evaluateConstraints.addStatement( loopPac );
		evaluateConstraints.release( runPac );
		evaluateConstraints.addLinebreak( );
	}

	//
	// Evaluate point constraints
	//

	for (unsigned i = 0, intRowOffset = 0, dim = 0; i < N + 1; ++i)
	{
		if (evaluatePointConstraints[ i ] == 0)
			continue;

		evaluateConstraints.addComment(
				string( "Evaluating constraint on node: #" ) + toString( i )
		);

		evaluateConstraints.addStatement(conValueIn.getCols(0, getNX()) == x.getRow( i ) );
		if (i < N)
		{
			evaluateConstraints.addStatement( conValueIn.getCols(NX, NX + NU) == u.getRow( i ) );
			evaluateConstraints.addStatement( conValueIn.getCols(NX + NU, NX + NU + NOD) == od.getRow( i ) );
		}
		else
			evaluateConstraints.addStatement( conValueIn.getCols(NX, NX + NOD) == od.getRow( i ) );

		evaluateConstraints.addFunctionCall(
				evaluatePointConstraints[ i ]->getName(), conValueIn, conValueOut );
		evaluateConstraints.addLinebreak();

		if (i < N)
			dim = evaluatePointConstraints[ i ]->getFunctionDim() / (1 + NX + NU);
		else
			dim = evaluatePointConstraints[ i ]->getFunctionDim() / (1 + NX);

		// Fill pocEvH, pocEvHx, pocEvHu
		evaluateConstraints.addStatement(
				pocEvH.getRows(intRowOffset, intRowOffset + dim) ==
						conValueOut.getTranspose().getRows(0, dim));
		evaluateConstraints.addLinebreak();

		evaluateConstraints.addStatement(
				pocEvHx.makeRowVector().getCols(intRowOffset * NX, (intRowOffset + dim) * NX)
						== conValueOut.getCols(dim, dim + dim * NX));
		evaluateConstraints.addLinebreak();

		if (i < N)
		{
			evaluateConstraints.addStatement(
					pocEvHu.makeRowVector().getCols(intRowOffset * NU, (intRowOffset + dim) * NU)
							== conValueOut.getCols(dim + dim * NX, dim + dim * NX + dim * NU));
			evaluateConstraints.addLinebreak();
		}

		intRowOffset += dim;
	}

	//
	// Copy data to QP solver structures
	//

	// TODO Make a function which is gonna set blocks of A, lbA, ubA for path constraints
	//      Here, an argument should be a offset from the beginning on the corresponding
	//      matrix or vector.

	ExportVariable tLbAValues, tUbAValues, tPacA;
	ExportIndex offsetPac("offset"), indPac( "ind" );

	tLbAValues.setup("lbAValues", dimPacH, 1, REAL, ACADO_LOCAL);
	tUbAValues.setup("ubAValues", dimPacH, 1, REAL, ACADO_LOCAL);
	tPacA.setup("tPacA", dimPacH, NX + NU, REAL, ACADO_LOCAL);

	setStagePac.setup("setStagePac", offsetPac, indPac, tPacA, tLbAValues, tUbAValues);

	if (pacEvHx.isGiven() == true)
		setStagePac << (tPacA.getSubMatrix(0, dimPacH, 0, NX) == pacEvHx);
	else
		setStagePac << (tPacA.getSubMatrix(0, dimPacH, 0, NX) ==
				pacEvHx.getSubMatrix(indPac * dimPacH, indPac * dimPacH + dimPacH, 0 , NX));

	if (pacEvHu.isGiven() == true)
		setStagePac << (tPacA.getSubMatrix(0, dimPacH, NX, NX + NU) == pacEvHu);
	else
		setStagePac << (tPacA.getSubMatrix(0, dimPacH, NX, NX + NU) ==
				pacEvHu.getSubMatrix(indPac * dimPacH, indPac * dimPacH + dimPacH, 0 , NU));

	setStagePac
		<< (qpLbA.getRows(offsetPac, offsetPac + dimPacH) == tLbAValues - pacEvH.getRows(indPac * dimPacH, indPac * dimPacH + dimPacH))
		<< (qpUbA.getRows(offsetPac, offsetPac + dimPacH) == tUbAValues - pacEvH.getRows(indPac * dimPacH, indPac * dimPacH + dimPacH));

	ExportVariable tPocA;
	tPocA.setup("tPocA", conValueOut.getDim(), NX + NU, REAL, ACADO_LOCAL);
	if ( dimPocH )
		evaluateConstraints.addVariable( tPocA );

	unsigned offsetEval = 0;
	unsigned offsetPoc = 0;
	for (unsigned i = 0; i < N; ++i)
	{
		if ( dimPacH )
		{
			evaluateConstraints.addFunctionCall(
					setStagePac,
					ExportIndex( offsetEval ), ExportIndex( i ),
					qpA.getAddress(offsetEval * (NX + NU)),
					evLbAValues.getAddress( offsetEval ), evUbAValues.getAddress( offsetEval )
			);

			offsetEval += dimPacH;
		}

		if ( evaluatePointConstraints[ i ] )
		{
			unsigned dim = evaluatePointConstraints[ i ]->getFunctionDim() / (1 + NX + NU);

			evaluateConstraints.addLinebreak();

			evaluateConstraints
				<< (tPocA.getSubMatrix(0, dim, 0, NX) == pocEvHx.getSubMatrix(offsetPoc, offsetPoc + dim, 0, NX))
				<< (tPocA.getSubMatrix(0, dim, NX, NX + NU) == pocEvHu.getSubMatrix(offsetPoc, offsetPoc + dim, 0, NU))
				<< (qpA.getRows(offsetEval * (NX + NU), (offsetEval + dim) * (NX + NU)) == tPocA.makeColVector().getRows(0, dim * (NX + NU)))
				<< (qpLbA.getRows(offsetEval, offsetEval + dim) ==
						evLbAValues.getRows(offsetEval, offsetEval + dim) - pocEvH.getRows(offsetPoc, offsetPoc + dim))
				<< (qpUbA.getRows(offsetEval, offsetEval + dim) ==
						evUbAValues.getRows(offsetEval, offsetEval + dim) - pocEvH.getRows(offsetPoc, offsetPoc + dim));

			offsetEval += dim;
			offsetPoc += dim;
		}
	}

	if ( evaluatePointConstraints[ N ] )
	{
		unsigned dim = evaluatePointConstraints[ N ]->getFunctionDim() / (1 + NX);

		evaluateConstraints
			<< (qpA.getRows(offsetEval * (NX + NU), offsetEval * (NX + NU) + dim * NX) ==
					pocEvHx.makeColVector().getRows(offsetPoc * (NX + NU), offsetPoc * (NX + NU) + dim * NX))
			<< (qpLbA.getRows(offsetEval, offsetEval + dim) ==
					evLbAValues.getRows(offsetEval, offsetEval + dim) - pocEvH.getRows(offsetPoc, offsetPoc + dim))
			<< (qpUbA.getRows(offsetEval, offsetEval + dim) ==
					evUbAValues.getRows(offsetEval, offsetEval + dim) - pocEvH.getRows(offsetPoc, offsetPoc + dim));
	}

	return SUCCESSFUL_RETURN;
}

returnValue ExportGaussNewtonQpDunes2::setupVariables( )
{
	if (initialStateFixed() == true)
	{
		x0.setup("x0",  NX, 1, REAL, ACADO_VARIABLES);
		x0.setDoc( (std::string)"Current state feedback vector." );
	}

	return SUCCESSFUL_RETURN;
}

returnValue ExportGaussNewtonQpDunes2::setupMultiplicationRoutines( )
{
	return SUCCESSFUL_RETURN;
}

returnValue ExportGaussNewtonQpDunes2::setupEvaluation( )
{
	stringstream ss;

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

	// XXX This is the MPC case!
	feedback.addStatement(
			qpLb0.getTranspose().getRows(0, NX) == x0 - x.getRow( 0 ).getTranspose()
	);
	feedback.addStatement(
			qpUb0.getCols(0, NX) == qpLb0.getCols(0, NX)
	);
	feedback.addLinebreak();

	//
	// Calculate objective residuals
	//
	feedback.addStatement( Dy -= y );
	feedback.addLinebreak();
	feedback.addStatement( DyN -= yN );
	feedback.addLinebreak();

	for (unsigned i = 0; i < N; ++i)
		feedback.addFunctionCall(setStagef, qpg.getAddress(i * (NX + NU)), ExportIndex( i ));
	feedback.addStatement( qpg.getRows(N * (NX + NU), N * (NX + NU) + NX) == QN2 * DyN );
	feedback.addLinebreak();

	feedback << returnValueFeedbackPhase.getFullName() << " = solveQpDunes();\n";

	//
	// Here we have to add the differences....
	//

	qpPrimal.setup("qpPrimal", N * (NX + NU) + NX, 1, REAL, ACADO_WORKSPACE);
	qpLambda.setup("qpLambda", N * NX, 1, REAL, ACADO_WORKSPACE);

	// XXX Janick, how to interpret those guys?
	qpMu.setup("qpMu", 2 * N * (NX + NU) + 2 * NX, 1, REAL, ACADO_WORKSPACE);

	// NOTE: Again, we are performing MS only

	ExportVariable stageOut("stageOut", 1, NX + NU, REAL, ACADO_LOCAL);
	ExportIndex index( "index" );
	acc.setup("accumulate", stageOut, index);

	acc	<< (x.getRow( index ) += stageOut.getCols(0, NX))
		<< (u.getRow( index ) += stageOut.getCols(NX, NX + NU));

	for (unsigned i = 0; i < N; ++i)
		feedback.addFunctionCall(acc, qpPrimal.getAddress(i * (NX + NU)), ExportIndex( i ));
	feedback.addLinebreak();
	feedback.addStatement(
			x.getRow( N ) += qpPrimal.getTranspose().getCols(N * (NX + NU), N * (NX + NU) + NX)
	);
	feedback.addLinebreak();

	////////////////////////////////////////////////////////////////////////////
	//
	// Shifting of QP data
	//
	////////////////////////////////////////////////////////////////////////////

	shiftQpData.setup( "shiftQpData" );
	ss.str( string() );
	ss	<< "qpDUNES_shiftLambda( &qpData );" << endl
		<< "qpDUNES_shiftIntervals( &qpData );" << endl;
	shiftQpData.addStatement( ss.str().c_str() );

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

returnValue ExportGaussNewtonQpDunes2::setupQPInterface( )
{
	//
	// Configure and export QP interface
	//

	qpInterface = std::tr1::shared_ptr< ExportQpDunesInterface >(new ExportQpDunesInterface("", commonHeaderName));

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

	qpInterface->configure(
			maxNumQPiterations,
			printLevel,
			qpH.getFullName(),
			qpg.getFullName(),
			" 0 ",
			qpC.getFullName(),
			qpc.getFullName(),
			qpA.getFullName(),
			qpLb0.getFullName(),
			qpUb0.getFullName(),
			qpLb.getFullName(),
			qpUb.getFullName(),
			qpLbA.getFullName(),
			qpUbA.getFullName(),
			qpPrimal.getFullName(),
			qpLambda.getFullName(),
			qpMu.getFullName(),
			qpConDim,
			"1",
			"1",
			"1"
	);

	return SUCCESSFUL_RETURN;
}

CLOSE_NAMESPACE_ACADO
