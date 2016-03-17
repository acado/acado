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
 *    \file src/code_generation/export_gauss_newton_forces.cpp
 *    \author Milan Vukov
 *    \date 2012 - 2014
 */

#include <acado/code_generation/export_gauss_newton_forces.hpp>
#include <acado/code_generation/export_module.hpp>

#include <acado/code_generation/export_forces_interface.hpp>
#include <acado/code_generation/export_forces_generator.hpp>

#include <acado/code_generation/templates/templates.hpp>

BEGIN_NAMESPACE_ACADO

using namespace std;

ExportGaussNewtonForces::ExportGaussNewtonForces(	UserInteraction* _userInteraction,
													const std::string& _commonHeaderName
													) : ExportNLPSolver( _userInteraction,_commonHeaderName )
{
	qpObjPrefix = "acadoForces";
	qpModuleName = "forces";
	diagH = diagHN = false;

	numLB = 0;
	numUB = 0;
}

returnValue ExportGaussNewtonForces::setup( )
{
	LOG( LVL_DEBUG ) << "Solver: setup initialization... " << endl;
	setupInitialization();
	// Add QP initialization call to the initialization
	ExportFunction initializeForces( "initializeForces" );
	initializeForces.setName( "initializeForces" );
	initialize.addFunctionCall( initializeForces );

	LOG( LVL_DEBUG ) << "done!" << endl;

	setupVariables();

	setupSimulation();

	setupObjectiveEvaluation();

	setupConstraintsEvaluation();

	setupEvaluation();

	setupAuxiliaryFunctions();

	return SUCCESSFUL_RETURN;
}

returnValue ExportGaussNewtonForces::getDataDeclarations(	ExportStatementBlock& declarations,
															ExportStruct dataStruct
															) const
{
	returnValue status;
	status = ExportNLPSolver::getDataDeclarations(declarations, dataStruct);
	if (status != SUCCESSFUL_RETURN)
		return status;

	declarations.addDeclaration(x0, dataStruct);

	declarations.addDeclaration(lbValues, dataStruct);
	declarations.addDeclaration(ubValues, dataStruct);

	return SUCCESSFUL_RETURN;
}

returnValue ExportGaussNewtonForces::getFunctionDeclarations(	ExportStatementBlock& declarations
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

returnValue ExportGaussNewtonForces::getCode(	ExportStatementBlock& code
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
	code.addFunction( setObjS1 );
	code.addFunction( setObjQN1QN2 );
	code.addFunction( setStageH );
	code.addFunction( setStagef );
	code.addFunction( evaluateObjective );

	code.addFunction( conSetGxGu );
	code.addFunction( conSetd );
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


unsigned ExportGaussNewtonForces::getNumQPvars( ) const
{
	return (N + 1) * NX + N * NU;
}

unsigned ExportGaussNewtonForces::getNumLowerBounds( ) const
{
	return numLB;
}

unsigned ExportGaussNewtonForces::getNumUpperBounds( ) const
{
	return numUB;
}

//
// PROTECTED FUNCTIONS:
//

returnValue ExportGaussNewtonForces::setupObjectiveEvaluation( void )
{
	evaluateObjective.setup("evaluateObjective");

	int variableObjS;
	get(CG_USE_VARIABLE_WEIGHTING_MATRIX, variableObjS);
	int forceDiagHessian;
	get(CG_FORCE_DIAGONAL_HESSIAN, forceDiagHessian);

	diagH = false;
	diagHN = false;
	unsigned dimHRows = NX + NU;
	unsigned dimHCols = NX + NU;
	unsigned dimHNRows = NX;
	unsigned dimHNCols = NX;
	if (objS.isGiven() == true || forceDiagHessian == true)
		if (objS.getGivenMatrix().isDiagonal())
		{
			diagH = true;
			dimHCols = 1;
		}

	if (objSEndTerm.isGiven() == true || forceDiagHessian == true)
		if (objSEndTerm.getGivenMatrix().isDiagonal() == true)
		{
			diagHN = true;
			dimHNCols = 1;
		}

	objHessians.resize(N + 1);
	for (unsigned i = 0; i < N; ++i)
	{
		objHessians[ i ].setup(string("H") + toString(i + 1), dimHRows, dimHCols, REAL, FORCES_PARAMS, false, qpObjPrefix);
	}
	objHessians[ N ].setup(string("H") + toString(N + 1), dimHNRows, dimHNCols, REAL, FORCES_PARAMS, false, qpObjPrefix);

	objGradients.resize(N + 1);
	for (unsigned i = 0; i < N; ++i)
	{
		objGradients[ i ].setup(string("f") + toString(i + 1), NX + NU, 1, REAL, FORCES_PARAMS, false, qpObjPrefix);
	}
	objGradients[ N ].setup(string("f") + toString(N + 1), NX, 1, REAL, FORCES_PARAMS, false, qpObjPrefix);

	//
	// LM regularization preparation
	//

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

	indexX = getNY();
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
	stageH.setup("stageH", dimHRows, dimHCols, REAL, ACADO_LOCAL);
	setStageH.setup("setStageH", stageH, index);

	if (Q1.isGiven() == false)
	{
		if (diagH == false)
			setStageH.addStatement(
					stageH.getSubMatrix(0, NX, 0, NX) == Q1.getSubMatrix(index * NX, (index + 1) * NX, 0, NX) + evLmX
			);
		else
			for (unsigned el = 0; el < NX; ++el)
				setStageH.addStatement(
						stageH.getElement(el, 0) == Q1.getElement(index * NX + el, el)
				);
	}
	else
	{
		setStageH << index.getFullName() << " = " << index.getFullName() << ";\n";
		if (diagH == false)
		{
			setStageH.addStatement(
					stageH.getSubMatrix(0, NX, 0, NX) == Q1 + evLmX
			);
		}
		else
		{
			setStageH.addStatement(
					stageH.getRows(0, NX) == Q1.getGivenMatrix().getDiag() + evLmX.getGivenMatrix().getDiag()
			);
		}
	}
	setStageH.addLinebreak();

	if (R1.isGiven() == false)
	{
		if (diagH == false)
			setStageH.addStatement(
					stageH.getSubMatrix(NX, NX + NU, NX, NX + NU) == R1.getSubMatrix(index * NU, (index + 1) * NU, 0, NU) + evLmU
			);
		else
			for (unsigned el = 0; el < NU; ++el)
				setStageH.addStatement(
						stageH.getElement(NX + el, 0) == R1.getElement(index * NU + el, el)
				);
	}
	else
	{
		if (diagH == false)
		{
			setStageH.addStatement(
					stageH.getSubMatrix(NX, NX + NU, NX, NX + NU) == R1 + evLmU
			);
		}
		else
		{
			setStageH.addStatement(
					stageH.getRows(NX, NX + NU) == R1.getGivenMatrix().getDiag() + evLmU.getGivenMatrix().getDiag()
			);
		}
	}
	setStageH.addLinebreak();

	if (diagH == false) {
		if (S1.isGiven() == false)
		{
			setStageH.addStatement(
					stageH.getSubMatrix(0, NX, NX, NX + NU) == S1.getSubMatrix(index * NX, (index + 1) * NX, 0, NU)
			);
			setStageH.addStatement(
					stageH.getSubMatrix(NX, NX + NU, 0, NX) == S1.getSubMatrix(index * NX, (index + 1) * NX, 0, NU).getTranspose()
			);
		}
		else if(S1.getGivenMatrix().isZero() == false)
		{
			setStageH.addStatement(
					stageH.getSubMatrix(0, NX, NX, NX + NU) == S1
			);
			setStageH.addStatement(
					stageH.getSubMatrix(NX, NX + NU, 0, NX) == S1.getTranspose()
			);
		}
	}

	if (Q1.isGiven() == true && R1.isGiven() == true && S1.isGiven() == true)
	{
		initialize <<
				setStageH.getName() << "( " << objHessians[ 0 ].getFullName() << ", " << "0" << " );\n";
		initialize.addLinebreak();
		if (diagHN == false)
		{
			initialize.addStatement(
					objHessians[ N ] == QN1 + evLmX
			);
		}
		else
		{
			initialize.addStatement(
					objHessians[ N ] == QN1.getGivenMatrix().getDiag() + evLmX.getGivenMatrix().getDiag()
			);
		}
	}
	else
	{
		for (unsigned i = 0; i < N; ++i)
			evaluateObjective.addFunctionCall(setStageH, objHessians[ i ], ExportIndex(i));
		evaluateObjective.addLinebreak();
		if (diagHN == false)
			evaluateObjective.addStatement(
					objHessians[ N ] == QN1 + evLmX
			);
		else
			for (unsigned el = 0; el < NX; ++el)
				evaluateObjective.addStatement(
						objHessians[ N ].getElement(el, 0) == QN1.getElement(el, el) + evLmX.getElement(el, el)
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
		setStagef <<  index.getFullName() << " = " << index.getFullName() << ";\n";
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

returnValue ExportGaussNewtonForces::setupConstraintsEvaluation( void )
{
	////////////////////////////////////////////////////////////////////////////
	//
	// Setup evaluation of box constraints on states and controls
	//
	////////////////////////////////////////////////////////////////////////////

	conLB.clear();
	conLB.resize(N + 1);

	conUB.clear();
	conUB.resize(N + 1);

	conLBIndices.clear();
	conLBIndices.resize(N + 1);

	conUBIndices.clear();
	conUBIndices.resize(N + 1);

	conABDimensions.clear();
	conABDimensions.resize(N + 1);

	conLBValues.clear();
	conLBValues.resize(N + 1);

	conUBValues.clear();
	conUBValues.resize(N + 1);

	DVector lbTmp, ubTmp;

	//
	// Stack state constraints
	//
	for (unsigned i = 0; i < xBounds.getNumPoints(); ++i)
	{
		lbTmp = xBounds.getLowerBounds( i );
		ubTmp = xBounds.getUpperBounds( i );

		if (isFinite( lbTmp ) == false && isFinite( ubTmp ) == false)
			continue;

		for (unsigned j = 0; j < lbTmp.getDim(); ++j)
		{
			if (acadoIsFinite( lbTmp( j ) ) == true)
			{
				conLBIndices[ i ].push_back( j );
				conLBValues[ i ].push_back( lbTmp( j ) );
				numLB++;
			}

			if (acadoIsFinite( ubTmp( j ) ) == true)
			{
				conUBIndices[ i ].push_back( j );
				conUBValues[ i ].push_back( ubTmp( j ) );
				numUB++;
			}
		}
	}

	//
	// Stack control constraints
	//
	for (unsigned i = 0; i < uBounds.getNumPoints() && i < N; ++i)
	{
		lbTmp = uBounds.getLowerBounds( i );
		ubTmp = uBounds.getUpperBounds( i );

		if (isFinite( lbTmp ) == false && isFinite( ubTmp ) == false)
			continue;

		for (unsigned j = 0; j < lbTmp.getDim(); ++j)
		{
			if (acadoIsFinite( lbTmp( j ) ) == true)
			{
				conLBIndices[ i ].push_back(NX + j);
				conLBValues[ i ].push_back( lbTmp( j ) );
				numLB++;
			}

			if (acadoIsFinite( ubTmp( j ) ) == true)
			{
				conUBIndices[ i ].push_back(NX + j);
				conUBValues[ i ].push_back( ubTmp( j ) );
				numUB++;
			}
		}
	}

	//
	// Setup variables
	//
	for (unsigned i = 0; i < N + 1; ++i)
	{
		conLB[ i ].setup(string("lb") + toString(i + 1), conLBIndices[ i ].size(), 1, REAL, FORCES_PARAMS, false, qpObjPrefix);
		conUB[ i ].setup(string("ub") + toString(i + 1), conUBIndices[ i ].size(), 1, REAL, FORCES_PARAMS, false, qpObjPrefix);
	}

	int hardcodeConstraintValues;
	get(CG_HARDCODE_CONSTRAINT_VALUES, hardcodeConstraintValues);
	uint numBounds = numLB+numUB;
	if (!hardcodeConstraintValues && numBounds > 0)
	{
		lbValues.setup("lbValues", numLB, 1, REAL, ACADO_VARIABLES);
		lbValues.setDoc( "Lower bounds values." );
		ubValues.setup("ubValues", numUB, 1, REAL, ACADO_VARIABLES);
		ubValues.setDoc( "Upper bounds values." );
	}

	evaluateConstraints.setup("evaluateConstraints");

	//
	// Export evaluation of simple box constraints
	//
	uint indexB = 0;
	for (unsigned i = 0; i < N + 1; ++i) {
		for (unsigned j = 0; j < conLBIndices[ i ].size(); ++j)
		{
			if( hardcodeConstraintValues ) {
				evaluateConstraints << conLB[ i ].getFullName() << "[ " << toString(j) << " ]" << " = " << toString(conLBValues[ i ][ j ]) << " - ";
			}
			else {
				evaluateConstraints << conLB[ i ].getFullName() << "[ " << toString(j) << " ]" << " = " << lbValues.get( indexB,0 ) << " - ";
			}
			indexB++;

			if (conLBIndices[ i ][ j ] < NX)
				evaluateConstraints << x.getFullName() << "[ " << toString(i * NX + conLBIndices[ i ][ j ]) << " ];\n";
			else
				evaluateConstraints << u.getFullName() << "[ " << toString(i * NU + conLBIndices[ i ][ j ] - NX) << " ];\n";
		}
	}
	evaluateConstraints.addLinebreak();

	indexB = 0;
	for (unsigned i = 0; i < N + 1; ++i)
		for (unsigned j = 0; j < conUBIndices[ i ].size(); ++j)
		{
			if( hardcodeConstraintValues ) {
				evaluateConstraints << conUB[ i ].getFullName() << "[ " << toString(j) << " ]" << " = " << toString(conUBValues[ i ][ j ]) << " - ";
			}
			else {
				evaluateConstraints << conUB[ i ].getFullName() << "[ " << toString(j) << " ]" << " = " << ubValues.get( indexB,0 ) << " - ";
			}
			indexB++;

			if (conUBIndices[ i ][ j ] < NX)
				evaluateConstraints << x.getFullName() << "[ " << toString(i * NX + conUBIndices[ i ][ j ]) << " ];\n";
			else
				evaluateConstraints << u.getFullName() << "[ " << toString(i * NU + conUBIndices[ i ][ j ] - NX) << " ];\n";
		}
	evaluateConstraints.addLinebreak();

	////////////////////////////////////////////////////////////////////////////
	//
	// Evaluation of the equality constraints
	//  - system dynamics only
	//
	////////////////////////////////////////////////////////////////////////////

	conC.clear();
	conC.resize( N );

	// XXX FORCES works with column major format
	//	if (initialStateFixed() == true)
	//		conC[ 0 ].setup("C1", NX + NU, 2 * NX, REAL, FORCES_PARAMS, false, qpObjPrefix);
	//	else
	//		conC[ 0 ].setup("C1", NX + NU, NX, REAL, FORCES_PARAMS, false, qpObjPrefix);

	//	for (unsigned i = 1; i < N; ++i)
	for (unsigned i = 0; i < N; ++i)
		conC[ i ].setup(string("C") + toString(i + 1), NX + NU, NX, REAL, FORCES_PARAMS, false, qpObjPrefix);

	ExportIndex index( "index" );
	conStageC.setup("conStageC", NX + NU, NX, REAL);
	conSetGxGu.setup("conSetGxGu", conStageC, index);

	conSetGxGu.addStatement(
			conStageC.getSubMatrix(0, NX, 0, NX) ==
					evGx.getSubMatrix(index * NX, (index + 1) * NX, 0, NX).getTranspose()
	);
	conSetGxGu.addLinebreak();
	conSetGxGu.addStatement(
			conStageC.getSubMatrix(NX, NX + NU, 0, NX) ==
					evGu.getSubMatrix(index * NX, (index + 1) * NX, 0, NU).getTranspose()
	);

	//	if (initialStateFixed() == true)
	//	{
	//		initialize.addStatement(
	//				conC[ 0 ].getSubMatrix(0, NX, 0, NX) == eye( NX )
	//		);
	//		evaluateConstraints.addLinebreak();
	//		evaluateConstraints.addStatement(
	//				conC[ 0 ].getSubMatrix(0, NX, NX, 2 * NX) == evGx.getSubMatrix(0, NX, 0, NX).getTranspose()
	//		);
	//		evaluateConstraints.addLinebreak();
	//		evaluateConstraints.addStatement(
	//				conC[ 0 ].getSubMatrix(NX, NX + NU, NX, 2 * NX) == evGu.getSubMatrix(0, NX, 0, NU).getTranspose()
	//		);
	//		evaluateConstraints.addLinebreak();
	//	}

	unsigned start = 0; //initialStateFixed() == true ? 1 : 0;
	for (unsigned i = start; i < N; ++i)
		evaluateConstraints.addFunctionCall(conSetGxGu, conC[ i ], ExportIndex( i ));
	evaluateConstraints.addLinebreak();

	cond.clear();

	unsigned dNum = initialStateFixed() == true ? N + 1 : N;
	cond.resize(dNum);

	//	if (initialStateFixed() == true)
	//		cond[ 0 ].setup("d1", 2 * NX, 1, REAL, FORCES_PARAMS, false, qpObjPrefix);
	//	else
	//		cond[ 0 ].setup("d1", NX, 1, REAL, FORCES_PARAMS, false, qpObjPrefix);

	for (unsigned i = 0; i < dNum; ++i)
		cond[ i ].setup(string("d") + toString(i + 1), NX, 1, REAL, FORCES_PARAMS, false, qpObjPrefix);

	ExportVariable staged, stagedNew;

	staged.setup("staged", NX, 1, REAL, ACADO_LOCAL);
	stagedNew.setup("stagedNew", NX, 1, REAL, ACADO_LOCAL);
	conSetd.setup("conSetd", stagedNew, index);
	conSetd.addStatement(
		stagedNew == zeros<double>(NX, 1) - d.getRows(index * NX, (index + 1) * NX)
	);

	//		evaluateConstraints.addStatement(
	//				cond[ 0 ].getRows(NX, 2 * NX) == dummyZero - d.getRows(0, NX)
	//		);
	//		evaluateConstraints.addLinebreak();

	if( initialStateFixed() ) {
		for (unsigned i = 1; i < dNum; ++i)
			evaluateConstraints.addFunctionCall(
					conSetd, cond[ i ], ExportIndex(i - 1)
			);
	}
	else {
		for (unsigned i = 0; i < dNum; ++i)
			evaluateConstraints.addFunctionCall(
					conSetd, cond[ i ], ExportIndex(i)
			);
	}

	return SUCCESSFUL_RETURN;
}

returnValue ExportGaussNewtonForces::setupVariables( )
{
	if (initialStateFixed() == true)
	{
		x0.setup("x0",  NX, 1, REAL, ACADO_VARIABLES);
		x0.setDoc( "Current state feedback vector." );
	}

	return SUCCESSFUL_RETURN;
}

returnValue ExportGaussNewtonForces::setupMultiplicationRoutines( )
{
	return SUCCESSFUL_RETURN;
}

returnValue ExportGaussNewtonForces::setupEvaluation( )
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

	feedback.addStatement(
			//			cond[ 0 ].getRows(0, NX) == x0 - x.getRow( 0 ).getTranspose()
			cond[ 0 ] == x0 - x.getRow( 0 ).getTranspose()
	);
	feedback.addLinebreak();

	// Calculate objective residuals
	feedback << (Dy -= y) << (DyN -= yN);
	feedback.addLinebreak();

	for (unsigned i = 0; i < N; ++i)
		feedback.addFunctionCall(setStagef, objGradients[ i ], ExportIndex( i ));
	feedback.addStatement( objGradients[ N ] == QN2 * DyN );
	feedback.addLinebreak();

	//
	// Configure output variables
	//
	std::vector< ExportVariable > vecQPVars;

	vecQPVars.clear();
	vecQPVars.resize(N + 1);
	for (unsigned i = 0; i < N; ++i)
		vecQPVars[ i ].setup(string("out") + toString(i + 1), NX + NU, 1, REAL, FORCES_OUTPUT, false, qpObjPrefix);
	vecQPVars[ N ].setup(string("out") + toString(N + 1), NX, 1, REAL, FORCES_OUTPUT, false, qpObjPrefix);

	//
	// In case warm starting is enabled, give an initial guess, based on the old solution
	//
	int hotstartQP;
	get(HOTSTART_QP, hotstartQP);

	if ( hotstartQP )
	{
		std::vector< ExportVariable > zInit;

		zInit.clear();
		zInit.resize(N + 1);
		for (unsigned i = 0; i < N; ++i)
		{
			string name = "z_init_";
			name = name + (i < 10 ? "0" : "") + toString( i );
			zInit[ i ].setup(name, NX + NU, 1, REAL, FORCES_PARAMS, false, qpObjPrefix);
		}
		string name = "z_init_";
		name = name + (N < 10 ? "0" : "") + toString( N );
		zInit[ N ].setup(name, NX, 1, REAL, FORCES_PARAMS, false, qpObjPrefix);

		// TODO This should be further investigated.

		//
		// 1) Just use the old solution
		//
		//		for (unsigned blk = 0; blk < N + 1; blk++)
		//			feedback.addStatement(zInit[ blk ] == vecQPVars[ blk ] );

		//
		// 2) Initialization by shifting
		//

		//		for (unsigned blk = 0; blk < N - 1; blk++)
		//			feedback.addStatement( zInit[ blk ] == vecQPVars[blk + 1] );
		//		for (unsigned el = 0; el < NX; el++)
		//			feedback.addStatement( zInit[N - 1].getElement(el, 0) == vecQPVars[ N ].getElement(el, 0) );
	}

	//
	// Call a QP solver
	// NOTE: we need two prefixes:
	// 1. module prefix
	// 2. structure instance prefix
	//
	ExportFunction solveQP;
	solveQP.setup("solve");
	solveQP.setName( "solve" );

	feedback
	<< returnValueFeedbackPhase.getFullName() << " = "
	<< qpModuleName << "_" << solveQP.getName() << "( "
	<< "&" << qpObjPrefix << "_" << "params" << ", "
	<< "&" << qpObjPrefix << "_" << "output" << ", "
	<< "&" << qpObjPrefix << "_" << "info" << " , NULL);\n";
	feedback.addLinebreak();

	//
	// Here we have to add the differences....
	//

	ExportVariable stageOut("stageOut", 1, NX + NU, REAL, ACADO_LOCAL);
	ExportIndex index( "index" );
	acc.setup("accumulate", stageOut, index);

	acc.addStatement( x.getRow( index ) += stageOut.getCols(0, NX) );
	acc.addLinebreak();
	acc.addStatement( u.getRow( index ) += stageOut.getCols(NX, NX + NU) );
	acc.addLinebreak();

	for (unsigned i = 0; i < N; ++i)
		feedback.addFunctionCall(acc, vecQPVars[ i ], ExportIndex( i ));
	feedback.addLinebreak();

	feedback.addStatement( x.getRow( N ) += vecQPVars[ N ].getTranspose() );
	feedback.addLinebreak();

	////////////////////////////////////////////////////////////////////////////
	//
	// TODO Setup evaluation of KKT
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

returnValue ExportGaussNewtonForces::setupQPInterface( )
{
	//
	// Configure and export QP interface
	//

	qpInterface = std::shared_ptr< ExportForcesInterface >(new ExportForcesInterface(FORCES_TEMPLATE, "", commonHeaderName));

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

	qpGenerator = std::shared_ptr< ExportForcesGenerator >(new ExportForcesGenerator(FORCES_GENERATOR, outFile, "", "real_t", "int", 16, "%"));

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

	qpGenerator->configure(
			NX,
			NU,
			N,
			conLBIndices,
			conUBIndices,
			conABDimensions,
			(Q1.isGiven() == true && R1.isGiven() == true) ? 1 : 0,
			diagH,
			diagHN,
			initialStateFixed(),
			qpModuleName,
			(PrintLevel)printLevel == HIGH ? 2 : 0,
			maxNumQPiterations,
			useOMP,
			true,
			hotstartQP
	);

	qpGenerator->exportCode();

	//
	// Export Python generator
	//

	outFile = folderName + "/acado_forces_generator.py";

	qpGenerator = std::shared_ptr< ExportForcesGenerator >(new ExportForcesGenerator(FORCES_GENERATOR_PYTHON, outFile, "", "real_t", "int", 16, "#"));

	qpGenerator->configure(
			NX,
			NU,
			N,
			conLBIndices,
			conUBIndices,
			conABDimensions,
			(Q1.isGiven() == true && R1.isGiven() == true) ? 1 : 0, // TODO Remove this one
			diagH,
			diagHN,
			initialStateFixed(),
			qpModuleName,
			(PrintLevel)printLevel == HIGH ? 2 : 0,
			maxNumQPiterations,
			useOMP,
			false,
			hotstartQP
	);

	qpGenerator->exportCode();

	return SUCCESSFUL_RETURN;
}

CLOSE_NAMESPACE_ACADO
