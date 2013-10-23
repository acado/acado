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
 *    \file src/code_generation/export_gauss_newton_forces.cpp
 *    \author Milan Vukov
 *    \date 2012 - 2013
 */

#include <acado/code_generation/export_gauss_newton_forces.hpp>
#include <acado/code_generation/export_module.hpp>

#include <acado/code_generation/export_forces_interface.hpp>
#include <acado/code_generation/export_forces_generator.hpp>

#include <acado/code_generation/templates/templates.hpp>

#include <sstream>

BEGIN_NAMESPACE_ACADO

using namespace std;

ExportGaussNewtonForces::ExportGaussNewtonForces(	UserInteraction* _userInteraction,
													const String& _commonHeaderName
													) : ExportNLPSolver( _userInteraction,_commonHeaderName )
{
	qpObjPrefix = "acadoForces";
	qpModuleName = "forces";
	diagH = diagHN = false;
}

returnValue ExportGaussNewtonForces::setup( )
{
	setupVariables();

	setupSimulation();

	//
	// Add QP initialization call to the initialization
	//
	ExportFunction initializeForces( "initializeForces" );
	initialize.addFunctionCall( initializeForces );

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

	code.addFunction( evaluateLSQ );
	code.addFunction( evaluateLSQEndTerm );
	code.addFunction( setObjQ1Q2 );
	code.addFunction( setObjR1R2 );
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

//
// PROTECTED FUNCTIONS:
//

returnValue ExportGaussNewtonForces::setupObjectiveEvaluation( void )
{
	evaluateObjective.setup("evaluateObjective");

	int variableObjS;
	get(CG_USE_VARIABLE_WEIGHTING_MATRIX, variableObjS);

	diagH = false;
	diagHN = false;
	unsigned dimHRows = NX + NU;
	unsigned dimHCols = NX + NU;
	unsigned dimHNRows = NX;
	unsigned dimHNCols = NX;
	if (objS.isGiven() == BT_TRUE)
		if (objS.getGivenMatrix().isDiagonal())
		{
			diagH = true;
			dimHCols = 1;
		}

	if (objSEndTerm.isGiven() == BT_TRUE)
		if (objSEndTerm.getGivenMatrix().isDiagonal() == BT_TRUE)
		{
			diagHN = true;
			dimHNCols = 1;
		}

	objHessians.resize(N + 1);
	for (unsigned i = 0; i < N; ++i)
	{
		objHessians[ i ].setup((String)"H" << (i + 1), dimHRows, dimHCols, REAL, FORCES_PARAMS, BT_FALSE, qpObjPrefix);
	}
	objHessians[ N ].setup((String)"H" << (N + 1), dimHNRows, dimHNCols, REAL, FORCES_PARAMS, BT_FALSE, qpObjPrefix);

	objGradients.resize(N + 1);
	for (unsigned i = 0; i < N; ++i)
	{
		objGradients[ i ].setup((String)"f" << (i + 1), NX + NU, 1, REAL, FORCES_PARAMS, BT_FALSE, qpObjPrefix);
	}
	objGradients[ N ].setup((String)"f" << (N + 1), NX, 1, REAL, FORCES_PARAMS, BT_FALSE, qpObjPrefix);

	//
	// LM regularization preparation
	//

	ExportVariable evLmX = zeros(NX, NX);
	ExportVariable evLmU = zeros(NU, NU);

	if (levenbergMarquardt > 0.0)
	{
		Matrix lmX = eye( NX );
		lmX *= levenbergMarquardt;

		Matrix lmU = eye( NU );
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
	loopObjective.addStatement( objValueIn.getCols(NX + NU, NX + NU + NP) == p );
	loopObjective.addLinebreak( );

	// Evaluate the objective function
	loopObjective.addFunctionCall( "evaluateLSQ", objValueIn, objValueOut );

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
	evaluateObjective.addFunctionCall( "evaluateLSQEndTerm", objValueIn, objValueOut );
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

	//
	// Hessian setup
	//

	ExportVariable stageH;
	ExportIndex index( "index" );
	stageH.setup("stageH", dimHRows, dimHCols, REAL, ACADO_LOCAL);
	setStageH.setup("setStageH", stageH, index.makeArgument());

	if (Q1.isGiven() == BT_FALSE)
		setStageH.addStatement(
				stageH.getSubMatrix(0, NX, 0, NX) == Q1.getSubMatrix(index * NX, (index + 1) * NX, 0, NX) + evLmX
		);
	else
	{
		setStageH.addStatement( index.getFullName() << " = " << index.getFullName() << ";\n" );
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

	if (R1.isGiven() == BT_FALSE)
		setStageH.addStatement(
				stageH.getSubMatrix(NX, NX + NU, NX, NX + NU) == R1.getSubMatrix(index * NU, (index + 1) * NU, 0, NU) + evLmU
		);
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

	if (Q1.isGiven() == BT_TRUE && R1.isGiven() == BT_TRUE)
	{
		initialize.addStatement(
				setStageH.getName() << "( " << objHessians[ 0 ].getFullName() << ", " << "0" << " );\n"
		);
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
					objHessians[ N ] == (QN1.getGivenMatrix().getDiag() + evLmX.getGivenMatrix().getDiag())
			);
		}
	}
	else
	{
		for (unsigned i = 0; i < N; ++i)
		{
			evaluateObjective.addStatement(
					setStageH.getName() << "( " << objHessians[ i ].getFullName() << ", "
					<< i << " );\n"
			);
		}
		evaluateObjective.addLinebreak();
		evaluateObjective.addStatement(
				objHessians[ N ] == QN1 + evLmX
		);
	}

	//
	// Gradient setup
	//

	ExportVariable stagef;
	stagef.setup("stagef", NX + NU, 1, REAL, ACADO_LOCAL);
	setStagef.setup("setStagef", stagef, index.makeArgument());

	if (Q2.isGiven() == BT_FALSE)
		setStagef.addStatement(
				stagef.getRows(0, NX) == Q2.getSubMatrix(index * NX, (index + 1) * NX, 0, NY) *
				Dy.getRows(index * NY, (index + 1) * NY)
		);
	else
	{
		setStagef.addStatement( index.getFullName() << " = " << index.getFullName() << ";\n" );
		setStagef.addStatement(
				stagef.getRows(0, NX) == Q2 *
				Dy.getRows(index * NY, (index + 1) * NY)
		);
	}
	setStagef.addLinebreak();

	if (R2.isGiven() == BT_FALSE)
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

	conLBValues.clear();
	conLBValues.resize(N + 1);

	conUBValues.clear();
	conUBValues.resize(N + 1);

	Vector lbTmp, ubTmp;

	//
	// Stack state constraints
	//
	unsigned numStateBox = 0;
	for (unsigned i = 0; i < xBounds.getNumPoints(); ++i)
	{
		lbTmp = xBounds.getLowerBounds( i );
		ubTmp = xBounds.getUpperBounds( i );

		if (lbTmp.isFinite() == BT_FALSE && ubTmp.isFinite() == BT_FALSE)
			continue;

		++numStateBox;

		for (unsigned j = 0; j < lbTmp.getDim(); ++j)
		{
			if (acadoIsFinite( lbTmp( j ) ) == BT_TRUE)
			{
				conLBIndices[ i ].push_back( j );
				conLBValues[ i ].push_back( lbTmp( j ) );
			}

			if (acadoIsFinite( ubTmp( j ) ) == BT_TRUE)
			{
				conUBIndices[ i ].push_back( j );
				conUBValues[ i ].push_back( ubTmp( j ) );
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

		if (lbTmp.isFinite() == BT_FALSE && ubTmp.isFinite() == BT_FALSE)
			continue;

		for (unsigned j = 0; j < lbTmp.getDim(); ++j)
		{
			if (acadoIsFinite( lbTmp( j ) ) == BT_TRUE)
			{
				conLBIndices[ i ].push_back(NX + j);
				conLBValues[ i ].push_back( lbTmp( j ) );
			}

			if (acadoIsFinite( ubTmp( j ) ) == BT_TRUE)
			{
				conUBIndices[ i ].push_back(NX + j);
				conUBValues[ i ].push_back( ubTmp( j ) );
			}
		}
	}

	//
	// Setup variables
	//
	for (unsigned i = 0; i < N + 1; ++i)
	{
		conLB[ i ].setup((String)"lb" << (i + 1), conLBIndices[ i ].size(), 1, REAL, FORCES_PARAMS, BT_FALSE, qpObjPrefix);
		conUB[ i ].setup((String)"ub" << (i + 1), conUBIndices[ i ].size(), 1, REAL, FORCES_PARAMS, BT_FALSE, qpObjPrefix);
	}

	evaluateConstraints.setup("evaluateConstraints");

	//
	// Export evaluation of simple box constraints
	//
	for (unsigned i = 0; i < N + 1; ++i)
		for (unsigned j = 0; j < conLBIndices[ i ].size(); ++j)
		{
			stringstream s;

			s << conLB[ i ].getFullName().getName() << "[ " << j << " ]" << " = " << conLBValues[ i ][ j ] << " - ";

			if (conLBIndices[ i ][ j ] < NX)
				s << x.getFullName().getName() << "[ " << i * NX + conLBIndices[ i ][ j ] << " ];\n";
			else
				s << u.getFullName().getName() << "[ " << i * NU + conLBIndices[ i ][ j ] - NX << " ];\n";

			evaluateConstraints.addStatement( (String)s.str().c_str() );
		}
	evaluateConstraints.addLinebreak();

	for (unsigned i = 0; i < N + 1; ++i)
		for (unsigned j = 0; j < conUBIndices[ i ].size(); ++j)
		{
			stringstream s;

			s << conUB[ i ].getFullName().getName() << "[ " << j << " ]" << " = " << conUBValues[ i ][ j ] << " - ";
			if (conUBIndices[ i ][ j ] < NX)
				s << x.getFullName().getName() << "[ " << i * NX + conUBIndices[ i ][ j ] << " ];\n";
			else
				s << u.getFullName().getName() << "[ " << i * NU + conUBIndices[ i ][ j ] - NX << " ];\n";

			evaluateConstraints.addStatement( (String)s.str().c_str() );
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
//	if (initialStateFixed() == BT_TRUE)
//		conC[ 0 ].setup("C1", NX + NU, 2 * NX, REAL, FORCES_PARAMS, BT_FALSE, qpObjPrefix);
//	else
//		conC[ 0 ].setup("C1", NX + NU, NX, REAL, FORCES_PARAMS, BT_FALSE, qpObjPrefix);

//	for (unsigned i = 1; i < N; ++i)
	for (unsigned i = 0; i < N; ++i)
		conC[ i ].setup((String)"C" << (i + 1), NX + NU, NX, REAL, FORCES_PARAMS, BT_FALSE, qpObjPrefix);

	ExportIndex index( "index" );
	conStageC.setup("conStageC", NX + NU, NX, REAL);
	conSetGxGu.setup("conSetGxGu", conStageC, index.makeArgument());

	conSetGxGu.addStatement(
			conStageC.getSubMatrix(0, NX, 0, NX) == evGx.
			getSubMatrix(index * NX, (index + 1) * NX, 0, NX).getTranspose()
	);
	conSetGxGu.addLinebreak();
	conSetGxGu.addStatement(
			conStageC.getSubMatrix(NX, NX + NU, 0, NX) == evGu.
				getSubMatrix(index * NX, (index + 1) * NX, 0, NU).getTranspose()
	);

//	if (initialStateFixed() == BT_TRUE)
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

	unsigned start = 0; //initialStateFixed() == BT_TRUE ? 1 : 0;
	for (unsigned i = start; i < N; ++i)
	{
		evaluateConstraints.addStatement(
				conSetGxGu.getName() << "( " <<
				conC[ i ].getFullName() <<
				", " << i << " );\n"
		);
	}
	evaluateConstraints.addLinebreak();

	cond.clear();

	unsigned dNum = initialStateFixed() == BT_TRUE ? N + 1 : N;
	cond.resize(dNum);

//	if (initialStateFixed() == BT_TRUE)
//		cond[ 0 ].setup("d1", 2 * NX, 1, REAL, FORCES_PARAMS, BT_FALSE, qpObjPrefix);
//	else
//		cond[ 0 ].setup("d1", NX, 1, REAL, FORCES_PARAMS, BT_FALSE, qpObjPrefix);


	for (unsigned i = 0; i < dNum; ++i)
		cond[ i ].setup((String)"d" << i + 1, NX, 1, REAL, FORCES_PARAMS, BT_FALSE, qpObjPrefix);

	ExportVariable staged, stagedNew;

	if (performsSingleShooting() == BT_FALSE)
	{
		staged.setup("staged", NX, 1, REAL, ACADO_LOCAL);
		stagedNew.setup("stagedNew", NX, 1, REAL, ACADO_LOCAL);
		conSetd.setup("conSetd", stagedNew, index.makeArgument());

		ExportVariable dummyZero( zeros(NX, 1) );

		if (initialStateFixed() == BT_TRUE)
		{
			conSetd.addStatement(
					stagedNew == dummyZero - d.getRows(index * NX, (index + 1) * NX)
			);
		}

//		evaluateConstraints.addStatement(
//				cond[ 0 ].getRows(NX, 2 * NX) == dummyZero - d.getRows(0, NX)
//		);
//		evaluateConstraints.addLinebreak();

		start = initialStateFixed() == BT_TRUE ? 1 : 0;
		for (unsigned i = start; i < dNum; ++i)
		{
			stringstream s;

			s << conSetd.getName().getName()
					<< "( " << cond[ i ].getFullName().getName() << ", " << i - 1 << ");" << endl;

			evaluateConstraints.addStatement( s.str().c_str() );
		}
	}

	return SUCCESSFUL_RETURN;
}

returnValue ExportGaussNewtonForces::setupVariables( )
{
	if (initialStateFixed() == BT_TRUE)
	{
		x0.setup("x0",  NX, 1, REAL, ACADO_VARIABLES);
		x0.setDoc( (String)"Current state feedback vector." );
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

	preparation.addFunctionCall( modelSimulation );
	preparation.addFunctionCall( evaluateObjective );
	preparation.addFunctionCall( evaluateConstraints );

	////////////////////////////////////////////////////////////////////////////
	//
	// Setup feedback phase
	//
	////////////////////////////////////////////////////////////////////////////
	ExportVariable stateFeedback("stateFeedback", NX, 1, REAL, ACADO_LOCAL);
	ExportVariable returnValueFeedbackPhase("retVal", 1, 1, INT, ACADO_LOCAL, BT_TRUE);
	returnValueFeedbackPhase.setDoc( "Status code of the FORCES QP solver." );
	feedback.setup("feedbackStep" );
	feedback.doc( "Feedback/estimation step of the RTI scheme." );
	feedback.setReturnValue( returnValueFeedbackPhase );

	feedback.addStatement(
//			cond[ 0 ].getRows(0, NX) == x0 - x.getRow( 0 ).getTranspose()
			cond[ 0 ] == x0 - x.getRow( 0 ).getTranspose()
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
	{
		feedback.addStatement(
				setStagef.getName() << "( " <<
				objGradients[ i ].getFullName() << ", " << i <<  " );\n"
		);
	}
	feedback.addLinebreak();
	feedback.addStatement( objGradients[ N ] == QN2 * DyN );
	feedback.addLinebreak();

	//
	// Call a QP solver
	// NOTE: we need two prefixes:
	// 1. module prefix
	// 2. structure instance prefix
	//
	ExportFunction solveQP;
	solveQP.setup("solve");
	String prefix;
	prefix = "forces";

	feedback.addStatement(
			returnValueFeedbackPhase.getFullName() << " = " <<
			(String) qpModuleName.getName() << "_" << solveQP.getName( ) << "( " <<
			"&" << qpObjPrefix << "_" << "params" << ", " <<
			"&" << qpObjPrefix << "_" << "output" << ", " <<
			"&" << qpObjPrefix << "_" << "info" << " );\n"
	);
	feedback.addLinebreak();

	//
	// Here we have to add the differences....
	//

	std::vector< ExportVariable > vecQPVars;

	vecQPVars.clear();
	vecQPVars.resize(N + 1);
	for (unsigned i = 0; i < N; ++i)
		vecQPVars[ i ].setup((String)"out" << i + 1, NX + NU, 1, REAL, FORCES_OUTPUT, BT_FALSE, qpObjPrefix);
	vecQPVars[ N ].setup((String)"out" << N + 1, NX, 1, REAL, FORCES_OUTPUT, BT_FALSE, qpObjPrefix);

	ExportVariable stageOut("stageOut", 1, NX + NU, REAL, ACADO_LOCAL);
	ExportIndex index( "index" );
	acc.setup("accumulate", stageOut, index.makeArgument());

	if (performsSingleShooting() == BT_TRUE)
	{
		acc.addStatement(
			u.getRow( index ) += stageOut.getCols(NX, NX + NU)
		);
		acc.addLinebreak();
	}
	else
	{
		acc.addStatement(
				x.getRow( index ) += stageOut.getCols(0, NX)
		);
		acc.addLinebreak();

		acc.addStatement(
				u.getRow( index ) += stageOut.getCols(NX, NX + NU)
		);
		acc.addLinebreak();
	}

	if (performsSingleShooting() == BT_TRUE)
	{
		feedback.addStatement(
				x.getRow( 0 ) += vecQPVars[ 0 ].getTranspose().getCols(0, NX)
		);
	}

	for (unsigned i = 0; i < N; ++i)
	{
		feedback.addStatement(
				acc.getName() << "( " <<
				vecQPVars[ i ].getFullName() << ", " <<
				i << " );\n"
		);
	}
	feedback.addLinebreak();

	if (performsSingleShooting() == BT_FALSE)
	{
		feedback.addStatement(
				x.getRow( N ) += vecQPVars[ N ].getTranspose()
		);
		feedback.addLinebreak();
	}

	////////////////////////////////////////////////////////////////////////////
	//
	// TODO Setup evaluation of KKT
	//
	////////////////////////////////////////////////////////////////////////////
	ExportVariable kkt("kkt", 1, 1, REAL, ACADO_LOCAL, BT_TRUE);

	getKKT.setup( "getKKT" );
	getKKT.doc( "Get the KKT tolerance of the current iterate. Under development." );
//	kkt.setDoc( "The KKT tolerance value." );
	kkt.setDoc( "0." );
	getKKT.setReturnValue( kkt );

	getKKT.addStatement( kkt == 0 );

	return SUCCESSFUL_RETURN;
}

returnValue ExportGaussNewtonForces::setupQPInterface( )
{
	//
	// Configure and export QP interface
	//

	qpInterface = std::tr1::shared_ptr< ExportForcesInterface >(new ExportForcesInterface(FORCES_TEMPLATE, "", commonHeaderName));

	ExportVariable tmp1("tmp", 1, 1, REAL, FORCES_PARAMS, BT_FALSE, qpObjPrefix);
	ExportVariable tmp2("tmp", 1, 1, REAL, FORCES_OUTPUT, BT_FALSE, qpObjPrefix);
	ExportVariable tmp3("tmp", 1, 1, REAL, FORCES_INFO, BT_FALSE, qpObjPrefix);

	string str( qpModuleName.getName() );
	string params = str;
	params += "_params";

	string output = str;
	output += "_output";

	string info = str;
	info += "_info";

	string header( qpModuleName.getName() );
	header += ".h";

	qpInterface->configure(
			string( header ),

			string( params ),
			string( tmp1.getDataStructString().getName() ),

			string( output ),
			string( tmp2.getDataStructString().getName() ),

			string( info ),
			string( tmp3.getDataStructString().getName() )
	);

	//
	// Configure and export QP generator
	//

	String folderName = dynamic_cast< ExportModule* >( userInteraction )->getExportFolderName();
	String outFile = folderName + "/acado_forces_generator.m";

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

	qpGenerator->configure(
			NX,
			NU,
			N,
			conLBIndices,
			conUBIndices,
			(Q1.isGiven() == BT_TRUE && R1.isGiven() == BT_TRUE) ? 1 : 0,
			diagH,
			diagHN,
			true, // TODO enable MHE
			qpModuleName.getName(),
			(PrintLevel)printLevel == HIGH ? 2 : 0,
			maxNumQPiterations,
			useOMP
	);

	qpGenerator->exportCode();

	return SUCCESSFUL_RETURN;
}

//
// Solver registration
//

ExportNLPSolver* createGaussNewtonForces(	UserInteraction* _userInteraction,
											const String& _commonHeaderName
											)
{
	return new ExportGaussNewtonForces(_userInteraction, _commonHeaderName);
}

RegisterGaussNewtonForces::RegisterGaussNewtonForces()
{
	NLPSolverFactory::instance().registerAlgorithm(GAUSS_NEWTON_FORCES, createGaussNewtonForces);
}

CLOSE_NAMESPACE_ACADO
