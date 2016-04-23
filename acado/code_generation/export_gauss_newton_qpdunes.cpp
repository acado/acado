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
 *    \date 2013 - 2014
 */

#include <acado/code_generation/export_gauss_newton_qpdunes.hpp>
#include <acado/code_generation/export_qpdunes_interface.hpp>

BEGIN_NAMESPACE_ACADO

using namespace std;

ExportGaussNewtonQpDunes::ExportGaussNewtonQpDunes(	UserInteraction* _userInteraction,
													const std::string& _commonHeaderName
													) : ExportNLPSolver( _userInteraction,_commonHeaderName )
{}

returnValue ExportGaussNewtonQpDunes::setup( )
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
	cleanupQpDunes.setName( "cleanupQpDunes" );
	cleanup.addFunctionCall( cleanupQpDunes );
	LOG( LVL_DEBUG ) << "done!" << endl;

	LOG( LVL_DEBUG ) << "Solver: setup setupVariables... " << endl;
	setupVariables();
	LOG( LVL_DEBUG ) << "done!" << endl;

	LOG( LVL_DEBUG ) << "Solver: setup setupSimulation... " << endl;
	setupSimulation();
	LOG( LVL_DEBUG ) << "done!" << endl;

	LOG( LVL_DEBUG ) << "Solver: setup setupObjectiveEvaluation... " << endl;
	setupObjectiveEvaluation();
	LOG( LVL_DEBUG ) << "done!" << endl;

	LOG( LVL_DEBUG ) << "Solver: setup setupConstraintsEvaluation... " << endl;
	setupConstraintsEvaluation();
	LOG( LVL_DEBUG ) << "done!" << endl;

	LOG( LVL_DEBUG ) << "Solver: setup setupEvaluation... " << endl;
	setupEvaluation();
	LOG( LVL_DEBUG ) << "done!" << endl;

	LOG( LVL_DEBUG ) << "Solver: setup setupAuxiliaryFunctions... " << endl;
	setupAuxiliaryFunctions();
	LOG( LVL_DEBUG ) << "done!" << endl;

	return SUCCESSFUL_RETURN;
}

returnValue ExportGaussNewtonQpDunes::getDataDeclarations(	ExportStatementBlock& declarations,
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
	declarations.addDeclaration(qpgN, dataStruct);
	declarations.addDeclaration(qpLb0, dataStruct);
	declarations.addDeclaration(qpUb0, dataStruct);
	declarations.addDeclaration(qpLb, dataStruct);
	declarations.addDeclaration(qpUb, dataStruct);
	declarations.addDeclaration(lbValues, dataStruct);
	declarations.addDeclaration(ubValues, dataStruct);
	declarations.addDeclaration(qpC, dataStruct);
	declarations.addDeclaration(qpc, dataStruct);

	declarations.addDeclaration(qpA, dataStruct);
	declarations.addDeclaration(qpLbA, dataStruct);
	declarations.addDeclaration(qpUbA, dataStruct);

	declarations.addDeclaration(qpPrimal, dataStruct);
	declarations.addDeclaration(qpLambda, dataStruct);
	declarations.addDeclaration(qpMu, dataStruct);

	// lagrange multipliers
	declarations.addDeclaration(mu, dataStruct);

	return SUCCESSFUL_RETURN;
}

returnValue ExportGaussNewtonQpDunes::getFunctionDeclarations(	ExportStatementBlock& declarations
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

returnValue ExportGaussNewtonQpDunes::getCode(	ExportStatementBlock& code
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


unsigned ExportGaussNewtonQpDunes::getNumQPvars( ) const
{
	return (N + 1) * NX + N * NU;
}

//
// PROTECTED FUNCTIONS:
//

returnValue ExportGaussNewtonQpDunes::setupObjectiveEvaluation( void )
{
	if (S1.getGivenMatrix().isZero() == false)
		ACADOWARNINGTEXT(RET_INVALID_ARGUMENTS,
				"Mixed control-state terms in the objective function are not supported at the moment.");

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

	//
	// Hessian setup
	//

	// LM regularization preparation

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

	// Interface variable to qpDUNES
	qpH.setup("qpH", N * (NX + NU) * (NX + NU) + NX * NX, 1, REAL, ACADO_WORKSPACE);

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
	int sensitivityProp;
	get( DYNAMIC_SENSITIVITY, sensitivityProp );
	bool adjoint = ((ExportSensitivityType) sensitivityProp == BACKWARD);

	// Interface variable to qpDUNES
	qpg.setup("qpG", N * (NX + NU) + NX, 1, REAL, ACADO_WORKSPACE);

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
	if( adjoint ) {
		setStagef.addStatement( stagef.getRows(0, NX) += objSlx.getRows( index*NX, index*NX+NX ) );
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
	if( adjoint ) {
		setStagef.addStatement( stagef.getRows(NX, NX+NU) += objSlu.getRows( index*NU, index*NU+NU ) );
	}

	// A buffer given to update the last node's gradient
	if (initialStateFixed() == false)
		qpgN.setup("qpgN", NX, 1, REAL, ACADO_WORKSPACE);

	return SUCCESSFUL_RETURN;
}

returnValue ExportGaussNewtonQpDunes::setupConstraintsEvaluation( void )
{
	////////////////////////////////////////////////////////////////////////////
	//
	// Setup evaluation of box constraints on states and controls
	//
	////////////////////////////////////////////////////////////////////////////

	int hardcodeConstraintValues;
	get(CG_HARDCODE_CONSTRAINT_VALUES, hardcodeConstraintValues);

	if (initialStateFixed() == true)
	{
		qpLb0.setup("qpLb0", 1, NX + NU, REAL, ACADO_WORKSPACE);
		qpUb0.setup("qpUb0", 1, NX + NU, REAL, ACADO_WORKSPACE);
	}
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

	ExportVariable evLbXValues("lbXValues", lbXValues, STATIC_CONST_REAL);
	ExportVariable evUbXValues("ubXValues", ubXValues, STATIC_CONST_REAL);

	DVector lbUInf( NU );
	lbUInf.setAll( -INFTY );
	DVector ubUInf( NU );
	ubUInf.setAll( INFTY );

	//
	// Stack control constraints
	//
	for (unsigned i = 0; i < N; ++i)
	{
		lbTmp = uBounds.getLowerBounds( i );
		if ( !lbTmp.getDim() )
			lbUValues.append( lbUInf );
		else
			lbUValues.append( lbTmp );

		ubTmp = uBounds.getUpperBounds( i );
		if ( !ubTmp.getDim() )
			ubUValues.append( ubUInf );
		else
			ubUValues.append( ubTmp );
	}

	ExportVariable evLbUValues("lbUValues", lbUValues, STATIC_CONST_REAL);
	ExportVariable evUbUValues("ubUValues", ubUValues, STATIC_CONST_REAL);

	//
	// Export evaluation of simple box constraints
	//
	evaluateConstraints.setup("evaluateConstraints");
	if( hardcodeConstraintValues == YES ) {
		evaluateConstraints.addVariable( evLbXValues );
		evaluateConstraints.addVariable( evUbXValues );
		evaluateConstraints.addVariable( evLbUValues );
		evaluateConstraints.addVariable( evUbUValues );
	}
	else {
		lbValues.setup("lbValues", 1, N * (NX + NU) + NX, REAL, ACADO_VARIABLES);
		lbValues.setDoc( "Lower bounds values." );
		ubValues.setup("ubValues", 1, N * (NX + NU) + NX, REAL, ACADO_VARIABLES);
		ubValues.setDoc( "Upper bounds values." );

		for( uint i = 0; i < N; i++ ) {
			for( uint j = 0; j < NX; j++ ) {
				initialize.addStatement(lbValues.getCol(i * (NX + NU) + j) == lbXValues(i * NX + j));
				initialize.addStatement(ubValues.getCol(i * (NX + NU) + j) == ubXValues(i * NX + j));
			}
			for( uint j = 0; j < NU; j++ ) {
				initialize.addStatement(lbValues.getCol(i * (NX + NU) + NX + j) == lbUValues(i * NU + j));
				initialize.addStatement(ubValues.getCol(i * (NX + NU) + NX + j) == ubUValues(i * NU + j));
			}
		}
		for( uint j = 0; j < NX; j++ ) {
			initialize.addStatement(lbValues.getCol(N * (NX + NU) + j) == lbXValues(N * NX + j));
			initialize.addStatement(ubValues.getCol(N * (NX + NU) + j) == ubXValues(N * NX + j));
		}
	}

	if (initialStateFixed() == true && hardcodeConstraintValues == YES)
	{
		evaluateConstraints.addStatement(
				qpLb0.getCols(NX, NX + NU) == evLbUValues.getTranspose().getCols(0, NU) - u.getRow( 0 )
		);
		evaluateConstraints.addStatement(
				qpUb0.getCols(NX, NX + NU) == evUbUValues.getTranspose().getCols(0, NU) - u.getRow( 0 )
		);
	}
	else if (initialStateFixed() == true) {
		evaluateConstraints.addStatement(
				qpLb0.getCols(NX, NX + NU) == lbValues.getCols(NX, NX + NU) - u.getRow( 0 )
		);
		evaluateConstraints.addStatement(
				qpUb0.getCols(NX, NX + NU) == ubValues.getCols(NX, NX + NU) - u.getRow( 0 )
		);
	}

	ExportIndex ind( "ind" );
	evaluateConstraints.addIndex( ind );
	ExportForLoop lbLoop(ind, 0, N);
	ExportForLoop ubLoop(ind, 0, N);

	if( hardcodeConstraintValues == YES ) {
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
	}
	else {
		lbLoop.addStatement(
				qpLb.getCols(ind * (NX + NU), ind * (NX + NU) + NX) ==
						lbValues.getCols(ind * (NX + NU), ind * (NX + NU) + NX) - x.getRow( ind )
		);
		lbLoop.addStatement(
				qpLb.getCols(ind * (NX + NU) + NX, (ind + 1) * (NX + NU)) ==
						lbValues.getCols(ind * (NX + NU) + NX, (ind + 1) * (NX + NU)) - u.getRow( ind )
		);

		ubLoop.addStatement(
				qpUb.getCols(ind * (NX + NU), ind * (NX + NU) + NX) ==
						ubValues.getCols(ind * (NX + NU), ind * (NX + NU) + NX) - x.getRow( ind )
		);
		ubLoop.addStatement(
				qpUb.getCols(ind * (NX + NU) + NX, (ind + 1) * (NX + NU)) ==
						ubValues.getCols(ind * (NX + NU) + NX, (ind + 1) * (NX + NU)) - u.getRow( ind )
		);
	}

	evaluateConstraints.addStatement( lbLoop );
	evaluateConstraints.addStatement( ubLoop );
	evaluateConstraints.addLinebreak();

	if( hardcodeConstraintValues == YES ) {
		evaluateConstraints.addStatement(
				qpLb.getCols(N * (NX + NU), N * (NX + NU) + NX) ==
						evLbXValues.getTranspose().getCols(N * NX, (N + 1) * NX) - x.getRow( N )
		);
		evaluateConstraints.addStatement(
				qpUb.getCols(N * (NX + NU), N * (NX + NU) + NX) ==
						evUbXValues.getTranspose().getCols(N * NX, (N + 1) * NX) - x.getRow( N )
		);
	}
	else {
		evaluateConstraints.addStatement(
				qpLb.getCols(N * (NX + NU), N * (NX + NU) + NX) ==
						lbValues.getCols(N * (NX + NU), N * (NX + NU) + NX) - x.getRow( N )
		);
		evaluateConstraints.addStatement(
				qpUb.getCols(N * (NX + NU), N * (NX + NU) + NX) ==
						ubValues.getCols(N * (NX + NU), N * (NX + NU) + NX) - x.getRow( N )
		);
	}
	evaluateConstraints.addLinebreak();

	////////////////////////////////////////////////////////////////////////////
	//
	// Evaluation of the system dynamics equality constraints
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

	//
	// Set QP c vector
	//
	evaluateConstraints.addStatement( qpc == d );
	evaluateConstraints.addLinebreak();

	////////////////////////////////////////////////////////////////////////////
	//
	// Setup evaluation of path and point constraints
	//
	////////////////////////////////////////////////////////////////////////////

	if (getNumComplexConstraints() == 0)
		return SUCCESSFUL_RETURN;
	else if(hardcodeConstraintValues == YES)
		return ACADOERROR( RET_NOT_IMPLEMENTED_YET );

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

	ExportVariable evLbAValues("lbAValues", lbAValues, STATIC_CONST_REAL);
	ExportVariable evUbAValues("ubAValues", ubAValues, STATIC_CONST_REAL);

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
	tPocA.setup("tPocA", conValueOut.getDim(), NX + NU, REAL);
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

returnValue ExportGaussNewtonQpDunes::setupVariables( )
{
	if (initialStateFixed() == true)
	{
		x0.setup("x0",  NX, 1, REAL, ACADO_VARIABLES);
		x0.setDoc( (std::string)"Current state feedback vector." );
	}

	return SUCCESSFUL_RETURN;
}

returnValue ExportGaussNewtonQpDunes::setupMultiplicationRoutines( )
{
	return SUCCESSFUL_RETURN;
}

returnValue ExportGaussNewtonQpDunes::setupEvaluation( )
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
	if( regularizeHessian.isDefined() ) preparation.addFunctionCall( regularizeHessian );
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

	qpPrimal.setup("qpPrimal", N * (NX + NU) + NX, 1, REAL, ACADO_WORKSPACE);
	qpLambda.setup("qpLambda", N * NX, 1, REAL, ACADO_WORKSPACE);
	qpMu.setup("qpMu", 2 * N * (NX + NU) + 2 * NX, 1, REAL, ACADO_WORKSPACE);

	//
	// Calculate objective residuals and call the QP solver
	//
	int sensitivityProp;
	get( DYNAMIC_SENSITIVITY, sensitivityProp );
	bool adjoint = ((ExportSensitivityType) sensitivityProp == BACKWARD);
	if( getNY() > 0 || getNYN() > 0 ) {
		feedback.addStatement( Dy -= y );
		feedback.addLinebreak();
		feedback.addStatement( DyN -= yN );
		feedback.addLinebreak();

		for (unsigned i = 0; i < N; ++i)
			feedback.addFunctionCall(setStagef, qpg.getAddress(i * (NX + NU)), ExportIndex( i ));
		feedback.addStatement( qpg.getRows(N * (NX + NU), N * (NX + NU) + NX) == QN2 * DyN );
		if( adjoint ) feedback.addStatement( qpg.getRows(N * (NX + NU), N * (NX + NU) + NX) += objSlx.getRows(N*NX,N*NX+NX) );
		feedback.addLinebreak();
	}

	if (initialStateFixed() == true)
	{
		feedback.addStatement(
				qpLb0.getTranspose().getRows(0, NX) == x0 - x.getRow( 0 ).getTranspose()
		);
		feedback.addStatement(
				qpUb0.getCols(0, NX) == qpLb0.getCols(0, NX)
		);
	}
	else
	{
		feedback << (qpgN == qpg.getRows(N * (NX + NU), N * (NX + NU) + NX));
	}
	feedback.addLinebreak();

	feedback << returnValueFeedbackPhase.getFullName() << " = solveQpDunes();\n";

	//
	// Here we have to accumulate the differences.
	//

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

	//
	// Pass the multipliers of the dynamic constraints in case of exact Hessian SQP.
	//
	int hessianApproximation;
	get( HESSIAN_APPROXIMATION, hessianApproximation );
	bool secondOrder = ((HessianApproximationMode)hessianApproximation == EXACT_HESSIAN);
	if( secondOrder )	feedback.addStatement( mu.makeColVector() == qpLambda );

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
	// Setup evaluation of the KKT tolerance
	//
	////////////////////////////////////////////////////////////////////////////
	ExportVariable kkt("kkt", 1, 1, REAL, ACADO_LOCAL, true);
	ExportVariable prd("prd", 1, 1, REAL, ACADO_LOCAL, true);
	ExportIndex index2( "index2" );

	getKKT.setup( "getKKT" );
	getKKT.doc( "Get the KKT tolerance of the current iterate." );
	kkt.setDoc( "KKT tolerance." );
	getKKT.setReturnValue( kkt );
//	getKKT.addVariable( prd );
	getKKT.addIndex( index );
	getKKT.addIndex( index2 );

	getKKT.addStatement( kkt == (qpg ^ qpPrimal) );
	getKKT << kkt.getFullName() << " = fabs( " << kkt.getFullName() << " );\n";

	ExportForLoop lamLoop(index, 0, N * NX);
	lamLoop << kkt.getFullName() << "+= fabs( " << d.get(index, 0) << " * " << qpLambda.get(index, 0) << ");\n";
	getKKT.addStatement( lamLoop );

	/*

	lambda are the multipliers of the coupling constraints
	i.e. lambda_i for x_{i+1} = A * x_i + B * u_i + c
	mu correspond to the bounds
	in the fashion
	mu = mu_0 … mu_N
	i.e. major ordering by the stages
	within each stage i
	i.e. within mu_i
	we have the minor ordering( I drop the i here)
	lb z_0, ub z_0, lb z_1, ub z_1, … lb z_nZ, ub z_nZ
	where z are the stage variables in the ordering z = [x u]
	signs are positive if active, zero if inactive

	 */

	if ( getNumComplexConstraints() )
	{
		ACADOWARNINGTEXT(RET_NOT_IMPLEMENTED_YET,
				"KKT Tolerance with affine stage constraints is under development");
		return SUCCESSFUL_RETURN;
	}

	if (initialStateFixed() == true)
	{
		for (unsigned el = 0; el < NX + NU; ++el)
		{
			getKKT << kkt.getFullName() << " += fabs("
				   << qpLb0.get(0, el) << " * " << qpMu.get(2 * el + 0, 0)  << ");\n";
			getKKT << kkt.getFullName() << " += fabs("
				   << qpUb0.get(0, el) << " * " << qpMu.get(2 * el + 1, 0)  << ");\n";
		}
	}

	ExportForLoop bndLoop(index, initialStateFixed() ? 1 : 0, N);
	ExportForLoop bndInLoop(index2, 0, NX + NU);
	bndInLoop << kkt.getFullName() << " += fabs("
			 << qpLb.get(0, index * (NX + NU) + index2) << " * " << qpMu.get(index * 2 * (NX + NU) + 2 * index2 + 0, 0)  << ");\n";
	bndInLoop << kkt.getFullName() << " += fabs("
			 << qpUb.get(0, index * (NX + NU) + index2) << " * " << qpMu.get(index * 2 * (NX + NU) + 2 * index2 + 1, 0)  << ");\n";
	bndLoop.addStatement( bndInLoop );
	getKKT.addStatement( bndLoop );

	for (unsigned el = 0; el < NX; ++el)
	{
		getKKT << kkt.getFullName() << " += fabs("
			   << qpLb.get(0, N * (NX + NU) + el) << " * " << qpMu.get(N * 2 * (NX + NU) + 2 * el + 0, 0)  << ");\n";
		getKKT << kkt.getFullName() << " += fabs("
			   << qpUb.get(0, N * (NX + NU) + el) << " * " << qpMu.get(N * 2 * (NX + NU) + 2 * el + 1, 0)  << ");\n";
	}

	return SUCCESSFUL_RETURN;
}

returnValue ExportGaussNewtonQpDunes::setupQPInterface( )
{
	//
	// Configure and export QP interface
	//

	qpInterface = std::shared_ptr< ExportQpDunesInterface >(new ExportQpDunesInterface("", commonHeaderName));

	int maxNumQPiterations;
	get(MAX_NUM_QP_ITERATIONS, maxNumQPiterations);

	// XXX If not specified, use default value
	if ( maxNumQPiterations <= 0 )
		maxNumQPiterations = getNumQPvars();

	int printLevel;
	get(PRINTLEVEL, printLevel);

	if ( (PrintLevel)printLevel >= MEDIUM )
		printLevel = 2;
	else
		printLevel = 0;

	qpInterface->configure(
			maxNumQPiterations,
			printLevel,
			qpH.getFullName(),
			qpg.getFullName(),
			initialStateFixed() ? "0" : qpgN.getFullName(),
			qpC.getFullName(),
			qpc.getFullName(),
			qpA.getFullName(),
			initialStateFixed() ? qpLb0.getFullName() : "0",
			initialStateFixed() ? qpUb0.getFullName() : "0",
			qpLb.getFullName(),
			qpUb.getFullName(),
			qpLbA.getFullName(),
			qpUbA.getFullName(),
			qpPrimal.getFullName(),
			qpLambda.getFullName(),
			qpMu.getFullName(),
			qpConDim,
			initialStateFixed() ? "1" : "0",
			diagonalH ? "1" : "0",
			diagonalHN ? "1" : "0",
			N, NX, NU
	);

	return SUCCESSFUL_RETURN;
}

CLOSE_NAMESPACE_ACADO
