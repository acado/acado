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
#include <acado/code_generation/export_qpoases3_interface.hpp>

using namespace std;

BEGIN_NAMESPACE_ACADO

ExportExactHessianCN2::ExportExactHessianCN2(	UserInteraction* _userInteraction,
											const std::string& _commonHeaderName
											) : ExportGaussNewtonCN2( _userInteraction,_commonHeaderName )
{}

returnValue ExportExactHessianCN2::setup( )
{
	std::cout << "NOTE: You are using the new (unstable) N2 condensing feature for exact Hessian based RTI..\n";

	if (performFullCondensing() == true && initialStateFixed() == false)
		return ACADOERRORTEXT( RET_INVALID_OPTION, "Impossible to perform full condensing, when the initial state is not fixed. You can use regular condensing instead." );
	if (performFullCondensing() == false && initialStateFixed() == true)
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
	ExportGaussNewtonCN2::getFunctionDeclarations( declarations );

	declarations.addDeclaration( regularization );

	return SUCCESSFUL_RETURN;
}

//
// PROTECTED FUNCTIONS:
//

returnValue ExportExactHessianCN2::setupObjectiveEvaluation( void )
{
	evaluateObjective.setup("evaluateObjective");

	int gradientUp;
	get( LIFTED_GRADIENT_UPDATE, gradientUp );
	bool gradientUpdate = (bool) gradientUp;

	//
	// A loop the evaluates objective and corresponding gradients
	//
	ExportIndex runObj( "runObj" );
	ExportForLoop loopObjective( runObj, 0, N );

	evaluateObjective.addIndex( runObj );

	unsigned offset = performFullCondensing() == true ? 0 : NX;

	int sensitivityProp;
	get( DYNAMIC_SENSITIVITY, sensitivityProp );
	bool adjoint = ((ExportSensitivityType) sensitivityProp == BACKWARD);

	if( evaluateStageCost.getFunctionDim() > 0 ) {
		loopObjective.addStatement( objValueIn.getCols(0, getNX()) == x.getRow( runObj ) );
		loopObjective.addStatement( objValueIn.getCols(NX, NX + NU) == u.getRow( runObj ) );
		loopObjective.addStatement( objValueIn.getCols(NX + NU, NX + NU + NOD) == od.getRow( runObj ) );
		loopObjective.addLinebreak( );

		// Evaluate the objective function
		loopObjective.addFunctionCall(evaluateStageCost, objValueIn, objValueOut);
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

		if( gradientUpdate || adjoint ) {
			loopObjective.addStatement( objValueOut.getCols(1,1+NX+NU) += objg.getRows(runObj*(NX+NU),(runObj+1)*(NX+NU)).getTranspose() );
		}
		loopObjective.addFunctionCall(
				setObjR1R2, QDy.getAddress(runObj * NX), g.getAddress(offset+runObj * NU, 0), objValueOut.getAddress(0, 1) );

		loopObjective.addLinebreak( );
	}
	else if( gradientUpdate || adjoint ) {
		loopObjective.addStatement( QDy.getRows(runObj*NX, runObj*NX+NX) == objg.getRows(runObj*(NX+NU),runObj*(NX+NU)+NX) );
		loopObjective.addStatement( g.getRows(offset+runObj*NU, offset+runObj*NU+NU) == objg.getRows(runObj*(NX+NU)+NX,(runObj+1)*(NX+NU)) );
	}
	else {
		DMatrix Du(NU,1); Du.setAll(0);
		DMatrix Dx(NX,1); Dx.setAll(0);
		loopObjective.addStatement( g.getRows(offset+runObj*NU, offset+runObj*NU+NU) == Du );
		loopObjective.addStatement( QDy.getRows(runObj*NX, runObj*NX+NX) == Dx );
	}

	evaluateObjective.addStatement( loopObjective );

	//
	// Evaluate the quadratic Mayer term
	//
	if( evaluateTerminalCost.getFunctionDim() > 0 ) {
		evaluateObjective.addStatement( objValueIn.getCols(0, NX) == x.getRow( N ) );
		evaluateObjective.addStatement( objValueIn.getCols(NX, NX + NOD) == od.getRow( N ) );

		// Evaluate the objective function, last node.
		evaluateObjective.addFunctionCall(evaluateTerminalCost, objValueIn, objValueOut);
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
		evaluateObjective.addStatement(objSEndTerm == hess);

		DMatrix Dx(NX,1); Dx.setAll(0);
		evaluateObjective.addStatement( QDy.getRows(N * NX, (N + 1) * NX) == Dx );
	}

	return SUCCESSFUL_RETURN;
}

returnValue ExportExactHessianCN2::setupHessianRegularization( )
{
    string moduleName;
	get(CG_MODULE_NAME, moduleName);
    
	ExportVariable block( "hessian_block", NX+NU, NX+NU );
	regularization = ExportFunction( "regularize", block );
	regularization.doc( "EVD-based regularization of a Hessian block." );
	regularization.addLinebreak();

	regularizeHessian.setup( "regularizeHessian" );
	regularizeHessian.doc( "Regularization procedure of the computed exact Hessian." );

	int hessianRegularization;
	get( HESSIAN_REGULARIZATION, hessianRegularization );

	ExportIndex oInd;
	regularizeHessian.acquire( oInd );

	ExportForLoop loopObjective(oInd, 0, N);
	if( (HessianRegularizationMode) hessianRegularization == BLOCK_REG ) {
		loopObjective.addFunctionCall( regularization, objS.getAddress(oInd*(NX+NU),0) );
	}
	loopObjective.addStatement( Q1.getRows(oInd*NX, oInd*NX+NX) == objS.getSubMatrix(oInd*(NX+NU), oInd*(NX+NU)+NX, 0, NX) );
	loopObjective.addStatement( S1.getRows(oInd*NX, oInd*NX+NX) == objS.getSubMatrix(oInd*(NX+NU), oInd*(NX+NU)+NX, NX, NX+NU) );
	loopObjective.addStatement( R1.getRows(oInd*NU, oInd*NU+NU) == objS.getSubMatrix(oInd*(NX+NU)+NX, oInd*(NX+NU)+NX+NU, NX, NX+NU) );
	regularizeHessian.addStatement( loopObjective );

	regularizeHessian.addStatement( QN1 == objSEndTerm );

	return SUCCESSFUL_RETURN;
}

CLOSE_NAMESPACE_ACADO
