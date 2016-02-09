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
 *    \file src/code_generation/export_exact_hessian_qpdunes.cpp
 *    \author Rien Quirynen
 *    \date 2014
 */

#include <acado/code_generation/export_exact_hessian_qpdunes.hpp>
#include <acado/code_generation/export_qpdunes_interface.hpp>

BEGIN_NAMESPACE_ACADO

using namespace std;

ExportExactHessianQpDunes::ExportExactHessianQpDunes(	UserInteraction* _userInteraction,
													const std::string& _commonHeaderName
													) : ExportGaussNewtonQpDunes( _userInteraction,_commonHeaderName )
{}

returnValue ExportExactHessianQpDunes::setup( )
{
	LOG( LVL_DEBUG ) << "Solver: setup initialization... " << endl;
	setupInitialization();

	//
	// Add QP initialization call to the initialization
	//
	initialize << "for( ret = 0; ret < ACADO_N*(ACADO_NX+ACADO_NU)*(ACADO_NX+ACADO_NU)+ACADO_NX*ACADO_NX; ret++ )  acadoWorkspace.qpH[ret] = 1.0;\n";  // TODO: this is added because of a bug in qpDUNES !!
	ExportFunction initializeQpDunes( "initializeQpDunes" );
	initialize
		<< "ret = (int)initializeQpDunes();\n"
		<< "if ((return_t)ret != QPDUNES_OK) return ret;\n";

	cleanup.setup( "cleanupSolver" );
	ExportFunction cleanupQpDunes( "cleanupQpDunes" );
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

	LOG( LVL_DEBUG ) << "Solver: setup hessian regularization... " << endl;
	setupHessianRegularization();
	LOG( LVL_DEBUG ) << "done!" << endl;

	LOG( LVL_DEBUG ) << "Solver: setup Evaluation... " << endl;
	setupEvaluation();
	LOG( LVL_DEBUG ) << "done!" << endl;

	LOG( LVL_DEBUG ) << "Solver: setup setupAuxiliaryFunctions... " << endl;
	setupAuxiliaryFunctions();
	LOG( LVL_DEBUG ) << "done!" << endl;

	return SUCCESSFUL_RETURN;
}

returnValue ExportExactHessianQpDunes::getFunctionDeclarations(	ExportStatementBlock& declarations
																) const
{
	ExportGaussNewtonQpDunes::getFunctionDeclarations( declarations );

	declarations.addDeclaration( regularization );

	return SUCCESSFUL_RETURN;
}

returnValue ExportExactHessianQpDunes::getCode(	ExportStatementBlock& code
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

	code.addFunction( regularizeHessian );

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

//
// PROTECTED FUNCTIONS:
//

returnValue ExportExactHessianQpDunes::setupObjectiveEvaluation( void )
{
	evaluateObjective.setup("evaluateObjective");

	//
	// A loop the evaluates objective and corresponding gradients
	//
	ExportIndex runObj( "runObj" );
	ExportForLoop loopObjective( runObj, 0, N );

	evaluateObjective.addIndex( runObj );

	// Interface variable to qpDUNES
	qpH.setup("qpH", N * (NX + NU) * (NX + NU) + NX * NX, 1, REAL, ACADO_WORKSPACE);   // --> to be used only after regularization to pass to qpDUNES
	qpg.setup("qpG", N * (NX + NU) + NX, 1, REAL, ACADO_WORKSPACE);

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

	ExportVariable stagef;
	stagef.setup("stagef", NX + NU, 1, REAL, ACADO_LOCAL);

	ExportVariable stageH;
	stageH.setup("stageH", NX + NU, NX + NU, REAL, ACADO_LOCAL);

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

		setStageH.setup("addObjTerm", tmpFxx, tmpFxu, tmpFuu, stageH);
		setStageH.addStatement( stageH.getSubMatrix(0,NX,0,NX) += tmpFxx + evLmX );
		setStageH.addStatement( stageH.getSubMatrix(0,NX,NX,NX+NU) += tmpFxu );
		setStageH.addStatement( stageH.getSubMatrix(NX,NX+NU,0,NX) += tmpFxu.getTranspose() );
		setStageH.addStatement( stageH.getSubMatrix(NX,NX+NU,NX,NX+NU) += tmpFuu + evLmU );

		loopObjective.addFunctionCall(
				setStageH, objValueOut.getAddress(0, 1+NX+NU), objValueOut.getAddress(0, 1+NX+NU+NX*NX),
				objValueOut.getAddress(0, 1+NX+NU+NX*(NX+NU)), objS.getAddress(runObj*(NX+NU), 0) );

		ExportVariable tmpDF;
		tmpDF.setup("tmpDF", NX+NU, 1, REAL, ACADO_LOCAL);
		setStagef.setup("addObjLinearTerm", tmpDF, stagef);
		setStagef.addStatement( stagef == tmpDF.getRows(0,NX+NU) );

		loopObjective.addFunctionCall(
				setStagef, objValueOut.getAddress(0, 1), qpg.getAddress(runObj * (NX+NU)) );

		loopObjective.addLinebreak( );
	}
	else {
		if(levenbergMarquardt > 0.0) {
			setStageH.setup("addObjTerm", stageH);
			setStageH.addStatement( stageH.getSubMatrix(0,NX,0,NX) += evLmX );
			setStageH.addStatement( stageH.getSubMatrix(NX,NX+NU,NX,NX+NU) += evLmU );

			loopObjective.addFunctionCall( setStageH, objS.getAddress(runObj*(NX+NU), 0) );
		}
		DMatrix D(NX+NU,1); D.setAll(0);
		loopObjective.addStatement( qpg.getRows(runObj*(NX+NU), runObj*(NX+NU)+NX+NU) == D );
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

		evaluateObjective.addStatement( objSEndTerm.makeRowVector() == objValueOut.getCols(1+NX,1+NX+NX*NX) + evLmX.makeRowVector() );
		evaluateObjective.addStatement( qpg.getRows(N * NX, (N + 1) * NX) == objValueOut.getCols(1,1+NX).getTranspose() );

		evaluateObjective.addLinebreak( );
	}
	else {
		if(levenbergMarquardt > 0.0) {
			evaluateObjective.addStatement( objSEndTerm == evLmX );
		}
		else {
			DMatrix hess(NX,NX); hess.setAll(0);
			evaluateObjective.addStatement( objSEndTerm == hess );
		}

		DMatrix Dx(NX,1); Dx.setAll(0);
		evaluateObjective.addStatement( qpg.getRows(N*NX, (N+1)*NX) == Dx );
	}

	return SUCCESSFUL_RETURN;
}

returnValue ExportExactHessianQpDunes::setupHessianRegularization( )
{
    string moduleName;
	get(CG_MODULE_NAME, moduleName);
    
	ExportVariable block( "hessian_block", NX+NU, NX+NU );
	regularization = ExportFunction( moduleName + "_regularize", block );
	regularization.doc( "EVD-based regularization of a Hessian block." );
	regularization.addLinebreak();

	regularizeHessian.setup( "regularizeHessian" );
	regularizeHessian.doc( "Regularization procedure of the computed exact Hessian." );

	ExportIndex oInd;
	regularizeHessian.acquire( oInd );

	ExportForLoop loopObjective(oInd, 0, N);
	loopObjective.addFunctionCall( regularization, objS.getAddress(oInd*(NX+NU),0) );
	for( uint row = 0; row < NX+NU; row++ ) {
		loopObjective.addStatement( qpH.getRows((oInd*(NX+NU)+row)*(NX+NU),(oInd*(NX+NU)+row+1)*(NX+NU)) == objS.getRow(oInd*(NX+NU)+row).getTranspose() );
	}
	regularizeHessian.addStatement( loopObjective );

	regularizeHessian.addStatement( qpH.getRows(N*(NX+NU)*(NX+NU), N*(NX+NU)*(NX+NU)+NX*NX) == objSEndTerm.makeColVector() );

	return SUCCESSFUL_RETURN;
}

CLOSE_NAMESPACE_ACADO
