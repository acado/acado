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
 *    \file src/code_generation/export_gauss_newton_block_qpdunes.cpp
 *    \author Rien Quirynen
 *    \date 2014
 */

#include <acado/code_generation/export_gauss_newton_block_qpdunes.hpp>

using namespace std;

BEGIN_NAMESPACE_ACADO

ExportGaussNewtonBlockQpDunes::ExportGaussNewtonBlockQpDunes(	UserInteraction* _userInteraction,
											const std::string& _commonHeaderName
											) : ExportGaussNewtonBlockCN2( _userInteraction,_commonHeaderName )
{}

returnValue ExportGaussNewtonBlockQpDunes::setup( )
{

	returnValue status = ExportGaussNewtonBlockCN2::setup();
	if( status != SUCCESSFUL_RETURN ) return status;

	LOG( LVL_DEBUG ) << "Solver: setup extra initialization... " << endl;
	//
	// Add QP initialization call to the initialization
	//
	stringstream ss;
	ss.str( string() );
	ss << "for( ret = 0; ret < " << getNumberOfBlocks()*getNumBlockVariables()*getNumBlockVariables()+NX*NX << "; ret++ )  acadoWorkspace.qpH[ret] += 1e-8;\n";  // TODO: this is added because of a bug in qpDUNES !!
	initialize << ss.str();
	initialize << "ret = (int)initializeQpDunes();\n"
	<< "if ((return_t)ret != QPDUNES_OK) return ret;\n";

	cleanup.setup( "cleanupSolver" );
	ExportFunction cleanupQpDunes( "cleanupQpDunes" );
	cleanupQpDunes.setName( "cleanupQpDunes" );
	cleanup.addFunctionCall( cleanupQpDunes );
	LOG( LVL_DEBUG ) << "done!" << endl;

	return SUCCESSFUL_RETURN;
}

returnValue ExportGaussNewtonBlockQpDunes::getCode(	ExportStatementBlock& code
											)
{
	setupQPInterface();
	code.addStatement( *qpInterface );

	code.addFunction( cleanup );
	code.addFunction( shiftQpData );

	code.addFunction( evaluateConstraints );

	return ExportGaussNewtonCN2::getCode( code );
}

returnValue ExportGaussNewtonBlockQpDunes::setupEvaluation( )
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
	ExportIndex index("index");
	preparation.addIndex( index );

	preparation	<< retSim.getFullName() << " = " << modelSimulation.getName() << "();\n";

	preparation.addFunctionCall( evaluateObjective );
	if( regularizeHessian.isDefined() ) preparation.addFunctionCall( regularizeHessian );
	preparation.addFunctionCall( evaluateConstraints );

	preparation.addLinebreak();
	preparation << (Dy -= y) << (DyN -= yN);
	preparation.addLinebreak();

	ExportForLoop condensePrepLoop( index, 0, getNumberOfBlocks() );
	condensePrepLoop.addFunctionCall( condensePrep, index );
	preparation.addStatement( condensePrepLoop );

	preparation.addStatement( qpH.getRows(getNumberOfBlocks()*getNumBlockVariables()*getNumBlockVariables(),getNumberOfBlocks()*getNumBlockVariables()*getNumBlockVariables()+NX*NX) == QN1.makeColVector() );
	if( levenbergMarquardt > ZERO_EPS ) {
		for( uint i = 0; i < NX; i++ ) {
			preparation.addStatement( qpH.getElement(getNumberOfBlocks()*getNumBlockVariables()*getNumBlockVariables()+i*NX+i,0) += levenbergMarquardt );
		}
	}
	preparation.addLinebreak();

	preparation.addStatement( g.getRows(getNumberOfBlocks()*getNumBlockVariables(), getNumQPvars()) == QN2 * DyN );
	int variableObjS;
	get(CG_USE_VARIABLE_WEIGHTING_MATRIX, variableObjS);
	ExportVariable SlxCall =
				objSlx.isGiven() == true || variableObjS == false ? objSlx : objSlx.getRows(N * NX, (N + 1) * NX);
	preparation.addStatement( g.getRows(getNumberOfBlocks()*getNumBlockVariables(), getNumQPvars()) += SlxCall );
	preparation.addLinebreak();

	stringstream prep;
	prep << retSim.getName() << " = prepareQpDunes( );" << endl;
	preparation << prep.str();
	preparation.addLinebreak();

	////////////////////////////////////////////////////////////////////////////
	//
	// Feedback phase
	//
	////////////////////////////////////////////////////////////////////////////

	ExportVariable tmp("tmp", 1, 1, INT, ACADO_LOCAL, true);
	tmp.setDoc( "Status code of the qpOASES QP solver." );

	feedback.setup("feedbackStep");
	feedback.doc( "Feedback/estimation step of the RTI scheme." );
	feedback.setReturnValue( tmp );
	feedback.addIndex( index );

	if (initialStateFixed() == true)
	{
		feedback.addStatement( qpLb0.getTranspose().getRows(0, NX) == x0 - x.getRow( 0 ).getTranspose() );
		feedback.addStatement( qpUb0.getCols(0, NX) == qpLb0.getCols(0, NX) );
	}
	else
	{
		feedback << (qpgN == g.getRows(getNumberOfBlocks()*getNumBlockVariables(), getNumQPvars()));
	}
	feedback.addLinebreak();

	stringstream s;
	s << tmp.getName() << " = solveQpDunes( );" << endl;
	feedback <<  s.str();
	feedback.addLinebreak();

	ExportForLoop expandLoop( index, 0, getNumberOfBlocks() );
	expandLoop.addFunctionCall( expand, index );
	feedback.addStatement( expandLoop );

	feedback.addStatement( (x.getRow(getNumberOfBlocks()*getBlockSize())).getTranspose() += xVars.getRows(getNumberOfBlocks()*getNumBlockVariables(), getNumberOfBlocks()*getNumBlockVariables()+NX) );

	////////////////////////////////////////////////////////////////////////////
	//
	// Shifting of QP data
	//
	////////////////////////////////////////////////////////////////////////////

	stringstream ss;
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

	getKKT.addStatement( kkt == (g ^ xVars) );
	getKKT << kkt.getFullName() << " = fabs( " << kkt.getFullName() << " );\n";

	ExportForLoop lamLoop(index, 0, getNumberOfBlocks() * NX);
	lamLoop << kkt.getFullName() << "+= fabs( " << qpc.get(index, 0) << " * " << qpLambda.get(index, 0) << ");\n";
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
		for (unsigned el = 0; el < getNumBlockVariables(); ++el)
		{
			getKKT << kkt.getFullName() << " += fabs("
					<< qpLb0.get(0, el) << " * " << qpMu.get(2 * el + 0, 0)  << ");\n";
			getKKT << kkt.getFullName() << " += fabs("
					<< qpUb0.get(0, el) << " * " << qpMu.get(2 * el + 1, 0)  << ");\n";
		}
	}

	ExportForLoop bndLoop(index, initialStateFixed() ? 1 : 0, getNumberOfBlocks());
	ExportForLoop bndInLoop(index2, 0, getNumBlockVariables());
	bndInLoop << kkt.getFullName() << " += fabs("
			<< lb.get(index * getNumBlockVariables() + index2, 0) << " * " << qpMu.get(index * 2 * getNumBlockVariables() + 2 * index2 + 0, 0)  << ");\n";
	bndInLoop << kkt.getFullName() << " += fabs("
			<< ub.get(index * getNumBlockVariables() + index2, 0) << " * " << qpMu.get(index * 2 * getNumBlockVariables() + 2 * index2 + 1, 0)  << ");\n";
	bndLoop.addStatement( bndInLoop );
	getKKT.addStatement( bndLoop );

	for (unsigned el = 0; el < NX; ++el)
	{
		getKKT << kkt.getFullName() << " += fabs("
				<< lb.get(getNumberOfBlocks() * getNumBlockVariables() + el, 0) << " * " << qpMu.get(2 * getNumberOfBlocks() * getNumBlockVariables() + 2 * el + 0, 0)  << ");\n";
		getKKT << kkt.getFullName() << " += fabs("
				<< ub.get(getNumberOfBlocks() * getNumBlockVariables() + el, 0) << " * " << qpMu.get(2 * getNumberOfBlocks() * getNumBlockVariables() + 2 * el + 1, 0)  << ");\n";
	}

	return SUCCESSFUL_RETURN;
}

returnValue ExportGaussNewtonBlockQpDunes::setupQPInterface( )
{
		//
		// Configure and export QP interface
		//

		qpInterface = std::shared_ptr< ExportSplitQpDunesInterface >(new ExportSplitQpDunesInterface("", commonHeaderName));

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
				g.getFullName(),
				initialStateFixed() ? "0" : qpgN.getFullName(),
				qpC.getFullName(),
				qpc.getFullName(),
				A.getFullName(),
				initialStateFixed() ? qpLb0.getFullName() : "0",
				initialStateFixed() ? qpUb0.getFullName() : "0",
				lb.getFullName(),
				ub.getFullName(),
				lbA.getFullName(),
				ubA.getFullName(),
				xVars.getFullName(),
				qpLambda.getFullName(),
				qpMu.getFullName(),
				qpConDim,
				initialStateFixed() ? "1" : "0",
				diagonalH ? "1" : "0",
				diagonalHN ? "1" : "0",
				getNumberOfBlocks(),
				NX,
				getBlockSize()*NU
		);

	return SUCCESSFUL_RETURN;
}


CLOSE_NAMESPACE_ACADO
