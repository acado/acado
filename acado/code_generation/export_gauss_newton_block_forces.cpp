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
 *    \file src/code_generation/export_gauss_newton_block_forces.cpp
 *    \author Rien Quirynen
 *    \date 2014
 */

#include <acado/code_generation/export_gauss_newton_block_forces.hpp>

#include <acado/code_generation/templates/templates.hpp>

using namespace std;

BEGIN_NAMESPACE_ACADO

ExportGaussNewtonBlockForces::ExportGaussNewtonBlockForces(	UserInteraction* _userInteraction,
											const std::string& _commonHeaderName
											) : ExportGaussNewtonBlockCN2( _userInteraction,_commonHeaderName )
{
	qpObjPrefix = "acadoForces";
	qpModuleName = "forces";
}

returnValue ExportGaussNewtonBlockForces::setup( )
{
	returnValue status = ExportGaussNewtonBlockCN2::setup();
	if( status != SUCCESSFUL_RETURN ) return status;

	LOG( LVL_DEBUG ) << "Solver: setup extra initialization... " << endl;
	// Add QP initialization call to the initialization
	ExportFunction initializeForces( "initializeForces" );
	initializeForces.setName( "initializeForces" );
	initialize.addFunctionCall( initializeForces );
	LOG( LVL_DEBUG ) << "done!" << endl;

	return SUCCESSFUL_RETURN;
}

returnValue ExportGaussNewtonBlockForces::getCode(	ExportStatementBlock& code
											)
{
	setupQPInterface();
	code.addStatement( *qpInterface );

	code.addFunction( cleanup );
	code.addFunction( shiftQpData );

	code.addFunction( evaluateConstraints );
	code.addFunction( evaluateAffineConstraints );

	return ExportGaussNewtonCN2::getCode( code );
}

returnValue ExportGaussNewtonBlockForces::setupCondensing( void )
{
	returnValue status = ExportGaussNewtonBlockCN2::setupCondensing();
	if( status != SUCCESSFUL_RETURN ) return status;

	objHessians.clear();
	objHessians.resize(getNumberOfBlocks() + 1);
	for (unsigned i = 0; i < getNumberOfBlocks(); ++i)
	{
		objHessians[ i ].setup(string("H") + toString(i + 1), getNumBlockVariables(), getNumBlockVariables(), REAL, FORCES_PARAMS, false, qpObjPrefix);
	}
	objHessians[ getNumberOfBlocks() ].setup(string("H") + toString(getNumberOfBlocks() + 1), NX, NX, REAL, FORCES_PARAMS, false, qpObjPrefix);


	objGradients.clear();
	objGradients.resize(getNumberOfBlocks() + 1);
	for (unsigned i = 0; i < getNumberOfBlocks(); ++i)
	{
		objGradients[ i ].setup(string("f") + toString(i + 1), getNumBlockVariables(), 1, REAL, FORCES_PARAMS, false, qpObjPrefix);
	}
	objGradients[ getNumberOfBlocks() ].setup(string("f") + toString(getNumberOfBlocks() + 1), NX, 1, REAL, FORCES_PARAMS, false, qpObjPrefix);


	conC.clear();
	conC.resize( getNumberOfBlocks() );
	// XXX FORCES works with column major format
	for (unsigned i = 0; i < getNumberOfBlocks(); ++i)
		conC[ i ].setup(string("C") + toString(i + 1), getNumBlockVariables(), NX, REAL, FORCES_PARAMS, false, qpObjPrefix);

	cond.clear();
	unsigned dNum = initialStateFixed() == true ? getNumberOfBlocks() + 1 : getNumberOfBlocks();
	cond.resize(dNum);
	for (unsigned i = 0; i < dNum; ++i)
		cond[ i ].setup(string("d") + toString(i + 1), NX, 1, REAL, FORCES_PARAMS, false, qpObjPrefix);


	// TODO: SET HESSIAN AND GRADIENT INFORMATION
	for (unsigned i = 0; i < getNumberOfBlocks(); ++i)
		condensePrep.addStatement(objHessians[ i ].makeColVector() == qpH.getRows(i*getNumBlockVariables()*getNumBlockVariables(),(i+1)*getNumBlockVariables()*getNumBlockVariables()));
	condensePrep.addLinebreak();

	for (unsigned i = 0; i < getNumberOfBlocks(); ++i)
		condensePrep.addStatement(objGradients[ i ] == g.getRows(i*getNumBlockVariables(),(i+1)*getNumBlockVariables()));
	condensePrep.addLinebreak();


	// TODO: SET EQUALITY CONSTRAINT VALUES
	for (unsigned i = 0; i < getNumberOfBlocks(); ++i)
		condensePrep.addStatement(conC[ i ] == qpC.getRows(i*NX,(i+1)*NX).getTranspose());
	condensePrep.addLinebreak();

	if( initialStateFixed() ) {
		for (unsigned i = 1; i < dNum; ++i)
			condensePrep.addStatement(cond[ i ] == zeros<double>(NX,1) - qpc.getRows((i-1)*NX,i*NX)); // TODO:CHECK THE SIGN
	}
	else {
		for (unsigned i = 0; i < dNum; ++i)
			condensePrep.addStatement(cond[ i ] == zeros<double>(NX,1) - qpc.getRows(i*NX,(i+1)*NX)); // TODO:CHECK THE SIGN
	}
	condensePrep.addLinebreak();


	// TODO: REWRITE EXPAND ROUTINE
	unsigned offset = performFullCondensing() == true ? 0 : NX;
	LOG( LVL_DEBUG ) << "Setup condensing: rewrite expand routine" << endl;
	ExportVariable stageOut("stageOut", getNumBlockVariables(), 1, REAL, ACADO_LOCAL);
	expand = ExportFunction( "expand", stageOut, blockI );

	for (unsigned i = 0; i < getBlockSize(); ++i ) {
		expand.addStatement( (u.getRow(blockI*getBlockSize()+i)).getTranspose() += stageOut.getRows(NX+i*NU, NX+(i+1)*NU) );
	}

	expand.addStatement( sbar.getRows(0, NX) == stageOut.getRows(0, NX) );
	expand.addStatement( (x.getRow(blockI*getBlockSize())).getTranspose() += sbar.getRows(0, NX) );

	if( getBlockSize() > 1 ) {
		expand.addStatement( sbar.getRows(NX, getBlockSize()*NX) == d.getRows(blockI*getBlockSize()*NX,(blockI+1)*getBlockSize()*NX-NX) );
	}

	for (unsigned row = 0; row < getBlockSize()-1; ++row ) {
		expand.addFunctionCall(
				expansionStep, evGx.getAddress((blockI*getBlockSize()+row) * NX), evGu.getAddress((blockI*getBlockSize()+row) * NX),
				stageOut.getAddress(offset + row * NU), sbar.getAddress(row * NX),
				sbar.getAddress((row + 1) * NX)
		);
		expand.addStatement( (x.getRow(blockI*getBlockSize()+row+1)).getTranspose() += sbar.getRows((row+1)*NX, (row+2)*NX) );
	}

	// !! TODO: Calculation of multipliers: !!


	return SUCCESSFUL_RETURN;
}

returnValue ExportGaussNewtonBlockForces::setupConstraintsEvaluation( void )
{
	returnValue status = ExportGaussNewtonBlockCN2::setupConstraintsEvaluation();
	if( status != SUCCESSFUL_RETURN ) return status;

	////////////////////////////////////////////////////////////////////////////
	//
	// Setup evaluation of box constraints on states and controls
	//
	////////////////////////////////////////////////////////////////////////////

	conLB.clear();
	conLB.resize(getNumberOfBlocks() + 1);

	conUB.clear();
	conUB.resize(getNumberOfBlocks() + 1);

	conA.clear();
	conA.resize(getNumberOfBlocks());

	conAB.clear();
	conAB.resize(getNumberOfBlocks());

	conLBIndices.clear();
	conLBIndices.resize(getNumberOfBlocks() + 1);

	conUBIndices.clear();
	conUBIndices.resize(getNumberOfBlocks() + 1);

	conABDimensions.clear();
	conABDimensions.resize(getNumberOfBlocks() + 1);

	DVector lbTmp, ubTmp;

	//
	// Stack state constraints
	//
	for (unsigned i = 0; i < getNumberOfBlocks(); ++i)
	{
		for (unsigned k = 0; k < getBlockSize(); ++k)
		{
			lbTmp = xBounds.getLowerBounds( i*getBlockSize()+k );
			ubTmp = xBounds.getUpperBounds( i*getBlockSize()+k );

			if (isFinite( lbTmp ) == false && isFinite( ubTmp ) == false)
				continue;

			for (unsigned j = 0; j < lbTmp.getDim(); ++j)
			{
				if (acadoIsFinite( lbTmp( j ) ) == true)
				{
					if( k == 0 ) {
						conLBIndices[ i ].push_back( j );
					}
				}

				if (acadoIsFinite( ubTmp( j ) ) == true)
				{
					if( k == 0 ) {
						conUBIndices[ i ].push_back( j );
					}
				}
			}
		}
		conABDimensions[ i ] = 2*getNumStateBoundsPerBlock();
	}
	conABDimensions[ getNumberOfBlocks() ] = 0;

	lbTmp = xBounds.getLowerBounds( N );
	ubTmp = xBounds.getUpperBounds( N );
	for (unsigned j = 0; j < lbTmp.getDim(); ++j)
	{
		if (acadoIsFinite( lbTmp( j ) ) == true)
		{
			conLBIndices[ getNumberOfBlocks() ].push_back( j );
		}

		if (acadoIsFinite( ubTmp( j ) ) == true)
		{
			conUBIndices[ getNumberOfBlocks() ].push_back( j );
		}
	}

	//
	// Stack control constraints
	//
	for (unsigned i = 0; i < getNumberOfBlocks(); ++i)
	{
		for (unsigned k = 0; k < getBlockSize(); ++k)
		{
			lbTmp = uBounds.getLowerBounds( i*getBlockSize()+k );
			ubTmp = uBounds.getUpperBounds( i*getBlockSize()+k );

			if (isFinite( lbTmp ) == false && isFinite( ubTmp ) == false)
				continue;

			for (unsigned j = 0; j < lbTmp.getDim(); ++j)
			{
				if (acadoIsFinite( lbTmp( j ) ) == true)
				{
					conLBIndices[ i ].push_back(NX + k*NU + j);
				}

				if (acadoIsFinite( ubTmp( j ) ) == true)
				{
					conUBIndices[ i ].push_back(NX + k*NU + j);
				}
			}
		}
	}

	//
	// Setup variables
	//
	for (unsigned i = 0; i < getNumberOfBlocks() + 1; ++i)
	{
		conLB[ i ].setup(string("lb") + toString(i + 1), conLBIndices[ i ].size(), 1, REAL, FORCES_PARAMS, false, qpObjPrefix);
		conUB[ i ].setup(string("ub") + toString(i + 1), conUBIndices[ i ].size(), 1, REAL, FORCES_PARAMS, false, qpObjPrefix);
	}

	//
	// FORCES evaluation of simple box constraints
	//
	for (unsigned i = 0; i < getNumberOfBlocks() + 1; ++i) {
		for (unsigned j = 0; j < conLBIndices[ i ].size(); ++j)
		{
			evaluateConstraints << conLB[ i ].getFullName() << "[ " << toString(j) << " ]" << " = " << lb.get( i*getNumBlockVariables()+conLBIndices[ i ][ j ],0 ) << ";\n";
		}
	}
	evaluateConstraints.addLinebreak();

	for (unsigned i = 0; i < getNumberOfBlocks() + 1; ++i)
		for (unsigned j = 0; j < conUBIndices[ i ].size(); ++j)
		{
			evaluateConstraints << conUB[ i ].getFullName() << "[ " << toString(j) << " ]" << " = " << ub.get( i*getNumBlockVariables()+conUBIndices[ i ][ j ],0 ) << ";\n";
		}
	evaluateConstraints.addLinebreak();

	//
	// Setup variables
	//
	for (unsigned i = 0; i < getNumberOfBlocks(); ++i)
	{
		conA[ i ].setup(string("A") + toString(i + 1), getNumBlockVariables(), conABDimensions[ i ], REAL, FORCES_PARAMS, false, qpObjPrefix);	// XXX FORCES works with column major format
		conAB[ i ].setup(string("Ab") + toString(i + 1), conABDimensions[ i ], 1, REAL, FORCES_PARAMS, false, qpObjPrefix);
	}


	evaluateAffineConstraints.setup("evaluateAffineConstraints");
	//
	// FORCES evaluation for the affine constraints after condensing
	//
	for (unsigned i = 0; i < getNumberOfBlocks(); ++i) {
		evaluateAffineConstraints.addStatement( conA[ i ].getCols(0,getNumStateBoundsPerBlock()) == A.getRows(i*getNumStateBoundsPerBlock(),(i+1)*getNumStateBoundsPerBlock()).getTranspose() );
		evaluateAffineConstraints.addStatement( conA[ i ].getCols(getNumStateBoundsPerBlock(),conABDimensions[ i ]) == zeros<double>(getNumBlockVariables(),getNumStateBoundsPerBlock())-A.getRows(i*getNumStateBoundsPerBlock(),(i+1)*getNumStateBoundsPerBlock()).getTranspose() );

		evaluateAffineConstraints.addStatement( conAB[ i ].getRows(0,getNumStateBoundsPerBlock()) == ubA.getRows(i*getNumStateBoundsPerBlock(),(i+1)*getNumStateBoundsPerBlock()) );
		evaluateAffineConstraints.addStatement( conAB[ i ].getRows(getNumStateBoundsPerBlock(),conABDimensions[ i ]) == zeros<double>(getNumStateBoundsPerBlock(),1)-lbA.getRows(i*getNumStateBoundsPerBlock(),(i+1)*getNumStateBoundsPerBlock()) );
	}

	return SUCCESSFUL_RETURN;
}

returnValue ExportGaussNewtonBlockForces::setupVariables( )
{
	returnValue status = ExportGaussNewtonBlockCN2::setupVariables();
	if( status != SUCCESSFUL_RETURN ) return status;

	xVars.setup("x", getNumQPvars(), 1, REAL, ACADO_LOCAL); // NOT USED
	yVars.setup("",0,0); // NOT USED

	return SUCCESSFUL_RETURN;
}

returnValue ExportGaussNewtonBlockForces::setupEvaluation( )
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

	preparation.addFunctionCall( evaluateAffineConstraints );

	preparation.addStatement( objHessians[getNumberOfBlocks()] == QN1 );
	DMatrix mReg = eye<double>( getNX() );
	mReg *= levenbergMarquardt;
	preparation.addStatement( objHessians[getNumberOfBlocks()] += mReg );
	preparation.addLinebreak();

	preparation.addStatement( objGradients[ getNumberOfBlocks() ] == QN2 * DyN );
	int variableObjS;
	get(CG_USE_VARIABLE_WEIGHTING_MATRIX, variableObjS);
	ExportVariable SlxCall =
				objSlx.isGiven() == true || variableObjS == false ? objSlx : objSlx.getRows(N * NX, (N + 1) * NX);
	preparation.addStatement( objGradients[ getNumberOfBlocks() ] += SlxCall );
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
		feedback.addStatement( cond[ 0 ] == x0 - x.getRow( 0 ).getTranspose() );
	}
	feedback.addLinebreak();

	//
	// Configure output variables
	//
	std::vector< ExportVariable > vecQPVars;

	vecQPVars.clear();
	vecQPVars.resize(getNumberOfBlocks() + 1);
	for (unsigned i = 0; i < getNumberOfBlocks(); ++i)
		vecQPVars[ i ].setup(string("out") + toString(i + 1), getNumBlockVariables(), 1, REAL, FORCES_OUTPUT, false, qpObjPrefix);
	vecQPVars[ getNumberOfBlocks() ].setup(string("out") + toString(getNumberOfBlocks() + 1), NX, 1, REAL, FORCES_OUTPUT, false, qpObjPrefix);

	//
	// In case warm starting is enabled, give an initial guess, based on the old solution
	//
	int hotstartQP;
	get(HOTSTART_QP, hotstartQP);

	if ( hotstartQP )
	{
		return ACADOERROR(RET_NOT_IMPLEMENTED_YET);
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
	<< tmp.getFullName() << " = "
	<< qpModuleName << "_" << solveQP.getName() << "( "
	<< "&" << qpObjPrefix << "_" << "params" << ", "
	<< "&" << qpObjPrefix << "_" << "output" << ", "
	<< "&" << qpObjPrefix << "_" << "info" << " );\n";
	feedback.addLinebreak();

	for (unsigned i = 0; i < getNumberOfBlocks(); ++i) {
		feedback.addFunctionCall( expand, vecQPVars[i], ExportIndex(i) );
	}

	feedback.addStatement( x.getRow( N ) += vecQPVars[ getNumberOfBlocks() ].getTranspose() );
	feedback.addLinebreak();

	////////////////////////////////////////////////////////////////////////////
	//
	// Setup evaluation of the KKT tolerance
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

returnValue ExportGaussNewtonBlockForces::setupQPInterface( )
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
			getBlockSize()*NU,
			getNumberOfBlocks(),
			conLBIndices,
			conUBIndices,
			conABDimensions,
			(Q1.isGiven() == true && R1.isGiven() == true) ? 1 : 0,
					diagonalH,
					diagonalHN,
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
//	ACADOWARNINGTEXT(RET_NOT_IMPLEMENTED_YET,
//			"A python code generator interface for block condensing with FORCES is under development.");

	return SUCCESSFUL_RETURN;
}


CLOSE_NAMESPACE_ACADO
