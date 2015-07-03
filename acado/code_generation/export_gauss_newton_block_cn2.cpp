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
 *    \file src/code_generation/export_gauss_newton_block_cn2.cpp
 *    \author Rien Quirynen
 *    \date 2014
 */

#include <acado/code_generation/export_gauss_newton_block_cn2.hpp>

using namespace std;

BEGIN_NAMESPACE_ACADO

ExportGaussNewtonBlockCN2::ExportGaussNewtonBlockCN2(	UserInteraction* _userInteraction,
											const std::string& _commonHeaderName
											) : ExportGaussNewtonCN2( _userInteraction,_commonHeaderName )
{}

returnValue ExportGaussNewtonBlockCN2::setup( )
{
	diagonalH = false;
	diagonalHN = false;

	LOG( LVL_DEBUG ) << "Condensing block size: " << getBlockSize() << endl;
	LOG( LVL_DEBUG ) << "Number of blocks     : " << getNumberOfBlocks() << endl;
	LOG( LVL_DEBUG ) << "# of variables each  : " << getNumBlockVariables() << endl;
	LOG( LVL_DEBUG ) << "# of QP variables    : " << getNumQPvars() << " / " << N*(NX+NU)+NX << endl;

	if (N/getBlockSize() != getNumberOfBlocks())
		return ACADOERRORTEXT(RET_INVALID_ARGUMENTS, "The condensing block size needs to be a divisor of the horizon length.");
	if (getBlockSize() == 0 || getBlockSize() > N)
		return ACADOERRORTEXT(RET_INVALID_ARGUMENTS, "The condensing block size needs to be a divisor of the horizon length.");
	if (getNumComplexConstraints() > 0)
		return ACADOERROR( RET_NOT_IMPLEMENTED_YET );
	if (performsSingleShooting() == true)
		return ACADOERROR( RET_NOT_IMPLEMENTED_YET );

	LOG( LVL_DEBUG ) << "Solver: setup initialization... " << endl;
	setupInitialization();
	LOG( LVL_DEBUG ) << "done!" << endl;

	LOG( LVL_DEBUG ) << "Solver: setup variables... " << endl;
	setupVariables();
	LOG( LVL_DEBUG ) << "done!" << endl;

	LOG( LVL_DEBUG ) << "# of state bounds per block: " << getNumStateBoundsPerBlock() << endl;

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

returnValue ExportGaussNewtonBlockCN2::getDataDeclarations(	ExportStatementBlock& declarations,
															ExportStruct dataStruct
															) const
{
	returnValue status;
	status = ExportGaussNewtonCN2::getDataDeclarations(declarations, dataStruct);
	if (status != SUCCESSFUL_RETURN)
		return status;

	declarations.addDeclaration(qpgN, dataStruct);
	declarations.addDeclaration(qpLb0, dataStruct);
	declarations.addDeclaration(qpUb0, dataStruct);
	declarations.addDeclaration(qpC, dataStruct);
	declarations.addDeclaration(qpc, dataStruct);
	declarations.addDeclaration(qpH, dataStruct);

	declarations.addDeclaration(qpLambda, dataStruct);
	declarations.addDeclaration(qpMu, dataStruct);

	return SUCCESSFUL_RETURN;
}

returnValue ExportGaussNewtonBlockCN2::getFunctionDeclarations(	ExportStatementBlock& declarations
															) const
{
	returnValue status;
	status = ExportGaussNewtonCN2::getFunctionDeclarations(declarations);
	if (status != SUCCESSFUL_RETURN)
		return status;

	declarations.addDeclaration( cleanup );
	declarations.addDeclaration( shiftQpData );

	return SUCCESSFUL_RETURN;
}


unsigned ExportGaussNewtonBlockCN2::getNumQPvars( ) const
{

	return getNumberOfBlocks()*getNumBlockVariables() + NX;
}

//
// PROTECTED FUNCTIONS:
//


returnValue ExportGaussNewtonBlockCN2::setupConstraintsEvaluation( void )
{
	ExportVariable tmp("tmp", 1, 1, REAL, ACADO_LOCAL, true);

	int hardcodeConstraintValues;
	get(CG_HARDCODE_CONSTRAINT_VALUES, hardcodeConstraintValues);

	////////////////////////////////////////////////////////////////////////////
	//
	// Setup the bounds on control variables
	//
	////////////////////////////////////////////////////////////////////////////

	if (initialStateFixed() == true)
	{
		qpLb0.setup("qpLb0", 1, NX + getBlockSize()*NU, REAL, ACADO_WORKSPACE);
		qpUb0.setup("qpUb0", 1, NX + getBlockSize()*NU, REAL, ACADO_WORKSPACE);
	}

	DVector lbTmp, ubTmp;
	DVector lbXValues, ubXValues;
	DVector lbXAValues(getNumberOfBlocks()*getNumStateBoundsPerBlock()), ubXAValues(getNumberOfBlocks()*getNumStateBoundsPerBlock());
	DVector lbUValues, ubUValues;

	DVector lbXInf( NX );
	lbXInf.setAll( -INFTY );
	DVector ubXInf( NX );
	ubXInf.setAll( INFTY );

	//
	// Stack state bounds
	//
	unsigned index = 0;
	unsigned numStateBounds = 0;
	if( getNumStateBoundsPerBlock() ) qpConDim.resize(getNumberOfBlocks() + 1, 0);
	for (unsigned i = 0; i < getNumberOfBlocks(); ++i) {
		lbTmp = xBounds.getLowerBounds( i*getBlockSize() );
		if ( !lbTmp.getDim() )
			lbXValues.append( lbXInf );
		else
			lbXValues.append( lbTmp );

		ubTmp = xBounds.getUpperBounds( i*getBlockSize() );
		if ( !ubTmp.getDim() )
			ubXValues.append( ubXInf );
		else
			ubXValues.append( ubTmp );

		while( index < xBoundsIdx.size() && xBoundsIdx[index] < (i*getBlockSize()+1)*NX ) { // SIMPLE BOUNDS
			index += 1;
		}
		for (unsigned j = 1; j < getBlockSize(); ++j) {
			lbTmp = xBounds.getLowerBounds( i*getBlockSize()+j );
			ubTmp = xBounds.getUpperBounds( i*getBlockSize()+j );
			while( index < xBoundsIdx.size() && xBoundsIdx[index] < (i*getBlockSize()+1+j)*NX ) {
				unsigned offset = (i*getBlockSize()+j)*NX;
				lbXAValues(numStateBounds) = lbTmp(xBoundsIdx[index]-offset);
				ubXAValues(numStateBounds) = ubTmp(xBoundsIdx[index]-offset);

				numStateBounds += 1;
				index += 1;
			}
		}
		if( getNumStateBoundsPerBlock() ) qpConDim[ i ] = getNumStateBoundsPerBlock();
	}
	lbTmp = xBounds.getLowerBounds( N );
	if ( !lbTmp.getDim() )
		lbXValues.append( lbXInf );
	else
		lbXValues.append( lbTmp );

	ubTmp = xBounds.getUpperBounds( N );
	if ( !ubTmp.getDim() )
		ubXValues.append( ubXInf );
	else
		ubXValues.append( ubTmp );

	ASSERT( numStateBounds == getNumStateBoundsPerBlock()*getNumberOfBlocks() );

	ExportVariable evLbXAValues("lbXAValues", lbXAValues, STATIC_CONST_REAL);
	ExportVariable evUbXAValues("ubXAValues", ubXAValues, STATIC_CONST_REAL);
	if( hardcodeConstraintValues == YES ) {
		condensePrep.addVariable( evLbXAValues );
		condensePrep.addVariable( evUbXAValues );
	}
	else {
		lbAValues.setup("lbAValues", 1, lbXAValues.getDim(), REAL, ACADO_VARIABLES);
		lbAValues.setDoc( "Lower state bounds values." );
		ubAValues.setup("ubAValues", 1, ubXAValues.getDim(), REAL, ACADO_VARIABLES);
		ubAValues.setDoc( "Upper state bounds values." );

		initialize.addStatement(lbAValues == lbXAValues.transpose());
		initialize.addStatement(ubAValues == ubXAValues.transpose());
	}

	// Add the constraint evaluations to the condensePrep function
	index = 0;
	numStateBounds = 0;
	while( index < xBoundsIdx.size() && xBoundsIdx[index] < NX ) {
		index += 1;
	}
	for (unsigned j = 1; j < getBlockSize(); ++j) {
		while( index < xBoundsIdx.size() && xBoundsIdx[index] < (1+j)*NX ) {
			if( hardcodeConstraintValues == YES ) {
				condensePrep.addStatement( lbA.getRow(blockI*getNumStateBoundsPerBlock()+numStateBounds) == evLbXAValues.getRow(blockI*getNumStateBoundsPerBlock()+numStateBounds) - x.getElement( blockI*getBlockSize()+j, xBoundsIdx[index]-j*NX ) );
			}
			else {
				condensePrep.addStatement( lbA.getRow(blockI*getNumStateBoundsPerBlock()+numStateBounds) == lbAValues.getCol(blockI*getNumStateBoundsPerBlock()+numStateBounds) - x.getElement( blockI*getBlockSize()+j, xBoundsIdx[index]-j*NX ) );
			}
			condensePrep.addStatement( lbA.getRow(blockI*getNumStateBoundsPerBlock()+numStateBounds) -= sbar.getRow(xBoundsIdx[index]) );

			if( hardcodeConstraintValues == YES ) {
				condensePrep.addStatement( ubA.getRow(blockI*getNumStateBoundsPerBlock()+numStateBounds) == evUbXAValues.getRow(blockI*getNumStateBoundsPerBlock()+numStateBounds) - x.getElement( blockI*getBlockSize()+j, xBoundsIdx[index]-j*NX ) );
			}
			else {
				condensePrep.addStatement( ubA.getRow(blockI*getNumStateBoundsPerBlock()+numStateBounds) == ubAValues.getCol(blockI*getNumStateBoundsPerBlock()+numStateBounds) - x.getElement( blockI*getBlockSize()+j, xBoundsIdx[index]-j*NX ) );
			}
			condensePrep.addStatement( ubA.getRow(blockI*getNumStateBoundsPerBlock()+numStateBounds) -= sbar.getRow(xBoundsIdx[index]) );

			condensePrep.addStatement( A.getSubMatrix(blockI*getNumStateBoundsPerBlock()+numStateBounds,blockI*getNumStateBoundsPerBlock()+numStateBounds+1,0,NX) == C.getRow(xBoundsIdx[index]-NX) );

			for (unsigned i = 0; i < j; ++i) { // loop over the controls within one block
				int offset = i * (2 * getBlockSize() - i + 1) / 2;
				condensePrep.addStatement( A.getSubMatrix(blockI*getNumStateBoundsPerBlock()+numStateBounds,blockI*getNumStateBoundsPerBlock()+numStateBounds+1,NX+i*NU,NX+(i+1)*NU) == E.getRow((offset-i-1)*NX+xBoundsIdx[index]) );
			}
			for (unsigned i = j; i < getBlockSize(); ++i) { // loop over the controls within one block
				condensePrep.addStatement( A.getSubMatrix(blockI*getNumStateBoundsPerBlock()+numStateBounds,blockI*getNumStateBoundsPerBlock()+numStateBounds+1,NX+i*NU,NX+(i+1)*NU) == zeros<double>(1,NU) );
			}

			numStateBounds += 1;
			index += 1;
		}
	}
	ASSERT( numStateBounds == getNumStateBoundsPerBlock() );


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
		lbValues.setup("lbValues", 1, lbXValues.getDim()+lbUValues.getDim(), REAL, ACADO_VARIABLES);
		lbValues.setDoc( "Lower bounds values." );
		ubValues.setup("ubValues", 1, ubXValues.getDim()+ubUValues.getDim(), REAL, ACADO_VARIABLES);
		ubValues.setDoc( "Upper bounds values." );

		for( uint i = 0; i < getNumberOfBlocks(); i++ ) {
			for( uint j = 0; j < NX; j++ ) {
				initialize.addStatement(lbValues.getCol(i * getNumBlockVariables() + j) == lbXValues(i * NX + j));
				initialize.addStatement(ubValues.getCol(i * getNumBlockVariables() + j) == ubXValues(i * NX + j));
			}
			for( uint j = 0; j < getBlockSize()*NU; j++ ) {
				initialize.addStatement(lbValues.getCol(i * getNumBlockVariables() + NX + j) == lbUValues(i * getBlockSize()*NU + j));
				initialize.addStatement(ubValues.getCol(i * getNumBlockVariables() + NX + j) == ubUValues(i * getBlockSize()*NU + j));
			}
		}
		for( uint j = 0; j < NX; j++ ) {
			initialize.addStatement(lbValues.getCol(getNumberOfBlocks()*getNumBlockVariables() + j) == lbXValues(getNumberOfBlocks() * NX + j));
			initialize.addStatement(ubValues.getCol(getNumberOfBlocks()*getNumBlockVariables() + j) == ubXValues(getNumberOfBlocks() * NX + j));
		}
	}

	if (initialStateFixed() == true)
	{
		for (unsigned i = 0; i < getBlockSize(); ++i) {
			if( hardcodeConstraintValues == YES ) {
				evaluateConstraints.addStatement( qpLb0.getCols(NX + i*NU, NX + (i+1)*NU) == evLbUValues.getTranspose().getCols(i*NU, (i+1)*NU) - u.getRow( i ) );
				evaluateConstraints.addStatement( qpUb0.getCols(NX + i*NU, NX + (i+1)*NU) == evUbUValues.getTranspose().getCols(i*NU, (i+1)*NU) - u.getRow( i ) );
			}
			else {
				evaluateConstraints.addStatement( qpLb0.getCols(NX + i*NU, NX + (i+1)*NU) == lbValues.getCols(NX + i*NU, NX + (i+1)*NU) - u.getRow( i ) );
				evaluateConstraints.addStatement( qpUb0.getCols(NX + i*NU, NX + (i+1)*NU) == ubValues.getCols(NX + i*NU, NX + (i+1)*NU) - u.getRow( i ) );
			}
		}
		evaluateConstraints.addStatement( lb.getRows(0,getNumBlockVariables()) == qpLb0.getTranspose() );
		evaluateConstraints.addStatement( ub.getRows(0,getNumBlockVariables()) == qpUb0.getTranspose() );
	}
	uint firstBlock = (initialStateFixed() == true) ? 1 : 0;

	ExportIndex iBlock( "iBlock" );
	ExportIndex uInd( "uInd" ), offset1( "offset1" ), offset2( "offset2" );
	evaluateConstraints.addIndex( iBlock );
	evaluateConstraints.addIndex( uInd );
	evaluateConstraints.addIndex( offset1 );
	if( hardcodeConstraintValues == YES ) evaluateConstraints.addIndex( offset2 );
	ExportForLoop blockLoop(iBlock, firstBlock, getNumberOfBlocks());
	ExportForLoop uLoop(uInd, 0, getBlockSize());

	if( hardcodeConstraintValues == YES ) {
		blockLoop.addStatement(
				lb.getRows(iBlock*getNumBlockVariables(), iBlock*getNumBlockVariables()+NX) ==
						evLbXValues.getRows(iBlock*NX, (iBlock + 1) * NX) - x.getRow( iBlock*getBlockSize() ).getTranspose()
		);
		blockLoop.addStatement(
				ub.getRows(iBlock*getNumBlockVariables(), iBlock*getNumBlockVariables()+NX) ==
						evUbXValues.getRows(iBlock*NX, (iBlock + 1) * NX) - x.getRow( iBlock*getBlockSize() ).getTranspose()
		);
	}
	else {
		blockLoop.addStatement(
				lb.getRows(iBlock*getNumBlockVariables(), iBlock*getNumBlockVariables()+NX) ==
						lbValues.getTranspose().getRows(iBlock*getNumBlockVariables(), iBlock*getNumBlockVariables()+NX) - x.getRow( iBlock*getBlockSize() ).getTranspose()
		);
		blockLoop.addStatement(
				ub.getRows(iBlock*getNumBlockVariables(), iBlock*getNumBlockVariables()+NX) ==
						ubValues.getTranspose().getRows(iBlock*getNumBlockVariables(), iBlock*getNumBlockVariables()+NX) - x.getRow( iBlock*getBlockSize() ).getTranspose()
		);
	}

	uLoop.addStatement( offset1 == iBlock*getNumBlockVariables()+uInd*NU+NX );

	if( hardcodeConstraintValues == YES ) {
		uLoop.addStatement( offset2 == iBlock*getBlockSize()*NU+uInd*NU );
		uLoop.addStatement(
				lb.getRows(offset1, offset1+NU) ==
						evLbUValues.getRows(offset2, offset2+NU) - u.getRow( iBlock*getBlockSize()+uInd ).getTranspose()
		);
		uLoop.addStatement(
				ub.getRows(offset1, offset1+NU) ==
						evUbUValues.getRows(offset2, offset2+NU) - u.getRow( iBlock*getBlockSize()+uInd ).getTranspose()
		);
	}
	else {
		uLoop.addStatement(
				lb.getRows(offset1, offset1+NU) ==
						lbValues.getTranspose().getRows(offset1, offset1+NU) - u.getRow( iBlock*getBlockSize()+uInd ).getTranspose()
		);
		uLoop.addStatement(
				ub.getRows(offset1, offset1+NU) ==
						ubValues.getTranspose().getRows(offset1, offset1+NU) - u.getRow( iBlock*getBlockSize()+uInd ).getTranspose()
		);
	}

	blockLoop.addStatement( uLoop );
	evaluateConstraints.addStatement( blockLoop );
	evaluateConstraints.addLinebreak();

	if( hardcodeConstraintValues == YES ) {
		evaluateConstraints.addStatement(
				lb.getRows(getNumberOfBlocks()*getNumBlockVariables(), getNumberOfBlocks()*getNumBlockVariables()+NX) ==
						evLbXValues.getRows(getNumberOfBlocks()*NX, (getNumberOfBlocks() + 1) * NX) - x.getRow( N ).getTranspose()
		);
		evaluateConstraints.addStatement(
				ub.getRows(getNumberOfBlocks()*getNumBlockVariables(), getNumberOfBlocks()*getNumBlockVariables()+NX) ==
						evUbXValues.getRows(getNumberOfBlocks()*NX, (getNumberOfBlocks() + 1) * NX) - x.getRow( N ).getTranspose()
		);
	}
	else {
		evaluateConstraints.addStatement(
				lb.getRows(getNumberOfBlocks()*getNumBlockVariables(), getNumberOfBlocks()*getNumBlockVariables()+NX) ==
						lbValues.getTranspose().getRows(getNumberOfBlocks()*getNumBlockVariables(), getNumberOfBlocks()*getNumBlockVariables()+NX) - x.getRow( N ).getTranspose()
		);
		evaluateConstraints.addStatement(
				ub.getRows(getNumberOfBlocks()*getNumBlockVariables(), getNumberOfBlocks()*getNumBlockVariables()+NX) ==
						ubValues.getTranspose().getRows(getNumberOfBlocks()*getNumBlockVariables(), getNumberOfBlocks()*getNumBlockVariables()+NX) - x.getRow( N ).getTranspose()
		);
	}
	evaluateConstraints.addLinebreak();

	////////////////////////////////////////////////////////////////////////////
	//
	// Setup evaluation of path and point constraints
	//
	////////////////////////////////////////////////////////////////////////////

	// TODO


	return SUCCESSFUL_RETURN;
}

returnValue ExportGaussNewtonBlockCN2::setupCondensing( void )
{
	condensePrep.setup("condensePrep", blockI);

	////////////////////////////////////////////////////////////////////////////
	//
	// Setup local memory for preparation phase: T1, T2, W1, W2
	//
	////////////////////////////////////////////////////////////////////////////

	// TODO

	////////////////////////////////////////////////////////////////////////////
	//
	// Compute Hessian block H00, H10 and C
	//
	////////////////////////////////////////////////////////////////////////////

	if (performFullCondensing() == false)
	{
		LOG( LVL_DEBUG ) << "Setup condensing: H00, H10 and C" << endl;

		if( getBlockSize() > 1 ) {
			T1.setup("T1", NX, NX, REAL, ACADO_WORKSPACE);
			T2.setup("T2", NX, NX, REAL, ACADO_WORKSPACE);
		}

		condensePrep.addFunctionCall(moveGxT, evGx.getAddress(blockI*getBlockSize()*NX), C.getAddress(0, 0));
		for (unsigned row = 1; row < getBlockSize(); ++row)
			condensePrep.addFunctionCall(
					multGxGx, evGx.getAddress((blockI*getBlockSize()+row)*NX), C.getAddress((row - 1) * NX), C.getAddress(row * NX));


		/* Algorithm for computation of H10 and H00

		T1 = Q_N * C_{N - 1}
		for i = N - 1: 1
		 	H_{i + 1, 0} =  B_i^T * T1
		 	H_{i + 1, 0} += S_i^T * C_{i - 1}

			T2 = A_i^T * T1
			T1 = T2 + Q_i * C_{i - 1}

		H_{1, 0} =  B_0^T * T1
		H_{1, 0} += S_0^T
		H_{0, 0} = Q_0 + A_0^T * T1

		 */

		// H10 Block
		condensePrep.addStatement( T1 == zeros<double>(NX,NX) ); // Because you never have a "terminal stage cost" for any of the blocks which are condensed!

		for (unsigned row = getBlockSize() - 1; row > 0; --row)
		{
			condensePrep.addFunctionCall(mult_BT_T1, evGu.getAddress((blockI*getBlockSize()+row)*NX), T1, ExportIndex( row ));

			if ((S1.isGiven() && S1.getGivenMatrix().isZero() == false) || S1.isGiven() == false)
			{
				ExportArgument S1Call = S1.isGiven() == false ? S1.getAddress((blockI*getBlockSize()+row)*NX) : S1;
				condensePrep.addFunctionCall(mac_ST_C, S1Call, C.getAddress((row - 1) * NX), ExportIndex( row ));
			}

			condensePrep.addFunctionCall(multGxTGx, evGx.getAddress((blockI*getBlockSize()+row)*NX), T1, T2);

			ExportArgument Q1Call = Q1.isGiven() == true ? Q1 : Q1.getAddress((blockI*getBlockSize()+row)*NX);
			condensePrep.addFunctionCall(macGxTGx, T2, Q1Call, C.getAddress((row - 1) * NX), T1);
		}

		if( getBlockSize() > 1 ) {
			condensePrep.addFunctionCall(mult_BT_T1, evGu.getAddress( blockI*getBlockSize()*NX ), T1, ExportIndex( 0 ));
		}
		if ((S1.isGiven() && S1.getGivenMatrix().isZero() == false) || S1.isGiven() == false)
		{
			condensePrep.addStatement(
					H.getSubMatrix(NX, NX + NU, 0, NX) += S1.getSubMatrix(blockI*getBlockSize()*NX, blockI*getBlockSize()*NX+NX, 0, NU).getTranspose()
			);
		}

		// H00 Block
		DMatrix mRegH00 = eye<double>( getNX() );
		mRegH00 *= levenbergMarquardt;
		ExportVariable Q1Var = Q1.isGiven() == true ? Q1 : Q1.getSubMatrix(blockI*getBlockSize()*NX, blockI*getBlockSize()*NX+NX, 0, NX);

		if( getBlockSize() > 1 ) {
			condensePrep.addStatement( H.getSubMatrix(0, NX, 0, NX) == Q1Var + (evGx.getSubMatrix(blockI*getBlockSize()*NX, blockI*getBlockSize()*NX+NX, 0, NX).getTranspose() * T1) );
		}
		else {
			condensePrep.addStatement( H.getSubMatrix(0, NX, 0, NX) == Q1Var );
		}
		condensePrep.addStatement( H.getSubMatrix(0, NX, 0, NX) += mRegH00 );
	}

	////////////////////////////////////////////////////////////////////////////
	//
	// Compute Hessian block H11
	//
	////////////////////////////////////////////////////////////////////////////

	LOG( LVL_DEBUG ) << "Setup condensing: H11 and E" << endl;

	/*

	This one is only for the case where we have input constraints only

	// Create E and H

	for i = 0: N - 1
	{
		// Storage for E: (N x nx) x nu

		E_0 = B_i;
		for k = 1: N - i - 1
			E_k = A_{i + k} * E_{k - 1};

		// Two temporary matrices W1, W2 of size nx x nu

		W1 = Q_N^T * E_{N - i - 1};

		for k = N - 1: i + 1
		{
			H_{k, i} = B_k^T * W1;

			W2 = A_k^T * W1;
			W1 = Q_k^T * E_{k - i - 1} + W2;
		}
		H_{i, i} = B_i^T * W1 + R_i^T
	}

	Else, in general case:

	for i = 0: N - 1
	{
		// Storage for E: (N * (N + 1) / 2 x nx) x nu

		j = 1 / 2 * i * (2 * N - i + 1);

		E_j = B_i;
		for k = 1: N - i - 1
			E_{j + k} = A_{i + k} * E_{j + k - 1};

		// Two temporary matrices W1, W2 of size nx x nu

		W1 = Q_N^T * E_{j + N - i - 1};

		for k = N - 1: i + 1
		{
			H_{k, i}  = B_k^T * W1;
			H_{k, i} += S_k^T * E_{j + k - i - 1};

			W2 = A_k^T * W1;
			W1 = Q_k^T * E_{j + k - i - 1} + W2;
		}
		H_{i, i} = B_i^T * W1 + R_i^T
	}

	 */

	if( getBlockSize() > 1 ) {
		W1.setup("W1", NX, NU, REAL, ACADO_WORKSPACE);
		W2.setup("W2", NX, NU, REAL, ACADO_WORKSPACE);
	}

	if (getBlockSize() <= 15)
	{
		for (unsigned col = 0; col < getBlockSize(); ++col)
		{
			int offset = col * (2 * getBlockSize() - col + 1) / 2;

			condensePrep.addComment( "Column: " + toString( col ) );
			condensePrep.addFunctionCall(
					moveGuE, evGu.getAddress((blockI*getBlockSize()+col) * NX), E.getAddress(offset * NX)
			);
			for (unsigned row = 1; row < getBlockSize() - col; ++row)
				condensePrep.addFunctionCall(
						multGxGu,
						evGx.getAddress((blockI*getBlockSize()+col + row) * NX),
						E.getAddress((offset + row - 1) * NX), E.getAddress((offset + row) * NX)
				);
			condensePrep.addLinebreak();

			condensePrep.addStatement( W1 == zeros<double>(NX,NU) ); // Because you never have a "terminal stage cost" for any of the blocks which are condensed!


			for (unsigned row = getBlockSize() - 1; col < row; --row)
			{
				condensePrep.addFunctionCall(
						multBTW1, evGu.getAddress((blockI*getBlockSize()+row) * NX), W1,
						ExportIndex( row ), ExportIndex( col )
				);

				if ((S1.isGiven() && S1.getGivenMatrix().isZero() == false) || S1.isGiven() == false)
				{
					ExportArgument S1Call = S1.isGiven() == false ? S1.getAddress((blockI*getBlockSize()+row) * NX) : S1;

					condensePrep.addFunctionCall(
							mac_S1T_E,
							S1Call, E.getAddress((offset + row - col - 1) * NX),
							ExportIndex( row ), ExportIndex( col )
					);
				}

				condensePrep.addFunctionCall(
						multGxTGu, evGx.getAddress((blockI*getBlockSize()+row) * NX), W1, W2
				);

				ExportArgument Q1Call = Q1.isGiven() == true ? Q1 : Q1.getAddress((blockI*getBlockSize()+row) * NX);
				condensePrep.addFunctionCall(
						macQEW2, Q1Call, E.getAddress((offset + row - col - 1) * NX), W2, W1
				);
			}

			if( getBlockSize() > 1 ) {
				ExportArgument R1Call = R1.isGiven() == true ? R1 : R1.getAddress((blockI*getBlockSize()+col) * NU);
				condensePrep.addFunctionCall(
					macBTW1_R1, R1Call, evGu.getAddress((blockI*getBlockSize()+col) * NX), W1,
					ExportIndex( col ) );
			}
			else {
				ExportVariable R1Var = R1.isGiven() == true ? R1 : R1.getRows((blockI*getBlockSize()) * NU, (blockI*getBlockSize()+1) * NU);
				DMatrix mRegH11 = eye<double>( getNU() );
				mRegH11 *= levenbergMarquardt;
				condensePrep.addStatement( H.getSubMatrix(NX, NX+NU, NX, NX+NU) == R1Var + mRegH11 );
			}


			condensePrep.addLinebreak();
		}
	}
	else
	{
		// Long horizons

		ExportIndex row, col, offset;
		condensePrep.acquire( row ).acquire( col ).acquire( offset );

		ExportForLoop cLoop(col, 0, getBlockSize());
		ExportForLoop fwdLoop(row, 1, getBlockSize() - col);
		ExportForLoop adjLoop(row, getBlockSize() - 1, col, -1);

		cLoop.addStatement( offset == col * (2 * getBlockSize() + 1 - col) / 2 );
		cLoop.addFunctionCall(
				moveGuE, evGu.getAddress((blockI*getBlockSize()+col) * NX), E.getAddress(offset * NX)
		);

		fwdLoop.addFunctionCall(
				multGxGu,
				evGx.getAddress((blockI*getBlockSize() + col + row) * NX),
				E.getAddress((offset + row - 1) * NX), E.getAddress((offset + row) * NX)
		);
		cLoop.addStatement( fwdLoop );
		cLoop.addLinebreak();

		cLoop.addStatement( W1 == zeros<double>(NX,NU) ); // Because you never have a "terminal stage cost" for any of the blocks which are condensed!

		adjLoop.addFunctionCall(
				multBTW1, evGu.getAddress((blockI*getBlockSize() + row) * NX), W1,
				row, col
		);

		if ((S1.isGiven() && S1.getGivenMatrix().isZero() == false) || S1.isGiven() == false)
		{
			ExportArgument S1Call = S1.isGiven() == false ? S1.getAddress((blockI*getBlockSize() + row) * NX) : S1;

			adjLoop.addFunctionCall(
					mac_S1T_E,
					S1Call, E.getAddress((offset + row - col - 1) * NX),
					row, col
			);
		}

		adjLoop.addFunctionCall(
				multGxTGu, evGx.getAddress((blockI*getBlockSize() + row) * NX), W1, W2
		);

		ExportArgument Q1Call = Q1.isGiven() == true ? Q1 : Q1.getAddress((blockI*getBlockSize() + row) * NX);
		adjLoop.addFunctionCall(
				macQEW2, Q1Call, E.getAddress((offset + row - col - 1) * NX), W2, W1
		);

		cLoop.addStatement( adjLoop );

		ExportArgument R1Call = R1.isGiven() == true ? R1 : R1.getAddress((blockI*getBlockSize() + col) * NU);
		cLoop.addFunctionCall(
				macBTW1_R1, R1Call, evGu.getAddress((blockI*getBlockSize() + col) * NX), W1,
				ExportIndex( col )
		);

		condensePrep.addStatement( cLoop );
		condensePrep.addLinebreak();

		condensePrep.release( row ).release( col ).release( offset );
	}

	/// NEW CODE END

	LOG( LVL_DEBUG ) << "---> Copy H11 lower part" << endl;

	// Copy to H11 upper lower part to upper triangular part, TODO: do this together with the copy to qpH !!!
	if (getBlockSize() <= 20)
	{
		for (unsigned ii = 0; ii < getBlockSize(); ++ii)
			for(unsigned jj = 0; jj < ii; ++jj)
				condensePrep.addFunctionCall(
						copyHTH, ExportIndex( jj ), ExportIndex( ii ));

		// Copy H10 to H01
		if( performFullCondensing() == false) {
			for (unsigned ii = 0; ii < getBlockSize(); ++ii)
				condensePrep.addFunctionCall( copyHTH1, ExportIndex( ii ) );
		}
	}
	else
	{
		ExportIndex ii, jj;

		condensePrep.acquire( ii );
		condensePrep.acquire( jj );

		ExportForLoop eLoopI(ii, 0, getBlockSize());
		ExportForLoop eLoopJ(jj, 0, ii);

		eLoopJ.addFunctionCall(copyHTH, jj, ii);
		eLoopI.addStatement( eLoopJ );
		condensePrep.addStatement( eLoopI );

		// Copy H10 to H01
		if( performFullCondensing() == false) {
			ExportForLoop eLoopK(ii, 0, getBlockSize());
			eLoopK.addFunctionCall(copyHTH1, ii);
			condensePrep.addStatement( eLoopK );
		}

		condensePrep.release( ii );
		condensePrep.release( jj );
	}
	condensePrep.addLinebreak();

	// Copy to qpH for qpDUNES:
	condensePrep.addStatement( qpH.getRows(blockI*getNumBlockVariables()*getNumBlockVariables(),(blockI+1)*getNumBlockVariables()*getNumBlockVariables()) == H.makeColVector() );

	////////////////////////////////////////////////////////////////////////////
	//
	// Compute gradient components g0 and g1
	//
	////////////////////////////////////////////////////////////////////////////

	LOG( LVL_DEBUG ) << "Setup condensing: create Dx0, Dy and DyN" << endl;

	unsigned offset = performFullCondensing() == true ? 0 : NX;

	int variableObjS;
	get(CG_USE_VARIABLE_WEIGHTING_MATRIX, variableObjS);

	// Compute RDy
	for(unsigned run1 = 0; run1 < getBlockSize(); ++run1)
	{
		ExportArgument R2Call = R2.isGiven() == true ? R2 : R2.getAddress((blockI*getBlockSize()+run1) * NU, 0);
		ExportArgument SluCall =
				objSlu.isGiven() == true || variableObjS == false ? objSlu : objSlu.getAddress((blockI*getBlockSize()+run1) * NU, 0);
		condensePrep.addFunctionCall(
				multRDy, R2Call, Dy.getAddress((blockI*getBlockSize()+run1) * NY, 0), SluCall, g.getAddress(blockI*getNumBlockVariables() + offset + run1 * NU, 0)
		);
	}
	condensePrep.addLinebreak();

	// Compute QDy
	for(unsigned run1 = 0; run1 < getBlockSize(); run1++ )
	{
		ExportArgument Q2Call = Q2.isGiven() == true ? Q2 : Q2.getAddress((blockI*getBlockSize()+run1) * NX);
		ExportArgument SlxCall =
				objSlx.isGiven() == true || variableObjS == false ? objSlx : objSlx.getAddress((blockI*getBlockSize()+run1) * NX, 0);
		condensePrep.addFunctionCall(
				multQDy, Q2Call, Dy.getAddress((blockI*getBlockSize()+run1) * NY), SlxCall, QDy.getAddress(run1 * NX) );
	}
	condensePrep.addLinebreak();

	if (performFullCondensing() == false)
		condensePrep.addStatement(g.getRows(blockI*getNumBlockVariables(), blockI*getNumBlockVariables()+NX) == QDy.getRows(0, NX));

	/*
	if partial condensing:
		g0 = q_0

	for k = 0: N - 1
		g1_k = r_k

	if partial condensing:
		sbar_0 = 0;
	else:
		sbar_0 = Dx_0;

	sbar(1: N) = d;

	for k = 0: N - 1
		sbar_{k + 1} += A_k sbar_k;

	w1 = Q_N^T * sbar_N + q_N;
	for k = N - 1: 1
	{
		g1_k += B_k^T * w1;
		g1_k += S_k^T * sbar_k;
		w2 = A_k^T * w1 + q_k;
		w1 = Q_k^T * sbar_k + w2;
	}

	g1_0 += B_0^T * w1;
	if partial condensing:
		g_0 += A_0^T * w1
	else:
		g1_0 += S^0^T * x0;

	*/

	if( getBlockSize() > 1 ) {
		w1.setup("w1", NX, 1, REAL, ACADO_WORKSPACE);
		w2.setup("w2", NX, 1, REAL, ACADO_WORKSPACE);
	}

	sbar.setup("sbar", (getBlockSize()+1)*NX, 1, REAL, ACADO_WORKSPACE);

//	if( performFullCondensing() == true ) {
//		condenseFdb.addStatement( sbar.getRows(0, NX) == Dx0 );
//	}
//	else {
	condensePrep.addStatement( sbar.getRows(0, NX) == zeros<double>(NX,1) );  // Dx0 is now a variable as well !!
//	}
	condensePrep.addStatement( sbar.getRows(NX, (getBlockSize() + 1) * NX) == d.getRows(blockI*getBlockSize()*NX,(blockI+1)*getBlockSize()*NX) );

	for (unsigned i = 1; i < getBlockSize(); ++i)
		condensePrep.addFunctionCall(
				macASbar, evGx.getAddress((blockI*getBlockSize()+i) * NX), sbar.getAddress(i * NX), sbar.getAddress((i + 1) * NX)
		);
	condensePrep.addLinebreak();

	condensePrep.addStatement( w1 == zeros<double>(NX,1) );
	for (unsigned i = getBlockSize() - 1; 0 < i; --i)
	{
		condensePrep.addFunctionCall(
				macBTw1, evGu.getAddress((blockI*getBlockSize()+i) * NX), w1, g.getAddress(blockI*getNumBlockVariables() + offset + i * NU)
		);

		if ((S1.isGiven() == true && S1.getGivenMatrix().isZero() == false) || S1.isGiven() == false)
		{
			ExportArgument S1Call = S1.isGiven() == false ? S1.getAddress((blockI*getBlockSize()+i) * NX) : S1;
			condensePrep.addFunctionCall(macS1TSbar, S1Call, sbar.getAddress(i * NX), g.getAddress(blockI*getNumBlockVariables() + offset + i * NU));
		}

		condensePrep.addFunctionCall(
				// TODO Check indexing for QDy
				macATw1QDy, evGx.getAddress((blockI*getBlockSize()+i) * NX), w1, QDy.getAddress(i * NX), w2
		);

		ExportArgument Q1Call = Q1.isGiven() == true ? Q1 : Q1.getAddress((blockI*getBlockSize()+i) * NX);
		condensePrep.addFunctionCall(
				macQSbarW2, Q1Call, sbar.getAddress(i * NX), w2, w1);
	}
	if( getBlockSize() > 1 ) {
		condensePrep.addFunctionCall(
			macBTw1, evGu.getAddress( blockI*getBlockSize()*NX ), w1, g.getAddress( blockI*getNumBlockVariables() + offset ) );
//	if( performFullCondensing() == true ) {
//		if ((S1.isGiven() == true && S1.getGivenMatrix().isZero() == false) || S1.isGiven() == false)
//		{
//			condenseFdb.addFunctionCall(macS1TSbar, S1, sbar.getAddress(0), g);
//		}
//	}
//	else {
		condensePrep.addStatement(g.getRows(blockI*getNumBlockVariables(), blockI*getNumBlockVariables()+NX) += evGx.getRows(blockI*getBlockSize()*NX, blockI*getBlockSize()*NX+NX).getTranspose() * w1);
//	}
	}
	condensePrep.addLinebreak();

	////////////////////////////////////////////////////////////////////////////
	//
	// Evaluation of the system dynamics equality constraints
	//
	////////////////////////////////////////////////////////////////////////////

	//
	// Set QP C matrix
	//

	condensePrep.addStatement( qpc.getRows(blockI*NX,(blockI+1)*NX) == sbar.getRows( getBlockSize()*NX, (getBlockSize()+1)*NX ) );
	condensePrep.addStatement( qpC.getSubMatrix(blockI*NX,(blockI+1)*NX,0,NX) == C.getRows( (getBlockSize()-1)*NX, getBlockSize()*NX ) );
	for( unsigned i = 0; i < getBlockSize(); i++ ) {
		unsigned offset2 = i * (2 * getBlockSize() - i + 1) / 2;
		condensePrep.addStatement( qpC.getSubMatrix(blockI*NX,(blockI+1)*NX,NX+i*NU,NX+(i+1)*NU) == E.getRows((offset2 + getBlockSize()-i-1)*NX,(offset2 + getBlockSize()-i)*NX) );
	}
	condensePrep.addLinebreak();


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
		Ds_{k + 1} += B_k * Du_k;
		s_{k + 1}  += Ds_{k + 1};
	}

	 */

	LOG( LVL_DEBUG ) << "Setup condensing: create expand routine" << endl;

	expand.setup( "expand", blockI );

//	if (performFullCondensing() == true)
//	{
//		expand.addStatement( u.makeRowVector() += xVars.getTranspose() );
//	}
//	else
//	{
	for (unsigned i = 0; i < getBlockSize(); ++i ) {
		expand.addStatement( (u.getRow(blockI*getBlockSize()+i)).getTranspose() += xVars.getRows(blockI*getNumBlockVariables()+NX+i*NU, blockI*getNumBlockVariables()+NX+(i+1)*NU) );
	}
//	}

//	if( performFullCondensing() == true ) {
//		expand.addStatement( sbar.getRows(0, NX) == Dx0 );
//	}
//	else {
	expand.addStatement( sbar.getRows(0, NX) == xVars.getRows(blockI*getNumBlockVariables(), blockI*getNumBlockVariables()+NX) );
	expand.addStatement( (x.getRow(blockI*getBlockSize())).getTranspose() += sbar.getRows(0, NX) );
//	}
	if( getBlockSize() > 1 ) {
		expand.addStatement( sbar.getRows(NX, getBlockSize()*NX) == d.getRows(blockI*getBlockSize()*NX,(blockI+1)*getBlockSize()*NX-NX) );
	}

	for (unsigned row = 0; row < getBlockSize()-1; ++row ) {
		expand.addFunctionCall(
				expansionStep, evGx.getAddress((blockI*getBlockSize()+row) * NX), evGu.getAddress((blockI*getBlockSize()+row) * NX),
				xVars.getAddress(blockI*getNumBlockVariables()+offset + row * NU), sbar.getAddress(row * NX),
				sbar.getAddress((row + 1) * NX)
		);
		expand.addStatement( (x.getRow(blockI*getBlockSize()+row+1)).getTranspose() += sbar.getRows((row+1)*NX, (row+2)*NX) );
	}

	// !! Calculation of multipliers: !!
	int hessianApproximation;
	get( HESSIAN_APPROXIMATION, hessianApproximation );
	bool secondOrder = ((HessianApproximationMode)hessianApproximation == EXACT_HESSIAN);
	if( secondOrder ) {
		return ACADOERROR( RET_NOT_IMPLEMENTED_YET );
		//	mu_N = lambda_N + q_N + Q_N^T * Ds_N  --> wrong in Joel's paper !!
		//		for i = N - 1: 1
		//			mu_k = Q_k^T * Ds_k + A_k^T * mu_{k + 1} + S_k * Du_k + q_k

		for (uint j = 0; j < NX; j++ ) {
			uint item = N*NX+j;
			uint IdxF = std::find(xBoundsIdx.begin(), xBoundsIdx.end(), item) - xBoundsIdx.begin();
			if( IdxF != xBoundsIdx.size() ) { // INDEX FOUND
				expand.addStatement( mu.getSubMatrix(N-1,N,j,j+1) == yVars.getRow(getNumQPvars()+IdxF) );
			}
			else { // INDEX NOT FOUND
				expand.addStatement( mu.getSubMatrix(N-1,N,j,j+1) == 0.0 );
			}
		}
		expand.addStatement( mu.getRow(N-1) += sbar.getRows(N*NX,(N+1)*NX).getTranspose()*QN1 );
		expand.addStatement( mu.getRow(N-1) += QDy.getRows(N*NX,(N+1)*NX).getTranspose() );
		for (int i = N - 1; i >= 1; i--) {
			for (uint j = 0; j < NX; j++ ) {
				uint item = i*NX+j;
				uint IdxF = std::find(xBoundsIdx.begin(), xBoundsIdx.end(), item) - xBoundsIdx.begin();
				if( IdxF != xBoundsIdx.size() ) { // INDEX FOUND
					expand.addStatement( mu.getSubMatrix(i-1,i,j,j+1) == yVars.getRow(getNumQPvars()+IdxF) );
				}
				else { // INDEX NOT FOUND
					expand.addStatement( mu.getSubMatrix(i-1,i,j,j+1) == 0.0 );
				}
			}
			expand.addFunctionCall(
					expansionStep2, QDy.getAddress(i*NX), Q1.getAddress(i * NX), sbar.getAddress(i*NX),
					S1.getAddress(i * NX), xVars.getAddress(offset + i * NU), evGx.getAddress(i * NX),
					mu.getAddress(i-1), mu.getAddress(i) );
		}
	}

	return SUCCESSFUL_RETURN;
}

returnValue ExportGaussNewtonBlockCN2::setupVariables( )
{
	ExportGaussNewtonCN2::setupVariables();

	blockI = ExportIndex( "blockI" );

	qpC.setup("qpC", getNumberOfBlocks()*NX, getNumBlockVariables(), REAL, ACADO_WORKSPACE);
	qpc.setup("qpc", getNumberOfBlocks()*NX, 1, REAL, ACADO_WORKSPACE);

	if (performFullCondensing() == false || getNumComplexConstraints() > 0)
	{
		C.setup("C", getBlockSize()*NX, NX, REAL, ACADO_WORKSPACE);
	}
	E.setup("E", getBlockSize() * (getBlockSize() + 1) / 2 * NX, NU, REAL, ACADO_WORKSPACE);

	QDy.setup ("QDy", getBlockSize()*NX, 1, REAL, ACADO_WORKSPACE);  // just for one block now

	qpH.setup("qpH", getNumberOfBlocks()*getNumBlockVariables()*getNumBlockVariables() + NX * NX, 1, REAL, ACADO_WORKSPACE);
	H.setup("H", getNumBlockVariables(), getNumBlockVariables(), REAL, ACADO_WORKSPACE);
	g.setup("g",  getNumQPvars(), 1, REAL, ACADO_WORKSPACE);
	A.setup("A", getNumberOfBlocks()*getNumStateBoundsPerBlock() + getNumComplexConstraints(), getNumBlockVariables(), REAL, ACADO_WORKSPACE);

	lb.setup("lb", getNumQPvars(), 1, REAL, ACADO_WORKSPACE);
	ub.setup("ub", getNumQPvars(), 1, REAL, ACADO_WORKSPACE);

	lbA.setup("lbA", getNumberOfBlocks()*getNumStateBoundsPerBlock() + getNumComplexConstraints(), 1, REAL, ACADO_WORKSPACE);
	ubA.setup("ubA", getNumberOfBlocks()*getNumStateBoundsPerBlock() + getNumComplexConstraints(), 1, REAL, ACADO_WORKSPACE);

	xVars.setup("x", getNumQPvars(), 1, REAL, ACADO_WORKSPACE);
	yVars.setup("",0,0); // NOT USED
	qpLambda.setup("qpLambda", getNumberOfBlocks()*NX, 1, REAL, ACADO_WORKSPACE);
	qpMu.setup("qpMu", 2*getNumberOfBlocks()*NX + 2*N*NU + 2*NX + 2*getNumberOfBlocks()*getNumStateBoundsPerBlock(), 1, REAL, ACADO_WORKSPACE);

	return SUCCESSFUL_RETURN;
}


returnValue ExportGaussNewtonBlockCN2::setupMultiplicationRoutines( )
{
	return ExportGaussNewtonCN2::setupMultiplicationRoutines();
}

uint ExportGaussNewtonBlockCN2::getBlockSize() const
{
	int blockSize;
	get(CONDENSING_BLOCK_SIZE, blockSize);

	return blockSize;
}

uint ExportGaussNewtonBlockCN2::getNumberOfBlocks() const
{
	uint blockSize = getBlockSize();
	uint numBlocks = acadoRound(((double)N)/((double)blockSize));

	return numBlocks;
}

uint ExportGaussNewtonBlockCN2::getNumBlockVariables() const
{
	return getBlockSize()*NU + NX;
}

unsigned ExportGaussNewtonBlockCN2::getNumStateBoundsPerBlock() const
{
	unsigned numStateBoundsBlock = 0; // NOTE: it is expected here that all blocks have the same state bounds !
	unsigned index = 0;
//	for (unsigned i = 0; i < getNumberOfBlocks(); ++i) {
		while( index < xBoundsIdx.size() && xBoundsIdx[index] < getBlockSize()*NX ) {
			if( xBoundsIdx[index] >= NX ) {
				numStateBoundsBlock += 1;
			}
			index += 1;
		}
//	}
	return numStateBoundsBlock;
}


CLOSE_NAMESPACE_ACADO
