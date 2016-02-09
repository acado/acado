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
 *    \file src/code_generation/integrators/dirk_export.cpp
 *    \author Rien Quirynen
 *    \date 2013
 */

#include <acado/code_generation/integrators/dirk_export.hpp>

#include <sstream>
using namespace std;



BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

DiagonallyImplicitRKExport::DiagonallyImplicitRKExport(	UserInteraction* _userInteraction,
														const std::string& _commonHeaderName
														) : ForwardIRKExport( _userInteraction,_commonHeaderName )
{

}

DiagonallyImplicitRKExport::DiagonallyImplicitRKExport( const DiagonallyImplicitRKExport& arg ) : ForwardIRKExport( arg )
{

}


DiagonallyImplicitRKExport::~DiagonallyImplicitRKExport( )
{
	if ( solver )
		delete solver;
	solver = 0;

	clear( );
}


DiagonallyImplicitRKExport& DiagonallyImplicitRKExport::operator=( const DiagonallyImplicitRKExport& arg ){

    if( this != &arg ){

    	ForwardIRKExport::operator=( arg );
    }
    return *this;
}


returnValue DiagonallyImplicitRKExport::solveInputSystem( ExportStatementBlock* block, const ExportIndex& index1, const ExportIndex& index2, const ExportIndex& index3, const ExportIndex& tmp_index, const ExportVariable& Ah )
{
	if( NX1 > 0 ) {
		ExportForLoop loop( index1,0,numStages );
		loop.addStatement( rk_xxx.getCols(0,NX1) == rk_eta.getCols(0,NX1) );
		ExportForLoop loop01( index2,0,NX1 );
		ExportForLoop loop02( index3,0,index1 );
		loop02.addStatement( rk_xxx.getCol( index2 ) += Ah.getElement(index1,index3)*rk_kkk.getElement(index2,index3) );
		loop01.addStatement( loop02 );
		loop.addStatement( loop01 );
		loop.addFunctionCall( lin_input.getName(), rk_xxx, rk_b.getAddress(0,0) );

		ExportForLoop loop5( index2,0,NX1 );
		loop5.addStatement( tmp_index == index1*NX1+index2 );
		loop5.addStatement( rk_kkk.getElement(index2,index1) == rk_mat1.getElement(tmp_index,0)*rk_b.getRow(0) );
		ExportForLoop loop6( index3,1,NX1 );
		loop6.addStatement( rk_kkk.getElement(index2,index1) += rk_mat1.getElement(tmp_index,index3)*rk_b.getRow(index3) );
		loop5.addStatement(loop6);
		loop.addStatement(loop5);
		block->addStatement(loop);
	}

	return SUCCESSFUL_RETURN;
}


returnValue DiagonallyImplicitRKExport::prepareInputSystem(	ExportStatementBlock& code )
{
	if( NX1 > 0 ) {
		DMatrix mat1 = formMatrix( M11, A11 );
		rk_mat1 = ExportVariable( "rk_mat1", mat1, STATIC_CONST_REAL );
		code.addDeclaration( rk_mat1 );
		// TODO: Ask Milan why this does NOT work properly !!
		rk_mat1 = ExportVariable( "rk_mat1", numStages*NX1, NX1, STATIC_CONST_REAL, ACADO_LOCAL );
		double h = (grid.getLastTime() - grid.getFirstTime())/grid.getNumIntervals();

		DMatrix sens = zeros<double>(NX1*(NX1+NU), numStages);
		uint i, j, k, s1, s2;
		for( i = 0; i < NX1; i++ ) {
			DVector vec(NX1);
			for( j = 0; j < numStages; j++ ) {
				for( k = 0; k < NX1; k++ ) {
					vec(k) = A11(k,i);
					for( s1 = 0; s1 < j; s1++ ) {
						for( s2 = 0; s2 < NX1; s2++ ) {
							vec(k) = vec(k) + AA(j,s1)*h*A11(k,s2)*sens(i*NX1+s2,s1);
						}
					}
				}
				DVector sol = mat1*vec;
				for( k = 0; k < NX1; k++ ) {
					sens(i*NX1+k,j) = sol(k);
				}
			}
		}
		for( i = 0; i < NU; i++ ) {
			DVector vec(NX1);
			for( j = 0; j < numStages; j++ ) {
				for( k = 0; k < NX1; k++ ) {
					vec(k) = B11(k,i);
					for( s1 = 0; s1 < j; s1++ ) {
						for( s2 = 0; s2 < NX1; s2++ ) {
							vec(k) = vec(k) + AA(j,s1)*h*A11(k,s2)*sens(NX1*NX1+i*NX1+s2,s1);
						}
					}
				}
				DVector sol = mat1*vec;
				for( k = 0; k < NX1; k++ ) {
					sens(NX1*NX1+i*NX1+k,j) = sol(k);
				}
			}
		}
		rk_dk1 = ExportVariable( "rk_dk1", sens, STATIC_CONST_REAL );
		code.addDeclaration( rk_dk1 );
		// TODO: Ask Milan why this does NOT work properly !!
		rk_dk1 = ExportVariable( "rk_dk1", NX1*(NX1+NU), numStages, STATIC_CONST_REAL, ACADO_LOCAL );
	}

	return SUCCESSFUL_RETURN;
}


DMatrix DiagonallyImplicitRKExport::formMatrix( const DMatrix& mass, const DMatrix& jacobian ) {
	if( jacobian.getNumRows() != jacobian.getNumCols() ) {
		return RET_UNABLE_TO_EXPORT_CODE;
	}
	double h = (grid.getLastTime() - grid.getFirstTime())/grid.getNumIntervals();
	uint vars = jacobian.getNumRows();
	uint i1, i2, j2;
	DMatrix result = zeros<double>(numStages*vars, vars);
	DMatrix tmp = zeros<double>(vars, vars);
	for( i1 = 0; i1 < numStages; i1++ ){
		for( i2 = 0; i2 < vars; i2++ ){
			for( j2 = 0; j2 < vars; j2++ ) {
				tmp(i2, j2) = mass(i2,j2) - AA(i1,i1)*h*jacobian(i2,j2);
			}
		}
		tmp = tmp.inverse();
		for( i2 = 0; i2 < vars; i2++ ){
			for( j2 = 0; j2 < vars; j2++ ) {
				result(i1*vars+i2, j2) = tmp(i2, j2);
			}
		}
	}

	return result;
}


returnValue DiagonallyImplicitRKExport::solveImplicitSystem( ExportStatementBlock* block, const ExportIndex& index1, const ExportIndex& index2, const ExportIndex& index3, const ExportIndex& tmp_index, const ExportVariable& Ah, const ExportVariable& C, const ExportVariable& det, bool DERIVATIVES )
{
	if( NX2 > 0 || NXA > 0 ) {

		if( REUSE ) block->addStatement( std::string( "if( " ) + reset_int.getFullName() + " ) {\n" );
		// Initialization iterations:
		ExportForLoop loop11( index2,0,numStages );
		ExportForLoop loop1( index1,0,numItsInit+1 ); // NOTE: +1 because 0 will lead to NaNs, so the minimum number of iterations is 1 at the initialization
		evaluateMatrix( &loop1, index2, index3, tmp_index, rk_A, Ah, C, true, DERIVATIVES );
		loop1.addStatement( det.getFullName() + " = " + ExportStatement::fcnPrefix + "_" + solver->getNameSolveFunction() + "( &" + rk_A.get(index2*(NX2+NXA),0) + ", " + rk_b.getFullName() + ", &" + rk_auxSolver.get(index2,0) + " );\n" );
		loop1.addStatement( rk_kkk.getSubMatrix( NX1,NX1+NX2,index2,index2+1 ) += rk_b.getRows( 0,NX2 ) );													// differential states
		if(NXA > 0) loop1.addStatement( rk_kkk.getSubMatrix( NX,NX+NXA,index2,index2+1 ) += rk_b.getRows( NX2,NX2+NXA ) );		// algebraic states
		loop11.addStatement( loop1 );
		block->addStatement( loop11 );
		if( REUSE ) block->addStatement( std::string( "}\n" ) );

		// the rest (numIts) of the Newton iterations with reuse of the Jacobian (no evaluation or factorization needed)
		ExportForLoop loop21( index2,0,numStages );
		ExportForLoop loop2( index1,0,numIts );
		evaluateStatesImplicitSystem( &loop2, Ah, C, index2, index3, tmp_index );
		evaluateRhsImplicitSystem( &loop2, index2 );
		loop2.addFunctionCall( solver->getNameSolveReuseFunction(),rk_A.getAddress(index2*(NX2+NXA),0),rk_b.getAddress(0,0),rk_auxSolver.getAddress(index2,0) );
		loop2.addStatement( rk_kkk.getSubMatrix( NX1,NX1+NX2,index2,index2+1 ) += rk_b.getRows( 0,NX2 ) );														// differential states
		if(NXA > 0) loop2.addStatement( rk_kkk.getSubMatrix( NX,NX+NXA,index2,index2+1 ) += rk_b.getRows( NX2,NX2+NXA ) );		// algebraic states
		loop21.addStatement( loop2 );
		block->addStatement( loop21 );

		if( DERIVATIVES ) {
			// solution calculated --> evaluate and save the necessary derivatives in rk_diffsTemp and update the matrix rk_A:
			ExportForLoop loop3( index2,0,numStages );
			evaluateMatrix( &loop3, index2, index3, tmp_index, rk_A, Ah, C, false, DERIVATIVES );
			block->addStatement( loop3 );
		}

		// IF DEBUG MODE:
		int debugMode;
		get( INTEGRATOR_DEBUG_MODE, debugMode );
		if ( (bool)debugMode == true ) {
			block->addStatement( debug_mat == rk_A );
		}
	}

	return SUCCESSFUL_RETURN;
}


returnValue DiagonallyImplicitRKExport::sensitivitiesImplicitSystem( ExportStatementBlock* block, const ExportIndex& index1, const ExportIndex& index2, const ExportIndex& index3, const ExportIndex& tmp_index1, const ExportIndex& tmp_index2, const ExportVariable& Ah, const ExportVariable& Bh, const ExportVariable& det, bool STATES, uint number )
{
	if( NX2 > 0 ) {
		DMatrix zeroM = zeros<double>( NX2+NXA,1 );
		DMatrix tempCoefs( evaluateDerivedPolynomial( 0.0 ) );
		uint i;

		ExportForLoop loop1( index2,0,numStages );
		if( STATES && number == 1 ) {
			ExportForLoop loop2( index3,0,NX1 );
			loop2.addStatement( std::string(rk_rhsTemp.get( index3,0 )) + " = -(" + index3.getName() + " == " + index1.getName() + ");\n" );
			ExportForLoop loop21( tmp_index1,0,index2+1 );
			loop21.addStatement( rk_rhsTemp.getRow( index3 ) -= rk_diffK.getElement( index3,tmp_index1 )*Ah.getElement(index2,tmp_index1) );
			loop2.addStatement( loop21 );
			loop1.addStatement( loop2 );
			ExportForLoop loop3( index3,0,NX2+NXA );
			loop3.addStatement( rk_b.getRow( index3 ) == rk_diffsTemp2.getSubMatrix( index2,index2+1,index3*(NVARS2),index3*(NVARS2)+NX1 )*rk_rhsTemp.getRows(0,NX1) );
			if( NDX2 > 0 ) {
				loop3.addStatement( rk_b.getRow( index3 ) -= rk_diffsTemp2.getSubMatrix( index2,index2+1,index3*(NVARS2)+NVARS2-NX1-NX2,index3*(NVARS2)+NVARS2-NX2 )*rk_diffK.getSubMatrix( 0,NX1,index2,index2+1 ) );
			}
			loop1.addStatement( loop3 );
		}
		else if( STATES && number == 2 ) {
			for( i = 0; i < NX2+NXA; i++ ) {
				loop1.addStatement( rk_b.getRow( i ) == zeroM.getRow( 0 ) - rk_diffsTemp2.getElement( index2,index1+i*(NVARS2) ) );
			}
		}
		else { // ~= STATES
			ExportForLoop loop2( index3,0,NX1 );
			loop2.addStatement( rk_rhsTemp.getRow( index3 ) == rk_diffK.getElement( index3,0 )*Ah.getElement(index2,0) );
			ExportForLoop loop21( tmp_index1,1,index2+1 );
			loop21.addStatement( rk_rhsTemp.getRow( index3 ) += rk_diffK.getElement( index3,tmp_index1 )*Ah.getElement(index2,tmp_index1) );
			loop2.addStatement( loop21 );
			loop1.addStatement( loop2 );
			ExportForLoop loop3( index3,0,NX2+NXA );
			loop3.addStatement( tmp_index2 == index1+index3*(NVARS2) );
			loop3.addStatement( rk_b.getRow( index3 ) == zeroM.getRow( 0 ) - rk_diffsTemp2.getElement( index2,tmp_index2+NX1+NX2+NXA ) );
			loop3.addStatement( rk_b.getRow( index3 ) -= rk_diffsTemp2.getSubMatrix( index2,index2+1,index3*(NVARS2),index3*(NVARS2)+NX1 )*rk_rhsTemp.getRows(0,NX1) );
			if( NDX2 > 0 ) {
				loop3.addStatement( rk_b.getRow( index3 ) -= rk_diffsTemp2.getSubMatrix( index2,index2+1,index3*(NVARS2)+NVARS2-NX1-NX2,index3*(NVARS2)+NVARS2-NX2 )*rk_diffK.getSubMatrix( 0,NX1,index2,index2+1 ) );
			}
			loop1.addStatement( loop3 );
		}
		ExportForLoop loop11( index3,0,NX2+NXA );
		ExportForLoop loop12( tmp_index1,0,index2 );
		ExportForLoop loop13( tmp_index2,NX1,NX1+NX2 );
		loop13.addStatement( std::string( rk_b.get(index3,0) ) + " -= " + Ah.get(index2,tmp_index1) + "*" + rk_diffsTemp2.get(index2,index3*NVARS2+tmp_index2) + "*" + rk_diffK.get(tmp_index2,tmp_index1) + ";\n" );
		loop12.addStatement( loop13 );
		loop11.addStatement( loop12 );
		loop1.addStatement( loop11 );
		if( STATES && (number == 1 || NX1 == 0) ) {
			loop1.addStatement( std::string( "if( 0 == " ) + index1.getName() + " ) {\n" );	// factorization of the new matrix rk_A not yet calculated!
			loop1.addStatement( det.getFullName() + " = " + ExportStatement::fcnPrefix + "_" + solver->getNameSolveFunction() + "( &" + rk_A.get(index2*(NX2+NXA),0) + ", " + rk_b.getFullName() + ", &" + rk_auxSolver.get(index2,0) + " );\n" );
			loop1.addStatement( std::string( "}\n else {\n" ) );
		}
		loop1.addFunctionCall( solver->getNameSolveReuseFunction(),rk_A.getAddress(index2*(NX2+NXA),0),rk_b.getAddress(0,0),rk_auxSolver.getAddress(index2,0) );
		if( STATES && (number == 1 || NX1 == 0) ) loop1.addStatement( std::string( "}\n" ) );
		// update rk_diffK with the new sensitivities:
		loop1.addStatement( rk_diffK.getSubMatrix(NX1,NX1+NX2,index2,index2+1) == rk_b.getRows(0,NX2) );
		loop1.addStatement( rk_diffK.getSubMatrix(NX,NX+NXA,index2,index2+1) == rk_b.getRows(NX2,NX2+NXA) );
		block->addStatement( loop1 );
		// update rk_diffsNew with the new sensitivities:
		ExportForLoop loop3( index2,0,NX2 );
		if( STATES && number == 2 ) loop3.addStatement( std::string(rk_diffsNew2.get( index2,index1 )) + " = (" + index2.getName() + " == " + index1.getName() + "-" + toString(NX1) + ");\n" );

		if( STATES && number == 2 ) loop3.addStatement( rk_diffsNew2.getElement( index2,index1 ) += rk_diffK.getRow( NX1+index2 )*Bh );
		else if( STATES )	loop3.addStatement( rk_diffsNew2.getElement( index2,index1 ) == rk_diffK.getRow( NX1+index2 )*Bh );
		else		 		loop3.addStatement( rk_diffsNew2.getElement( index2,index1+NX1+NX2 ) == rk_diffK.getRow( NX1+index2 )*Bh );
		block->addStatement( loop3 );
		if( NXA > 0 ) {
			block->addStatement( std::string("if( run == 0 ) {\n") );
			ExportForLoop loop4( index2,0,NXA );
			if( STATES ) loop4.addStatement( rk_diffsNew2.getElement( index2+NX2,index1 ) == rk_diffK.getRow( NX+index2 )*tempCoefs );
			else 		 loop4.addStatement( rk_diffsNew2.getElement( index2+NX2,index1+NX1+NX2 ) == rk_diffK.getRow( NX+index2 )*tempCoefs );
			block->addStatement( loop4 );
			block->addStatement( std::string("}\n") );
		}
	}

	return SUCCESSFUL_RETURN;
}


returnValue DiagonallyImplicitRKExport::evaluateMatrix( ExportStatementBlock* block, const ExportIndex& index1, const ExportIndex& index2, const ExportIndex& tmp_index, const ExportVariable& _rk_A, const ExportVariable& Ah, const ExportVariable& C, bool evaluateB, bool DERIVATIVES )
{
	evaluateStatesImplicitSystem( block, Ah, C, index1, index2, tmp_index );

	ExportIndex indexDiffs(index1);
	if( !DERIVATIVES ) indexDiffs = ExportIndex(0);

	block->addFunctionCall( getNameDiffsRHS(), rk_xxx, rk_diffsTemp2.getAddress(indexDiffs,0) );
	ExportForLoop loop2( index2,0,NX2+NXA );
	loop2.addStatement( tmp_index == index1*(NX2+NXA)+index2 );
	if( NDX2 == 0 ) {
		loop2.addStatement( _rk_A.getSubMatrix( tmp_index,tmp_index+1,0,NX2 ) == Ah.getElement( 0,0 )*rk_diffsTemp2.getSubMatrix( indexDiffs,indexDiffs+1,index2*(NVARS2)+NX1,index2*(NVARS2)+NX1+NX2 ) );
		loop2.addStatement( _rk_A.getElement( tmp_index,index2 ) -= 1 );
	}
	else {
		loop2.addStatement( _rk_A.getSubMatrix( tmp_index,tmp_index+1,0,NX2 ) == Ah.getElement( 0,0 )*rk_diffsTemp2.getSubMatrix( indexDiffs,indexDiffs+1,index2*(NVARS2)+NX1,index2*(NVARS2)+NX1+NX2 ) );
		loop2.addStatement( _rk_A.getSubMatrix( tmp_index,tmp_index+1,0,NX2 ) += rk_diffsTemp2.getSubMatrix( indexDiffs,indexDiffs+1,index2*(NVARS2)+NVARS2-NX2,index2*(NVARS2)+NVARS2 ) );
	}
	if( NXA > 0 ) {
		DMatrix zeroM = zeros<double>( 1,NXA );
		loop2.addStatement( _rk_A.getSubMatrix( tmp_index,tmp_index+1,NX2,NX2+NXA ) == rk_diffsTemp2.getSubMatrix( indexDiffs,indexDiffs+1,index2*(NVARS2)+NX1+NX2,index2*(NVARS2)+NX1+NX2+NXA ) );
	}
	block->addStatement( loop2 );
	if( evaluateB ) {
		evaluateRhsImplicitSystem( block, index1 );
	}

	return SUCCESSFUL_RETURN;
}


returnValue DiagonallyImplicitRKExport::evaluateStatesImplicitSystem( ExportStatementBlock* block, const ExportVariable& Ah, const ExportVariable& C, const ExportIndex& stage, const ExportIndex& i, const ExportIndex& j )
{
	ExportForLoop loop1( i, 0, NX1+NX2 );
	loop1.addStatement( rk_xxx.getCol( i ) == rk_eta.getCol( i ) );
	ExportForLoop loop2( j, 0, stage+1 );
	loop2.addStatement( rk_xxx.getCol( i ) += Ah.getElement(stage,j)*rk_kkk.getElement( i,j ) );
	loop1.addStatement( loop2 );
	block->addStatement( loop1 );

	ExportForLoop loop3( i, 0, NXA );
	loop3.addStatement( rk_xxx.getCol( NX+i ) == rk_kkk.getElement( NX+i,stage ) );
	block->addStatement( loop3 );

	ExportForLoop loop4( i, 0, NDX2 );
	loop4.addStatement( rk_xxx.getCol( inputDim-diffsDim+i ) == rk_kkk.getElement( i,stage ) );
	block->addStatement( loop4 );

	if( C.getDim() > 0 ) {	// There is a time dependence, so it must be set
		block->addStatement( rk_xxx.getCol( inputDim-diffsDim+NDX2 ) == C.getCol(stage) );
	}

	return SUCCESSFUL_RETURN;
}


returnValue DiagonallyImplicitRKExport::evaluateRhsImplicitSystem( ExportStatementBlock* block, const ExportIndex& stage )
{
	DMatrix zeroM = zeros<double>( NX2+NXA,1 );
	block->addFunctionCall( getNameRHS(), rk_xxx, rk_rhsTemp.getAddress(0,0) );
	// matrix rk_b:
	if( NDX2 == 0 ) {
		block->addStatement( rk_b.getRows( 0,NX2 ) == rk_kkk.getSubMatrix( NX1,NX1+NX2,stage,stage+1 ) - rk_rhsTemp.getRows( 0,NX2 ) );
	}
	else {
		block->addStatement( rk_b.getRows( 0,NX2 ) == zeroM.getRows( 0,NX2-1 ) - rk_rhsTemp.getRows( 0,NX2 ) );
	}
	if( NXA > 0 ) {
		block->addStatement( rk_b.getRows( NX2,NX2+NXA ) == zeroM.getRows( 0,NXA-1 ) - rk_rhsTemp.getRows( NX2,NX2+NXA ) );
	}

	return SUCCESSFUL_RETURN;
}


returnValue DiagonallyImplicitRKExport::solveOutputSystem( ExportStatementBlock* block, const ExportIndex& index1, const ExportIndex& index2, const ExportIndex& index3, const ExportIndex& tmp_index, const ExportVariable& Ah, bool DERIVATIVES )
{
	if( NX3 > 0 ) {
		ExportForLoop loop( index1,0,numStages );
		evaluateStatesOutputSystem( &loop, Ah, index1 );
		ExportForLoop loop01( index2,NX1+NX2,NX );
		ExportForLoop loop02( index3,0,index1 );
		loop02.addStatement( rk_xxx.getCol( index2 ) += Ah.getElement(index1,index3)*rk_kkk.getElement(index2,index3) );
		loop01.addStatement( loop02 );
		loop.addStatement( loop01 );
		loop.addFunctionCall( getNameOutputRHS(), rk_xxx, rk_b.getAddress(0,0) );
		if( DERIVATIVES )	loop.addFunctionCall( getNameOutputDiffs(), rk_xxx, rk_diffsTemp3.getAddress(index1,0) );

		ExportForLoop loop5( index2,0,NX3 );
		loop5.addStatement( tmp_index == index1*NX3+index2 );
		loop5.addStatement( rk_kkk.getElement(NX1+NX2+index2,index1) == rk_mat3.getElement(tmp_index,0)*rk_b.getRow(0) );
		ExportForLoop loop6( index3,1,NX3 );
		loop6.addStatement( rk_kkk.getElement(NX1+NX2+index2,index1) += rk_mat3.getElement(tmp_index,index3)*rk_b.getRow(index3) );
		loop5.addStatement(loop6);
		loop.addStatement(loop5);
		block->addStatement(loop);
	}

	return SUCCESSFUL_RETURN;
}


returnValue DiagonallyImplicitRKExport::sensitivitiesOutputSystem( ExportStatementBlock* block, const ExportIndex& index1, const ExportIndex& index2, const ExportIndex& index3, const ExportIndex& index4, const ExportIndex& tmp_index1, const ExportIndex& tmp_index2, const ExportVariable& Ah, const ExportVariable& Bh, bool STATES, uint number )
{
	if( NX3 > 0 ) {
		uint i, j;
		ExportForLoop loop1( index2,0,numStages );
		if( STATES && number == 1 ) {
			ExportForLoop loop2( index3,0,NX1 );
			loop2.addStatement( std::string(rk_rhsTemp.get( index3,0 )) + " = (" + index3.getName() + " == " + index1.getName() + ");\n" );
			for( i = 0; i < numStages; i++ ) {
				loop2.addStatement( rk_rhsTemp.getRow( index3 ) += rk_diffK.getElement( index3,i )*Ah.getElement(index2,i) );
			}
			loop1.addStatement( loop2 );
			ExportForLoop loop3( index3,NX1,NX1+NX2 );
			loop3.addStatement( rk_rhsTemp.getRow( index3 ) == 0.0 );
			for( i = 0; i < numStages; i++ ) {
				loop3.addStatement( rk_rhsTemp.getRow( index3 ) += rk_diffK.getElement( index3,i )*Ah.getElement(index2,i) );
			}
			loop1.addStatement( loop3 );
			ExportForLoop loop4( index3,0,NX3 );
			loop4.addStatement( rk_b.getRow( index3 ) == rk_diffsTemp3.getSubMatrix( index2,index2+1,index3*(NVARS3),index3*(NVARS3)+NX1+NX2 )*rk_rhsTemp.getRows(0,NX1+NX2) );
			if( NXA3 > 0 ) {
				loop4.addStatement( rk_b.getRow( index3 ) += rk_diffsTemp3.getSubMatrix( index2,index2+1,index3*(NVARS3)+NX1+NX2,index3*(NVARS3)+NX1+NX2+NXA )*rk_diffK.getSubMatrix( NX,NX+NXA,index2,index2+1 ) );
			}
			if( NDX3 > 0 ) {
				loop4.addStatement( rk_b.getRow( index3 ) += rk_diffsTemp3.getSubMatrix( index2,index2+1,index3*(NVARS3)+NVARS3-NX1-NX2,index3*(NVARS3)+NVARS3 )*rk_diffK.getSubMatrix( 0,NX1+NX2,index2,index2+1 ) );
			}
			loop1.addStatement( loop4 );
		}
		else if( STATES && number == 2 ) {
			ExportForLoop loop3( index3,NX1,NX1+NX2 );
			loop3.addStatement( std::string(rk_rhsTemp.get( index3,0 )) + " = (" + index3.getName() + " == " + index1.getName() + ");\n" );
			for( i = 0; i < numStages; i++ ) {
				loop3.addStatement( rk_rhsTemp.getRow( index3 ) += rk_diffK.getElement( index3,i )*Ah.getElement(index2,i) );
			}
			loop1.addStatement( loop3 );
			ExportForLoop loop4( index3,0,NX3 );
			loop4.addStatement( rk_b.getRow( index3 ) == rk_diffsTemp3.getSubMatrix( index2,index2+1,index3*(NVARS3)+NX1,index3*(NVARS3)+NX1+NX2 )*rk_rhsTemp.getRows(NX1,NX1+NX2) );
			if( NXA3 > 0 ) {
				loop4.addStatement( rk_b.getRow( index3 ) += rk_diffsTemp3.getSubMatrix( index2,index2+1,index3*(NVARS3)+NX1+NX2,index3*(NVARS3)+NX1+NX2+NXA )*rk_diffK.getSubMatrix( NX,NX+NXA,index2,index2+1 ) );
			}
			if( NDX3 > 0 ) {
				loop4.addStatement( rk_b.getRow( index3 ) += rk_diffsTemp3.getSubMatrix( index2,index2+1,index3*(NVARS3)+NVARS3-NX2,index3*(NVARS3)+NVARS3 )*rk_diffK.getSubMatrix( NX1,NX1+NX2,index2,index2+1 ) );
			}
			loop1.addStatement( loop4 );
		}
		else if( !STATES ) {
			ExportForLoop loop2( index3,0,NX1 );
			loop2.addStatement( rk_rhsTemp.getRow( index3 ) == rk_diffK.getElement( index3,0 )*Ah.getElement(index2,0) );
			for( i = 1; i < numStages; i++ ) {
				loop2.addStatement( rk_rhsTemp.getRow( index3 ) += rk_diffK.getElement( index3,i )*Ah.getElement(index2,i) );
			}
			loop1.addStatement( loop2 );
			ExportForLoop loop3( index3,NX1,NX1+NX2 );
			loop3.addStatement( rk_rhsTemp.getRow( index3 ) == rk_diffK.getElement( index3,0 )*Ah.getElement(index2,0) );
			for( i = 1; i < numStages; i++ ) {
				loop3.addStatement( rk_rhsTemp.getRow( index3 ) += rk_diffK.getElement( index3,i )*Ah.getElement(index2,i) );
			}
			loop1.addStatement( loop3 );
			ExportForLoop loop4( index3,0,NX3 );
			loop4.addStatement( tmp_index2 == index1+index3*(NVARS3) );
			loop4.addStatement( rk_b.getRow( index3 ) == rk_diffsTemp3.getElement( index2,tmp_index2+NX1+NX2+NXA3 ) );
			loop4.addStatement( rk_b.getRow( index3 ) += rk_diffsTemp3.getSubMatrix( index2,index2+1,index3*(NVARS3),index3*(NVARS3)+NX1+NX2 )*rk_rhsTemp.getRows(0,NX1+NX2) );
			if( NXA3 > 0 ) {
				loop4.addStatement( rk_b.getRow( index3 ) += rk_diffsTemp3.getSubMatrix( index2,index2+1,index3*(NVARS3)+NX1+NX2,index3*(NVARS3)+NX1+NX2+NXA )*rk_diffK.getSubMatrix( NX,NX+NXA,index2,index2+1 ) );
			}
			if( NDX3 > 0 ) {
				loop4.addStatement( rk_b.getRow( index3 ) += rk_diffsTemp3.getSubMatrix( index2,index2+1,index3*(NVARS3)+NVARS3-NX1-NX2,index3*(NVARS3)+NVARS3 )*rk_diffK.getSubMatrix( 0,NX1+NX2,index2,index2+1 ) );
			}
			loop1.addStatement( loop4 );
		}
		if( !STATES || number != 3 ) {
			ExportForLoop loop12( tmp_index1,0,index2 );
			for( i = 0; i < NX3; i++ ) {
				for( j = NX1+NX2; j < NX; j++ ) {
					if( acadoRoundAway(A33(i,j-NX1-NX2)) != 0 ) {
						loop12.addStatement( std::string( rk_b.get(i,0) ) + " += " + Ah.get(index2,tmp_index1) + "*" + toString(A33(i,j-NX1-NX2)) + "*" + rk_diffK.get(j,tmp_index1) + ";\n" );
					}
				}
			}
			loop1.addStatement( loop12 );
		}

		// update rk_diffK with the new sensitivities:
		if( STATES && number == 3 ) {
			block->addStatement( rk_diffK.getRows(NX1+NX2,NX) == rk_dk3.getRows(index1*NX3-(NX1+NX2)*NX3,index1*NX3+NX3-(NX1+NX2)*NX3) );
		}
		else {
			ExportForLoop loop5( index3,0,NX3 );
			loop5.addStatement( tmp_index1 == index2*NX3+index3 );
			loop5.addStatement( rk_diffK.getElement(NX1+NX2+index3,index2) == rk_mat3.getElement(tmp_index1,0)*rk_b.getRow(0) );
			ExportForLoop loop6( index4,1,NX3 );
			loop6.addStatement( rk_diffK.getElement(NX1+NX2+index3,index2) += rk_mat3.getElement(tmp_index1,index4)*rk_b.getRow(index4) );
			loop5.addStatement(loop6);
			loop1.addStatement(loop5);
			block->addStatement( loop1 );
		}
		// update rk_diffsNew with the new sensitivities:
		if( grid.getNumIntervals() > 1 || !equidistantControlGrid() ) block->addStatement( std::string( "if( run == 0 ) {\n" ) );
		ExportForLoop loop8( index2,0,NX3 );
		if( STATES && number == 3 ) loop8.addStatement( std::string(rk_diffsNew3.get( index2,index1 )) + " = (" + index2.getName() + " == " + index1.getName() + "-" + toString(NX1+NX2) + ");\n" );

		if( STATES && number == 3 ) loop8.addStatement( rk_diffsNew3.getElement( index2,index1 ) += rk_diffK.getRow( NX1+NX2+index2 )*Bh );
		else if( STATES )	loop8.addStatement( rk_diffsNew3.getElement( index2,index1 ) == rk_diffK.getRow( NX1+NX2+index2 )*Bh );
		else		 		loop8.addStatement( rk_diffsNew3.getElement( index2,index1+NX ) == rk_diffK.getRow( NX1+NX2+index2 )*Bh );
		block->addStatement( loop8 );
		if( grid.getNumIntervals() > 1 || !equidistantControlGrid() ) block->addStatement( std::string( "}\n" ) );
	}

	return SUCCESSFUL_RETURN;
}


returnValue DiagonallyImplicitRKExport::prepareOutputSystem(	ExportStatementBlock& code )
{
	if( NX3 > 0 ) {
		DMatrix mat3 = formMatrix( M33, A33 );
		rk_mat3 = ExportVariable( "rk_mat3", mat3, STATIC_CONST_REAL );
		code.addDeclaration( rk_mat3 );
		// TODO: Ask Milan why this does NOT work properly !!
		rk_mat3 = ExportVariable( "rk_mat3", numStages*NX3, NX3, STATIC_CONST_REAL, ACADO_LOCAL );
		double h = (grid.getLastTime() - grid.getFirstTime())/grid.getNumIntervals();

		DMatrix sens = zeros<double>(NX3*NX3, numStages);
		uint i, j, k, s1, s2;
		for( i = 0; i < NX3; i++ ) {
			DVector vec(NX3);
			for( j = 0; j < numStages; j++ ) {
				for( k = 0; k < NX3; k++ ) {
					vec(k) = A33(k,i);
					for( s1 = 0; s1 < j; s1++ ) {
						for( s2 = 0; s2 < NX3; s2++ ) {
							vec(k) = vec(k) + AA(j,s1)*h*A33(k,s2)*sens(i*NX3+s2,s1);
						}
					}
				}
				DVector sol = mat3*vec;
				for( k = 0; k < NX3; k++ ) {
					sens(i*NX3+k,j) = sol(k);
				}
			}
		}
		rk_dk3 = ExportVariable( "rk_dk3", sens, STATIC_CONST_REAL );
		code.addDeclaration( rk_dk3 );
		// TODO: Ask Milan why this does NOT work properly !!
		rk_dk3 = ExportVariable( "rk_dk3", NX3*NX3, numStages, STATIC_CONST_REAL, ACADO_LOCAL );
	}

	return SUCCESSFUL_RETURN;
}


returnValue DiagonallyImplicitRKExport::setup( )
{
	returnValue IRKsetup = ForwardIRKExport::setup();

	int debugMode;
	get( INTEGRATOR_DEBUG_MODE, debugMode );

	int useOMP;
	get(CG_USE_OPENMP, useOMP);
	ExportStruct structWspace;
	structWspace = useOMP ? ACADO_LOCAL : ACADO_WORKSPACE;

	rk_A = ExportVariable( "rk_A", numStages*(NX2+NXA), NX2+NXA, REAL, structWspace );
	if ( (bool)debugMode == true && useOMP ) {
		return ACADOERROR( RET_INVALID_OPTION );
	}
	else {
		debug_mat = ExportVariable( "debug_mat", numStages*(NX2+NXA), NX2+NXA, REAL, ACADO_VARIABLES );
	}
	uint Xmax = NX1;
	if( NX2 > Xmax ) Xmax = NX2;
	if( NX3 > Xmax ) Xmax = NX3;
	rk_b = ExportVariable( "rk_b", Xmax+NXA, 1, REAL, structWspace );

	if( NX2 > 0 || NXA > 0 ) {
		solver->init( NX2+NXA );
		solver->setup();
		rk_auxSolver = solver->getGlobalExportVariable( numStages );
	}

    return IRKsetup;
}


CLOSE_NAMESPACE_ACADO

// end of file.
