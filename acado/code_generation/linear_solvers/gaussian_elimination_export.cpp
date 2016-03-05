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
 *    \file src/code_generation/gaussian_elimination_export.cpp
 *    \author Rien Quirynen
 *    \date 2012
 */

#include <acado/code_generation/linear_solvers/gaussian_elimination_export.hpp>

using namespace std;

BEGIN_NAMESPACE_ACADO

//
// PUBLIC MEMBER FUNCTIONS:
//

ExportGaussElim::ExportGaussElim( UserInteraction* _userInteraction,
									const std::string& _commonHeaderName
									) : ExportLinearSolver( _userInteraction,_commonHeaderName )
{
}

ExportGaussElim::~ExportGaussElim( )
{}

returnValue ExportGaussElim::getDataDeclarations(	ExportStatementBlock& declarations,
														ExportStruct dataStruct
														) const
{
	declarations.addDeclaration( rk_swap,dataStruct );			// needed for the row swaps
	if( REUSE ) {
		declarations.addDeclaration( rk_bPerm,dataStruct );		// reordered right-hand side
		if( TRANSPOSE ) {
			declarations.addDeclaration( rk_bPerm_trans,dataStruct );
		}
	}

	return SUCCESSFUL_RETURN;
}


returnValue ExportGaussElim::getFunctionDeclarations(	ExportStatementBlock& declarations
															) const
{
	declarations.addDeclaration( solve );
	declarations.addDeclaration( solveTriangular );
	if( REUSE ) {
		declarations.addDeclaration( solveReuse );
	}

	return SUCCESSFUL_RETURN;
}


returnValue ExportGaussElim::getCode(	ExportStatementBlock& code
											)
{

	if (nRightHandSides > 0) {
		if( !REUSE ) return ACADOERROR(RET_INVALID_OPTION);

		setupFactorization( solve, rk_swap, determinant, string("fabs") );
		code.addFunction( solve );

		setupSolveReuseComplete( solveReuse, rk_bPerm );
		code.addFunction( solveReuse );

		if( TRANSPOSE ) {
			setupSolveReuseTranspose( solveReuseTranspose, rk_bPerm_trans );
			code.addFunction( solveReuseTranspose );
		}
	}
	else {
		setupSolveUpperTriangular( solveTriangular );
		code.addFunction( solveTriangular );

		setupSolve( solve, solveTriangular, rk_swap, determinant, string("fabs") );
		code.addFunction( solve );

		if( REUSE ) { // Also export the extra function which reuses the factorization of the matrix A

			setupSolveReuse( solveReuse, solveTriangular, rk_bPerm );
			code.addFunction( solveReuse );
		}
//		if( TRANSPOSE ) return ACADOERROR( RET_NOT_YET_IMPLEMENTED );
		if( TRANSPOSE ) {
			setupSolveReuseTranspose( solveReuseTranspose, rk_bPerm_trans );
			code.addFunction( solveReuseTranspose );
		}
	}

	return SUCCESSFUL_RETURN;
}


returnValue ExportGaussElim::setupSolveUpperTriangular( ExportFunction& _solveTriangular ) {

	if (nRightHandSides > 0)
		return ACADOERROR(RET_INVALID_OPTION);

	uint run1, run2;
	// Solve the upper triangular system of equations:
	for( run1 = dim; run1 > 0; run1--) {
		for( run2 = dim-1; run2 > (run1-1); run2--) {
			_solveTriangular.addStatement( b.getRow( (run1-1) ) -= A.getSubMatrix( (run1-1),(run1-1)+1,run2,run2+1 ) * b.getRow( run2 ) );
		}
		_solveTriangular << "b[" << toString( (run1-1) ) << "] = b[" << toString( (run1-1) ) << "]/A[" << toString( (run1-1)*dim+(run1-1) ) << "];\n";
	}

	return SUCCESSFUL_RETURN;
}


returnValue ExportGaussElim::setupSolve( ExportFunction& _solve, ExportFunction& _solveTriangular, ExportVariable& _swap, ExportVariable& _determinant, const string& absF ) {

	uint run1, run2, run3;
	ExportIndex i( "i" );
	_solve.addIndex( i );
	ExportIndex j( "j" );
	ExportIndex k( "k" );
	ExportVariable indexMax( "indexMax", 1, 1, INT, ACADO_LOCAL, true );
	ExportVariable intSwap( "intSwap", 1, 1, INT, ACADO_LOCAL, true );
	ExportVariable valueMax( "valueMax", 1, 1, REAL, ACADO_LOCAL, true );
	ExportVariable temp( "temp", 1, 1, REAL, ACADO_LOCAL, true );
	if( !UNROLLING ) {
		_solve.addIndex( j );
		_solve.addIndex( k );
		_solve.addDeclaration( indexMax );
		if( REUSE ) _solve.addDeclaration( intSwap );
		_solve.addDeclaration( valueMax );
		_solve.addDeclaration( temp );
	}

	if (nRightHandSides > 0)
		return ACADOERROR(RET_INVALID_OPTION);

	// initialise rk_perm (the permutation vector)
	if( REUSE ) {
		ExportForLoop loop1( i,0,dim );
		loop1 << rk_perm.get( 0,i ) << " = " << i.getName() << ";\n";
		_solve.addStatement( loop1 );
	}

	_solve.addStatement( _determinant == 1 );
	if( UNROLLING || dim <= 5 ) {
		// Start the factorization:
		for( run1 = 0; run1 < (dim-1); run1++ ) {
			// Search for pivot in column run1:
			for( run2 = run1+1; run2 < dim; run2++ ) {
				// add the test (if or else if):
				stringstream test;
				if( run2 == (run1+1) ) {
					test << "if(";
				} else {
					test << "else if(";
				}
				test << absF << "(A[" << toString( run2*dim+run1 ) << "]) > " << absF << "(A[" << toString( run1*dim+run1 ) << "])";
				for( run3 = run1+1; run3 < dim; run3++ ) {
					if( run3 != run2) {
						test << " && " << absF << "(A[" << toString( run2*dim+run1 ) << "]) > " << absF << "(A[" << toString( run3*dim+run1 ) << "])";
					}
				}
				test << ") {\n";
				_solve.addStatement( test.str() );

				// do the row swaps:
				// for A:
				for( run3 = 0; run3 < dim; run3++ ) {
					_solve.addStatement( _swap == A.getSubMatrix( run1,run1+1,run3,run3+1 ) );
					_solve.addStatement( A.getSubMatrix( run1,run1+1,run3,run3+1 ) == A.getSubMatrix( run2,run2+1,run3,run3+1 ) );
					_solve.addStatement( A.getSubMatrix( run2,run2+1,run3,run3+1 ) == _swap );
				}
				// for b:
				_solve.addStatement( _swap == b.getRow( run1 ) );
				_solve.addStatement( b.getRow( run1 ) == b.getRow( run2 ) );
				_solve.addStatement( b.getRow( run2 ) == _swap );

				if( REUSE ) { // rk_perm also needs to be updated if it needs to be possible to reuse the factorization
					_solve.addStatement( intSwap == rk_perm.getCol( run1 ) );
					_solve.addStatement( rk_perm.getCol( run1 ) == rk_perm.getCol( run2 ) );
					_solve.addStatement( rk_perm.getCol( run2 ) == intSwap );
				}

				_solve.addStatement( "}\n" );
			}
			// potentially needed row swaps are done
			_solve.addLinebreak();
			// update of the next rows:
			for( run2 = run1+1; run2 < dim; run2++ ) {
				_solve << "A[" << toString( run2*dim+run1 ) << "] = -A[" << toString( run2*dim+run1 ) << "]/A[" << toString( run1*dim+run1 ) << "];\n";
				_solve.addStatement( A.getSubMatrix( run2,run2+1,run1+1,dim ) += A.getSubMatrix( run2,run2+1,run1,run1+1 ) * A.getSubMatrix( run1,run1+1,run1+1,dim ) );
				_solve.addStatement( b.getRow( run2 ) += A.getSubMatrix( run2,run2+1,run1,run1+1 ) * b.getRow( run1 ) );
				_solve.addLinebreak();
			}
			_solve.addStatement( _determinant == _determinant*A.getSubMatrix(run1,run1+1,run1,run1+1) );
			_solve.addLinebreak();
		}
		_solve.addStatement( _determinant == _determinant*A.getSubMatrix(dim-1,dim,dim-1,dim) );
		_solve.addLinebreak();
	}
	else { // without UNROLLING:
		_solve << "for( i=0; i < (" << toString( dim-1 ) << "); i++ ) {\n";
		_solve << "	indexMax = i;\n";
		_solve << "	valueMax = " << absF << "(A[i*" << toString( dim ) << "+i]);\n";
		_solve << "	for( j=(i+1); j < " << toString( dim ) << "; j++ ) {\n";
		_solve << "		temp = " << absF << "(A[j*" << toString( dim ) << "+i]);\n";
		_solve << "		if( temp > valueMax ) {\n";
		_solve << "			indexMax = j;\n";
		_solve << "			valueMax = temp;\n";
		_solve << "		}\n";
		_solve << "	}\n";
		_solve << "	if( indexMax > i ) {\n";
		ExportForLoop loop2( k,0,dim );
		loop2 << "	" << _swap.getFullName() << " = A[i*" << toString( dim ) << "+" << k.getName() << "];\n";
		loop2 << "	A[i*" << toString( dim ) << "+" << k.getName() << "] = A[indexMax*" << toString( dim ) << "+" << k.getName() << "];\n";
		loop2 << "	A[indexMax*" << toString( dim ) << "+" << k.getName() << "] = " << _swap.getFullName() << ";\n";
		_solve.addStatement( loop2 );
		_solve << "	" << _swap.getFullName() << " = b[i];\n";
		_solve << "	b[i] = b[indexMax];\n";
		_solve << "	b[indexMax] = " << _swap.getFullName() << ";\n";
		if( REUSE ) {
			_solve << "	" << intSwap.getFullName() << " = " << rk_perm.getFullName() << "[i];\n";
			_solve << "	" << rk_perm.getFullName() << "[i] = " << rk_perm.getFullName() << "[indexMax];\n";
			_solve << "	" << rk_perm.getFullName() << "[indexMax] = " << intSwap.getFullName() << ";\n";
		}
		_solve << "	}\n";
		_solve << "	" << _determinant.getFullName() << " *= A[i*" << toString( dim ) << "+i];\n";
		_solve << "	for( j=i+1; j < " << toString( dim ) << "; j++ ) {\n";
		_solve << "		A[j*" << toString( dim ) << "+i] = -A[j*" << toString( dim ) << "+i]/A[i*" << toString( dim ) << "+i];\n";
		_solve << "		for( k=i+1; k < " << toString( dim ) << "; k++ ) {\n";
		_solve << "			A[j*" << toString( dim ) << "+k] += A[j*" << toString( dim ) << "+i] * A[i*" << toString( dim ) << "+k];\n";
		_solve << "		}\n";
		_solve << "		b[j] += A[j*" << toString( dim ) << "+i] * b[i];\n";
		_solve << "	}\n";
		_solve << "}\n";
		_solve << _determinant.getFullName() << " *= A[" << toString( (dim-1)*dim+(dim-1) ) << "];\n";
	}
	_solve << _determinant.getFullName() << " = " << absF << "(" << _determinant.getFullName() << ");\n";

	_solve.addFunctionCall( _solveTriangular, A, b );

	return SUCCESSFUL_RETURN;
}


returnValue ExportGaussElim::setupFactorization( ExportFunction& _solve, ExportVariable& _swap, ExportVariable& _determinant, const string& absF ) {

	uint run1, run2, run3;
	ExportIndex i( "i" );
	_solve.addIndex( i );
	ExportIndex j( "j" );
	ExportIndex k( "k" );
	ExportVariable indexMax( "indexMax", 1, 1, INT, ACADO_LOCAL, true );
	ExportVariable intSwap( "intSwap", 1, 1, INT, ACADO_LOCAL, true );
	ExportVariable valueMax( "valueMax", 1, 1, REAL, ACADO_LOCAL, true );
	ExportVariable temp( "temp", 1, 1, REAL, ACADO_LOCAL, true );
	if( !UNROLLING ) {
		_solve.addIndex( j );
		_solve.addIndex( k );
		_solve.addDeclaration( indexMax );
		if( REUSE ) _solve.addDeclaration( intSwap );
		_solve.addDeclaration( valueMax );
		_solve.addDeclaration( temp );
	}

	// initialise rk_perm (the permutation vector)
	if( REUSE ) {
		ExportForLoop loop1( i,0,dim );
		loop1 << rk_perm.get( 0,i ) << " = " << i.getName() << ";\n";
		_solve.addStatement( loop1 );
	}

	_solve.addStatement( _determinant == 1 );
	if( UNROLLING || dim <= 5 ) {
		// Start the factorization:
		for( run1 = 0; run1 < (dim-1); run1++ ) {
			// Search for pivot in column run1:
			for( run2 = run1+1; run2 < dim; run2++ ) {
				// add the test (if or else if):
				stringstream test;
				if( run2 == (run1+1) ) {
					test << "if(";
				} else {
					test << "else if(";
				}
				test << absF << "(A[" << toString( run2*dim+run1 ) << "]) > " << absF << "(A[" << toString( run1*dim+run1 ) << "])";
				for( run3 = run1+1; run3 < dim; run3++ ) {
					if( run3 != run2) {
						test << " && " << absF << "(A[" << toString( run2*dim+run1 ) << "]) > " << absF << "(A[" << toString( run3*dim+run1 ) << "])";
					}
				}
				test << ") {\n";
				_solve.addStatement( test.str() );

				// do the row swaps:
				// for A:
				for( run3 = 0; run3 < dim; run3++ ) {
					_solve.addStatement( _swap == A.getSubMatrix( run1,run1+1,run3,run3+1 ) );
					_solve.addStatement( A.getSubMatrix( run1,run1+1,run3,run3+1 ) == A.getSubMatrix( run2,run2+1,run3,run3+1 ) );
					_solve.addStatement( A.getSubMatrix( run2,run2+1,run3,run3+1 ) == _swap );
				}

				if( REUSE ) { // rk_perm also needs to be updated if it needs to be possible to reuse the factorization
					_solve.addStatement( intSwap == rk_perm.getCol( run1 ) );
					_solve.addStatement( rk_perm.getCol( run1 ) == rk_perm.getCol( run2 ) );
					_solve.addStatement( rk_perm.getCol( run2 ) == intSwap );
				}

				_solve.addStatement( "}\n" );
			}
			// potentially needed row swaps are done
			_solve.addLinebreak();
			// update of the next rows:
			for( run2 = run1+1; run2 < dim; run2++ ) {
				_solve << "A[" << toString( run2*dim+run1 ) << "] = -A[" << toString( run2*dim+run1 ) << "]/A[" << toString( run1*dim+run1 ) << "];\n";
				_solve.addStatement( A.getSubMatrix( run2,run2+1,run1+1,dim ) += A.getSubMatrix( run2,run2+1,run1,run1+1 ) * A.getSubMatrix( run1,run1+1,run1+1,dim ) );
				_solve.addLinebreak();
			}
			_solve.addStatement( _determinant == _determinant*A.getSubMatrix(run1,run1+1,run1,run1+1) );
			_solve.addLinebreak();
		}
		_solve.addStatement( _determinant == _determinant*A.getSubMatrix(dim-1,dim,dim-1,dim) );
		_solve.addLinebreak();
	}
	else { // without UNROLLING:
		_solve << "for( i=0; i < (" << toString( dim-1 ) << "); i++ ) {\n";
		_solve << "	indexMax = i;\n";
		_solve << "	valueMax = " << absF << "(A[i*" << toString( dim ) << "+i]);\n";
		_solve << "	for( j=(i+1); j < " << toString( dim ) << "; j++ ) {\n";
		_solve << "		temp = " << absF << "(A[j*" << toString( dim ) << "+i]);\n";
		_solve << "		if( temp > valueMax ) {\n";
		_solve << "			indexMax = j;\n";
		_solve << "			valueMax = temp;\n";
		_solve << "		}\n";
		_solve << "	}\n";
		_solve << "	if( indexMax > i ) {\n";
		ExportForLoop loop2( k,0,dim );
		loop2 << "	" << _swap.getFullName() << " = A[i*" << toString( dim ) << "+" << k.getName() << "];\n";
		loop2 << "	A[i*" << toString( dim ) << "+" << k.getName() << "] = A[indexMax*" << toString( dim ) << "+" << k.getName() << "];\n";
		loop2 << "	A[indexMax*" << toString( dim ) << "+" << k.getName() << "] = " << _swap.getFullName() << ";\n";
		_solve.addStatement( loop2 );
		if( REUSE ) {
			_solve << "	" << intSwap.getFullName() << " = " << rk_perm.getFullName() << "[i];\n";
			_solve << "	" << rk_perm.getFullName() << "[i] = " << rk_perm.getFullName() << "[indexMax];\n";
			_solve << "	" << rk_perm.getFullName() << "[indexMax] = " << intSwap.getFullName() << ";\n";
		}
		_solve << "	}\n";
		_solve << "	" << _determinant.getFullName() << " *= A[i*" << toString( dim ) << "+i];\n";
		_solve << "	for( j=i+1; j < " << toString( dim ) << "; j++ ) {\n";
		_solve << "		A[j*" << toString( dim ) << "+i] = -A[j*" << toString( dim ) << "+i]/A[i*" << toString( dim ) << "+i];\n";
		_solve << "		for( k=i+1; k < " << toString( dim ) << "; k++ ) {\n";
		_solve << "			A[j*" << toString( dim ) << "+k] += A[j*" << toString( dim ) << "+i] * A[i*" << toString( dim ) << "+k];\n";
		_solve << "		}\n";
		_solve << "	}\n";
		_solve << "}\n";
		_solve << _determinant.getFullName() << " *= A[" << toString( (dim-1)*dim+(dim-1) ) << "];\n";
	}
	_solve << _determinant.getFullName() << " = " << absF << "(" << _determinant.getFullName() << ");\n";
	
	return SUCCESSFUL_RETURN;
}


returnValue ExportGaussElim::setupSolveReuse( ExportFunction& _solveReuse, ExportFunction& _solveTriangular, ExportVariable& _bPerm ) {

	uint run1, run2;

	if (nRightHandSides > 0)
		return ACADOERROR(RET_INVALID_OPTION);

	for( run1 = 0; run1 < dim; run1++ ) {
		_solveReuse << _bPerm.get( run1,0 ) << " = b[" << rk_perm.getFullName() << "[" << toString( run1 ) << "]];\n";
	}

	for( run2 = 1; run2 < dim; run2++ ) { 		// row run2
		for( run1 = 0; run1 < run2; run1++ ) { 	// column run1
			_solveReuse << _bPerm.get( run2,0 ) << " += A[" << toString( run2*dim+run1 ) << "]*" << _bPerm.getFullName() << "[" << toString( run1 ) << "];\n";
		}
		_solveReuse.addLinebreak();
	}
	_solveReuse.addLinebreak();

	_solveReuse.addFunctionCall( _solveTriangular, A, _bPerm );
	_solveReuse.addStatement( b == _bPerm );

	return SUCCESSFUL_RETURN;
}


returnValue ExportGaussElim::setupSolveReuseComplete( ExportFunction& _solveReuse, ExportVariable& _bPerm ) {

	ExportIndex run1( "i" );
	ExportIndex run2( "j" );
	ExportIndex tmp_index1( "index1" );
	ExportIndex tmp_index2( "index2" );
	ExportVariable tmp( "tmp_var", 1, 1, _bPerm.getType(), ACADO_LOCAL, true );
	_solveReuse.addIndex( run1 );
	_solveReuse.addIndex( run2 );
	_solveReuse.addIndex( tmp_index1 );
	_solveReuse.addIndex( tmp_index2 );
	_solveReuse.addDeclaration(tmp);
	uint run3;

	if (nRightHandSides <= 0)
		return ACADOERROR(RET_INVALID_OPTION);

	ExportForLoop loop1( run1, 0, dim );
	loop1 << run2.getName() << " = " << rk_perm.getFullName() << "[" << run1.getName() << "]*" << toString(nRightHandSides) << ";\n";
	for( run3 = 0; run3 < nRightHandSides; run3++ ) {
		loop1 << _bPerm.get( run1,run3 ) << " = b[" << run2.getName() << "+" << toString(run3) << "];\n";
	}
	_solveReuse.addStatement( loop1 );

	ExportForLoop loop2( run2, 1, dim );	// row run2
	loop2.addStatement( tmp_index1 == run2*nRightHandSides );
	ExportForLoop loop3( run1, 0, run2 );	// column run1
	loop3.addStatement( tmp_index2 == run1*nRightHandSides );
	loop3.addStatement( tmp == A.getElement(run2,run1) );
	for( run3 = 0; run3 < nRightHandSides; run3++ ) {
//		loop3.addStatement( _bPerm.getElement( run2,run3 ) += tmp * _bPerm.getElement( run1,run3 ) );
		loop3 << _bPerm.getFullName() << "[" << tmp_index1.getName() << "+" << toString(run3) << "] += " << tmp.getName() << "*" << _bPerm.getFullName() << "[" << tmp_index2.getName() << "+" << toString(run3) << "];\n";
	}
	loop2.addStatement( loop3 );
	_solveReuse.addStatement( loop2 );


	// Solve the upper triangular system of equations:
	ExportForLoop loop4( run1, dim-1, -1, -1 );
	loop4.addStatement( tmp_index1 == run1*nRightHandSides );
	ExportForLoop loop5( run2, dim-1, run1, -1 );
	loop5.addStatement( tmp_index2 == run2*nRightHandSides );
	loop5.addStatement( tmp == A.getElement( run1,run2 ) );
	for( run3 = 0; run3 < nRightHandSides; run3++ ) {
//		loop5.addStatement( _bPerm.getElement( run1,run3 ) -= tmp * _bPerm.getElement( run2,run3 ) );
		loop5 << _bPerm.getFullName() << "[" << tmp_index1.getName() << "+" << toString(run3) << "] -= " << tmp.getName() << "*" << _bPerm.getFullName() << "[" << tmp_index2.getName() << "+" << toString(run3) << "];\n";
	}
	loop4.addStatement( loop5 );
	loop4 << tmp.getName() << " = 1.0/A[" << run1.getName() << "*" << toString(dim+1) << "];\n";
	for( run3 = 0; run3 < nRightHandSides; run3++ ) {
//		loop4 << _bPerm.get( run1,run3 ) << " = " << _bPerm.get( run1,run3 ) << "*" << tmp.getName() << ";\n";
		loop4 << _bPerm.getFullName() << "[" << tmp_index1.getName() << "+" << toString(run3) << "] = " << tmp.getName() << "*" << _bPerm.getFullName() << "[" << tmp_index1.getName() << "+" << toString(run3) << "];\n";
	}
	_solveReuse.addStatement( loop4 );

	_solveReuse.addStatement( b == _bPerm );

	return SUCCESSFUL_RETURN;
}


returnValue ExportGaussElim::setupSolveReuseTranspose( ExportFunction& _solveReuse, ExportVariable& _bPerm ) {

	ExportIndex run1( "i" );
	ExportIndex run2( "j" );
	ExportVariable tmp( "tmp_var", 1, 1, _bPerm.getType(), ACADO_LOCAL, true );
	_solveReuse.addIndex( run1 );
	_solveReuse.addIndex( run2 );
	_solveReuse.addDeclaration(tmp);

	_solveReuse.addStatement( _bPerm == b_trans );

	ExportForLoop loop2( run2, 0, dim );	// row run2
	ExportForLoop loop3( run1, 0, run2 );	// column run1
	loop3.addStatement( _bPerm.getRow(run2) -= A.getElement(run1,run2)*_bPerm.getRow(run1) );
	loop2.addStatement( loop3 );
	loop2 << tmp.getName() << " = 1.0/A[" << run2.getName() << "*" << toString(dim+1) << "];\n";
	loop2.addStatement( _bPerm.getRow(run2) == _bPerm.getRow(run2)*tmp );
	_solveReuse.addStatement( loop2 );


	// Solve the upper triangular system of equations:
	ExportForLoop loop4( run1, dim-1, -1, -1 );
	ExportForLoop loop5( run2, dim-1, run1, -1 );
	loop5.addStatement( _bPerm.getRow(run1) += A.getElement(run2,run1)*_bPerm.getRow(run2) );
	loop4.addStatement( loop5 );
	_solveReuse.addStatement( loop4 );


	// The permutation now happens HERE!
	ExportForLoop loop1( run1, 0, dim );
	loop1 << run2.getName() << " = " << rk_perm.getFullName() << "[" << run1.getName() << "];\n";
	loop1.addStatement( b_trans.getRow(run2) == _bPerm.getRow(run1) );
	_solveReuse.addStatement( loop1 );

	return SUCCESSFUL_RETURN;
}


returnValue ExportGaussElim::appendVariableNames( stringstream& string ) {

	string << ", " << rk_swap.getFullName();
	if( REUSE ) {
//		string << ", " << rk_perm.getFullName().getName();
		string << ", " << rk_bPerm.getFullName();
	}

	return SUCCESSFUL_RETURN;
}


returnValue ExportGaussElim::setup( )
{
	// Other cases are not implemented...
	ASSERT_RETURN(nCols == nRows);

	int useOMP;
	get(CG_USE_OPENMP, useOMP);
	ExportStruct structWspace;
	structWspace = useOMP ? ACADO_LOCAL : ACADO_WORKSPACE;

	rk_swap = ExportVariable( std::string( "rk_" ) + identifier + "swap", 1, 1, REAL, structWspace, true );
	A = ExportVariable( "A", dim, dim, REAL );
	if (nRightHandSides > 0) {
		b = ExportVariable( "b", dim, nRightHandSides, REAL );
		rk_bPerm = ExportVariable( std::string( "rk_" ) + identifier + "bPerm", dim, nRightHandSides, REAL, structWspace );
		solve = ExportFunction( getNameSolveFunction(), A, rk_perm );
	}
	else {
		b = ExportVariable( "b", dim, 1, REAL );
		rk_bPerm = ExportVariable( std::string( "rk_" ) + identifier + "bPerm", dim, 1, REAL, structWspace );
		solve = ExportFunction( getNameSolveFunction(), A, b, rk_perm );
	}
	rk_perm = ExportVariable( "rk_perm", 1, dim, INT );
	solve.setReturnValue( determinant, false );
	solve.addLinebreak( );	// FIX: TO MAKE SURE IT GETS EXPORTED
	if (nRightHandSides <= 0) {
		solveTriangular = ExportFunction( std::string( "solve_" ) + identifier + "triangular", A, b );
		solveTriangular.addLinebreak( );	// FIX: TO MAKE SURE IT GETS EXPORTED
	}
	
	if( REUSE ) {
		solveReuse = ExportFunction( getNameSolveReuseFunction(), A, b, rk_perm );
		solveReuse.addLinebreak( );	// FIX: TO MAKE SURE IT GETS EXPORTED
		if( TRANSPOSE ) {
			b_trans = ExportVariable( "b", dim, 1, REAL );
			rk_bPerm_trans = ExportVariable( std::string( "rk_" ) + identifier + "bPerm_trans", dim, 1, REAL, structWspace );
			solveReuseTranspose = ExportFunction( getNameSolveTransposeReuseFunction(), A, b, rk_perm );
			solveReuseTranspose.addLinebreak( );	// FIX: TO MAKE SURE IT GETS EXPORTED
		}
	}
	
	int unrollOpt;
	userInteraction->get( UNROLL_LINEAR_SOLVER, unrollOpt );
	UNROLLING = (bool) unrollOpt;

    return SUCCESSFUL_RETURN;
}


ExportVariable ExportGaussElim::getGlobalExportVariable( const uint factor ) const {

	int useOMP;
	get(CG_USE_OPENMP, useOMP);
	ExportStruct structWspace;
	structWspace = useOMP ? ACADO_LOCAL : ACADO_WORKSPACE;

	return ExportVariable( std::string( "rk_" ) + identifier + "perm", factor, dim, INT, structWspace );
}


//
// PROTECTED MEMBER FUNCTIONS:
//



CLOSE_NAMESPACE_ACADO

// end of file.
