/*
 *    This file is part of ACADO Toolkit.
 *
 *    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
 *    Copyright (C) 2008-2009 by Boris Houska and Hans Joachim Ferreau, K.U.Leuven.
 *    Developed within the Optimization in Engineering Center (OPTEC) under
 *    supervision of Moritz Diehl. All rights reserved.
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

#include <acado/code_generation/gaussian_elimination_export.hpp>


BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

ExportGaussElim::ExportGaussElim( UserInteraction* _userInteraction,
									const String& _commonHeaderName
									) : ExportLinearSolver( _userInteraction,_commonHeaderName )
{
}


ExportGaussElim::ExportGaussElim( const ExportGaussElim& arg ) : ExportLinearSolver( arg )
{
}


ExportGaussElim::~ExportGaussElim( )
{
}


ExportGaussElim& ExportGaussElim::operator=( const ExportGaussElim& arg )
{
	if( this != &arg )
	{
		ExportLinearSolver::operator=( arg );
	}

	return *this;
}


returnValue ExportGaussElim::getDataDeclarations(	ExportStatementBlock& declarations,
														ExportStruct dataStruct
														) const
{
	declarations.addDeclaration( rk_swap,dataStruct );			// needed for the row swaps
	if( REUSE == BT_TRUE ) {
		declarations.addDeclaration( rk_perm,dataStruct );		// permutation vector
		declarations.addDeclaration( rk_bPerm,dataStruct );		// reordered right-hand side
	}

	return SUCCESSFUL_RETURN;
}


returnValue ExportGaussElim::getFunctionDeclarations(	ExportStatementBlock& declarations
															) const
{
	declarations.addDeclaration( solve );
	if( REUSE == BT_TRUE ) {
		declarations.addDeclaration( solveReuse );
	}

	return SUCCESSFUL_RETURN;
}


returnValue ExportGaussElim::getCode(	ExportStatementBlock& code
											)
{
	uint run1, run2, run3;
	if( !UNROLLING ) {
		solve.addStatement( String( "int indexMax, i, j, k;\n" ) );
		solve.addStatement( String( "real_t valueMax, temp;\n" ) );
	}
	
	// initialise rk_perm (the permutation vector)
	if( REUSE == BT_TRUE ) {
		for( run1 = 0; run1 < dim; run1++ ) {
			solve.addStatement( String( "acadoWorkspace.rk_" ) << identifier << "perm[" << String( run1 ) << String( "] = " ) << String( run1 ) << String( ";\n" ) );
		}
	}
	
	if( UNROLLING || dim <= 5 ) {
		// Start the factorization:
		for( run1 = 0; run1 < (dim-1); run1++ ) {
			// Search for pivot in column run1:
			for( run2 = run1+1; run2 < dim; run2++ ) {
				// add the test (if or else if):
				String test;
				if( run2 == (run1+1) ) {
					test << "if(";
				} else {
					test << "else if(";
				}
				test << "fabs(A[" << String( run2*dim+run1 ) << "]) > fabs(A[" << String( run1*dim+run1 ) << "])";
				for( run3 = run1+1; run3 < dim; run3++ ) {
					if( run3 != run2) {
						test << " && fabs(A[" << String( run2*dim+run1 ) << "]) > fabs(A[" << String( run3*dim+run1 ) << "])";
					}
				}
				test << ") {\n";
				solve.addStatement( test );
			
				// do the row swaps:
				// for A:
				for( run3 = 0; run3 < dim; run3++ ) {
					solve.addStatement( rk_swap == A.getSubMatrix( run1,run1+1,run3,run3+1 ) );
					solve.addStatement( A.getSubMatrix( run1,run1+1,run3,run3+1 ) == A.getSubMatrix( run2,run2+1,run3,run3+1 ) );
					solve.addStatement( A.getSubMatrix( run2,run2+1,run3,run3+1 ) == rk_swap );
				}
				// for b:
				solve.addStatement( rk_swap == b.getRow( run1 ) );
				solve.addStatement( b.getRow( run1 ) == b.getRow( run2 ) );
				solve.addStatement( b.getRow( run2 ) == rk_swap );
			
				if( REUSE == BT_TRUE ) { // rk_perm also needs to be updated if it needs to be possible to reuse the factorization
					solve.addStatement( rk_swap == rk_perm.getRow( run1 ) );
					solve.addStatement( rk_perm.getRow( run1 ) == rk_perm.getRow( run2 ) );
					solve.addStatement( rk_perm.getRow( run2 ) == rk_swap );
				}
			
				solve.addStatement( String( "}\n" ) );
			}
			// potentially needed row swaps are done
			solve.addLinebreak();
			// update of the next rows:
			for( run2 = run1+1; run2 < dim; run2++ ) {
				solve.addStatement( String( "A[" ) << String( run2*dim+run1 ) << "] = -A[" << String( run2*dim+run1 ) << "]/A[" << String( run1*dim+run1 ) << "];\n" );
				solve.addStatement( A.getSubMatrix( run2,run2+1,run1+1,dim ) += A.getSubMatrix( run2,run2+1,run1,run1+1 ) * A.getSubMatrix( run1,run1+1,run1+1,dim ) );
				solve.addStatement( b.getRow( run2 ) += A.getSubMatrix( run2,run2+1,run1,run1+1 ) * b.getRow( run1 ) );
				solve.addLinebreak();
			}
			solve.addLinebreak();
		}
		solve.addLinebreak();
	}
	else { // without UNROLLING:
		solve.addStatement( String( "for( i=0; i < (" ) << String( dim-1 ) << "); i++ ) {\n" );
		solve.addStatement( String( "	indexMax = i;\n") );
		solve.addStatement( String( "	valueMax = fabs(A[i*" ) << String( dim ) << "+i]);\n" );
		solve.addStatement( String( "	for( j=(i+1); j < " ) << String( dim ) << "; j++ ) {\n" );
		solve.addStatement( String( "		temp = fabs(A[j*" ) << String( dim ) << "+i]);\n" );
		solve.addStatement( String( "		if( temp > valueMax ) {\n" ) );
		solve.addStatement( String( "			indexMax = j;\n" ) );
		solve.addStatement( String( "			valueMax = temp;\n" ) );
		solve.addStatement( String( "		}\n" ) );
		solve.addStatement( String( "	}\n" ) );
		solve.addStatement( String( "	if( indexMax > i ) {\n" ) );
		for( run1 = 0; run1 < dim; run1++ ) {
			solve.addStatement( String( "	acadoWorkspace.rk_" ) << identifier << "swap = A[i*" << String( dim ) << "+" << String( run1 ) << "];\n" );
			solve.addStatement( String( "	A[i*" ) << String( dim ) << "+" << String( run1 ) << "] = A[indexMax*" << String( dim ) << "+" << String( run1 ) << "];\n" );
			solve.addStatement( String( "	A[indexMax*" ) << String( dim ) << "+" << String( run1 ) << "] = acadoWorkspace.rk_" << identifier << "swap;\n" );
		}
		solve.addStatement( String( "	acadoWorkspace.rk_" ) << identifier << "swap = b[i];\n" );
		solve.addStatement( String( "	b[i] = b[indexMax];\n" ) );
		solve.addStatement( String( "	b[indexMax] = acadoWorkspace.rk_" ) << identifier << "swap;\n" );
		if( REUSE == BT_TRUE ) {
			solve.addStatement( String( "	acadoWorkspace.rk_" ) << identifier << "swap = acadoWorkspace.rk_" << identifier << "perm[i];\n" );
			solve.addStatement( String( "	acadoWorkspace.rk_" ) << identifier << "perm[i] = acadoWorkspace.rk_" << identifier << "perm[indexMax];\n" );
			solve.addStatement( String( "	acadoWorkspace.rk_" ) << identifier << "perm[indexMax] = acadoWorkspace.rk_" << identifier << "swap;\n" );
		}
		solve.addStatement( String( "	}\n" ) );
		solve.addStatement( String( "	for( j=i+1; j < " ) << String( dim ) << "; j++ ) {\n" );
		solve.addStatement( String( "		A[j*" ) << String( dim ) << "+i] = -A[j*" << String( dim ) << "+i]/A[i*" << String( dim ) << "+i];\n" );
		solve.addStatement( String( "		for( k=i+1; k < " ) << String( dim ) << "; k++ ) {\n" );
		solve.addStatement( String( "			A[j*" ) << String( dim ) << "+k] += A[j*" << String( dim ) << "+i] * A[i*" << String( dim ) << "+k];\n" );
		solve.addStatement( String( "		}\n" ) );
		solve.addStatement( String( "		b[j] += A[j*" ) << String( dim ) << "+i] * b[i];\n" );
		solve.addStatement( String( "	}\n" ) );
		solve.addStatement( String( "}\n" ) );
	}
	
	// Solve the upper triangular system of equations which is left, using back substitution:
	for( run1 = dim; run1 > 0; run1--) {
		solve.addStatement( x.getRow( (run1-1) ) == b.getRow( (run1-1) ) );
		for( run2 = dim-1; run2 > (run1-1); run2--) {
			solve.addStatement( x.getRow( (run1-1) ) -= A.getSubMatrix( (run1-1),(run1-1)+1,run2,run2+1 ) * x.getRow( run2 ) );
		}
		solve.addStatement( String( "x[" ) << String( (run1-1) ) << "] = x[" << String( (run1-1) ) << "]/A[" << String( (run1-1)*dim+(run1-1) ) << "];\n" );
	}
	
	code.addFunction( solve );
	
    code.addLinebreak( 2 );
	if( REUSE == BT_TRUE ) { // Also export the extra function which reuses the factorization of the matrix A
		
		for( run1 = 0; run1 < dim; run1++ ) {
			solveReuse.addStatement( ((String)rk_bPerm.get( run1,0 ) << " = b[acadoWorkspace.rk_" << identifier << "perm[" << String( run1 ) << "]];\n" ) );
		}
		
		for( run1 = 0; run1 < (dim-1); run1++ ) { 		// column run1
			for( run2 = run1+1; run2 < dim; run2++ ) { 	// update row run2
				solveReuse.addStatement( ((String)rk_bPerm.get( run2,0 ) << " += A[" << String( run2*dim+run1 ) << "]*acadoWorkspace.rk_" << identifier << "bPerm[" << String( run1 ) << "];\n" ) );
			}
			solveReuse.addLinebreak();
		}
		solveReuse.addLinebreak();
		
		// Solve the upper triangular system of equations:
		for( run1 = dim; run1 > 0; run1--) {
			solveReuse.addStatement( String( "x[" ) << String( (run1-1) ) << "] = " << rk_bPerm.get( run1-1,0 ) << ";\n" );
			for( run2 = dim-1; run2 > (run1-1); run2--) {
				solveReuse.addStatement( x.getRow( (run1-1) ) -= A.getSubMatrix( (run1-1),(run1-1)+1,run2,run2+1 ) * x.getRow( run2 ) );
			}
			solveReuse.addStatement( String( "x[" ) << String( (run1-1) ) << "] = x[" << String( (run1-1) ) << "]/A[" << String( (run1-1)*dim+(run1-1) ) << "];\n" );
		}
		
		code.addFunction( solveReuse );
	}
	
	return SUCCESSFUL_RETURN;
}


returnValue ExportGaussElim::setup( )
{
	rk_swap = ExportVariable( String( "rk_" ) << identifier << "swap", 1, 1, REAL, ACADO_WORKSPACE, BT_TRUE );
	rk_perm = ExportVariable( String( "rk_" ) << identifier << "perm", dim, 1, INT, ACADO_WORKSPACE );
	rk_bPerm = ExportVariable( String( "rk_" ) << identifier << "bPerm", dim, 1, REAL, ACADO_WORKSPACE );
	A = ExportVariable( "A", dim, dim, REAL );
	b = ExportVariable( "b", dim, 1, REAL );
	x = ExportVariable( "x", dim, 1, REAL );
	solve = ExportFunction( getNameSolveFunction(), A, b, x);
	solve.addLinebreak( );	// FIX: TO MAKE SURE IT GETS EXPORTED
	
	if( REUSE == BT_TRUE ) {
		solveReuse = ExportFunction( getNameSolveReuseFunction(), A, b, x);
		solveReuse.addLinebreak( );	// FIX: TO MAKE SURE IT GETS EXPORTED
	}
	
	int unrollOpt;
	userInteraction->get( UNROLL_LINEAR_SOLVER, unrollOpt );
	UNROLLING = (BooleanType) unrollOpt;

    return SUCCESSFUL_RETURN;
}


//
// PROTECTED MEMBER FUNCTIONS:
//



CLOSE_NAMESPACE_ACADO

// end of file.
