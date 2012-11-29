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
 *    \file src/code_generation/householder_qr_export.cpp
 *    \author Rien Quirynen
 *    \date 2012
 */

#include <acado/code_generation/linear_solvers/householder_qr_export.hpp>


BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

ExportHouseholderQR::ExportHouseholderQR( UserInteraction* _userInteraction,
									const String& _commonHeaderName
									) : ExportLinearSolver( _userInteraction,_commonHeaderName )
{
}


ExportHouseholderQR::ExportHouseholderQR( const ExportHouseholderQR& arg ) : ExportLinearSolver( arg )
{
}


ExportHouseholderQR::~ExportHouseholderQR( )
{
}


ExportHouseholderQR& ExportHouseholderQR::operator=( const ExportHouseholderQR& arg )
{
	if( this != &arg )
	{
		ExportLinearSolver::operator=( arg );
	}

	return *this;
}


returnValue ExportHouseholderQR::getDataDeclarations(	ExportStatementBlock& declarations,
														ExportStruct dataStruct
														) const
{
	declarations.addDeclaration( rk_temp,dataStruct );

	return SUCCESSFUL_RETURN;
}


returnValue ExportHouseholderQR::getFunctionDeclarations(	ExportStatementBlock& declarations
															) const
{
	declarations.addDeclaration( solve );
	if( REUSE ) {
		declarations.addDeclaration( solveReuse );
	}

	return SUCCESSFUL_RETURN;
}


returnValue ExportHouseholderQR::getCode(	ExportStatementBlock& code
											)
{
	uint run1, run2, run3;
	ExportIndex i( "i" );
	ExportIndex j( "j" );
	ExportIndex k( "k" );
	if( !UNROLLING ) {
		solve.addIndex( i );
		solve.addIndex( j );
		solve.addIndex( k );
	}
	
	if( UNROLLING || dim <= 5 ) {
		// Start the factorization:
		for( run1 = 0; run1 < (dim-1); run1++ ) {
			for( run2 = run1; run2 < dim; run2++ ) {
				solve.addStatement( rk_temp.getCol( run2 ) == A.getSubMatrix( run2,run2+1,run1,run1+1 ) );
			}
			// calculate norm:
			solve.addStatement( rk_temp.getCol( dim ) == rk_temp.getCol( run1 )*rk_temp.getCol( run1 ) );
			for( run2 = run1+1; run2 < dim; run2++ ) {
				solve.addStatement( rk_temp.getCol( dim ) += rk_temp.getCol( run2 )*rk_temp.getCol( run2 ) );
			}
			solve.addStatement( String( "acadoWorkspace.rk_" ) << identifier << "temp[" << String( dim ) << "] = sqrt(acadoWorkspace.rk_" << identifier << "temp[" << String( dim ) << "]);\n" );
			
			// update first element:
			solve.addStatement( String( "acadoWorkspace.rk_" ) << identifier << "temp[" << String( run1 ) << "] += (acadoWorkspace.rk_" << identifier << "temp[" << String( run1 ) << "] < 0 ? -1 : 1)*acadoWorkspace.rk_" << identifier << "temp[" << String( dim ) << "];\n" );
			
			// calculate norm:
			solve.addStatement( rk_temp.getCol( dim ) == rk_temp.getCol( run1 )*rk_temp.getCol( run1 ) );
			for( run2 = run1+1; run2 < dim; run2++ ) {
				solve.addStatement( rk_temp.getCol( dim ) += rk_temp.getCol( run2 )*rk_temp.getCol( run2 ) );
			}
			solve.addStatement( String( "acadoWorkspace.rk_" ) << identifier << "temp[" << String( dim ) << "] = sqrt(acadoWorkspace.rk_" << identifier << "temp[" << String( dim ) << "]);\n" );
			
			// normalization:
			for( run2 = run1; run2 < dim; run2++ ) {
				solve.addStatement( String( "acadoWorkspace.rk_" ) << identifier << "temp[" << String( run2 ) << "] = acadoWorkspace.rk_" << identifier << "temp[" << String( run2 ) << "]/acadoWorkspace.rk_" << identifier << "temp[" << String( dim ) << "];\n" );
			}
			
			// update current column:
			solve.addStatement( rk_temp.getCol( dim ) == rk_temp.getCols( run1,dim )*A.getSubMatrix( run1,dim,run1,run1+1 ) );
			solve.addStatement( String( "acadoWorkspace.rk_" ) << identifier << "temp[" << String( dim ) << "] *= 2;\n" );
			solve.addStatement( A.getSubMatrix( run1,run1+1,run1,run1+1 ) -= rk_temp.getCol( run1 )*rk_temp.getCol( dim ) );
			if( REUSE ) {
				// replace zeros by results that can be reused:
				for( run2 = run1; run2 < dim-1; run2++ ) {
					solve.addStatement( A.getSubMatrix( run2+1,run2+2,run1,run1+1 ) == rk_temp.getCol( run2 ) );
				}
			}
			
			// update following columns:
			for( run2 = run1+1; run2 < dim; run2++ ) {
				solve.addStatement( rk_temp.getCol( dim ) == rk_temp.getCols( run1,dim )*A.getSubMatrix( run1,dim,run2,run2+1 ) );
				solve.addStatement( String( "acadoWorkspace.rk_" ) << identifier << "temp[" << String( dim ) << "] *= 2;\n" );
				for( run3 = run1; run3 < dim; run3++ ) {
					solve.addStatement( A.getSubMatrix( run3,run3+1,run2,run2+1 ) -= rk_temp.getCol( run3 )*rk_temp.getCol( dim ) );
				}
			}
			// update right-hand side:
			solve.addStatement( rk_temp.getCol( dim ) == rk_temp.getCols( run1,dim )*b.getRows( run1,dim ) );
			solve.addStatement( String( "acadoWorkspace.rk_" ) << identifier << "temp[" << String( dim ) << "] *= 2;\n" );
			for( run3 = run1; run3 < dim; run3++ ) {
				solve.addStatement( b.getRow( run3 ) -= rk_temp.getCol( run3 )*rk_temp.getCol( dim ) );
			}
			
			if( REUSE ) {
				// store last element to be reused:
				solve.addStatement( rk_temp.getCol( run1 ) == rk_temp.getCol( dim-1 ) );
			}
		}
	}
	else {
		solve.addStatement( String( "for( i=0; i < " ) << String( dim-1 ) << "; i++ ) {\n" );
		solve.addStatement( String( "	for( j=i; j < " ) << String( dim ) << "; j++ ) {\n" );
		solve.addStatement( String( "		acadoWorkspace.rk_" ) << identifier << "temp[j] = A[j*" << String( dim ) << "+i];\n" );
		solve.addStatement( String( "	}\n" ) );
		solve.addStatement( String( "	acadoWorkspace.rk_" ) << identifier << "temp[" << String( dim ) << "] = acadoWorkspace.rk_" << identifier << "temp[i]*acadoWorkspace.rk_" << identifier << "temp[i];\n" );
		solve.addStatement( String( "	for( j=i+1; j < " ) << String( dim ) << "; j++ ) {\n" );
		solve.addStatement( String( "		acadoWorkspace.rk_" ) << identifier << "temp[" << String( dim ) << "] += acadoWorkspace.rk_" << identifier << "temp[j]*acadoWorkspace.rk_" << identifier << "temp[j];\n" );
		solve.addStatement( String( "	}\n" ) );
		solve.addStatement( String( "	acadoWorkspace.rk_" ) << identifier << "temp[" << String( dim ) << "] = sqrt(acadoWorkspace.rk_" << identifier << "temp[" << String( dim ) << "]);\n" );
		// update first element:
		solve.addStatement( String( "	acadoWorkspace.rk_" ) << identifier << "temp[i] += (acadoWorkspace.rk_" << identifier << "temp[i] < 0 ? -1 : 1)*acadoWorkspace.rk_" << identifier << "temp[" << String( dim ) << "];\n" );
		solve.addStatement( String( "	acadoWorkspace.rk_" ) << identifier << "temp[" << String( dim ) << "] = acadoWorkspace.rk_" << identifier << "temp[i]*acadoWorkspace.rk_" << identifier << "temp[i];\n" );
		solve.addStatement( String( "	for( j=i+1; j < " ) << String( dim ) << "; j++ ) {\n" );
		solve.addStatement( String( "		acadoWorkspace.rk_" ) << identifier << "temp[" << String( dim ) << "] += acadoWorkspace.rk_" << identifier << "temp[j]*acadoWorkspace.rk_" << identifier << "temp[j];\n" );
		solve.addStatement( String( "	}\n" ) );
		solve.addStatement( String( "	acadoWorkspace.rk_" ) << identifier << "temp[" << String( dim ) << "] = sqrt(acadoWorkspace.rk_" << identifier << "temp[" << String( dim ) << "]);\n" );
		solve.addStatement( String( "	for( j=i; j < " ) << String( dim ) << "; j++ ) {\n" );
		solve.addStatement( String( "		acadoWorkspace.rk_" ) << identifier << "temp[j] = acadoWorkspace.rk_" << identifier << "temp[j]/acadoWorkspace.rk_" << identifier << "temp[" << String( dim ) << "];\n" );
		solve.addStatement( String( "	}\n" ) );
		solve.addStatement( String( "	acadoWorkspace.rk_" ) << identifier << "temp[" << String( dim ) << "] = acadoWorkspace.rk_" << identifier << "temp[i]*A[i*" << String( dim ) << "+i];\n" );
		solve.addStatement( String( "	for( j=i+1; j < " ) << String( dim ) << "; j++ ) {\n" );
		solve.addStatement( String( "		acadoWorkspace.rk_" ) << identifier << "temp[" << String( dim ) << "] += acadoWorkspace.rk_" << identifier << "temp[j]*A[j*" << String( dim ) << "+i];\n" );
		solve.addStatement( String( "	}\n" ) );
		solve.addStatement( String( "	acadoWorkspace.rk_" ) << identifier << "temp[" << String( dim ) << "] *= 2;\n" );
		solve.addStatement( String( "	A[i*" ) << String( dim ) << "+i] -= acadoWorkspace.rk_" << identifier << "temp[i]*acadoWorkspace.rk_" << identifier << "temp[" << String( dim ) << "];\n" );
		if( REUSE ) {
			solve.addStatement( String( "	for( j=i; j < (" ) << String( dim ) << "-1); j++ ) {\n" );
			solve.addStatement( String( "		A[(j+1)*" ) << String( dim ) << "+i] = acadoWorkspace.rk_" << identifier << "temp[j];\n" );
			solve.addStatement( String( "	}\n" ) );
		}
		solve.addStatement( String( "	for( j=i+1; j < " ) << String( dim ) << "; j++ ) {\n" );
		solve.addStatement( String( "		acadoWorkspace.rk_" ) << identifier << "temp[" << String( dim ) << "] = acadoWorkspace.rk_" << identifier << "temp[i]*A[i*" << String( dim ) << "+j];\n" );
		solve.addStatement( String( "		for( k=i+1; k < " ) << String( dim ) << "; k++ ) {\n" );
		solve.addStatement( String( "			acadoWorkspace.rk_" ) << identifier << "temp[" << String( dim ) << "] += acadoWorkspace.rk_" << identifier << "temp[k]*A[k*" << String( dim ) << "+j];\n" );
		solve.addStatement( String( "		}\n" ) );
		solve.addStatement( String( "		acadoWorkspace.rk_" ) << identifier << "temp[" << String( dim ) << "] *= 2;\n" );
		solve.addStatement( String( "		for( k=i; k < " ) << String( dim ) << "; k++ ) {\n" );
		solve.addStatement( String( "			A[k*" ) << String( dim ) << "+j] -= acadoWorkspace.rk_" << identifier << "temp[k]*acadoWorkspace.rk_" << identifier << "temp[" << String( dim ) << "];\n" );
		solve.addStatement( String( "		}\n" ) );
		solve.addStatement( String( "	}\n" ) );
		solve.addStatement( String( "	acadoWorkspace.rk_" ) << identifier << "temp[" << String( dim ) << "] = acadoWorkspace.rk_" << identifier << "temp[i]*b[i];\n" );
		solve.addStatement( String( "	for( k=i+1; k < " ) << String( dim ) << "; k++ ) {\n" );
		solve.addStatement( String( "		acadoWorkspace.rk_" ) << identifier << "temp[" << String( dim ) << "] += acadoWorkspace.rk_" << identifier << "temp[k]*b[k];\n" );
		solve.addStatement( String( "	}\n" ) );
		solve.addStatement( String( "	acadoWorkspace.rk_" ) << identifier << "temp[" << String( dim ) << "] *= 2;\n" );
		solve.addStatement( String( "	for( k=i; k < " ) << String( dim ) << "; k++ ) {\n" );
		solve.addStatement( String( "		b[k] -= acadoWorkspace.rk_" ) << identifier << "temp[k]*acadoWorkspace.rk_" << identifier << "temp[" << String( dim ) << "];\n" );
		solve.addStatement( String( "	}\n" ) );
		if( REUSE ) {
			solve.addStatement( String( "	acadoWorkspace.rk_" ) << identifier << "temp[i] = acadoWorkspace.rk_" << identifier << "temp[" << String( dim-1 ) << "];\n" );
		}
		solve.addStatement( String( "}\n" ) );
	}
	// updates last column:
	solve.addStatement( String( "A[" ) << String( dim*dim-1 ) << "] *= -1;\n" );
	solve.addStatement( String( "b[" ) << String( dim-1 ) << "] *= -1;\n" );
	
	solve.addLinebreak();
	
	// Solve the upper triangular system of equations which is left, using back substitution:
	solve.addStatement( String( "for( i=" ) << String(dim) << "; i>0; i-- ) {\n" );
	solve.addStatement( String( "	x[i-1] = b[i-1];\n" ) );
	solve.addStatement( String( "	for( j=" ) << String(dim-1) << "; j>(i-1); j-- ) {\n" );
	solve.addStatement( String( "		x[i-1] -= A[" ) << String( dim ) << "*(i-1)+j]*x[j];\n" );
	solve.addStatement( String( "	}\n" ) );
	solve.addStatement( String( "	x[i-1] = x[i-1]/A[" ) << String( dim ) << "*(i-1)+i-1];\n" );
	solve.addStatement( String( "}\n" ) );
	
	code.addFunction( solve );
	
    code.addLinebreak( 2 );
	if( REUSE ) { // Also export the extra function which reuses the factorization of the matrix A
		// update right-hand side:
		for( run1 = 0; run1 < (dim-1); run1++ ) {
			solveReuse.addStatement( rk_temp.getCol( dim ) == A.getSubMatrix( run1+1,run1+2,run1,run1+1 )*b.getRow( run1 ) );
			for( run2 = run1+1; run2 < (dim-1); run2++ ) {
				solveReuse.addStatement( rk_temp.getCol( dim ) += A.getSubMatrix( run2+1,run2+2,run1,run1+1 )*b.getRow( run2 ) );
			}
			solveReuse.addStatement( rk_temp.getCol( dim ) += rk_temp.getCol( run1 )*b.getRow( dim-1 ) );
			solveReuse.addStatement( String( "acadoWorkspace.rk_" ) << identifier << "temp[" << String( dim ) << "] *= 2;\n" );
			for( run3 = run1; run3 < (dim-1); run3++ ) {
				solveReuse.addStatement( b.getRow( run3 ) -= A.getSubMatrix( run3+1,run3+2,run1,run1+1 )*rk_temp.getCol( dim ) );
			}
			solveReuse.addStatement( b.getRow( dim-1 ) -= rk_temp.getCol( run1 )*rk_temp.getCol( dim ) );
		}
		solveReuse.addStatement( String( "b[" ) << String( dim-1 ) << "] *= -1;\n" );
		solveReuse.addLinebreak();

		// Solve the upper triangular system of equations which is left, using back substitution:
		for( run1 = dim; run1 > 0; run1--) {
			solveReuse.addStatement( x.getRow( (run1-1) ) == b.getRow( (run1-1) ) );
			for( run2 = dim-1; run2 > (run1-1); run2--) {
				solveReuse.addStatement( x.getRow( (run1-1) ) -= A.getSubMatrix( (run1-1),(run1-1)+1,run2,run2+1 ) * x.getRow( run2 ) );
			}
			solveReuse.addStatement( String( "x[" ) << String( (run1-1) ) << "] = x[" << String( (run1-1) ) << "]/A[" << String( (run1-1)*dim+(run1-1) ) << "];\n" );
		}

		code.addFunction( solveReuse );
	}
	
	return SUCCESSFUL_RETURN;
}


returnValue ExportHouseholderQR::setup( )
{
	rk_temp = ExportVariable( String( "rk_" ) << identifier << "temp", 1, dim+1, REAL, ACADO_WORKSPACE );
	A = ExportVariable( "A", dim, dim, REAL );
	b = ExportVariable( "b", dim, 1, REAL );
	x = ExportVariable( "x", dim, 1, REAL );
	solve = ExportFunction( getNameSolveFunction(), A, b, x);
	solve.addLinebreak( );	// FIX: TO MAKE SURE IT GETS EXPORTED
	
	if( REUSE ) {
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
