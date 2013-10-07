/*
 *    This file is part of ACADO Toolkit.
 *
 *    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
 *    Copyright (C) 2008-2013 by Boris Houska, Hans Joachim Ferreau,
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
	return SUCCESSFUL_RETURN;
}


returnValue ExportHouseholderQR::getFunctionDeclarations(	ExportStatementBlock& declarations
															) const
{
	declarations.addDeclaration( solve );
	declarations.addDeclaration( solveTriangular );
	if( REUSE ) {
		declarations.addDeclaration( solveReuse );
	}

	return SUCCESSFUL_RETURN;
}


returnValue ExportHouseholderQR::getCode(	ExportStatementBlock& code
											)
{
	unsigned run1, run2, run3;

	//
	// Solve the upper triangular system of equations:
	//
	for (run1 = nCols; run1 > (nCols - nBacksolves); run1--)
	{
		for (run2 = nCols - 1; run2 > (run1 - 1); run2--)
		{
			solveTriangular.addStatement(
					b.getRow(run1 - 1) -= A.getSubMatrix((run1 - 1), run1, run2, run2 + 1) * b.getRow(run2));
		}
		solveTriangular.addStatement(
				String("b[") << String((run1 - 1)) << "] = b["
				<< String((run1 - 1)) << "]/A["
				<< String((run1 - 1) * nCols + (run1 - 1)) << "];\n");
	}
	code.addFunction(solveTriangular);
	
	//
	// Main solver function
	//
	solve.addStatement( determinant == 1.0 );

	if( UNROLLING || nRows <= 5 )
	{
		// Start the factorization:
		for (run1 = 0; run1 < nCols; run1++)
		{
			for (run2 = run1; run2 < nRows; run2++)
			{
				solve.addStatement(
						rk_temp.getCol(run2)
								== A.getSubMatrix(run2, run2 + 1, run1,
										run1 + 1));
			}
			// calculate norm:
			solve.addStatement(rk_temp.getCol(nRows) ==
					rk_temp.getCols(run1, nRows) * rk_temp.getTranspose().getRows(run1, nRows));
			solve.addStatement(
					rk_temp.getFullName() << "[" << String(nRows) << "] = sqrt("
							<< rk_temp.getFullName() << "[" << String(nRows)
							<< "]);\n");

			// update first element:
			solve.addStatement(
					rk_temp.getFullName() << "[" << String(run1) << "] += ("
							<< rk_temp.getFullName() << "[" << String(run1)
							<< "] < 0 ? -1 : 1)*" << rk_temp.getFullName()
							<< "[" << String(nRows) << "];\n");

			// calculate norm:
			solve.addStatement(rk_temp.getCol(nRows) ==
					rk_temp.getCols(run1, nRows) * rk_temp.getTranspose().getRows(run1, nRows));
			solve.addStatement(
					rk_temp.getFullName() << "[" << String(nRows) << "] = sqrt("
							<< rk_temp.getFullName() << "[" << String(nRows)
							<< "]);\n");

			// normalization:
			for (run2 = run1; run2 < nRows; run2++)
			{
				solve.addStatement(
						rk_temp.getFullName() << "[" << String(run2) << "] = "
								<< rk_temp.getFullName() << "[" << String(run2)
								<< "]/" << rk_temp.getFullName() << "["
								<< String(nRows) << "];\n");
			}

			// update current column:
			solve.addStatement(
					rk_temp.getCol(nRows)
							== rk_temp.getCols(run1, nRows)
									* A.getSubMatrix(run1, nRows, run1,
											run1 + 1));
			solve.addStatement(
					rk_temp.getFullName() << "[" << String(nRows) << "] *= 2;\n");
			solve.addStatement(
					A.getSubMatrix(run1, run1 + 1, run1, run1 + 1) -=
							rk_temp.getCol(run1) * rk_temp.getCol(nRows));

			solve.addStatement( determinant == determinant * A.getElement(run1, run1) );

			if (REUSE)
			{
				// replace zeros by results that can be reused:
				for (run2 = run1; run2 < nRows - 1; run2++)
				{
					solve.addStatement(
							A.getSubMatrix(run2 + 1, run2 + 2, run1, run1 + 1)
									== rk_temp.getCol(run2));
				}
			}

			// update following columns:
			for (run2 = run1 + 1; run2 < nCols; run2++)
			{
				solve.addStatement(
						rk_temp.getCol(nRows)
								== rk_temp.getCols(run1, nRows)
										* A.getSubMatrix(run1, nRows, run2,
												run2 + 1));
				solve.addStatement(
						rk_temp.getFullName() << "[" << String(nRows)
								<< "] *= 2;\n");
				for (run3 = run1; run3 < nRows; run3++)
				{
					solve.addStatement(
							A.getSubMatrix(run3, run3 + 1, run2, run2 + 1) -=
									rk_temp.getCol(run3) * rk_temp.getCol(nRows));
				}
			}
			// update right-hand side:
			solve.addStatement(
					rk_temp.getCol(nRows)
							== rk_temp.getCols(run1, nRows)
									* b.getRows(run1, nRows));
			solve.addStatement(
					rk_temp.getFullName() << "[" << String(nRows) << "] *= 2;\n");
			for (run3 = run1; run3 < nRows; run3++)
			{
				solve.addStatement( b.getRow(run3) -= rk_temp.getCol(run3) * rk_temp.getCol(nRows));
			}

			if (REUSE)
			{
				// store last element to be reused:
				solve.addStatement(
						rk_temp.getCol(run1) == rk_temp.getCol(nRows - 1));
			}
		}
	}
	else
	{
		ExportIndex i( "i" );
		ExportIndex j( "j" );
		ExportIndex k( "k" );

		solve.addIndex( i );
		solve.addIndex( j );
		solve.addIndex( k );

		solve.addStatement( String( "for( i=0; i < " ) << String( nCols ) << "; i++ ) {\n" );
		solve.addStatement( String( "	for( j=i; j < " ) << String( nRows ) << "; j++ ) {\n" );
		solve.addStatement( String( "		" ) << rk_temp.getFullName() << "[j] = A[j*" << String( nCols ) << "+i];\n" );
		solve.addStatement( String( "	}\n" ) );
		solve.addStatement( String( "	" ) << rk_temp.getFullName() << "[" << String( nRows ) << "] = " << rk_temp.getFullName() << "[i]*" << rk_temp.getFullName() << "[i];\n" );
		solve.addStatement( String( "	for( j=i+1; j < " ) << String( nRows ) << "; j++ ) {\n" );
		solve.addStatement( String( "		" ) << rk_temp.getFullName() << "[" << String( nRows ) << "] += " << rk_temp.getFullName() << "[j]*" << rk_temp.getFullName() << "[j];\n" );
		solve.addStatement( String( "	}\n" ) );
		solve.addStatement( String( "	" ) << rk_temp.getFullName() << "[" << String( nRows ) << "] = sqrt(" << rk_temp.getFullName() << "[" << String( nRows ) << "]);\n" );
		// update first element:
		solve.addStatement( String( "	" ) << rk_temp.getFullName() << "[i] += (" << rk_temp.getFullName() << "[i] < 0 ? -1 : 1)*" << rk_temp.getFullName() << "[" << String( nRows ) << "];\n" );
		solve.addStatement( String( "	" ) << rk_temp.getFullName() << "[" << String( nRows ) << "] = " << rk_temp.getFullName() << "[i]*" << rk_temp.getFullName() << "[i];\n" );
		solve.addStatement( String( "	for( j=i+1; j < " ) << String( nRows ) << "; j++ ) {\n" );
		solve.addStatement( String( "		" ) << rk_temp.getFullName() << "[" << String( nRows ) << "] += " << rk_temp.getFullName() << "[j]*" << rk_temp.getFullName() << "[j];\n" );
		solve.addStatement( String( "	}\n" ) );
		solve.addStatement( String( "	" ) << rk_temp.getFullName() << "[" << String( nRows ) << "] = sqrt(" << rk_temp.getFullName() << "[" << String( nRows ) << "]);\n" );
		solve.addStatement( String( "	for( j=i; j < " ) << String( nRows ) << "; j++ ) {\n" );
		solve.addStatement( String( "		" ) << rk_temp.getFullName() << "[j] = " << rk_temp.getFullName() << "[j]/" << rk_temp.getFullName() << "[" << String( nRows ) << "];\n" );
		solve.addStatement( String( "	}\n" ) );
		solve.addStatement( String( "	" ) << rk_temp.getFullName() << "[" << String( nRows ) << "] = " << rk_temp.getFullName() << "[i]*A[i*" << String( nCols ) << "+i];\n" );
		solve.addStatement( String( "	for( j=i+1; j < " ) << String( nRows ) << "; j++ ) {\n" );
		solve.addStatement( String( "		" ) << rk_temp.getFullName() << "[" << String( nRows ) << "] += " << rk_temp.getFullName() << "[j]*A[j*" << String( nCols ) << "+i];\n" );
		solve.addStatement( String( "	}\n" ) );
		solve.addStatement( String( "	" ) << rk_temp.getFullName() << "[" << String( nRows ) << "] *= 2;\n" );
		solve.addStatement( String( "	A[i*" ) << String( nCols ) << "+i] -= " << rk_temp.getFullName() << "[i]*" << rk_temp.getFullName() << "[" << String( nRows ) << "];\n" );

		solve.addStatement( String( "	" ) << determinant.getFullName() << " *= " << "	A[i * " << String( nCols ) << " + i];\n" );

		if( REUSE ) {
			solve.addStatement( String( "	for( j=i; j < (" ) << String( nRows ) << "-1); j++ ) {\n" );
			solve.addStatement( String( "		A[(j+1)*" ) << String( nCols ) << "+i] = " << rk_temp.getFullName() << "[j];\n" );
			solve.addStatement( String( "	}\n" ) );
		}
		solve.addStatement( String( "	for( j=i+1; j < " ) << String( nCols ) << "; j++ ) {\n" );
		solve.addStatement( String( "		" ) << rk_temp.getFullName() << "[" << String( nRows ) << "] = " << rk_temp.getFullName() << "[i]*A[i*" << String( nCols ) << "+j];\n" );
		solve.addStatement( String( "		for( k=i+1; k < " ) << String( nRows ) << "; k++ ) {\n" );
		solve.addStatement( String( "			" ) << rk_temp.getFullName() << "[" << String( nRows ) << "] += " << rk_temp.getFullName() << "[k]*A[k*" << String( nCols ) << "+j];\n" );
		solve.addStatement( String( "		}\n" ) );
		solve.addStatement( String( "		" ) << rk_temp.getFullName() << "[" << String( nRows ) << "] *= 2;\n" );
		solve.addStatement( String( "		for( k=i; k < " ) << String( nRows ) << "; k++ ) {\n" );
		solve.addStatement( String( "			A[k*" ) << String( nCols ) << "+j] -= " << rk_temp.getFullName() << "[k]*" << rk_temp.getFullName() << "[" << String( nRows ) << "];\n" );
		solve.addStatement( String( "		}\n" ) );
		solve.addStatement( String( "	}\n" ) );
		solve.addStatement( String( "	" ) << rk_temp.getFullName() << "[" << String( nRows ) << "] = " << rk_temp.getFullName() << "[i]*b[i];\n" );
		solve.addStatement( String( "	for( k=i+1; k < " ) << String( nRows ) << "; k++ ) {\n" );
		solve.addStatement( String( "		" ) << rk_temp.getFullName() << "[" << String( nRows ) << "] += " << rk_temp.getFullName() << "[k]*b[k];\n" );
		solve.addStatement( String( "	}\n" ) );
		solve.addStatement( String( "	" ) << rk_temp.getFullName() << "[" << String( nRows ) << "] *= 2;\n" );
		solve.addStatement( String( "	for( k=i; k < " ) << String( nRows ) << "; k++ ) {\n" );
		solve.addStatement( String( "		b[k] -= " ) << rk_temp.getFullName() << "[k]*" << rk_temp.getFullName() << "[" << String( nRows ) << "];\n" );
		solve.addStatement( String( "	}\n" ) );
		if( REUSE ) {
			solve.addStatement( String( "	" ) << rk_temp.getFullName() << "[i] = " << rk_temp.getFullName() << "[" << String( nRows-1 ) << "];\n" );
		}
		solve.addStatement( String( "}\n" ) );
	}
	solve.addLinebreak();

	solve.addFunctionCall(solveTriangular, A, b);
	code.addFunction( solve );
	
    code.addLinebreak( 2 );
	if( REUSE ) { // Also export the extra function which reuses the factorization of the matrix A
		// update right-hand side:
		for( run1 = 0; run1 < nCols; run1++ ) {
			solveReuse.addStatement( rk_temp.getCol( nRows ) == A.getSubMatrix( run1+1,run1+2,run1,run1+1 )*b.getRow( run1 ) );
			for( run2 = run1+1; run2 < (nRows-1); run2++ ) {
				solveReuse.addStatement( rk_temp.getCol( nRows ) += A.getSubMatrix( run2+1,run2+2,run1,run1+1 )*b.getRow( run2 ) );
			}
			solveReuse.addStatement( rk_temp.getCol( nRows ) += rk_temp.getCol( run1 )*b.getRow( nRows-1 ) );
			solveReuse.addStatement( String( "" ) << rk_temp.getFullName() << "[" << String( nRows ) << "] *= 2;\n" );
			for( run3 = run1; run3 < (nRows-1); run3++ ) {
				solveReuse.addStatement( b.getRow( run3 ) -= A.getSubMatrix( run3+1,run3+2,run1,run1+1 )*rk_temp.getCol( nRows ) );
			}
			solveReuse.addStatement( b.getRow( nRows-1 ) -= rk_temp.getCol( run1 )*rk_temp.getCol( nRows ) );
		}
		solveReuse.addLinebreak();

		solveReuse.addFunctionCall( solveTriangular, A, b );
		code.addFunction( solveReuse );
	}
	
	return SUCCESSFUL_RETURN;
}


returnValue ExportHouseholderQR::appendVariableNames( stringstream& string )
{
	return SUCCESSFUL_RETURN;
}


returnValue ExportHouseholderQR::setup( )
{
	int useOMP;
	get(CG_USE_OPENMP, useOMP);

	A = ExportVariable("A", nRows, nCols, REAL);
	b = ExportVariable("b", nRows, 1, REAL);
	rk_temp = ExportVariable("rk_temp", 1, nRows + 1, REAL);
	solve = ExportFunction(getNameSolveFunction(), A, b, rk_temp);
	solve.setReturnValue(determinant, BT_FALSE);
	solve.addLinebreak( );	// FIX: TO MAKE SURE IT GETS EXPORTED
	solveTriangular = ExportFunction( String( "solve_" ) << identifier << "triangular", A, b);
	solveTriangular.addLinebreak( );	// FIX: TO MAKE SURE IT GETS EXPORTED
	
	if (REUSE)
	{
		solveReuse = ExportFunction(getNameSolveReuseFunction(), A, b, rk_temp);
		solveReuse.addLinebreak();	// FIX: TO MAKE SURE IT GETS EXPORTED
	}
	
	int unrollOpt;
	userInteraction->get(UNROLL_LINEAR_SOLVER, unrollOpt);
	UNROLLING = (BooleanType) unrollOpt;

    return SUCCESSFUL_RETURN;
}


ExportVariable ExportHouseholderQR::getGlobalExportVariable( const uint factor ) const {

	int useOMP;
	get(CG_USE_OPENMP, useOMP);
	ExportStruct structWspace;
	structWspace = useOMP ? ACADO_LOCAL : ACADO_WORKSPACE;

	return ExportVariable( String( "rk_" ) << identifier << "temp", factor, nRows+1, REAL, structWspace );
}


//
// PROTECTED MEMBER FUNCTIONS:
//



CLOSE_NAMESPACE_ACADO

// end of file.
