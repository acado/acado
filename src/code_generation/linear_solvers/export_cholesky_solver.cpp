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
 *    \file src/code_generation/export_cholesky_solver.cpp
 *    \author Milan Vukov
 *    \date 2014
 */

#include <acado/code_generation/linear_solvers/export_cholesky_solver.hpp>

using namespace std;

BEGIN_NAMESPACE_ACADO

ExportCholeskySolver::ExportCholeskySolver(	UserInteraction* _userInteraction,
											const std::string& _commonHeaderName
											) : ExportLinearSolver(_userInteraction, _commonHeaderName)
{
	nColsB = 0;
}

ExportCholeskySolver::~ExportCholeskySolver()
{}

returnValue ExportCholeskySolver::init(	unsigned _dimA,
										unsigned _numColsB,
										const std::string& _id
										)
{
	nRows = nCols = _dimA;
	nColsB = _numColsB;

	identifier = _id;

	A.setup("A", nRows, nCols, REAL, ACADO_LOCAL);
	B.setup("B", nRows, nColsB, REAL, ACADO_LOCAL);

	chol.setup(identifier + "_chol", A);
	solve.setup(identifier + "_solve", A, B);

	REUSE = false;

	return SUCCESSFUL_RETURN;
}

returnValue ExportCholeskySolver::setup()
{
	if (REUSE == true)
		return RET_NOT_IMPLEMENTED_YET;

	/* Setup evaluation of the Cholesky factorization. */

	ExportVariable sum("sum", 1, 1, REAL, ACADO_LOCAL, true);
	ExportVariable div("div", 1, 1, REAL, ACADO_LOCAL, true);
	chol.addVariable( sum );
	chol.addVariable( div );

	for(int ii = 0; ii < (int)nRows; ++ii)
	{
		for (int k = 0; k < ii; ++k)
			chol.addStatement( A.getElement(ii, k) == 0.0 );


		/* j == i */
//		sum = H[ii * nCols + ii];
		chol.addStatement( sum == A.getElement(ii, ii) );

		chol.addComment( "j == i" );
		for(int k = (ii - 1); k >= 0; --k)
//			sum -= A[k*NVMAX + i] * A[k*NVMAX + i];
			chol.addStatement( sum -= A.getElement(k, ii) * A.getElement(k, ii) );

//		if ( sum > 0.0 )
//			R[i*NVMAX + i] = sqrt( sum );
//		else
//		{
//			hessianType = HST_SEMIDEF;
//			return THROWERROR( RET_HESSIAN_NOT_SPD );
//		}

		chol << A.getElement(ii, ii).get(0, 0) << " = sqrt(" << sum.getFullName() << ");\n";
		chol << div.getFullName() << " = 1.0 / " << A.getElement(ii, ii).get(0, 0) << ";\n";

		/* j > i */
		chol.addComment( "j loop" );
		for(int jj = (ii + 1); jj < (int)nRows; ++jj)
		{
//			jj = FR_idx[j];
//			sum = H[jj*NVMAX + ii];
			chol.addStatement( sum == A.getElement(jj, ii) );

			chol.addComment( "k loop" );
			for(int k = (ii - 1); k >= 0; --k)
//				sum -= R[k * NVMAX + ii] * R[k * NVMAX + jj];
				chol.addStatement( sum -= A.getElement(k, ii) * A.getElement(k, jj) );

//			R[ii * NVMAX + jj] = sum / R[ii * NVMAX + ii];
			chol.addStatement( A.getElement(ii, jj) == sum * div );
		}
	}

	/* Setup evaluation of the solve function. */
	/* Implements R' X = B -> X = R^{-T} * B. B is replaced by the solution. */

	/* solve R^T*a = b */

	solve.addVariable( sum );

	for (unsigned col = 0; col < nColsB; ++col)
		for(int i = 0; i < int(nRows); ++i)
		{
//			sum = b[i];
			solve.addStatement( sum == B.getElement(i, col) );

			for(int j = 0; j < i; ++j)
//				sum -= R[j*NVMAX + i] * a[j];
				solve.addStatement( sum-= A.getElement(j, i) * B.getElement(j, col) );

//			if ( getAbs( R[i*NVMAX + i] ) > ZERO )
//				a[i] = sum / R[i*NVMAX + i];
//			else
//				return THROWERROR( RET_DIV_BY_ZERO );

			solve << B.getElement(i, col).get(0, 0) << " = " << sum.getFullName() << " / " << A.getElement(i, i).get(0, 0) << ";\n";
		}


	return SUCCESSFUL_RETURN;
}

returnValue ExportCholeskySolver::getCode( ExportStatementBlock& code )
{
	code.addFunction( chol );
	code.addFunction( solve );

	return SUCCESSFUL_RETURN;
}

returnValue ExportCholeskySolver::getDataDeclarations(	ExportStatementBlock& declarations,
														ExportStruct dataStruct
														) const
{
	return SUCCESSFUL_RETURN;
}

returnValue ExportCholeskySolver::getFunctionDeclarations(	ExportStatementBlock& declarations
															) const
{
	declarations.addDeclaration( chol );
	declarations.addDeclaration( solve );

	return SUCCESSFUL_RETURN;
}

const ExportFunction& ExportCholeskySolver::getCholeskyFunction() const
{
	return chol;
}

const ExportFunction& ExportCholeskySolver::getSolveFunction() const
{
	return solve;
}

returnValue ExportCholeskySolver::appendVariableNames( std::stringstream& string )
{
	return SUCCESSFUL_RETURN;
}

CLOSE_NAMESPACE_ACADO

// end of file.
