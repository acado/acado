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
 *    \file src/code_generation/linear_solvers/irk_4stage_simplified_newton_export.cpp
 *    \author Rien Quirynen
 *    \date 2015
 */

#include <acado/code_generation/linear_solvers/irk_4stage_simplified_newton_export.hpp>

using namespace std;

BEGIN_NAMESPACE_ACADO

//
// PUBLIC MEMBER FUNCTIONS:
//

ExportIRK4StageSimplifiedNewton::ExportIRK4StageSimplifiedNewton( UserInteraction* _userInteraction,
									const std::string& _commonHeaderName
									) : ExportGaussElim( _userInteraction,_commonHeaderName )
{
	stepsize = 0;
	implicit = false;
}

ExportIRK4StageSimplifiedNewton::~ExportIRK4StageSimplifiedNewton( )
{}

returnValue ExportIRK4StageSimplifiedNewton::getDataDeclarations(	ExportStatementBlock& declarations,
														ExportStruct dataStruct
														) const
{
	ExportGaussElim::getDataDeclarations( declarations, dataStruct );

	declarations.addDeclaration( rk_swap_complex,dataStruct );			// needed for the row swaps
	if( REUSE ) {
		declarations.addDeclaration( rk_bPerm_complex,dataStruct );		// reordered right-hand side
	}
	declarations.addDeclaration( A_mem_complex1,dataStruct );
	declarations.addDeclaration( b_mem_complex1,dataStruct );

	declarations.addDeclaration( A_mem_complex2,dataStruct );
	declarations.addDeclaration( b_mem_complex2,dataStruct );

	if( TRANSPOSE ) {
		declarations.addDeclaration( b_mem_complex1_trans,dataStruct );
		declarations.addDeclaration( b_mem_complex2_trans,dataStruct );
		declarations.addDeclaration( rk_bPerm_complex_trans,dataStruct );
	}

	return SUCCESSFUL_RETURN;
}


returnValue ExportIRK4StageSimplifiedNewton::getFunctionDeclarations(	ExportStatementBlock& declarations
															) const
{
	ExportGaussElim::getFunctionDeclarations( declarations );

	declarations.addDeclaration( solve_complex );
	if( REUSE ) {
		declarations.addDeclaration( solveReuse_complex );
	}

	declarations.addDeclaration( solve_full );
	if( REUSE ) {
		declarations.addDeclaration( solveReuse_full );
	}

	return SUCCESSFUL_RETURN;
}


returnValue ExportIRK4StageSimplifiedNewton::getCode(	ExportStatementBlock& code
											)
{
	if( eig.isEmpty() || transf1.isEmpty() || transf2.isEmpty() || fabs(stepsize) <= ZERO_EPS ) return ACADOERROR(RET_INVALID_OPTION);

//	if( TRANSPOSE ) return ACADOERROR( RET_NOT_YET_IMPLEMENTED );

	setupFactorization( solve_complex, rk_swap_complex, determinant_complex, string("cabs") );
	code.addFunction( solve_complex );

	if( REUSE ) { // Also export the extra function which reuses the factorization of the matrix A
		setupSolveReuseComplete( solveReuse_complex, rk_bPerm_complex );
		code.addFunction( solveReuse_complex );

		if( TRANSPOSE ) {
			setupSolveReuseTranspose( solveReuse_complexTranspose, rk_bPerm_complex_trans );
			code.addFunction( solveReuse_complexTranspose );
		}
	}
	
	ExportVariable eig_var( 1.0/stepsize*eig );

	// SETUP solve_full
	ExportVariable det_complex1( "det_complex1", 1, 1, REAL, ACADO_LOCAL, true );
	ExportVariable det_complex2( "det_complex2", 1, 1, REAL, ACADO_LOCAL, true );
	solve_full.addDeclaration( det_complex1 );
	solve_full.addDeclaration( det_complex2 );
	ExportIndex i( "i" );
	solve_full.addIndex(i);

	// form the real and complex linear system matrices
	solve_full.addStatement( A_mem_complex1 == A_full );
	if( implicit ) {
		ExportForLoop loop1( i, 0, dim*dim );
		loop1 << A_mem_complex1.getFullName() << "[i] += (" << eig_var.get(0,0) << "+" << eig_var.get(0,1) << "*I)*" << I_full.getFullName() << "[i];\n";
		solve_full.addStatement( loop1 );
	}
	else {
		ExportForLoop loop1( i, 0, dim );
		loop1 << A_mem_complex1.getFullName() << "[i*" << toString(dim) << "+i] -= (" << eig_var.get(0,0) << "+" << eig_var.get(0,1) << "*I);\n";
		solve_full.addStatement( loop1 );
	}

	solve_full.addStatement( A_mem_complex2 == A_full );
	if( implicit ) {
		ExportForLoop loop1( i, 0, dim*dim );
		loop1 << A_mem_complex2.getFullName() << "[i] += (" << eig_var.get(1,0) << "+" << eig_var.get(1,1) << "*I)*" << I_full.getFullName() << "[i];\n";
		solve_full.addStatement( loop1 );
	}
	else {
		ExportForLoop loop1( i, 0, dim );
		loop1 << A_mem_complex2.getFullName() << "[i*" << toString(dim) << "+i] -= (" << eig_var.get(1,0) << "+" << eig_var.get(1,1) << "*I);\n";
		solve_full.addStatement( loop1 );
	}

	// factorize the real and complex linear systems
	solve_full.addFunctionCall(getNameSolveComplexFunction(),A_mem_complex1,rk_perm_full.getAddress(0,0));
	solve_full.addFunctionCall(getNameSolveComplexFunction(),A_mem_complex2,rk_perm_full.getAddress(1,0));

	code.addFunction( solve_full );

	// SETUP solveReuse_full
	if( REUSE ) {
		solveReuse_full.addIndex(i);

		// transform the right-hand side
		transformRightHandSide( solveReuse_full, b_mem_complex1, b_mem_complex2, b_full, transf1, i, false );

		// solveReuse the real and complex linear systems
		solveReuse_full.addFunctionCall(getNameSolveComplexReuseFunction(),A_mem_complex1,b_mem_complex1,rk_perm_full.getAddress(0,0));
		solveReuse_full.addFunctionCall(getNameSolveComplexReuseFunction(),A_mem_complex2,b_mem_complex2,rk_perm_full.getAddress(1,0));

		// transform back to the solution
		transformSolution( solveReuse_full, b_mem_complex1, b_mem_complex2, b_full, transf2, i, false );

		code.addFunction( solveReuse_full );

		if( TRANSPOSE ) {
			uint NUM_RHS = nRightHandSides;
			nRightHandSides = 1;
			solveReuseTranspose_full.addIndex(i);

			// transform the right-hand side
			transformRightHandSide( solveReuseTranspose_full, b_mem_complex1_trans, b_mem_complex2_trans, b_full_trans, transf1_T, i, true );

			// solveReuse the real and complex linear systems
			solveReuseTranspose_full.addFunctionCall(getNameSolveComplexTransposeReuseFunction(),A_mem_complex1,b_mem_complex1_trans,rk_perm_full.getAddress(0,0));
			solveReuseTranspose_full.addFunctionCall(getNameSolveComplexTransposeReuseFunction(),A_mem_complex2,b_mem_complex2_trans,rk_perm_full.getAddress(1,0));

			// transform back to the solution
			transformSolution( solveReuseTranspose_full, b_mem_complex1_trans, b_mem_complex2_trans, b_full_trans, transf2_T, i, true );

			code.addFunction( solveReuseTranspose_full );
			nRightHandSides = NUM_RHS;
		}
	}

	return SUCCESSFUL_RETURN;
}


returnValue ExportIRK4StageSimplifiedNewton::transformRightHandSide(	ExportStatementBlock& code, const ExportVariable& b_mem1, const ExportVariable& b_mem2, const ExportVariable& b_full_, const ExportVariable& transf_, const ExportIndex& index, const bool transpose )
{
	uint i, j;

	ExportVariable transf1_var( transf_ );
	ExportVariable stepSizeV( 1.0/stepsize );

	ExportForLoop loop1( index, 0, dim );
	for( j = 0; j < nRightHandSides; j++ ) {
		loop1.addStatement( b_mem1.getElement(index,j) == 0.0 );
		for( i = 0; i < 4; i++ ) {
			loop1.addStatement( b_mem1.getElement(index,j) += transf1_var.getElement(0,i)*b_full_.getElement(index+i*dim,j) );
		}
		stringstream ss1;
		if( transpose ) ss1 << b_mem1.get(index,j) << " -= (";
		else 			ss1 << b_mem1.get(index,j) << " += (";
		for( i = 0; i < 4; i++ ) {
			if( i > 0 ) ss1 << " + ";
			ss1 << transf1_var.get(1,i) << "*" << b_full_.get(index+i*dim,j);
		}
		ss1 << ")*I;\n";
		ss1 << b_mem1.get(index,j) << " *= " << stepSizeV.get(0,0) << ";\n";
		loop1 << ss1.str();


		loop1.addStatement( b_mem2.getElement(index,j) == 0.0 );
		for( i = 0; i < 4; i++ ) {
			loop1.addStatement( b_mem2.getElement(index,j) += transf1_var.getElement(2,i)*b_full_.getElement(index+i*dim,j) );
		}
		stringstream ss2;
		if( transpose ) ss2 << b_mem2.get(index,j) << " -= (";
		else 			ss2 << b_mem2.get(index,j) << " += (";
		for( i = 0; i < 4; i++ ) {
			if( i > 0 ) ss2 << " + ";
			ss2 << transf1_var.get(3,i) << "*" << b_full_.get(index+i*dim,j);
		}
		ss2 << ")*I;\n";
		ss2 << b_mem2.get(index,j) << " *= " << stepSizeV.get(0,0) << ";\n";
		loop1 << ss2.str();
	}
	code.addStatement( loop1 );

	return SUCCESSFUL_RETURN;
}


returnValue ExportIRK4StageSimplifiedNewton::transformSolution(	ExportStatementBlock& code, const ExportVariable& b_mem1, const ExportVariable& b_mem2, const ExportVariable& b_full_, const ExportVariable& transf_, const ExportIndex& index, const bool transpose )
{
	uint j;
	ExportVariable transf2_var( transf_ );

	ExportForLoop loop1( index, 0, dim );
	for( j = 0; j < nRightHandSides; j++ ) {
		stringstream ss;
		ss << b_full_.get(index,j) << " = ";
		if( transpose ) ss << transf2_var.get(0,0) << "*creal(" << b_mem1.get(index,j) << ") - ";
		else 			ss << transf2_var.get(0,0) << "*creal(" << b_mem1.get(index,j) << ") + ";
		ss << transf2_var.get(0,1) << "*cimag(" << b_mem1.get(index,j) << ") + ";
		if( transpose ) ss << transf2_var.get(0,2) << "*creal(" << b_mem2.get(index,j) << ") - ";
		else 			ss << transf2_var.get(0,2) << "*creal(" << b_mem2.get(index,j) << ") + ";
		ss << transf2_var.get(0,3) << "*cimag(" << b_mem2.get(index,j) << ");\n";

		ss << b_full_.get(index+dim,j) << " = ";
		if( transpose ) ss << transf2_var.get(1,0) << "*creal(" << b_mem1.get(index,j) << ") - ";
		else			ss << transf2_var.get(1,0) << "*creal(" << b_mem1.get(index,j) << ") + ";
		ss << transf2_var.get(1,1) << "*cimag(" << b_mem1.get(index,j) << ") + ";
		if( transpose ) ss << transf2_var.get(1,2) << "*creal(" << b_mem2.get(index,j) << ") - ";
		else 			ss << transf2_var.get(1,2) << "*creal(" << b_mem2.get(index,j) << ") + ";
		ss << transf2_var.get(1,3) << "*cimag(" << b_mem2.get(index,j) << ");\n";

		ss << b_full_.get(index+2*dim,j) << " = ";
		if( transpose ) ss << transf2_var.get(2,0) << "*creal(" << b_mem1.get(index,j) << ") - ";
		else			ss << transf2_var.get(2,0) << "*creal(" << b_mem1.get(index,j) << ") + ";
		ss << transf2_var.get(2,1) << "*cimag(" << b_mem1.get(index,j) << ") + ";
		if( transpose ) ss << transf2_var.get(2,2) << "*creal(" << b_mem2.get(index,j) << ") - ";
		else			ss << transf2_var.get(2,2) << "*creal(" << b_mem2.get(index,j) << ") + ";
		ss << transf2_var.get(2,3) << "*cimag(" << b_mem2.get(index,j) << ");\n";

		ss << b_full_.get(index+3*dim,j) << " = ";
		if( transpose ) ss << transf2_var.get(3,0) << "*creal(" << b_mem1.get(index,j) << ") - ";
		else			ss << transf2_var.get(3,0) << "*creal(" << b_mem1.get(index,j) << ") + ";
		ss << transf2_var.get(3,1) << "*cimag(" << b_mem1.get(index,j) << ") + ";
		if( transpose ) ss << transf2_var.get(3,2) << "*creal(" << b_mem2.get(index,j) << ") - ";
		else			ss << transf2_var.get(3,2) << "*creal(" << b_mem2.get(index,j) << ") + ";
		ss << transf2_var.get(3,3) << "*cimag(" << b_mem2.get(index,j) << ");\n";
		loop1 << ss.str();
	}
	code.addStatement( loop1 );

	return SUCCESSFUL_RETURN;
}


returnValue ExportIRK4StageSimplifiedNewton::appendVariableNames( stringstream& string ) {

	ExportGaussElim::appendVariableNames( string );
	string << ", " << rk_swap_complex.getFullName();
	if( REUSE ) {
//		string << ", " << rk_perm.getFullName().getName();
		string << ", " << rk_bPerm_complex.getFullName();
	}

	return SUCCESSFUL_RETURN;
}


returnValue ExportIRK4StageSimplifiedNewton::setup( )
{
	ExportGaussElim::setup( );

	if (nRightHandSides <= 0)
		return ACADOERROR(RET_INVALID_OPTION);

	int useOMP;
	get(CG_USE_OPENMP, useOMP);
	ExportStruct structWspace;
	structWspace = useOMP ? ACADO_LOCAL : ACADO_WORKSPACE;

	determinant_complex = ExportVariable("det", 1, 1, COMPLEX, ACADO_LOCAL, true);
	rk_swap_complex = ExportVariable( std::string( "rk_complex_" ) + identifier + "swap", 1, 1, COMPLEX, structWspace, true );
	rk_bPerm_complex = ExportVariable( std::string( "rk_complex_" ) + identifier + "bPerm", dim, nRightHandSides, COMPLEX, structWspace );

	A_mem_complex1 = ExportVariable( std::string( "rk_complex1_" ) + identifier + "A", dim, dim, COMPLEX, structWspace );
	b_mem_complex1 = ExportVariable( std::string( "rk_complex1_" ) + identifier + "b", dim, nRightHandSides, COMPLEX, structWspace );

	A_mem_complex2 = ExportVariable( std::string( "rk_complex2_" ) + identifier + "A", dim, dim, COMPLEX, structWspace );
	b_mem_complex2 = ExportVariable( std::string( "rk_complex2_" ) + identifier + "b", dim, nRightHandSides, COMPLEX, structWspace );

	A_complex = ExportVariable( "A", dim, dim, COMPLEX );
	b_complex = ExportVariable( "b", dim, nRightHandSides, COMPLEX );
	rk_perm_complex = ExportVariable( "rk_perm", 1, dim, INT );

	solve_complex = ExportFunction( getNameSolveComplexFunction(), A_complex, rk_perm_complex );
	solve_complex.setReturnValue( determinant_complex, false );
	solve_complex.addLinebreak( );	// FIX: TO MAKE SURE IT GETS EXPORTED
	
	if( REUSE ) {
		solveReuse_complex = ExportFunction( getNameSolveComplexReuseFunction(), A_complex, b_complex, rk_perm_complex );
		solveReuse_complex.addLinebreak( );	// FIX: TO MAKE SURE IT GETS EXPORTED

		if( TRANSPOSE ) {
			b_complex_trans = ExportVariable( "b", dim, 1, COMPLEX );
			rk_bPerm_complex_trans = ExportVariable( std::string( "rk_complex_trans_" ) + identifier + "bPerm", dim, 1, COMPLEX, structWspace );

			b_mem_complex1_trans = ExportVariable( std::string( "rk_complex1_trans_" ) + identifier + "b", dim, 1, COMPLEX, structWspace );
			b_mem_complex2_trans = ExportVariable( std::string( "rk_complex2_trans_" ) + identifier + "b", dim, 1, COMPLEX, structWspace );

			solveReuse_complexTranspose = ExportFunction( getNameSolveComplexTransposeReuseFunction(), A_complex, b_complex_trans, rk_perm_complex );
			solveReuse_complexTranspose.addLinebreak( );	// FIX: TO MAKE SURE IT GETS EXPORTED
		}
	}

	A_full = ExportVariable( "A", dim, dim, REAL );
	I_full = ExportVariable( "A_I", dim, dim, REAL );
	b_full = ExportVariable( "b", 4*dim, nRightHandSides, REAL );
	rk_perm_full = ExportVariable( "rk_perm", 2, dim, INT );

	if( implicit ) {
		solve_full = ExportFunction( getNameSolveFunction(), A_full, I_full, rk_perm_full );   // Only perform the LU factorization!
	}
	else {
		solve_full = ExportFunction( getNameSolveFunction(), A_full, rk_perm_full );   // Only perform the LU factorization!
	}
	solve_full.setReturnValue( determinant, false );
	solve_full.addLinebreak( );	// FIX: TO MAKE SURE IT GETS EXPORTED
	if( REUSE ) {
		if( implicit ) {
			solveReuse_full = ExportFunction( getNameSolveReuseFunction(), A_full, I_full, b_full, rk_perm_full );
			if( TRANSPOSE ) return ACADOERROR( RET_NOT_YET_IMPLEMENTED );
		}
		else {
			solveReuse_full = ExportFunction( getNameSolveReuseFunction(), A_full, b_full, rk_perm_full );
			if( TRANSPOSE ) {
				b_full_trans = ExportVariable( "b", 4*dim, 1, REAL );
				solveReuseTranspose_full = ExportFunction( getNameSolveTransposeReuseFunction(), A_full, b_full_trans, rk_perm_full );
			}
		}
		solveReuse_full.addLinebreak( );	// FIX: TO MAKE SURE IT GETS EXPORTED
	}

    return SUCCESSFUL_RETURN;
}


returnValue ExportIRK4StageSimplifiedNewton::setEigenvalues( const DMatrix& _eig ) {
	eig = _eig;

	if( _eig.getNumRows() != 2 || _eig.getNumCols() != 2 ) return ACADOERROR( RET_INVALID_ARGUMENTS );
	if( _eig.isZero() ) return ACADOERROR( RET_INVALID_ARGUMENTS );
	// each row represents a complex conjugate pair of eigenvalues

	return SUCCESSFUL_RETURN;
}


returnValue ExportIRK4StageSimplifiedNewton::setTransformations( const DMatrix& _transf1, const DMatrix& _transf2, const DMatrix& _transf1_T, const DMatrix& _transf2_T ) {
	transf1 = _transf1;
	transf2 = _transf2;
	transf1_T = _transf1_T;
	transf2_T = _transf2_T;

	if( _transf1.getNumRows() != 4 || _transf1.getNumCols() != 4 ) return ACADOERROR( RET_INVALID_ARGUMENTS );
	if( _transf2.getNumRows() != 4 || _transf2.getNumCols() != 4 ) return ACADOERROR( RET_INVALID_ARGUMENTS );
	if( _transf1.isZero() || _transf2.isZero() ) return ACADOERROR( RET_INVALID_ARGUMENTS );

	return SUCCESSFUL_RETURN;
}


returnValue ExportIRK4StageSimplifiedNewton::setStepSize( double _stepsize ) {
	stepsize = _stepsize;

	return SUCCESSFUL_RETURN;
}


const std::string ExportIRK4StageSimplifiedNewton::getNameSolveComplexFunction() {

	return string( "solve_complex_" ) + identifier + "system";
}


const std::string ExportIRK4StageSimplifiedNewton::getNameSolveComplexReuseFunction() {

	return string( "solve_complex_" ) + identifier + "system_reuse";
}


const std::string ExportIRK4StageSimplifiedNewton::getNameSolveComplexTransposeReuseFunction() {

	return string( "solve_complex_trans_" ) + identifier + "system_reuse";
}

returnValue ExportIRK4StageSimplifiedNewton::setImplicit( BooleanType _implicit ) {

	implicit = _implicit;

	return SUCCESSFUL_RETURN;
}

//
// PROTECTED MEMBER FUNCTIONS:
//



CLOSE_NAMESPACE_ACADO

// end of file.
