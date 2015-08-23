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
 *    \file src/code_generation/linear_solver_export.cpp
 *    \author Rien Quirynen
 *    \date 2012
 */

#include <acado/code_generation/linear_solvers/linear_solver_export.hpp>

using namespace std;

BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

ExportLinearSolver::ExportLinearSolver(	UserInteraction* _userInteraction,
										const std::string& _commonHeaderName
										) : ExportAlgorithm(_userInteraction, _commonHeaderName)
{
	REUSE = true;
	TRANSPOSE = false;
	UNROLLING = false;
	dim = nRows = nCols = nBacksolves = nRightHandSides = 0;

	determinant = ExportVariable("det", 1, 1, REAL, ACADO_LOCAL, true);
}


ExportLinearSolver::~ExportLinearSolver( )
{}


returnValue ExportLinearSolver::init(	const uint newDim,
										const bool& reuse,
										const bool& unrolling
										)
{
	return init(newDim, newDim, newDim, 0, reuse, unrolling, std::string( "dim" ) + toString( newDim ) + "_");
}

returnValue ExportLinearSolver::init(	const unsigned newDim,
										const unsigned _nRightHandSides,
										const bool& reuse,
										const bool& unroll
										)
{
	return init(newDim, newDim, newDim, _nRightHandSides, reuse, unroll, std::string( "dim" ) + toString( newDim ) + "_");
}


returnValue ExportLinearSolver::init(	const uint newDim,
										const bool& reuse,
										const bool& unrolling,
										const std::string& newId
										)
{
	return init(newDim, newDim, newDim, 0, reuse, unrolling, newId);
}

returnValue ExportLinearSolver::init(	unsigned _nRows,
										unsigned _nCols,
										unsigned _nBacksolves,
										bool _reuse,
										bool _unroll,
										const std::string& _id
										)
{
	return init(_nRows, _nCols, _nBacksolves, 0, _reuse, _unroll, _id);
}

returnValue ExportLinearSolver::init(	unsigned _nRows,
										unsigned _nCols,
										unsigned _nBacksolves,
										unsigned _nRightHandSides,
										bool _reuse,
										bool _unroll,
										const std::string& _id
										)
{
	ASSERT_RETURN(_nRows >= _nCols);
	ASSERT_RETURN(_nBacksolves <= _nCols);
	ASSERT_RETURN(_nRightHandSides >= 0);

	nRows = _nRows;
	nCols = _nCols;
	nBacksolves = _nBacksolves;
	nRightHandSides = _nRightHandSides;
	REUSE = _reuse;
	UNROLLING = _unroll;
	identifier = _id;

	// This is more for compatibility reasons and should be deprecated.
	dim = _nRows;

	return setup();
}

uint ExportLinearSolver::getDim() const {
	
	return dim;
}


bool ExportLinearSolver::getReuse() const {
	
	return REUSE;
}


returnValue ExportLinearSolver::setReuse( const bool& reuse ) {
	
	REUSE = reuse;
	
	return SUCCESSFUL_RETURN;
} 


bool ExportLinearSolver::getTranspose() const {

	return TRANSPOSE;
}


returnValue ExportLinearSolver::setTranspose( const bool& transpose ) {

	TRANSPOSE = transpose;

	return SUCCESSFUL_RETURN;
}


bool ExportLinearSolver::getUnrolling() const {
	
	return UNROLLING;
}


returnValue ExportLinearSolver::setUnrolling( const bool& unrolling ) {
	
	UNROLLING = unrolling;
	
	return SUCCESSFUL_RETURN;
} 


const std::string ExportLinearSolver::getNameSolveFunction() {
	
	return string( "solve_" ) + identifier + "system";
}


const std::string ExportLinearSolver::getNameSolveReuseFunction() {
	
	return string( "solve_" ) + identifier + "system_reuse";
}


const std::string ExportLinearSolver::getNameSolveTransposeReuseFunction() {

	return string( "solve_" ) + identifier + "transpose_reuse";
}

ExportVariable ExportLinearSolver::getGlobalExportVariable( const uint factor ) const
{
	ASSERT(1 == 0);
	return ExportVariable();
}


//
// PROTECTED MEMBER FUNCTIONS:
//



CLOSE_NAMESPACE_ACADO

// end of file.
