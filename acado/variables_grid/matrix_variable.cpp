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
 *    \file src/variables_grid/matrix_variable.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 31.05.2008
 */


#include <acado/variables_grid/matrix_variable.hpp>



BEGIN_NAMESPACE_ACADO



//
// PUBLIC MEMBER FUNCTIONS:
//

MatrixVariable::MatrixVariable( ) : DMatrix( ), VariableSettings( )
{
}


MatrixVariable::MatrixVariable(	uint _nRows,
								uint _nCols,
								VariableType _type,
								const char** const _names,
								const char** const _units,
								DVector _scaling,
								DVector _lb,
								DVector _ub,
								BooleanType _autoInit
								) : DMatrix( _nRows,_nCols ), VariableSettings( _nRows*_nCols,_type,_names,_units,_scaling,_lb,_ub,_autoInit )
{
}


MatrixVariable::MatrixVariable( const MatrixVariable& rhs ) : DMatrix( rhs ), VariableSettings( rhs )
{
}


MatrixVariable::MatrixVariable(	const DMatrix& _matrix,
								VariableType _type
								) : DMatrix( _matrix ), VariableSettings( _matrix.getDim(),_type )
{
}


MatrixVariable::~MatrixVariable( )
{
}


MatrixVariable& MatrixVariable::operator=( const MatrixVariable& rhs )
{
    if ( this != &rhs )
    {
		VariableSettings::operator=( rhs );
		DMatrix::operator=( rhs );
    }

    return *this;
}


MatrixVariable& MatrixVariable::operator=( const DMatrix& rhs )
{
    if ( this != &rhs )
    {
		VariableSettings::operator=( rhs.getDim() );
		DMatrix::operator=( rhs );
    }

    return *this;
}



returnValue MatrixVariable::init(	uint _nRows,
									uint _nCols,
									VariableType _type,
									const char** const _names,
									const char** const _units,
									DVector _scaling,
									DVector _lb,
									DVector _ub,
									BooleanType _autoInit
									)
{
	DMatrix::init( _nRows,_nCols );

	if ( VariableSettings::init( _nRows*_nCols,_type,_names,_units,_scaling,_lb,_ub,_autoInit ) != SUCCESSFUL_RETURN )
		return ACADOERROR( RET_UNKNOWN_BUG );

	return SUCCESSFUL_RETURN;
}



MatrixVariable MatrixVariable::getRows(	uint startIdx,
										uint endIdx
										) const
{
	MatrixVariable newMatrixVariable;

	if ( ( startIdx >= getNumRows( ) ) || ( endIdx >= getNumRows( ) ) )
		return newMatrixVariable;

	if ( startIdx > endIdx )
		return newMatrixVariable;

	newMatrixVariable.init( endIdx-startIdx+1,getNumCols() );
	newMatrixVariable.operator=( DMatrix::getRows( startIdx,endIdx ) );
	// needs to be implemented for VariableSettings!!

	return newMatrixVariable;
}


MatrixVariable MatrixVariable::getCols(	uint startIdx,
										uint endIdx
										) const
{
	MatrixVariable newMatrixVariable;

	if ( ( startIdx >= getNumCols( ) ) || ( endIdx >= getNumCols( ) ) )
		return newMatrixVariable;

	if ( startIdx > endIdx )
		return newMatrixVariable;


	newMatrixVariable.init( getNumRows(),endIdx-startIdx+1 );
	newMatrixVariable.operator=( DMatrix::getCols( startIdx,endIdx ) );
	// needs to be implemented for VariableSettings!!

	return newMatrixVariable;
}


//
// PROTECTED MEMBER FUNCTIONS:
//



CLOSE_NAMESPACE_ACADO


/*
 *	end of file
 */
