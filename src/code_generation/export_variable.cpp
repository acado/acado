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
 *    \file src/code_generation/export_variable.cpp
 *    \authors Hans Joachim Ferreau, Boris Houska, Milan Vukov
 *    \date 2010-2013
 */


#include <acado/code_generation/export_variable.hpp>
#include <acado/code_generation/export_variable_internal.hpp>
#include <acado/code_generation/export_arithmetic_statement.hpp>
#include <acado/symbolic_expression/acado_syntax.hpp>


BEGIN_NAMESPACE_ACADO


static const double undefinedEntry = 1073741824.03125; // = 2^30 + 2^-5



//
// PUBLIC MEMBER FUNCTIONS:
//


ExportVariable::ExportVariable(	const String& _name,
								uint _nRows,
								uint _nCols,
								ExportType _type,
								ExportStruct _dataStruct,
								BooleanType _callItByValue,
								const String& _prefix
								)
{
	Matrix m(_nRows, _nCols);
	m.setAll( undefinedEntry );

	assignNode(new ExportVariableInternal(_name, matrixPtr(new Matrix( m )), _type, _dataStruct, _callItByValue, _prefix));
}


ExportVariable::ExportVariable(	const String& _name,
								const Matrix& _data,
								ExportType _type,
								ExportStruct _dataStruct,
								BooleanType _callItByValue,
								const String& _prefix
								)
{
	assignNode(new ExportVariableInternal(_name, matrixPtr(new Matrix( _data )), _type, _dataStruct, _callItByValue, _prefix));
}

ExportVariable::ExportVariable(	unsigned _nRows,
								unsigned _nCols,
								ExportType _type,
								ExportStruct _dataStruct,
								BooleanType _callItByValue,
								const String& _prefix
								)
{
	Matrix m(_nRows, _nCols);
	m.setAll( undefinedEntry );

	assignNode(new ExportVariableInternal("var", matrixPtr(new Matrix( m )), _type, _dataStruct, _callItByValue, _prefix));
}

ExportVariable::ExportVariable(	const String& _name,
								const matrixPtr& _data,
								ExportType _type,
								ExportStruct _dataStruct,
								BooleanType _callItByValue,
								const String& _prefix
								)
{
	assignNode(new ExportVariableInternal(_name, _data, _type, _dataStruct, _callItByValue, _prefix));
}

ExportVariable::ExportVariable( const Matrix& _data )
{
	assignNode(new ExportVariableInternal("var", matrixPtr(new Matrix( _data ))));
}

ExportVariable::ExportVariable( const Vector& _data )
{
	assignNode(new ExportVariableInternal("var", matrixPtr(new Matrix( _data ))));
}

ExportVariable::ExportVariable( const double _data )
{
	assignNode(new ExportVariableInternal("var", matrixPtr(new Matrix( _data ))));
}


ExportVariable::~ExportVariable( )
{}

ExportVariable ExportVariable::clone() const
{
	ExportVariable ret;
	if( !isNull() )
		ret.assignNode( (*this)->clone() );

	return ret;
}


ExportVariableInternal* ExportVariable::operator->()
{
	return (ExportVariableInternal*)(ExportArgument::operator->());
}


const ExportVariableInternal* ExportVariable::operator->() const
{
	return (const ExportVariableInternal*)(ExportArgument::operator->());
}


ExportVariable& ExportVariable::setup(	const String& _name,
										uint _nRows,
										uint _nCols,
										ExportType _type,
										ExportStruct _dataStruct,
										BooleanType _callItByValue,
										const String& _prefix
										)
{
	Matrix m(_nRows, _nCols);
	m.setAll( undefinedEntry );

	assignNode(new ExportVariableInternal(_name, matrixPtr(new Matrix( m )), _type, _dataStruct, _callItByValue, _prefix));

	return *this;
}


ExportVariable& ExportVariable::setup(	const String& _name,
										const Matrix& _data,
										ExportType _type,
										ExportStruct _dataStruct,
										BooleanType _callItByValue,
										const String& _prefix
										)
{
	assignNode(new ExportVariableInternal(_name, matrixPtr(new Matrix( _data )), _type, _dataStruct, _callItByValue, _prefix));

	return *this;
}


double ExportVariable::operator()(	uint rowIdx,
									uint colIdx
									) const
{
	return (*this)->data->operator()(rowIdx, colIdx);
}


double ExportVariable::operator()(	uint totalIdx
									) const
{
	return (*this)->data->operator()(totalIdx / (*this)->data->getNumCols(), totalIdx % (*this)->data->getNumCols());
}


ExportVariable ExportVariable::operator()(	const String& _name
											) const
{
	ExportVariable tmp = deepcopy( *this );

	tmp.setName( _name );

	return tmp;
}


returnValue ExportVariable::resetAll( )
{
	return (*this)->resetAll();
}


returnValue ExportVariable::resetDiagonal( )
{
	return (*this)->resetDiagonal();
}


BooleanType ExportVariable::isZero(	const ExportIndex& rowIdx,
									const ExportIndex& colIdx
									) const
{
	return (*this)->isZero(rowIdx, colIdx);
}


BooleanType ExportVariable::isOne(	const ExportIndex& rowIdx,
									const ExportIndex& colIdx
									) const
{
	return (*this)->isOne(rowIdx, colIdx);
}


BooleanType ExportVariable::isGiven(	const ExportIndex& rowIdx,
										const ExportIndex& colIdx
										) const
{
	return (*this)->isGiven(rowIdx, colIdx);
}


BooleanType ExportVariable::isGiven( ) const
{
	return (*this)->isGiven();
}


const String ExportVariable::get(	const ExportIndex& rowIdx,
									const ExportIndex& colIdx
									) const
{
	return (*this)->get(rowIdx, colIdx);
}

uint ExportVariable::getNumRows( ) const
{
	return (*this)->getNumRows();
}

uint ExportVariable::getNumCols( ) const
{
	return (*this)->getNumCols();
}


uint ExportVariable::getDim( ) const
{
	return (*this)->getDim();
}


ExportArithmeticStatement operator+(	const ExportVariable& arg1,
										const ExportVariable& arg2
										)
{
//	ASSERT( getNumRows() == arg.getNumRows() );
//	ASSERT( getNumCols() == arg.getNumCols() );

	return ExportArithmeticStatement(0, ESO_ASSIGN, arg1, ESO_ADD, arg2);
}


ExportArithmeticStatement operator-(	const ExportVariable& arg1,
										const ExportVariable& arg2
										)
{
//	ASSERT( getNumRows() == arg.getNumRows() );
//	ASSERT( getNumCols() == arg.getNumCols() );

	return ExportArithmeticStatement(0, ESO_ASSIGN, arg1, ESO_SUBTRACT, arg2);
}


ExportArithmeticStatement operator+=(	const ExportVariable& arg1,
										const ExportVariable& arg2
										)
{
//	ASSERT( getNumRows() == arg.getNumRows() );
//	ASSERT( getNumCols() == arg.getNumCols() );

	return ExportArithmeticStatement(arg1, ESO_ASSIGN, arg2, ESO_ADD_ASSIGN, 0);
}


ExportArithmeticStatement operator-=(	const ExportVariable& arg1,
										const ExportVariable& arg2
										)
{
//	ASSERT( getNumRows() == arg.getNumRows() );
//	ASSERT( getNumCols() == arg.getNumCols() );

	return ExportArithmeticStatement(arg1, ESO_ASSIGN, arg2, ESO_SUBTRACT_ASSIGN, 0);
}


ExportArithmeticStatement operator*(	const ExportVariable& arg1,
										const ExportVariable& arg2
										)
{
//	ASSERT( getNumCols() == arg.getNumRows() );

	return ExportArithmeticStatement(0, ESO_ASSIGN, arg1, ESO_MULTIPLY, arg2);
}


ExportArithmeticStatement operator^(	const ExportVariable& arg1,
										const ExportVariable& arg2
										)
{
//	ASSERT( getNumRows() == arg.getNumRows() );

	return ExportArithmeticStatement(0, ESO_ASSIGN, arg1, ESO_MULTIPLY_TRANSPOSE, arg2);
}


ExportArithmeticStatement operator==(	const ExportVariable& arg1,
										const ExportVariable& arg2
										)
{
//	ASSERT( getNumRows() == arg.getNumRows() );
//	ASSERT( getNumCols() == arg.getNumCols() );

	return ExportArithmeticStatement(arg1, ESO_ASSIGN, arg2, ESO_ASSIGN, 0);
}


ExportArithmeticStatement ExportVariable::operator==(	ExportArithmeticStatement arg
														) const
{
//	ASSERT( getNumRows() == arg.getNumRows() );
//	ASSERT( getNumCols() == arg.getNumCols() );

	return ExportArithmeticStatement(*this, ESO_ASSIGN, arg.rhs1, arg.op1, arg.rhs2, arg.op2, arg.rhs3);
}


ExportArithmeticStatement ExportVariable::operator+(	ExportArithmeticStatement arg
														) const
{
//	ASSERT( getNumRows() == arg.getNumRows() );
//	ASSERT( getNumCols() == arg.getNumCols() );
//
//	ASSERT( ( arg.op1 == ESO_MULTIPLY ) || ( arg.op1 == ESO_MULTIPLY_TRANSPOSE ) );

	return ExportArithmeticStatement( 0,ESO_UNDEFINED,arg.rhs1,arg.op1,arg.rhs2,ESO_ADD,*this );
}


ExportArithmeticStatement ExportVariable::operator-(	ExportArithmeticStatement arg
														) const
{
//	ASSERT( getNumRows() == arg.getNumRows() );
//	ASSERT( getNumCols() == arg.getNumCols() );
//
//	ASSERT( ( arg.op1 == ESO_MULTIPLY ) || ( arg.op1 == ESO_MULTIPLY_TRANSPOSE ) );

	return ExportArithmeticStatement( 0,ESO_UNDEFINED,arg.rhs1,arg.op1,arg.rhs2,ESO_SUBTRACT,*this );
}


ExportArithmeticStatement ExportVariable::operator+=(	ExportArithmeticStatement arg
														) const
{
//	ASSERT( getNumRows() == arg.getNumRows() );
//	ASSERT( getNumCols() == arg.getNumCols() );
//
//	ASSERT( ( arg.op1 == ESO_MULTIPLY ) ||
//			( arg.op1 == ESO_MULTIPLY_TRANSPOSE ) ||
//			( arg.op1 == ESO_ADD ) ||
//			( arg.op1 == ESO_SUBTRACT ) );

	return ExportArithmeticStatement( *this,ESO_ADD_ASSIGN,arg.rhs1,arg.op1,arg.rhs2,arg.op2,arg.rhs3 );
}


ExportArithmeticStatement ExportVariable::operator-=(	ExportArithmeticStatement arg
														) const
{
//	ASSERT( getNumRows() == arg.getNumRows() );
//	ASSERT( getNumCols() == arg.getNumCols() );
//
//	ASSERT( ( arg.op1 == ESO_MULTIPLY ) ||
//			( arg.op1 == ESO_MULTIPLY_TRANSPOSE ) ||
//			( arg.op1 == ESO_ADD ) ||
//			( arg.op1 == ESO_SUBTRACT ) );

	return ExportArithmeticStatement( *this,ESO_SUBTRACT_ASSIGN,arg.rhs1,arg.op1,arg.rhs2,arg.op2,arg.rhs3 );
}


ExportVariable ExportVariable::getTranspose( ) const
{
	return (*this)->getTranspose();
}

ExportVariable ExportVariable::getRow(	const ExportIndex& idx
										) const
{
	return (*this)->getRow( idx );
}


ExportVariable ExportVariable::getCol(	const ExportIndex& idx
										) const
{
	return (*this)->getCol( idx );
}


ExportVariable ExportVariable::getRows(	const ExportIndex& idx1,
										const ExportIndex& idx2
										) const
{
	return (*this)->getRows(idx1, idx2);
}


ExportVariable ExportVariable::getCols(	const ExportIndex& idx1,
										const ExportIndex& idx2
										) const
{
	return (*this)->getCols(idx1, idx2);
}


ExportVariable ExportVariable::getSubMatrix(	const ExportIndex& rowIdx1,
												const ExportIndex& rowIdx2,
												const ExportIndex& colIdx1,
												const ExportIndex& colIdx2
												) const
{
	return (*this)->getSubMatrix(rowIdx1, rowIdx2, colIdx1, colIdx2);
}

ExportVariable ExportVariable::getElement(	const ExportIndex& rowIdx,
											const ExportIndex& colIdx
											) const
{
	return (*this)->getSubMatrix(rowIdx, rowIdx + 1, colIdx, colIdx + 1);
}


ExportVariable ExportVariable::makeRowVector( ) const
{
	return (*this)->makeRowVector();
}


ExportVariable ExportVariable::makeColVector( ) const
{
	return (*this)->makeColVector();
}


BooleanType ExportVariable::isVector( ) const
{
	return (*this)->isVector();
}


Matrix ExportVariable::getGivenMatrix( ) const
{
	return (*this)->getGivenMatrix();
}


returnValue ExportVariable::print( ) const
{
	return (*this)->print();
}

ExportVariable diag(	const String& _name,
						unsigned int _n )
{
	ExportVariable t(_name, _n, _n);
	t = eye( _n );
	t.resetDiagonal( );
	return t;
}

BooleanType ExportVariable::isSubMatrix() const
{
	return (*this)->isSubMatrix();
}

CLOSE_NAMESPACE_ACADO

// end of file.
