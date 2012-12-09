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
 *    \file src/code_generation/export_variable.cpp
 *    \author Hans Joachim Ferreau, Boris Houska, Milan Vukov
 *    \date 2010 - 2012
 */

#include <acado/code_generation/export_variable.hpp>
#include <acado/code_generation/export_variable_internal.hpp>

#include <sstream>
#include <iomanip>

BEGIN_NAMESPACE_ACADO

using namespace std;


static const double undefinedEntry = 1073741824.03125; // = 2^30 + 2^-5


//
// PUBLIC MEMBER FUNCTIONS:
//

ExportVariableInternal::ExportVariableInternal( ) : ExportArgumentInternal( )
{
	doAccessTransposed = BT_FALSE;

	rowOffset = 0;
	colOffset = 0;

	rowDim = 0;
	colDim = 0;
	nRows = 0;
	nCols = 0;
}

ExportVariableInternal::ExportVariableInternal(	const String& _name,
												const Matrix& _data,
												ExportType _type,
												ExportStruct _dataStruct,
												BooleanType _callItByValue,
												const String& _prefix
												) : ExportArgumentInternal(_name, _data, _type, _dataStruct, _callItByValue, emptyConstExportIndex, _prefix)
{
	doAccessTransposed = BT_FALSE;

	rowOffset = 0;
	colOffset = 0;

	rowDim = _data.getNumRows();
	colDim = _data.getNumCols();
	nRows = 0;
	nCols = 0;
}


ExportVariableInternal::~ExportVariableInternal( )
{}


ExportVariableInternal* ExportVariableInternal::clone() const
{
	return new ExportVariableInternal( *this );
}


returnValue ExportVariableInternal::resetAll( )
{
	return data.setAll( undefinedEntry );
}


returnValue ExportVariableInternal::resetDiagonal( )
{
	if ( getNumRows() != getNumCols() )
		return ACADOERROR( RET_MATRIX_NOT_SQUARE );

	for( uint i=0; i<getNumRows(); ++i )
		data(i, i) = undefinedEntry;

	return SUCCESSFUL_RETURN;
}


BooleanType ExportVariableInternal::isZero(	const ExportIndex& rowIdx,
											const ExportIndex& colIdx
											) const
{
	return hasValue(rowIdx, colIdx, 0.0);
}

BooleanType ExportVariableInternal::isOne(	const ExportIndex& rowIdx,
											const ExportIndex& colIdx
											) const
{
	return hasValue(rowIdx, colIdx, 1.0);
}


BooleanType ExportVariableInternal::isGiven(	const ExportIndex& rowIdx,
												const ExportIndex& colIdx
												) const
{
	if (hasValue(rowIdx, colIdx, undefinedEntry) == BT_TRUE)
		return BT_FALSE;

	return BT_TRUE;
}

BooleanType ExportVariableInternal::isGiven() const
{
	return ExportArgumentInternal::isGiven();
}


const String ExportVariableInternal::get(	const ExportIndex& rowIdx,
											const ExportIndex& colIdx
											) const
{
	stringstream s;
	s.precision( 16 );
	s << scientific;

	ExportIndex totalIdx = getTotalIdx(rowIdx + rowOffset, colIdx + colOffset);

	if ( ( totalIdx.isGiven() == BT_TRUE ) && ( rowIdx.isGiven() == BT_TRUE ) && ( colIdx.isGiven() == BT_TRUE ) )
	{
		if (isGiven(rowIdx, colIdx) == BT_FALSE)
		{
			if ( ( isCalledByValue() == BT_TRUE ) && ( totalIdx.getGivenValue() == 0 ) )
				s << getFullName().getName();
			else
				s << getFullName().getName() << "[" << totalIdx.getGivenValue() << "]";
		}
		else
			s << "(real_t)" << data(rowIdx.getGivenValue(), colIdx.getGivenValue());
	}
	else
		s << getFullName().getName() << "[" << totalIdx.get( ).getName() << "]";

	String str( s.str().c_str() );

	return str;
}


uint ExportVariableInternal::getNumRows( ) const
{
	if ( nRows > 0 )
		return nRows;

	return data.getNumRows( );
}


uint ExportVariableInternal::getNumCols( ) const
{
	if ( nCols > 0 )
		return nCols;

	return data.getNumCols( );
}


uint ExportVariableInternal::getDim( ) const
{
	return (getNumRows() * getNumCols());
}


ExportVariable ExportVariableInternal::getTranspose( ) const
{
	unsigned nc = getNumCols();
	unsigned nr = getNumRows();

	Matrix m(nc, nr);

	for(unsigned i = 0; i < nc; ++i)
		for(unsigned j = 0; j < nr; ++j)
			m(i, j) = data(j, i);

	ExportVariableInternal tmp(name, m, type, dataStruct, callItByValue, prefix);
	tmp.setSubmatrixOffsets(colOffset, rowOffset, colDim, rowDim, nCols, nRows);
	tmp.doAccessTransposed = BT_TRUE;

	ExportVariable transposed;
	transposed.assignNode( tmp.clone() );

	return transposed;
}


ExportVariable ExportVariableInternal::getRow(	const ExportIndex& idx
												) const
{
	ASSERT( isAccessedTransposed() == BT_FALSE );

	ExportVariable tmp;
	tmp.assignNode( this->clone() );

	// OR, proposed by Joel
//	ExportVariable tmp = shared_from_this<ExportVariable>();
//	tmp.makeUnique();

	if (idx.isGiven() == BT_TRUE)
	{
		if (	idx.getGivenValue() < 0 ||
				idx.getGivenValue() > (int) getNumRows( ) )
			ACADOERRORTEXT(RET_INVALID_ARGUMENTS, "getRow: invalid row arguments");
	}

	tmp->setSubmatrixOffsets(idx, 0, getNumRows(), getNumCols( ), 1, getNumCols( ));

	return tmp;
}


ExportVariable ExportVariableInternal::getCol(	const ExportIndex& idx
												) const
{
	ASSERT( isAccessedTransposed() == BT_FALSE );

	ExportVariable tmp;
	tmp.assignNode( this->clone() );

	if (idx.isGiven() == BT_TRUE)
	{
		if (	idx.getGivenValue() < 0 ||
				idx.getGivenValue() > (int) getNumCols( ) )
			ACADOERRORTEXT(RET_INVALID_ARGUMENTS, "getCol: invalid row arguments");
	}

	tmp->setSubmatrixOffsets(0, idx, getNumRows(), getNumCols( ), getNumRows(), 1);

	return tmp;
}


ExportVariable ExportVariableInternal::getRows(	const ExportIndex& idx1,
												const ExportIndex& idx2
												) const
{
	ASSERT( isAccessedTransposed() == BT_FALSE );

	ExportVariable tmp;

	ExportIndex size = idx2 - idx1;

	if (size.isGiven() == BT_TRUE && size.getGivenValue() == 0)
		return tmp;

	if (idx1.isGiven() == BT_TRUE && idx2.isGiven() == BT_TRUE)
	{
		if (	idx1.getGivenValue() < 0 ||
				idx1.getGivenValue() > idx2.getGivenValue() ||
				idx1.getGivenValue() > (int) getNumRows( ) )
			ACADOERRORTEXT(RET_INVALID_ARGUMENTS, "getRows: invalid row arguments");
	}
	else if (size.isGiven() == BT_FALSE)
		ACADOERRORTEXT(RET_INVALID_ARGUMENTS, "getRows: Cannot determine size");

	tmp.assignNode( this->clone() );
	tmp->setSubmatrixOffsets(idx1, 0, getNumRows(), getNumCols( ), size.getGivenValue(), getNumCols( ));

	return tmp;
}


ExportVariable ExportVariableInternal::getCols(	const ExportIndex& idx1,
												const ExportIndex& idx2
												) const
{
	ASSERT( isAccessedTransposed() == BT_FALSE );

	ExportVariable tmp;

	// OR, proposed by Joel
//	ExportVariable tmp = shared_from_this<ExportVariable>();
//	tmp.makeUnique();

	ExportIndex size = idx2 - idx1;

	if (size.isGiven() == BT_TRUE && size.getGivenValue() == 0)
		return tmp;

	if (idx1.isGiven() == BT_TRUE && idx2.isGiven() == BT_TRUE)
	{
		if (	idx1.getGivenValue() < 0 ||
				idx1.getGivenValue() > idx2.getGivenValue() ||
				idx1.getGivenValue() > (int) getNumCols( ) )
			ACADOERRORTEXT(RET_INVALID_ARGUMENTS, "getCols: invalid row arguments");
	}
	else if (size.isGiven() == BT_FALSE)
		ACADOERRORTEXT(RET_INVALID_ARGUMENTS, "getCols: Cannot determine size");

	tmp.assignNode( this->clone() );
	tmp->setSubmatrixOffsets(0, idx1, getNumRows(), getNumCols( ), getNumRows( ), size.getGivenValue() );
	return tmp;
}


ExportVariable ExportVariableInternal::getSubMatrix(	const ExportIndex& _rowIdx1,
														const ExportIndex& _rowIdx2,
														const ExportIndex& _colIdx1,
														const ExportIndex& _colIdx2
														) const
{
	ASSERT( isAccessedTransposed() == BT_FALSE );

	ExportVariable tmp;

	ExportIndex sizeRow = _rowIdx2 - _rowIdx1;
	ExportIndex sizeCol = _colIdx2 - _colIdx1;

	if (sizeRow.isGiven() == BT_TRUE && sizeRow.getGivenValue() == 0)
		return tmp;

	if (sizeCol.isGiven() == BT_TRUE && sizeCol.getGivenValue() == 0)
		return tmp;

	if (_rowIdx1.isGiven() == BT_TRUE && _rowIdx2.isGiven() == BT_TRUE)
	{
		if (	_rowIdx1.getGivenValue() < 0 ||
				_rowIdx1.getGivenValue() > _rowIdx2.getGivenValue() ||
				_rowIdx1.getGivenValue() > (int) getNumRows( ) )
			ACADOERRORTEXT(RET_INVALID_ARGUMENTS, "getSubMatrix: invalid row arguments");
	}
	else if (sizeRow.isGiven() == BT_FALSE)
		ACADOERRORTEXT(RET_INVALID_ARGUMENTS, "getSubMatrix: cannot determine row size");

	if (_colIdx1.isGiven() == BT_TRUE && _colIdx2.isGiven() == BT_TRUE)
	{
		if (	_colIdx1.getGivenValue() < 0 ||
				_colIdx1.getGivenValue() > _colIdx2.getGivenValue() ||
				_colIdx1.getGivenValue() > (int) getNumCols( ) )
			ACADOERRORTEXT(RET_INVALID_ARGUMENTS, "getSubMatrix: invalid column arguments");
	}
	else if (sizeCol.isGiven() == BT_FALSE)
		ACADOERRORTEXT(RET_INVALID_ARGUMENTS, "getSubMatrix: cannot determine column size");

	tmp.assignNode( this->clone() );
	tmp->setSubmatrixOffsets(_rowIdx1, _colIdx1, getNumRows(), getNumCols( ), sizeRow.getGivenValue(), sizeCol.getGivenValue() );

	return tmp;
}


ExportVariable ExportVariableInternal::makeRowVector( ) const
{
	ASSERT( ( nRows == 0 ) && ( nCols == 0 ) );

	Matrix m(1, getDim());
	unsigned nc = getNumCols();

	for ( uint i=0; i<getNumRows(); ++i )
		for ( uint j=0; j<getNumCols(); ++j )
			m(0,i * nc + j) = data(i, j);

	ExportVariable tmp(name, m, type, dataStruct, callItByValue, prefix);

	return tmp;
}


ExportVariable ExportVariableInternal::makeColVector( ) const
{
	ASSERT( ( nRows == 0 ) && ( nCols == 0 ) );

	Matrix m(getDim(), 1);
	unsigned nc = getNumCols();

	for ( uint i=0; i<getNumRows(); ++i )
		for ( uint j=0; j<getNumCols(); ++j )
			m(i * nc + j, 0) = data(i, j);

	ExportVariable tmp(name, m, type, dataStruct, callItByValue, prefix);

	return tmp;
}


BooleanType ExportVariableInternal::isVector( ) const
{
	if ( ( getNumRows( ) == 1 ) || ( getNumCols( ) == 1 ) )
		return BT_TRUE;
	else
		return BT_FALSE;
}


Matrix ExportVariableInternal::getGivenMatrix( ) const
{
	if ( isGiven() == BT_TRUE )
		return data;
	else
		return Matrix();
}


returnValue ExportVariableInternal::print( ) const
{
	return data.print( name.getName() );
}


//
// PROTECTED MEMBER FUNCTIONS:
//


uint ExportVariableInternal::getColDim( ) const
{
	if (doAccessTransposed == BT_TRUE)
		return rowDim;

	return colDim;
}


ExportIndex	ExportVariableInternal::getTotalIdx(	const ExportIndex& _rowIdx,
													const ExportIndex& _colIdx
													) const
{
	ExportIndex tmp;

	if ( doAccessTransposed == BT_FALSE )
		tmp = _rowIdx * getColDim() + _colIdx;
	else
		tmp = _colIdx * getColDim() + _rowIdx;

	return tmp;
}


returnValue ExportVariableInternal::setSubmatrixOffsets(	const ExportIndex& _rowOffset,
															const ExportIndex& _colOffset,
															unsigned _rowDim,
															unsigned _colDim,
															unsigned _nRows,
															unsigned _nCols
															)
{
	if ( ( _rowOffset.isGiven() == BT_TRUE ) && ( _rowOffset.getGivenValue() < 0 ) )
		return ACADOERROR( RET_INVALID_ARGUMENTS );

	if ( ( _colOffset.isGiven() == BT_TRUE ) && ( _colOffset.getGivenValue() < 0 ) )
		return ACADOERROR( RET_INVALID_ARGUMENTS );

	if ( ( _colOffset.isGiven() == BT_TRUE ) && ( _colOffset.getGivenValue() > (int)_colDim ) )
		return ACADOERROR( RET_INVALID_ARGUMENTS );

	rowOffset = _rowOffset;
	colOffset = _colOffset;
	rowDim    = _rowDim;
	colDim    = _colDim;

	nRows = _nRows;
	nCols = _nCols;

	return SUCCESSFUL_RETURN;
}


BooleanType ExportVariableInternal::hasValue(	const ExportIndex& rowIdx,
												const ExportIndex& colIdx,
												double _value
												) const
{
	if ( ( rowIdx.isGiven() == BT_TRUE ) && ( colIdx.isGiven() == BT_TRUE ) )
		return acadoIsEqual(data(rowIdx.getGivenValue(), colIdx.getGivenValue()), _value);

	return BT_FALSE;
}


CLOSE_NAMESPACE_ACADO

// end of file.
