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
 *    \file src/code_generation/export_variable.cpp
 *    \author Hans Joachim Ferreau, Boris Houska, Milan Vukov
 *    \date 2010 - 2013
 */

#include <acado/code_generation/export_variable.hpp>
#include <acado/code_generation/export_variable_internal.hpp>

BEGIN_NAMESPACE_ACADO

using namespace std;

static const double undefinedEntry = 1073741824.03125; // = 2^30 + 2^-5


//
// PUBLIC MEMBER FUNCTIONS:
//

ExportVariableInternal::ExportVariableInternal( ) : ExportArgumentInternal( )
{
	doAccessTransposed = false;

	rowOffset = 0;
	colOffset = 0;

	rowDim = 0;
	colDim = 0;
	nRows = 0;
	nCols = 0;
}

ExportVariableInternal::ExportVariableInternal(	const std::string& _name,
												const DMatrixPtr& _data,
												ExportType _type,
												ExportStruct _dataStruct,
												bool _callItByValue,
												const std::string& _prefix
												)
	: ExportArgumentInternal(_name, _data, _type, _dataStruct, _callItByValue, emptyConstExportIndex, _prefix)
{
	doAccessTransposed = false;

	rowOffset = 0;
	colOffset = 0;

	rowDim = _data->getNumRows();
	colDim = _data->getNumCols();
	nRows = 0;
	nCols = 0;
}


ExportVariableInternal::~ExportVariableInternal( )
{}


ExportVariableInternal* ExportVariableInternal::clone() const
{
	return new ExportVariableInternal( *this );
}

bool ExportVariableInternal::isZero(	const ExportIndex& rowIdx,
										const ExportIndex& colIdx
										) const
{
	return hasValue(rowIdx, colIdx, 0.0);
}

bool ExportVariableInternal::isOne(	const ExportIndex& rowIdx,
									const ExportIndex& colIdx
									) const
{
	return hasValue(rowIdx, colIdx, 1.0);
}


bool ExportVariableInternal::isGiven(	const ExportIndex& rowIdx,
										const ExportIndex& colIdx
										) const
{
	if (hasValue(rowIdx, colIdx, undefinedEntry) == true)
		return false;

	return true;
}

bool ExportVariableInternal::isGiven() const
{
	return ExportArgumentInternal::isGiven();
}


const std::string ExportVariableInternal::get(	const ExportIndex& rowIdx,
												const ExportIndex& colIdx
												) const
{
	stringstream s;

	IoFormatter iof( s );

	iof.set(	getType() == INT || getType() == STATIC_CONST_INT ? 5 : 16,
				iof.width,
				getType() == INT || getType() == STATIC_CONST_INT ? ios::fixed : ios::scientific
				);

	ExportIndex totalIdx = getTotalIdx(rowIdx + rowOffset, colIdx + colOffset);

	if ( ( totalIdx.isGiven() == true ) && ( rowIdx.isGiven() == true ) && ( colIdx.isGiven() == true ) )
	{
		if (isGiven(rowIdx, colIdx) == false)
		{
			if ( ( isCalledByValue() == true ) && ( totalIdx.getGivenValue() == 0 ) )
				s << getFullName();
			else
				s << getFullName() << "[" << totalIdx.getGivenValue() << "]";
		}
		else
		{
			s << "(real_t)" << data->operator()(totalIdx.getGivenValue());
		}
	}
	else
		s << getFullName() << "[" << totalIdx.get( ) << "]";

	return s.str();
}


uint ExportVariableInternal::getNumRows( ) const
{
	if ( nRows > 0 )
		return nRows;

	return data->getNumRows( );
}


uint ExportVariableInternal::getNumCols( ) const
{
	if ( nCols > 0 )
		return nCols;

	return data->getNumCols( );
}


uint ExportVariableInternal::getDim( ) const
{
	return (getNumRows() * getNumCols());
}


ExportVariable ExportVariableInternal::getTranspose( ) const
{
	DMatrix m = data->transpose();

	ExportVariable transposed(name, m, type, dataStruct, callItByValue, prefix);
	transposed->setSubmatrixOffsets(colOffset, rowOffset, colDim, rowDim, nCols, nRows);
	transposed->doAccessTransposed = true;

	return transposed;
}


ExportVariable ExportVariableInternal::getRow(	const ExportIndex& idx
												) const
{
	ASSERT( doAccessTransposed == false );

	ExportVariable tmp(name, data, type, dataStruct, callItByValue, prefix);

	if (idx.isGiven() == true
			&& (idx.getGivenValue() < 0 || idx.getGivenValue() > ((int) getNumRows( ) - 1)) )
	{
		LOG( LVL_ERROR )
						<< "getRow: invalid row arguments, row index "
						<< idx.getGivenValue() << " of variable " << getFullName()
						<< " does not lie in the admissible range " << "0 - " << getNumRows( ) - 1 << endl;
	}

	tmp->setSubmatrixOffsets(idx, 0, getNumRows(), getNumCols( ), 1, getNumCols( ));

	return tmp;
}


ExportVariable ExportVariableInternal::getCol(	const ExportIndex& idx
												) const
{
	ASSERT( doAccessTransposed == false );

	ExportVariable tmp(name, data, type, dataStruct, callItByValue, prefix);

	if (idx.isGiven() == true
			&& (idx.getGivenValue() < 0 || idx.getGivenValue() > (int) getNumCols( ) - 1) )
	{
		LOG( LVL_ERROR )
			<< "getCol: invalid column arguments, column index "
			<< idx.getGivenValue() << " of variable " << getFullName()
			<< " does not lie in the admissible range " << "0 - " << getNumCols( ) - 1 << endl;
	}

	tmp->setSubmatrixOffsets(0, idx, getNumRows(), getNumCols( ), getNumRows(), 1);

	return tmp;
}


ExportVariable ExportVariableInternal::getRows(	const ExportIndex& idx1,
												const ExportIndex& idx2
												) const
{
	if (doAccessTransposed == true) ASSERT(data->getNumCols() == 1 || data->getNumRows() == 1);

	ExportVariable tmp(name, data, type, dataStruct, callItByValue, prefix);

	ExportIndex size = idx2 - idx1;

	if (size.isGiven() == true && size.getGivenValue() == 0)
		return tmp;

	if (idx1.isGiven() == true && idx2.isGiven() == true
			&& (	idx1.getGivenValue() < 0 ||
					idx1.getGivenValue() > idx2.getGivenValue() ||
					idx2.getGivenValue() > (int) getNumRows( ) ))
	{
		LOG( LVL_ERROR ) << getFullName() << ": getRows: invalid row arguments" << endl;
	}
	else if (size.isGiven() == false)
	{
		LOG( LVL_ERROR ) << getFullName() << ": getRows: Cannot determine size" << endl;
	}
	else
		tmp->setSubmatrixOffsets(idx1, 0, getNumRows(), getNumCols( ), size.getGivenValue(), getNumCols( ));

	return tmp;
}


ExportVariable ExportVariableInternal::getCols(	const ExportIndex& idx1,
												const ExportIndex& idx2
												) const
{
	if (doAccessTransposed == true) ASSERT(data->getNumCols() == 1 || data->getNumRows() == 1);

	ExportVariable tmp(name, data, type, dataStruct, callItByValue, prefix);

	ExportIndex size = idx2 - idx1;

	if (size.isGiven() == true && size.getGivenValue() == 0)
		return tmp;

	if (idx1.isGiven() == true && idx2.isGiven() == true
			&& (	idx1.getGivenValue() < 0 ||
					idx1.getGivenValue() > idx2.getGivenValue() ||
					idx2.getGivenValue() > (int) getNumCols( ) ))
	{
		LOG( LVL_ERROR ) << getFullName() << ": getCols: invalid column arguments" << endl;
	}
	else if (size.isGiven() == false)
	{
		LOG( LVL_ERROR ) << getFullName() << ": getCols: Cannot determine size" << endl;
	}
	else
		tmp->setSubmatrixOffsets(0, idx1, getNumRows(), getNumCols( ), getNumRows( ), size.getGivenValue() );

	return tmp;
}


ExportVariable ExportVariableInternal::getSubMatrix(	const ExportIndex& _rowIdx1,
														const ExportIndex& _rowIdx2,
														const ExportIndex& _colIdx1,
														const ExportIndex& _colIdx2
														) const
{
	ASSERT(doAccessTransposed == false);

	ExportVariable tmp;

	ExportIndex sizeRow = _rowIdx2 - _rowIdx1;
	ExportIndex sizeCol = _colIdx2 - _colIdx1;

	if (sizeRow.isGiven() == true && sizeRow.getGivenValue() == 0)
		return tmp;

	if (sizeCol.isGiven() == true && sizeCol.getGivenValue() == 0)
		return tmp;

	if (_rowIdx1.isGiven() == true && _rowIdx2.isGiven() == true
			&& (	_rowIdx1.getGivenValue() < 0 ||
					_rowIdx1.getGivenValue() > _rowIdx2.getGivenValue() ||
					_rowIdx2.getGivenValue() > (int) getNumRows( ) ))
	{
		LOG( LVL_ERROR ) << getFullName() << ": getSubMatrix: invalid row arguments" << endl;
		return tmp;
	}
	else if (sizeRow.isGiven() == false)
	{
		LOG( LVL_ERROR ) << getFullName() << ": getSubMatrix: cannot determine row size" << endl;
		return tmp;
	}

	if (_colIdx1.isGiven() == true && _colIdx2.isGiven() == true
			&& (	_colIdx1.getGivenValue() < 0 ||
					_colIdx1.getGivenValue() > _colIdx2.getGivenValue() ||
					_colIdx2.getGivenValue() > (int) getNumCols( ) ))
	{
		LOG( LVL_ERROR ) << getFullName() << ": getSubMatrix: invalid column arguments" << endl;
		return tmp;
	}
	else if (sizeCol.isGiven() == false)
	{
		LOG( LVL_ERROR ) << getFullName() << ": getSubMatrix: cannot determine column size" << endl;
		return tmp;
	}

	tmp.assignNode(new ExportVariableInternal(name, data, type, dataStruct, callItByValue, prefix));
	tmp->setSubmatrixOffsets(_rowIdx1, _colIdx1, getNumRows(), getNumCols( ), sizeRow.getGivenValue(), sizeCol.getGivenValue());

	return tmp;
}


ExportVariable ExportVariableInternal::makeRowVector( ) const
{
	ASSERT( ( nRows == 0 ) && ( nCols == 0 ) );

	DMatrix foo( *data.get() );
	foo.makeVector().transposeInPlace();

	ExportVariable tmp(name, foo, type, dataStruct, callItByValue, prefix);

	return tmp;
}


ExportVariable ExportVariableInternal::makeColVector( ) const
{
	ASSERT( ( nRows == 0 ) && ( nCols == 0 ) );

	DMatrix foo( *data.get() );
	foo.makeVector();

	ExportVariable tmp(name, foo, type, dataStruct, callItByValue, prefix);

	return tmp;
}


bool ExportVariableInternal::isVector( ) const
{
	if (getNumRows( ) == 1 || getNumCols( ) == 1)
		return true;

	return false;
}


const DMatrix& ExportVariableInternal::getGivenMatrix( ) const
{
	if ( isGiven() == true )
		return *data.get();

	return emptyConstMatrix;
}


returnValue ExportVariableInternal::print( ) const
{
	return data->print( );
}


//
// PROTECTED MEMBER FUNCTIONS:
//


uint ExportVariableInternal::getColDim( ) const
{
	if (doAccessTransposed == true)
		return rowDim;

	return colDim;
}


ExportIndex	ExportVariableInternal::getTotalIdx(	const ExportIndex& _rowIdx,
													const ExportIndex& _colIdx
													) const
{
	ExportIndex tmp;

	if ( doAccessTransposed == false )
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
	if ( ( _rowOffset.isGiven() == true ) && ( _rowOffset.getGivenValue() < 0 ) )
		return ACADOERROR( RET_INVALID_ARGUMENTS );

	if ( ( _colOffset.isGiven() == true ) && ( _colOffset.getGivenValue() < 0 ) )
		return ACADOERROR( RET_INVALID_ARGUMENTS );

	if ( ( _colOffset.isGiven() == true ) && ( _colOffset.getGivenValue() > (int)_colDim ) )
		return ACADOERROR( RET_INVALID_ARGUMENTS );

	rowOffset = _rowOffset;
	colOffset = _colOffset;
	rowDim    = _rowDim;
	colDim    = _colDim;

	nRows = _nRows;
	nCols = _nCols;

	return SUCCESSFUL_RETURN;
}


bool ExportVariableInternal::hasValue(	const ExportIndex& rowIdx,
										const ExportIndex& colIdx,
										double _value
										) const
{
	if ((getType() == STATIC_CONST_INT || getType() == STATIC_CONST_REAL) &&
			acadoIsEqual(_value, undefinedEntry) == true)
		return true;

	ExportIndex ind = getTotalIdx(rowIdx + rowOffset, colIdx + colOffset);

	if (ind.isGiven() == true)
		return acadoIsEqual(data->operator()(ind.getGivenValue()), _value);

	return false;
}

bool ExportVariableInternal::isSubMatrix() const
{
	if (nRows == 0 && nCols == 0)
		return false;

	return true;
}

bool ExportVariableInternal::isDiagonal() const
{
	if (isSubMatrix() == true)
	{
		LOG( LVL_DEBUG ) << "Digonal check works for non-sub-matrices only ATM" << endl;
		return false;
	}

	DMatrix foo( *data );
	Eigen::DiagonalMatrix<double, Eigen::Dynamic, Eigen::Dynamic> bar( foo.diagonal() );

	return (foo == bar);
}


CLOSE_NAMESPACE_ACADO

// end of file.
