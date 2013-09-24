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
 *    \file src/code_generation/export_argument.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 2010-2011
 */

#include <acado/code_generation/export_index.hpp>
#include <acado/code_generation/export_argument.hpp>
#include <acado/code_generation/export_argument_internal.hpp>


BEGIN_NAMESPACE_ACADO

static const double undefinedEntry = 1073741824.03125; // = 2^30 + 2^-5

//
// PUBLIC MEMBER FUNCTIONS:
//

ExportArgument::ExportArgument( )
{
	Matrix m(0, 0);
	assignNode(new ExportArgumentInternal(
				"defaultArgumentName", matrixPtr(new Matrix( m )), REAL, ACADO_LOCAL, BT_FALSE, 0, emptyConstString));
}


ExportArgument::ExportArgument(	const String& _name,
								uint _nRows,
								uint _nCols,
								ExportType _type,
								ExportStruct _dataStruct,
								BooleanType _callItByValue,
								const ExportIndex& _addressIdx,
								const String& _prefix
								)
{
	Matrix m(_nRows, _nCols);
	m.setAll( undefinedEntry );

	assignNode(new ExportArgumentInternal(
			_name, matrixPtr(new Matrix( m )), _type, _dataStruct, _callItByValue, _addressIdx, _prefix));
}


ExportArgument::ExportArgument(	const String& _name,
								const matrixPtr& _data,
								ExportType _type,
								ExportStruct _dataStruct,
								BooleanType _callItByValue,
								const ExportIndex& _addressIdx,
								const String& _prefix
								)
{
	assignNode(new ExportArgumentInternal(
			_name, _data, _type, _dataStruct, _callItByValue, _addressIdx, _prefix));
}

ExportArgument::ExportArgument( const Matrix& _data
								)
{
	assignNode(new ExportArgumentInternal(
			"defaultArgumentName", matrixPtr(new Matrix( _data )), REAL, ACADO_LOCAL, BT_FALSE, 0, emptyConstString));
}

ExportArgumentInternal* ExportArgument::operator->()
{
	return (ExportArgumentInternal*)(ExportData::operator->());
}

const ExportArgumentInternal* ExportArgument::operator->() const
{
	return (const ExportArgumentInternal*)(ExportData::operator->());
}

ExportArgument ExportArgument::getAddress(	const ExportIndex& _rowIdx,
											const ExportIndex& _colIdx
											) const
{
	return (*this)->getAddress(_rowIdx, _colIdx);
}

const String ExportArgument::getAddressString(	BooleanType withDataStruct
												) const
{
	return (*this)->getAddressString( withDataStruct );
}


uint ExportArgument::getNumRows( ) const
{
	return (*this)->getNumRows();
}


uint ExportArgument::getNumCols( ) const
{
	return (*this)->getNumCols();
}


uint ExportArgument::getDim( ) const
{
	return (*this)->getDim();
}



BooleanType ExportArgument::isGiven( ) const
{
	return (*this)->isGiven();
}



BooleanType ExportArgument::isCalledByValue( ) const
{
	return (*this)->isCalledByValue();
}


returnValue ExportArgument::callByValue( )
{
	return (*this)->callByValue();
}



returnValue ExportArgument::exportDataDeclaration(	FILE* file,
													const String& _realString,
													const String& _intString,
													int _precision
													) const
{
	return (*this)->exportDataDeclaration(file, _realString, _intString, _precision);
}

CLOSE_NAMESPACE_ACADO

// end of file.
