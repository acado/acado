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
	DMatrix m(0, 0);
	assignNode(new ExportArgumentInternal(
				"defaultArgumentName", DMatrixPtr(new DMatrix( m )), REAL, ACADO_LOCAL, false, 0, ""));
}


ExportArgument::ExportArgument(	const std::string& _name,
								uint _nRows,
								uint _nCols,
								ExportType _type,
								ExportStruct _dataStruct,
								bool _callItByValue,
								const ExportIndex& _addressIdx,
								const std::string& _prefix
								)
{
	DMatrix m(_nRows, _nCols);
	m.setAll( undefinedEntry );

	assignNode(new ExportArgumentInternal(
			_name, DMatrixPtr(new DMatrix( m )), _type, _dataStruct, _callItByValue, _addressIdx, _prefix));
}


ExportArgument::ExportArgument(	const std::string& _name,
								const DMatrixPtr& _data,
								ExportType _type,
								ExportStruct _dataStruct,
								bool _callItByValue,
								const ExportIndex& _addressIdx,
								const std::string& _prefix
								)
{
	assignNode(new ExportArgumentInternal(
			_name, _data, _type, _dataStruct, _callItByValue, _addressIdx, _prefix));
}

ExportArgument::ExportArgument( const DMatrix& _data
								)
{
	assignNode(new ExportArgumentInternal(
			"defaultArgumentName", DMatrixPtr(new DMatrix( _data )), REAL, ACADO_LOCAL, false, 0, ""));
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

const std::string ExportArgument::getAddressString(	bool withDataStruct
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



bool ExportArgument::isGiven( ) const
{
	return (*this)->isGiven();
}



bool ExportArgument::isCalledByValue( ) const
{
	return (*this)->isCalledByValue();
}


returnValue ExportArgument::callByValue( )
{
	return (*this)->callByValue();
}



returnValue ExportArgument::exportDataDeclaration(	std::ostream& stream,
													const std::string& _realString,
													const std::string& _intString,
													int _precision
													) const
{
	return (*this)->exportDataDeclaration(stream, _realString, _intString, _precision);
}

CLOSE_NAMESPACE_ACADO

// end of file.
