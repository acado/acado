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
 *    \author Hans Joachim Ferreau, Boris Houska, Milan Vukov
 *    \date 2010-2012
 */


#include <acado/code_generation/export_argument_internal.hpp>
#include <acado/code_generation/export_argument.hpp>

#include <sstream>


BEGIN_NAMESPACE_ACADO

using namespace std;


static const double undefinedEntry = 1073741824.03125; // = 2^30 + 2^-5


//
// PUBLIC MEMBER FUNCTIONS:
//

ExportArgumentInternal::ExportArgumentInternal( ) : ExportDataInternal()
{
	data->init(0, 0);

	callItByValue = BT_FALSE;
}

ExportArgumentInternal::ExportArgumentInternal(	const String& _name,
												const matrixPtr& _data,
												ExportType _type,
												ExportStruct _dataStruct,
												BooleanType _callItByValue,
												const ExportIndex& _addressIdx,
												const String& _prefix
												)
	: ExportDataInternal(_name, _type, _dataStruct, _prefix)
{
	data = _data ;
	callItByValue = _callItByValue;
	addressIdx =  _addressIdx;
}

ExportArgumentInternal::~ExportArgumentInternal( )
{}

ExportArgumentInternal* ExportArgumentInternal::clone() const
{
	return new ExportArgumentInternal( *this );
}

void ExportArgumentInternal::deepCopyMembers(	std::map<CasADi::SharedObjectNode*, CasADi::SharedObject>& already_copied
												)
{
	data.reset(new Matrix( *data ));
}

ExportArgument ExportArgumentInternal::getAddress(	const ExportIndex& rowIdx,
													const ExportIndex& colIdx
													) const
{
	if ( rowIdx.isGiven( ) )
	{
		ASSERT( rowIdx.getGivenValue() < (int)getNumRows() );
	}
	if( colIdx.isGiven() )
	{
		ASSERT( colIdx.getGivenValue() < (int)getNumCols() );
	}

	ExportIndex ind = getTotalIdx(rowIdx, colIdx);

	ExportArgument tmp(name, data, type, dataStruct, BT_FALSE, ind, prefix);

	return tmp;
}


const String ExportArgumentInternal::getAddressString(	BooleanType withDataStruct
														) const
{
	stringstream s;

	String nameStr;

	if (withDataStruct == BT_TRUE)
		nameStr = getFullName();
	else
		nameStr = getName();

	if ( addressIdx.isGiven() == BT_TRUE )
	{
		if ( addressIdx.getGivenValue() == 0 )
			s << nameStr.getName();
		else
			s << "&(" << nameStr.getName() << "[ " << addressIdx.getGivenValue() << " ])";
	}
	else
	{
		s << "&(" << nameStr.getName() << "[ " << addressIdx.get().getName()  << " ])";
	}

	String str( s.str().c_str() );

	return str;
}


uint ExportArgumentInternal::getNumRows( ) const
{
	return data->getNumRows( );
}


uint ExportArgumentInternal::getNumCols( ) const
{
	return data->getNumCols( );
}


uint ExportArgumentInternal::getDim( ) const
{
	return data->getDim( );
}


BooleanType ExportArgumentInternal::isGiven( ) const
{
	if ( getDim() == 0 )
		return BT_TRUE;

	for (uint i = 0; i < getNumRows(); ++i)
		for (uint j = 0; j < getNumCols(); ++j)
			if (acadoIsEqual(data->operator()(i, j), undefinedEntry) == BT_TRUE)
				return BT_FALSE;

	return BT_TRUE;
}



BooleanType ExportArgumentInternal::isCalledByValue( ) const
{
	return callItByValue;
}


returnValue ExportArgumentInternal::callByValue( )
{
	callItByValue = BT_TRUE;
	return SUCCESSFUL_RETURN;
}


returnValue ExportArgumentInternal::exportDataDeclaration(	FILE* file,
															const String& _realString,
															const String& _intString,
															int _precision
															) const
{
	// Variable not in use, thus no declaration necessary
	if ( getDim( ) == 0 )
		return SUCCESSFUL_RETURN;

	// Variable will be hard-coded
	if (isGiven() == BT_TRUE && getDataStruct() != ACADO_LOCAL)
		return SUCCESSFUL_RETURN;

	if ( ( isCalledByValue() == BT_TRUE ) && ( getDim() == 1 ) )
	{
		acadoFPrintf( file,"%s %s", getTypeString( _realString,_intString ).getName(),name.getName() );
	}
	else
	{
		if (data->getNumCols() > 1 && data->getNumRows() > 1)
		{
			acadoFPrintf(file,
					"/** %s %d %s %d %s", "Matrix of size:", data->getNumRows(), "x", data->getNumCols(), "(row major format)");
		}
		else
		{
			if (data->getNumCols() == 1)
				acadoFPrintf(file,"/** %s %d",      "Column vector of size:", data->getNumRows());
			else
				acadoFPrintf(file,"/** %s %d",      "Row vector of size:", data->getNumCols());
		}

		if (description.isEmpty() == BT_FALSE)
		{
			acadoFPrintf(file, "\n * \n *  %s\n", description.getName());
		}

		acadoFPrintf(file," */\n");

		acadoFPrintf( file,"%s %s[ %d ]", getTypeString( _realString,_intString ).getName(),name.getName(),getDim() );
	}

	if ( isGiven() == BT_FALSE )
	{
		acadoFPrintf( file,";\n\n" );
	}
	else
	{
		acadoFPrintf( file," = " );

		switch ( getType() )
		{
		case INT:
		case STATIC_CONST_INT:
			data->printToFile( file, "", "{ "," };\n", 5, 0, ", ",", \n" );
			break;

		case REAL:
		case STATIC_CONST_REAL:
			data->printToFile( file, "", "{ "," };\n", 1, 16, ", ",", \n" );
			break;

		default:
			return ACADOERROR( RET_NOT_YET_IMPLEMENTED );
		}
	}

	return SUCCESSFUL_RETURN;
}



//
// PROTECTED MEMBER FUNCTIONS:
//


uint ExportArgumentInternal::getColDim( ) const
{
	return data->getNumCols( );
}

ExportIndex	ExportArgumentInternal::getTotalIdx(	const ExportIndex& rowIdx,
													const ExportIndex& colIdx
													) const
{
	return rowIdx * getNumCols() + colIdx;
}


CLOSE_NAMESPACE_ACADO

// end of file.
