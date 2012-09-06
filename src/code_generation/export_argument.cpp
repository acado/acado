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
 *    \file src/code_generation/export_argument.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 2010-2011
 */


#include <acado/code_generation/export_argument.hpp>
// #include <acado/code_generation/export_index.hpp>


BEGIN_NAMESPACE_ACADO


static const double undefinedEntry = 1073741824.03125; // = 2^30 + 2^-5
static char exportDataString[1024];


//
// PUBLIC MEMBER FUNCTIONS:
//

ExportArgument::ExportArgument( ) : ExportData( )
{
	init( "",0,0 );
}


ExportArgument::ExportArgument(	const String& _name,
								uint _nRows,
								uint _nCols,
								ExportType _type,
								ExportStruct _dataStruct,
								BooleanType _callItByValue,
								const ExportIndex& _addressIdx
								) : ExportData( )
{
	init( _name,_nRows,_nCols,_type,_dataStruct,_callItByValue,_addressIdx );
}


ExportArgument::ExportArgument(	const String& _name,
								const Matrix& _data,
								ExportType _type,
								ExportStruct _dataStruct,
								BooleanType _callItByValue,
								const ExportIndex& _addressIdx
								) : ExportData( )
{
	init( _name,_data,_type,_dataStruct,_callItByValue,_addressIdx );
}


// ExportArgument::ExportArgument(	const ExportIndex& _arg
// 										)
// {
// 	init( _arg.name,_arg.type,1,1,0,BT_TRUE );
// }


ExportArgument::ExportArgument(	const ExportArgument& arg
								) : ExportData( )
{
	init( arg.name, arg.data, arg.type, arg.dataStruct, arg.callItByValue, arg.addressIdx );
}


ExportArgument::~ExportArgument( )
{
	clear( );
}


ExportArgument& ExportArgument::operator=(	const ExportArgument& arg
											)
{
	if( this != &arg )
	{
		init( arg.name, arg.data, arg.type, arg.dataStruct, arg.callItByValue, arg.addressIdx );
	}

	return *this;
}


ExportArgument& ExportArgument::operator=(	const Matrix& arg	
											)
{
	String tmp;
	if ( name.isEmpty() == BT_FALSE )
		tmp = name;
	else
		tmp = "M";

	init( tmp,arg );
	return *this;
}


ExportData* ExportArgument::clone( ) const
{
	return new ExportArgument(*this);
}




returnValue ExportArgument::init(	const String& _name,
									uint _nRows,
									uint _nCols,
									ExportType _type,
									ExportStruct _dataStruct,
									BooleanType _callItByValue,
									const ExportIndex& _addressIdx
									)
{
	clear( );

	data.init( _nRows,_nCols );
	data.setAll( undefinedEntry );

	ExportData::init( _name,_type,_dataStruct );

	callItByValue = _callItByValue;
	addressIdx  = _addressIdx;

	return SUCCESSFUL_RETURN;
}


returnValue ExportArgument::init(	const String& _name,
									const Matrix& _data,
									ExportType _type,
									ExportStruct _dataStruct,
									BooleanType _callItByValue,
									const ExportIndex& _addressIdx
									)
{
	clear( );

	data = _data;

	ExportData::init( _name,_type,_dataStruct );

	callItByValue = _callItByValue;
	addressIdx  = _addressIdx;

	return SUCCESSFUL_RETURN;
}


ExportArgument ExportArgument::getAddress(	uint rowIdx,
											uint colIdx
											) const
{
	ASSERT( rowIdx < getNumRows() );
	ASSERT( colIdx < getNumCols() );

	ExportIndex tmpAddressIdx( "i" );
	tmpAddressIdx = getTotalIdx( rowIdx,colIdx );

	ExportArgument tmp(	name,getNumRows(),getNumCols(),
						type,dataStruct,BT_FALSE,tmpAddressIdx );

	return tmp;
}


ExportArgument ExportArgument::getAddress(	const ExportIndex& rowIdx,
											uint colIdx
											) const
{
	if ( rowIdx.isGiven( ) )
	{
		ASSERT( rowIdx.getGivenValue() < (int)getNumRows() );
	}
	
	ASSERT( colIdx < getNumCols() );
	
	ExportArgument tmp(	name,getNumRows(),getNumCols(),
						type,dataStruct,BT_FALSE,getTotalIdx(rowIdx,colIdx) );

	return tmp;
}



const char* ExportArgument::getAddressString(	BooleanType withDataStruct
												) const
{
	if ( withDataStruct == BT_TRUE )
	{
		if ( addressIdx.isGiven() == BT_TRUE )
		{
			if ( addressIdx.getGivenValue() == 0 )
				sprintf( exportDataString,"%s", getFullName().getName() );
			else
				sprintf( exportDataString,"&(%s[%d])", getFullName().getName(),addressIdx.getGivenValue() );
		}
		else
		{
			sprintf( exportDataString,"&(%s[%s])", getFullName().getName(),addressIdx.get() );
		}
	}
	else
	{
		if ( addressIdx.isGiven() == BT_TRUE )
		{
			if ( addressIdx.getGivenValue() == 0 )
				sprintf( exportDataString,"%s", getName().getName() );
			else
				sprintf( exportDataString,"&(%s[%d])", getName().getName(),addressIdx.getGivenValue() );
		}
		else
		{
			sprintf( exportDataString,"&(%s[%s])", getName().getName(),addressIdx.get() );
		}
	}

	return exportDataString;
}



uint ExportArgument::getNumRows( ) const
{
	return data.getNumRows( );
}


uint ExportArgument::getNumCols( ) const
{
	return data.getNumCols( );
}


uint ExportArgument::getDim( ) const
{
	return data.getDim( );
}



BooleanType ExportArgument::isGiven( ) const
{
	if ( getDim() == 0 )
		return BT_TRUE;

	for ( uint i=0; i<getNumRows(); ++i )
		for ( uint j=0; j<getNumCols(); ++j )
			if ( acadoIsEqual( data(i,j),undefinedEntry ) == BT_TRUE )
				return BT_FALSE;
			
	return BT_TRUE;
}



BooleanType ExportArgument::isCalledByValue( ) const
{
	return callItByValue;
}


returnValue ExportArgument::callByValue( )
{
	callItByValue = BT_TRUE;
	return SUCCESSFUL_RETURN;
}



returnValue ExportArgument::exportDataDeclaration(	FILE* file,
													const String& _realString,
													const String& _intString,
													int _precision
													) const
{
	// variable not in use, thus no declaration necessary
	if ( getDim( ) == 0 )
		return SUCCESSFUL_RETURN;

	if ( ( isCalledByValue() == BT_TRUE ) && ( getDim() == 1 ) )
			acadoFPrintf( file,"%s %s", getTypeString( _realString,_intString ).getName(),name.getName() );
		else
			acadoFPrintf( file,"%s %s[%d]", getTypeString( _realString,_intString ).getName(),name.getName(),getDim() );

	if ( isGiven() == BT_FALSE )
	{
		acadoFPrintf( file,";\n" );
	}
	else
	{
		if ( getDim( ) == 1 )
			acadoFPrintf( file," = %e;\n", data(0,0) );
		else
			return ACADOERROR( RET_NOT_YET_IMPLEMENTED );
	}

	return SUCCESSFUL_RETURN;
}



//
// PROTECTED MEMBER FUNCTIONS:
//

returnValue ExportArgument::clear( )
{
	return SUCCESSFUL_RETURN;
}



uint ExportArgument::getColDim( ) const
{
	return data.getNumCols( );
}



uint ExportArgument::getTotalIdx(	uint rowIdx,
									uint colIdx
									) const
{
	return rowIdx*getNumCols()+colIdx;
}


ExportIndex	ExportArgument::getTotalIdx(	const ExportIndex& rowIdx,
											const ExportIndex& colIdx
											) const
{
	return rowIdx*getNumCols()+colIdx;
}


CLOSE_NAMESPACE_ACADO

// end of file.
