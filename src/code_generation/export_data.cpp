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
 *    \file src/code_generation/export_data.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 2010-2011
 */


#include <acado/code_generation/export_data.hpp>

BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

ExportData::ExportData( )
{
	name = "default_name";
	type = REAL;
	prefix = "";
	dataStruct = ACADO_LOCAL;
	fullName = name;
}


ExportData::ExportData(	const String& _name,
						ExportType _type,
						ExportStruct _dataStruct,
						const String& _prefix
						)
{
	init(_name, _type, _dataStruct, _prefix);
}


ExportData::ExportData(	const ExportData& arg
						)
{
	init(arg.name, arg.type, arg.dataStruct, arg.prefix);
}


ExportData::~ExportData( )
{
}


ExportData& ExportData::operator=(	const ExportData& arg
									)
{
	if( this != &arg )
	{
		init(arg.name, arg.type, arg.dataStruct, arg.prefix);
	}

	return *this;
}


returnValue ExportData::init(	const String& _name,
								ExportType _type,
								ExportStruct _dataStruct,
								const String& _prefix
								)
{
	if ( _name.isEmpty() == BT_FALSE )
	{
		dataStruct = ACADO_LOCAL;
		setName( _name );
	}

	setType( _type );
	setPrefix( _prefix );
	setDataStruct( _dataStruct );

	return SUCCESSFUL_RETURN;
}



returnValue	ExportData::setName(	const String& _name
									)
{
	if ( _name.isEmpty() == BT_TRUE )
		return ACADOERROR( RET_INVALID_ARGUMENTS );

	name = _name;

	setFullName();

	return SUCCESSFUL_RETURN;
}


returnValue	ExportData::setType(	ExportType _type
									)
{
	type = _type;

	setFullName();

	return SUCCESSFUL_RETURN;
}

returnValue ExportData::setPrefix(	const String& _prefix
									)
{
	prefix = _prefix;

	setFullName();

	return SUCCESSFUL_RETURN;
}

returnValue	ExportData::setDataStruct(	ExportStruct _dataStruct
										)
{
	dataStruct = _dataStruct;

	setFullName();

	return SUCCESSFUL_RETURN;
}



String ExportData::getName( ) const
{
	return name;
}

ExportType ExportData::getType( ) const
{
	return type;
}

String ExportData::getPrefix() const
{
	return prefix;
}

String ExportData::getTypeString(	const String& _realString,
									const String& _intString
									) const
{
	switch ( type )
	{
		case INT:
			return _intString;
			
		case REAL:
			return _realString;
	}
	
	return (String)"<unknown_type>";
}


ExportStruct ExportData::getDataStruct( ) const
{
	return dataStruct;
}


String ExportData::getDataStructString( ) const
{
	String tmp = prefix;

	if (tmp.isEmpty() == BT_FALSE)
		tmp << "_";

	switch ( dataStruct )
	{
		case ACADO_VARIABLES:
			tmp << "acadoVariables";
			break;
			
		case ACADO_WORKSPACE:
			tmp << "acadoWorkspace";
			break;

		case ACADO_PARAMS:
			tmp << "params";
			break;
			
		case ACADO_VARS:
			tmp << "vars";
			break;

		case FORCES_PARAMS:
			tmp << "params";
			break;

		case FORCES_OUTPUT:
			tmp << "output";
			break;

		case FORCES_INFO:
			tmp << "info";
			break;

		default:
			tmp << "";
			break;
	}

	return tmp;
}


String ExportData::getFullName( ) const
{
	if ( fullName.isEmpty() == BT_TRUE )
		return name;
	else
		return fullName;
}



//
// PROTECTED MEMBER FUNCTIONS:
//


//
// PRIVATE MEMBER FUNCTIONS:
//
returnValue ExportData::setFullName()
{
	if ( dataStruct == ACADO_LOCAL )
	{
		if ( prefix.isEmpty() == BT_FALSE )
		{
			fullName = prefix;
			fullName << "_" << name;
		}
		else
		{
			fullName = "";
		}
	}
	else
	{
//		if ( prefix.isEmpty() == BT_FALSE )
//		{
//			fullName = prefix;
//			fullName << "_" << getDataStructString();
//		}
//		else
//		{
			fullName = getDataStructString();
//		}

		fullName << "." << name;
	}

	return SUCCESSFUL_RETURN;
}

CLOSE_NAMESPACE_ACADO

// end of file.
