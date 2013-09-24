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
#include <acado/code_generation/export_data_internal.hpp>

BEGIN_NAMESPACE_ACADO

using namespace CasADi;


//
// PUBLIC MEMBER FUNCTIONS:
//

ExportDataInternal::ExportDataInternal(	const String& _name,
										ExportType _type,
										ExportStruct _dataStruct,
										const String& _prefix
										)
	: SharedObjectNode(), name( _name ), type( _type ), prefix( _prefix ), dataStruct( _dataStruct ),
	  description()
{
	setFullName();
}

ExportDataInternal::~ExportDataInternal( )
{
}

returnValue	ExportDataInternal::setName(	const String& _name
											)
{
	if ( _name.isEmpty() == BT_TRUE )
		return ACADOERROR( RET_INVALID_ARGUMENTS );

	name = _name;

	setFullName();

	return SUCCESSFUL_RETURN;
}


returnValue	ExportDataInternal::setType(	ExportType _type
											)
{
	type = _type;

	setFullName();

	return SUCCESSFUL_RETURN;
}

returnValue ExportDataInternal::setPrefix(	const String& _prefix
											)
{
	prefix = _prefix;

	setFullName();

	return SUCCESSFUL_RETURN;
}

returnValue	ExportDataInternal::setDataStruct(	ExportStruct _dataStruct
												)
{
	dataStruct = _dataStruct;

	setFullName();

	return SUCCESSFUL_RETURN;
}



String ExportDataInternal::getName( ) const
{
	return name;
}

ExportType ExportDataInternal::getType( ) const
{
	return type;
}

String ExportDataInternal::getPrefix() const
{
	return prefix;
}

String ExportDataInternal::getTypeString(	const String& _realString,
									const String& _intString
									) const
{
	switch ( type )
	{
	case INT:
		return _intString;

	case REAL:
		return _realString;

	case STATIC_CONST_INT:
		return String("static const ") << _intString;

	case STATIC_CONST_REAL:
		return String("static const ") << _realString;
	}

	return (String)"unknownType";
}


ExportStruct ExportDataInternal::getDataStruct( ) const
{
	return dataStruct;
}


String ExportDataInternal::getDataStructString( ) const
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


String ExportDataInternal::getFullName( ) const
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

//ExportDataInternal::ExportDataInternal()
//{}

returnValue ExportDataInternal::setFullName()
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

returnValue ExportDataInternal::setDoc( const String& _doc )
{
	description = _doc;

	return SUCCESSFUL_RETURN;
}

String ExportDataInternal::getDoc() const
{
	return description;
}

CLOSE_NAMESPACE_ACADO
