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
#include <acado/code_generation/export_data_internal.hpp>

BEGIN_NAMESPACE_ACADO

std::string ExportDataInternal::fcnPrefix = "acado";

        
using namespace CasADi;


//
// PUBLIC MEMBER FUNCTIONS:
//

ExportDataInternal::ExportDataInternal(	const std::string& _name,
										ExportType _type,
										ExportStruct _dataStruct,
										const std::string& _prefix
										)
	: SharedObjectNode(), name( _name ), type( _type ), prefix( _prefix ), dataStruct( _dataStruct ),
	  description()
{
	setFullName();
}

ExportDataInternal::~ExportDataInternal( )
{
}

returnValue	ExportDataInternal::setName(	const std::string& _name
											)
{
	if ( _name.empty() == true )
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

returnValue ExportDataInternal::setPrefix(	const std::string& _prefix
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



std::string ExportDataInternal::getName( ) const
{
	return name;
}

ExportType ExportDataInternal::getType( ) const
{
	return type;
}

std::string ExportDataInternal::getPrefix() const
{
	return prefix;
}

std::string ExportDataInternal::getTypeString(	const std::string& _realString,
												const std::string& _intString
												) const
{
	switch ( type )
	{
	case INT:
		return _intString;

	case REAL:
		return _realString;

	case COMPLEX:
		return std::string("double complex");

	case STATIC_CONST_INT:
		return std::string("static const ") + _intString;

	case STATIC_CONST_REAL:
		return std::string("static const ") + _realString;
	}

	return std::string("unknownType");
}


ExportStruct ExportDataInternal::getDataStruct( ) const
{
	return dataStruct;
}


std::string ExportDataInternal::getDataStructString( ) const
{
	std::stringstream tmp;

	tmp << prefix;

	if (prefix.empty() == false)
		tmp << "_";

	switch ( dataStruct )
	{
		case ACADO_VARIABLES:
			tmp << fcnPrefix + "Variables";
			break;

		case ACADO_WORKSPACE:
			tmp << fcnPrefix + "Workspace";
			break;

		case ACADO_PARAMS:
			tmp << fcnPrefix + "Params";
			break;

		case ACADO_VARS:
			tmp << fcnPrefix + "Vars";
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

	return tmp.str();
}


std::string ExportDataInternal::getFullName( ) const
{
	if ( fullName.empty() == true )
		return name;

	return fullName;
}



//
// PROTECTED MEMBER FUNCTIONS:
//


//
// PRIVATE MEMBER FUNCTIONS:
//

returnValue ExportDataInternal::setFullName()
{
	if ( dataStruct == ACADO_LOCAL )
	{
		if ( prefix.empty() == false )
		{
			fullName = prefix;
			fullName += std::string("_") + name;
		}
		else
		{
			fullName = "";
		}
	}
	else
	{
//		if ( prefix.isEmpty() == false )
//		{
//			fullName = prefix;
//			fullName << "_" << getDataStructstd::string();
//		}
//		else
//		{
			fullName = getDataStructString();
//		}

		fullName += std::string(".") + name;
	}

	return SUCCESSFUL_RETURN;
}

returnValue ExportDataInternal::setDoc( const std::string& _doc )
{
	description = _doc;

	return SUCCESSFUL_RETURN;
}

std::string ExportDataInternal::getDoc() const
{
	return description;
}

CLOSE_NAMESPACE_ACADO
