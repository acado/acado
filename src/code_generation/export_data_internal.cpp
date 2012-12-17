
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
	: SharedObjectNode(), name( _name ), type( _type ), prefix( _prefix ), dataStruct( _dataStruct )
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
		case STATIC_CONSTANT:
				return String("const static ") << _intString;

		case INT:
			return _intString;

		case REAL:
			return _realString;
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

CLOSE_NAMESPACE_ACADO
