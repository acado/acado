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
 *    \file src/code_generation/export_function_call.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 2010-2011
 */

#include <acado/code_generation/export_function_call.hpp>


BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

ExportFunctionCall::ExportFunctionCall(	const std::string& _name,
										const ExportArgument& _argument1,
										const ExportArgument& _argument2,
										const ExportArgument& _argument3,
										const ExportArgument& _argument4,
										const ExportArgument& _argument5,
										const ExportArgument& _argument6,
										const ExportArgument& _argument7,
										const ExportArgument& _argument8,
										const ExportArgument& _argument9
										) : ExportStatement( )
{
	init(	_name,
			_argument1,_argument2,_argument3,
			_argument4,_argument5,_argument6,
			_argument7,_argument8,_argument9 );
}


ExportFunctionCall::ExportFunctionCall(	const ExportFunction& _f,
										const ExportArgument& _argument1,
										const ExportArgument& _argument2,
										const ExportArgument& _argument3,
										const ExportArgument& _argument4,
										const ExportArgument& _argument5,
										const ExportArgument& _argument6,
										const ExportArgument& _argument7,
										const ExportArgument& _argument8,
										const ExportArgument& _argument9
										) : ExportStatement( )
{
	init(	_f,
			_argument1,_argument2,_argument3,
			_argument4,_argument5,_argument6,
			_argument7,_argument8,_argument9 );
}


ExportFunctionCall::ExportFunctionCall( const ExportFunctionCall& arg ) : ExportStatement( arg )
{
	setName( arg.name );

	functionArguments = arg.functionArguments;
}


ExportFunctionCall::~ExportFunctionCall( )
{
	clear( );
}


ExportFunctionCall& ExportFunctionCall::operator=( const ExportFunctionCall& arg )
{
	if( this != &arg )
	{
		ExportStatement::operator=( arg );
		
		setName( arg.name );

		functionArguments = arg.functionArguments;
	}

	return *this;
}

ExportStatement* ExportFunctionCall::clone( ) const
{
	return new ExportFunctionCall(*this);
}



returnValue ExportFunctionCall::init(	const std::string& _name,
										const ExportArgument& _argument1,
										const ExportArgument& _argument2,
										const ExportArgument& _argument3,
										const ExportArgument& _argument4,
										const ExportArgument& _argument5,
										const ExportArgument& _argument6,
										const ExportArgument& _argument7,
										const ExportArgument& _argument8,
										const ExportArgument& _argument9
										)
{
	clear( );

	// only append fcnPrefix iff _name does not alreads start with it
    if ( ( _name.length() > fcnPrefix.length() ) && 
         ( _name.compare( 0,fcnPrefix.length(),fcnPrefix ) == 0 ) )
    {
        setName( _name );
    }
	else
	{
        setName( fcnPrefix + "_" + _name );
    }

	functionArguments.addArgument( 	_argument1,_argument2,_argument3,
									_argument4,_argument5,_argument6,
									_argument7,_argument8,_argument9 );

	return SUCCESSFUL_RETURN;
}


returnValue ExportFunctionCall::init(	const ExportFunction& _f,
										const ExportArgument& _argument1,
										const ExportArgument& _argument2,
										const ExportArgument& _argument3,
										const ExportArgument& _argument4,
										const ExportArgument& _argument5,
										const ExportArgument& _argument6,
										const ExportArgument& _argument7,
										const ExportArgument& _argument8,
										const ExportArgument& _argument9
										)
{
	clear( );

	setName( _f.getName() );

	if (_f.isDefined() == false)
	{
		LOG( LVL_DEBUG ) << "ExportFunctionCall: " << _f.getName() << " is empty" << std::endl;
		return SUCCESSFUL_RETURN;
	}

	functionArguments.addArgument( 	_argument1,_argument2,_argument3,
									_argument4,_argument5,_argument6,
									_argument7,_argument8,_argument9 );

	if ( _f.getNumArguments() != functionArguments.getNumArguments() )
	{
		LOG( LVL_ERROR ) << "Function " << _f.getName()
				<< " expects " << _f.getNumArguments() << " argument(s), but you provided "
				<< functionArguments.getNumArguments() << std::endl;
		return ACADOERROR( RET_INVALID_CALL_TO_EXPORTED_FUNCTION );
	}

	return SUCCESSFUL_RETURN;
}



returnValue ExportFunctionCall::exportCode(	std::ostream& stream,
											const std::string& _realString,
											const std::string& _intString,
											int _precision
											) const
{
	if ( name.empty() == true )
		return ACADOERROR( RET_MEMBER_NOT_INITIALISED );

	stream << name << "( ";
	functionArguments.exportCode(stream, _realString, _intString, _precision);
	stream << " );\n";

	return SUCCESSFUL_RETURN;
}



//
// PROTECTED MEMBER FUNCTIONS:
//

returnValue ExportFunctionCall::clear( )
{
	functionArguments.clear( );
	functionArguments.doNotIncludeType( );

	return SUCCESSFUL_RETURN;
}



returnValue	ExportFunctionCall::setName(	const std::string& _name
											)
{
	if ( _name.empty() == true )
		return ACADOERROR( RET_INVALID_ARGUMENTS );

	name = _name;

	return SUCCESSFUL_RETURN;
}



CLOSE_NAMESPACE_ACADO

// end of file.
