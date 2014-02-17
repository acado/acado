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
 *    \file src/code_generation/export_argument_list.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 */


#include <acado/code_generation/export_argument_list.hpp>



BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

ExportArgumentList::ExportArgumentList( )
{
	doIncludeType( );
}


ExportArgumentList::ExportArgumentList(	const ExportArgument& _argument1,
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
	addArgument( _argument1,_argument2,_argument3,
				 _argument4,_argument5,_argument6,
				 _argument7,_argument8,_argument9 );
}


ExportArgumentList::ExportArgumentList( const ExportArgumentList& arg )
{
	arguments = arg.arguments;
	
	includeType = arg.includeType;
}


ExportArgumentList::~ExportArgumentList( )
{
	clear( );
}


ExportArgumentList& ExportArgumentList::operator=( const ExportArgumentList& arg )
{
	if ( this != &arg )
	{
		clear( );

		arguments = arg.arguments;
		
		includeType = arg.includeType;
	}

	return *this;
}



returnValue ExportArgumentList::addArgument(	const ExportArgument& _argument1,
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
	addSingleArgument( _argument1 );
	addSingleArgument( _argument2 );
	addSingleArgument( _argument3 );
	addSingleArgument( _argument4 );
	addSingleArgument( _argument5 );
	addSingleArgument( _argument6 );
	addSingleArgument( _argument7 );
	addSingleArgument( _argument8 );
	addSingleArgument( _argument9 );

	return SUCCESSFUL_RETURN;
}


uint ExportArgumentList::getNumArguments( ) const
{
	return arguments.size();
}



returnValue ExportArgumentList::exportCode(	std::ostream& stream,
											const std::string& _realString,
											const std::string& _intString,
											int _precision
											) const
{
	bool started = false;
	for (unsigned i = 0; i < arguments.size(); ++i)
	{
		// Allow only undefined arguments and defined integer scalars
		if (	arguments[ i ].isGiven( ) == true &&
				(arguments[ i ].getDim() > 1 || arguments[ i ].getType() != INT) &&
				arguments[ i ].getType() != STATIC_CONST_INT &&
				arguments[ i ].getType() != STATIC_CONST_REAL
				)
			continue;

		if (i > 0 && started == true)
			stream << ", ";

		if ( includeType == true )
		{
			if ( arguments[ i ].isCalledByValue( ) == true )
				stream << arguments[ i ].getTypeString(_realString, _intString) << " ";
			else
				stream << arguments[ i ].getTypeString(_realString, _intString) << "* const ";
		}

		if ( includeType == false )
			stream << arguments[ i ].getAddressString( );
		else
			stream << arguments[ i ].getAddressString( false );

		started = true;
	}

	return SUCCESSFUL_RETURN;
}



returnValue ExportArgumentList::clear( )
{
	doIncludeType( );

	return SUCCESSFUL_RETURN;
}



returnValue ExportArgumentList::doIncludeType( )
{
	includeType = true;
	return SUCCESSFUL_RETURN;
}


returnValue ExportArgumentList::doNotIncludeType( )
{
	includeType = false;
	return SUCCESSFUL_RETURN;
}



//
// PROTECTED MEMBER FUNCTIONS:
//

returnValue ExportArgumentList::addSingleArgument(	const ExportArgument& _argument
													)
{
	if ( _argument.isNull() )
		return SUCCESSFUL_RETURN;
	if (_argument.getDim() == 0)
		return SUCCESSFUL_RETURN;

	arguments.push_back( _argument );

	return SUCCESSFUL_RETURN;
}

const std::vector< ExportArgument >& ExportArgumentList::get() const
{
	return arguments;
}


CLOSE_NAMESPACE_ACADO

// end of file.
