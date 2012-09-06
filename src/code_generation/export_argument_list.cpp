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
	nArguments = 0;
	arguments  = 0;
	
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
	nArguments = 0;
	arguments  = 0;
	
	addArgument( _argument1,_argument2,_argument3,
				 _argument4,_argument5,_argument6,
				 _argument7,_argument8,_argument9 );
}


ExportArgumentList::ExportArgumentList( const ExportArgumentList& arg )
{
	nArguments = arg.nArguments;

	if ( arg.arguments != 0 )
	{
		arguments = (ExportArgument**) calloc( nArguments,sizeof(ExportArgument*) );
		for( uint i=0; i<nArguments; ++i )
			arguments[i] = new ExportArgument( *(arg.arguments[i]) );
	}
	else
	{
		nArguments = 0;
		arguments = 0;
	}
	
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

		nArguments = arg.nArguments;
	
		if ( arg.arguments != 0 )
		{
			arguments = (ExportArgument**) calloc( nArguments,sizeof(ExportArgument*) );
			for( uint i=0; i<nArguments; ++i )
				arguments[i] = new ExportArgument( *(arg.arguments[i]) );
		}
		else
		{
			nArguments = 0;
			arguments = 0;
		}
		
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
	return nArguments;
}



returnValue ExportArgumentList::exportCode(	FILE* file,
											const String& _realString,
											const String& _intString,
											int _precision
											) const
{
	for( uint i=0; i<nArguments; ++i )
	{
		// only export arguments that are not explicitly given
		if ( arguments[i]->isGiven( ) == BT_FALSE )
		{
			if ( includeType == BT_TRUE )
			{
				if ( arguments[i]->isCalledByValue( ) == BT_TRUE )
					acadoFPrintf( file,"%s ", arguments[i]->getTypeString( _realString,_intString ).getName() );
				else
					acadoFPrintf( file,"%s* ", arguments[i]->getTypeString( _realString,_intString ).getName() );
			}

			if ( includeType == BT_FALSE )
				acadoFPrintf( file,"%s", arguments[i]->getAddressString( ) );
			else
				acadoFPrintf( file,"%s", arguments[i]->getAddressString( BT_FALSE ) );
			
			if ( i < nArguments-1 )
				acadoFPrintf( file,", " );
		}
	}

	return SUCCESSFUL_RETURN;
}



returnValue ExportArgumentList::clear( )
{
	if ( arguments != 0 )
	{
		for( uint i=0; i<nArguments; ++i )
		{
			delete arguments[i];
			arguments[i] = 0;
		}
		free( arguments );
		arguments = 0;
		
		nArguments = 0;
	}

	doIncludeType( );

	return SUCCESSFUL_RETURN;
}



returnValue ExportArgumentList::doIncludeType( )
{
	includeType = BT_TRUE;
	return SUCCESSFUL_RETURN;
}


returnValue ExportArgumentList::doNotIncludeType( )
{
	includeType = BT_FALSE;
	return SUCCESSFUL_RETURN;
}



//
// PROTECTED MEMBER FUNCTIONS:
//

returnValue ExportArgumentList::addSingleArgument(	const ExportArgument& _argument
													)
{
	// nothing to do?
	if ( _argument.getDim( ) == 0 )
		return SUCCESSFUL_RETURN;

	++nArguments;

	arguments = (ExportArgument**) realloc( arguments,nArguments*sizeof(ExportArgument*) );
	arguments[ nArguments-1 ] = new ExportArgument( _argument );

	return SUCCESSFUL_RETURN;
}


CLOSE_NAMESPACE_ACADO

// end of file.
