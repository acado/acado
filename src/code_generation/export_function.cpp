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
 *    \file src/code_generation/export_function.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 2010-2011
 */

#include <acado/code_generation/export_function.hpp>
#include <acado/code_generation/export_function_call.hpp>


BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

ExportFunction::ExportFunction(	const String& _name,
								const ExportArgument& _argument1,
								const ExportArgument& _argument2,
								const ExportArgument& _argument3,
								const ExportArgument& _argument4,
								const ExportArgument& _argument5,
								const ExportArgument& _argument6,
								const ExportArgument& _argument7,
								const ExportArgument& _argument8,
								const ExportArgument& _argument9
								) : ExportStatementBlock( )
{
	functionReturnValue = 0;
	returnAsPointer = BT_FALSE;

	init( _name,_argument1,_argument2,_argument3,
				_argument4,_argument5,_argument6,
				_argument7,_argument8,_argument9 );
}


ExportFunction::ExportFunction( const ExportFunction& arg ) : ExportStatementBlock( arg )
{
	functionReturnValue = 0;

	name = arg.name;
	functionArguments = arg.functionArguments;
	
	if ( arg.functionReturnValue != 0 )
		setReturnValue( *(arg.functionReturnValue),arg.returnAsPointer );

	localVariables.resize(arg.localVariables.size(), 0);
	for (unsigned i = 0; i < arg.localVariables.size(); ++i)
	{
		if ( arg.localVariables[ i ] )
		{
			localVariables[ i ] = arg.localVariables[ i ]->clone();
		}
	}
}


ExportFunction::~ExportFunction( )
{
	clear( );
}


ExportFunction& ExportFunction::operator=( const ExportFunction& arg )
{
	if( this != &arg )
	{
		init( arg.name );

		ExportStatementBlock::operator=( arg );
		functionArguments = arg.functionArguments;
		
		if ( arg.functionReturnValue != 0 )
			setReturnValue( *(arg.functionReturnValue),arg.returnAsPointer );

		localVariables.resize(arg.localVariables.size(), 0);
		for (unsigned i = 0; i < arg.localVariables.size(); ++i)
		{
			if ( arg.localVariables[ i ] )
			{
				localVariables[ i ] = arg.localVariables[ i ]->clone();
			}
		}
	}

	return *this;
}


ExportStatement* ExportFunction::clone( ) const
{
	return new ExportFunction(*this);
}


ExportFunction* ExportFunction::cloneFunction( ) const
{
	return new ExportFunction(*this);
}



returnValue ExportFunction::init(	const String& _name,
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
	ExportStatementBlock::clear( );
	clear( );

	setName( _name );

	addArgument( 	_argument1,_argument2,_argument3,
					_argument4,_argument5,_argument6,
					_argument7,_argument8,_argument9 );

	return SUCCESSFUL_RETURN;
}


ExportFunction& ExportFunction::setup(	const String& _name,
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
	init( 	_name,
			_argument1,_argument2,_argument3,
			_argument4,_argument5,_argument6,
			_argument7,_argument8,_argument9 );

	return *this;
}



returnValue ExportFunction::addArgument(	const ExportArgument& _argument1,
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
	return functionArguments.addArgument( 	_argument1,_argument2,_argument3,
											_argument4,_argument5,_argument6,
											_argument7,_argument8,_argument9 );
}



returnValue ExportFunction::setReturnValue(	const ExportVariable& _functionReturnValue,
											BooleanType _returnAsPointer
											)
{
	if ( functionReturnValue != 0 )
		*functionReturnValue = _functionReturnValue;
	else
		functionReturnValue = new ExportVariable( _functionReturnValue );
	
	returnAsPointer = _returnAsPointer;

	return SUCCESSFUL_RETURN;
}


returnValue	ExportFunction::setName(	const String& _name
										)
{
	if ( _name.isEmpty() == BT_TRUE )
		return ACADOERROR( RET_INVALID_ARGUMENTS );

	name = _name;
	
	return SUCCESSFUL_RETURN;
}


String ExportFunction::getName( ) const
{
	return name;
}



returnValue ExportFunction::exportDataDeclaration(	FILE* file,
													const String& _realString,
													const String& _intString,
													int _precision
													) const
{
	return SUCCESSFUL_RETURN;
}


returnValue ExportFunction::exportForwardDeclaration(	FILE *file,
														const String& _realString,
														const String& _intString,
														int _precision
														) const
{
	// do not export undefined (empty) functions
	if ( isDefined() == BT_FALSE )
		return SUCCESSFUL_RETURN;

	if ( functionReturnValue != 0 )
	{
		acadoFPrintf( file,"%s", functionReturnValue->getTypeString( _realString,_intString ).getName() );
		if ( returnAsPointer == BT_TRUE )
			acadoFPrintf( file,"*" );
	}
	else
	{
		acadoFPrintf( file,"void" );
	}

	acadoFPrintf( file," %s( ", name.getName() );
	functionArguments.exportCode( file,_realString,_intString,_precision );
	acadoFPrintf( file," );\n" );

	return SUCCESSFUL_RETURN;
}


returnValue ExportFunction::exportCode(	FILE *file,
										const String& _realString,
										const String& _intString,
										int _precision
										) const
{
	// do not export undefined (empty) functions
	if ( isDefined() == BT_FALSE )
		return SUCCESSFUL_RETURN;

	if ( functionReturnValue != 0 )
	{
		acadoFPrintf( file,"%s", functionReturnValue->getTypeString( _realString,_intString ).getName() );
		if ( returnAsPointer == BT_TRUE )
			acadoFPrintf( file,"*" );
	}
	else
	{
		acadoFPrintf( file,"void" );
	}
	
	acadoFPrintf( file," %s( ", name.getName() );
	functionArguments.exportCode( file,_realString,_intString,_precision );
	acadoFPrintf( file," )\n{\n");

	if ( functionReturnValue )
	{
		acadoFPrintf(file, "%s ", functionReturnValue->getTypeString( _realString,_intString ).getName());
		acadoFPrintf(file, "%s;\n", functionReturnValue->getName().getName());
	}

	for (unsigned i = 0; i < localVariables.size(); ++i)
		localVariables[ i ]->exportDataDeclaration(file, _realString, _intString, _precision);

	ExportStatementBlock::exportDataDeclaration( file,_realString,_intString,_precision );

	ExportStatementBlock::exportCode( file,_realString,_intString,_precision );
	
	if ( functionReturnValue != 0 )
		acadoFPrintf( file,"return %s;\n", functionReturnValue->getFullName().getName() );
	acadoFPrintf( file,"}\n\n");

	return SUCCESSFUL_RETURN;
}



ExportVariable ExportFunction::getGlobalExportVariable( ) const
{
	ASSERT( 1==0 );
	return ExportVariable();
}



BooleanType ExportFunction::isDefined( ) const
{
	if ( ( name.isEmpty() == BT_FALSE ) && 
		 ( ( getNumStatements( ) > 0 ) || ( functionReturnValue != 0 ) ) )
		return BT_TRUE;
	else
		return BT_FALSE;
}



uint ExportFunction::getNumArguments( ) const
{
	return functionArguments.getNumArguments( );
}



//
// PROTECTED MEMBER FUNCTIONS:
//

returnValue ExportFunction::clear( )
{
	if ( functionReturnValue != 0 )
	{
		delete functionReturnValue;
		functionReturnValue = 0;
	}
	
	for (std::vector< ExportData* >::iterator it = localVariables.begin(); it != localVariables.end(); ++it)
		delete *it;
	localVariables.clear();

	returnAsPointer = BT_FALSE;

	return SUCCESSFUL_RETURN;
}

returnValue ExportFunction::addIndex(const ExportIndex& _index)
{
	localVariables.push_back( new ExportIndex( _index ) );

	return SUCCESSFUL_RETURN;
}

returnValue ExportFunction::addVariables(const ExportVariable& _variable)
{
	localVariables.push_back( new ExportVariable( _variable ) );

	return SUCCESSFUL_RETURN;
}


CLOSE_NAMESPACE_ACADO

// end of file.
