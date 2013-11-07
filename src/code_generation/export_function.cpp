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
 *    \file src/code_generation/export_function.cpp
 *    \authors Hans Joachim Ferreau, Boris Houska, Milan Vukov
 *    \date 2010 - 2013
 */

#include <acado/code_generation/export_function.hpp>
#include <acado/code_generation/export_function_call.hpp>

using namespace std;

BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

ExportFunction::ExportFunction(	const std::string& _name,
								const ExportArgument& _argument1,
								const ExportArgument& _argument2,
								const ExportArgument& _argument3,
								const ExportArgument& _argument4,
								const ExportArgument& _argument5,
								const ExportArgument& _argument6,
								const ExportArgument& _argument7,
								const ExportArgument& _argument8,
								const ExportArgument& _argument9
								) : ExportStatementBlock( ), description()
{
	returnAsPointer = BT_FALSE;
	flagPrivate = BT_FALSE;

	memAllocator = MemoryAllocatorPtr( new MemoryAllocator );

	init( _name,_argument1,_argument2,_argument3,
				_argument4,_argument5,_argument6,
				_argument7,_argument8,_argument9 );
}


ExportFunction::ExportFunction( const ExportFunction& arg ) : ExportStatementBlock( arg )
{
	name = arg.name;
	functionArguments = arg.functionArguments;
	flagPrivate = arg.flagPrivate;
	returnAsPointer = arg.returnAsPointer;
	
	retVal = arg.retVal;

	memAllocator = arg.memAllocator;
	localVariables = arg.localVariables;

	description = arg.description;
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
		retVal = arg.retVal;
		returnAsPointer = arg.returnAsPointer;

		memAllocator = arg.memAllocator;
		localVariables = arg.localVariables;

		description = arg.description;
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



returnValue ExportFunction::init(	const std::string& _name,
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


ExportFunction& ExportFunction::setup(	const std::string& _name,
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



ExportFunction& ExportFunction::setReturnValue(	const ExportVariable& _functionReturnValue,
												BooleanType _returnAsPointer
												)
{
	retVal = std::tr1::shared_ptr< ExportVariable >(new ExportVariable( _functionReturnValue ));
	returnAsPointer = _returnAsPointer;

	return *this;
}


ExportFunction&	ExportFunction::setName(	const std::string& _name
											)
{
	if ( _name.empty() == BT_TRUE )
		ACADOERROR( RET_INVALID_ARGUMENTS );

	name = _name;
	
	return *this;
}


std::string ExportFunction::getName( ) const
{
	return name;
}



returnValue ExportFunction::exportDataDeclaration(	std::ostream& stream,
													const std::string& _realString,
													const std::string& _intString,
													int _precision
													) const
{
	return SUCCESSFUL_RETURN;
}


returnValue ExportFunction::exportForwardDeclaration(	std::ostream& stream,
														const std::string& _realString,
														const std::string& _intString,
														int _precision
														) const
{
	// do not export undefined (empty) functions
	if (isDefined() == BT_FALSE)
		return SUCCESSFUL_RETURN;

	if (flagPrivate == BT_TRUE)
		return SUCCESSFUL_RETURN;

	if (description.empty() == BT_FALSE)
	{
		stream <<  "\n/** " << description;

		if (functionArguments.getNumArguments() > 0)
		{
			vector< ExportArgument > args = functionArguments.get();

			stream << "\n *\n";

			for (unsigned i = 0; i < args.size(); ++i)
			{
				if (args[ i ].isGiven() == BT_TRUE || args[ i ].getDoc().empty() == BT_TRUE)
					continue;

				stream << " *  \\param " << args[ i ].getName() << " " << args[ i ].getDoc() << endl;
			}
		}
		else
		{
			stream << "\n";
		}

		if (retVal != 0)
		{
			std::string tmp = retVal->getDoc();
			if (tmp.empty() == BT_FALSE)
				stream << " *\n *  \\return " << tmp << endl;
		}
		stream << " */\n";
	}


	if (retVal != 0)
	{
		stream << retVal->getTypeString(_realString, _intString);
		if ( returnAsPointer == BT_TRUE )
			stream << "*";
	}
	else
	{
		stream << "void";
	}

	stream << " " << name << "( ";
	functionArguments.exportCode(stream, _realString, _intString, _precision);
	stream << " );\n";

	return SUCCESSFUL_RETURN;
}


returnValue ExportFunction::exportCode(	std::ostream& stream,
										const std::string& _realString,
										const std::string& _intString,
										int _precision
										) const
{
	//
	// Do not export undefined (empty) functions
	//
	if ( isDefined() == BT_FALSE )
		return SUCCESSFUL_RETURN;

	//
	// Set return value type
	//
	if ( retVal != 0 )
	{
		stream << retVal->getTypeString(_realString, _intString);
		if ( returnAsPointer == BT_TRUE )
			stream << "*";
	}
	else
	{
		stream << "void";
	}
	
	stream << " " << name << "( ";
	functionArguments.exportCode(stream, _realString, _intString, _precision);
	stream << " )\n{\n";

	if (retVal && retVal->getDataStruct() == ACADO_LOCAL)
	{
		stream << retVal->getTypeString(_realString, _intString) << " ";
		stream << retVal->getName() << endl;
	}

	// ExportStatementBlock::exportDataDeclaration( file,_realString,_intString,_precision );

	//
	// Set parent pointers, and run memory allocation
	//
	StatementPtrArray::const_iterator it = statements.begin();
	for(; it != statements.end(); ++it)
		(*it)->allocate( memAllocator );

	//
	// Open a temporary file and export statements to the temporary file
	//
	stringstream ss;
	ExportStatementBlock::exportCode(ss, _realString, _intString, _precision);

	//
	// Export local indices (allocated previously)
	//
	const std::vector< ExportIndex > indices = memAllocator->getPool();
	for (unsigned i = 0; i < indices.size(); ++i)
		indices[ i ].exportDataDeclaration(stream, _realString, _intString, _precision);

	//
	// Export local variables -- still done in a very primitive way
	//
	for(unsigned i = 0; i < localVariables.size(); ++i)
		localVariables[ i ].exportDataDeclaration(stream, _realString, _intString, _precision);

	//
	// Copy temporary file to main file, and close te temporary file aftwarards
	//
	stream << ss;

	//
	// Finish the export of the function
	//
	if ( retVal != 0 )
		stream << "return " <<  retVal->getFullName() << ";\n";
	stream << "}\n\n";

	return SUCCESSFUL_RETURN;
}


BooleanType ExportFunction::isDefined( ) const
{
	if ( ( name.empty() == BT_FALSE ) &&
		 ( ( getNumStatements( ) > 0 ) || ( retVal != 0 ) ) )
		return BT_TRUE;
	else
		return BT_FALSE;
}


unsigned ExportFunction::getNumArguments( ) const
{
	return functionArguments.getNumArguments( );
}


//
// PROTECTED MEMBER FUNCTIONS:
//

returnValue ExportFunction::clear( )
{
	returnAsPointer = BT_FALSE;

	return SUCCESSFUL_RETURN;
}

ExportFunction& ExportFunction::addIndex(const ExportIndex& _index)
{
	memAllocator->add( _index );

	return *this;
}

ExportFunction& ExportFunction::acquire(ExportIndex& obj)
{
	memAllocator->acquire( obj );

	return *this;
}

ExportFunction& ExportFunction::release(const ExportIndex& obj)
{
	memAllocator->release( obj );

	return *this;
}

ExportFunction& ExportFunction::addVariable(const ExportVariable& _var)
{
	localVariables.push_back( _var );

	return *this;
}

ExportFunction& ExportFunction::doc(const std::string& _doc)
{
	description = _doc;

	return *this;
}

ExportFunction& ExportFunction::setPrivate(BooleanType _set)
{
	flagPrivate = _set;

	return *this;
}

BooleanType ExportFunction::isPrivate() const
{
	return flagPrivate;
}

CLOSE_NAMESPACE_ACADO

// end of file.
