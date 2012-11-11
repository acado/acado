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
 *    \file src/code_generation/export_statement_block.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 2010-2011
 */


#include <acado/code_generation/export_statement_block.hpp>
#include <acado/code_generation/export_function.hpp>
#include <acado/code_generation/export_ode_function.hpp>
#include <acado/code_generation/export_function_call.hpp>
#include <acado/code_generation/export_statement_string.hpp>
#include <acado/code_generation/export_function_declaration.hpp>



BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

ExportStatementBlock::ExportStatementBlock( ) : ExportStatement( )
{
}


ExportStatementBlock::ExportStatementBlock( const ExportStatementBlock& arg ) : ExportStatement( arg )
{
	statements = arg.statements;
}


ExportStatementBlock::~ExportStatementBlock( )
{
	clear( );
}


ExportStatementBlock& ExportStatementBlock::operator=( const ExportStatementBlock& arg )
{
	if ( this != &arg )
	{
		clear( );

		ExportStatement::operator=( arg );

		statements = arg.statements;
	}

	return *this;
}


ExportStatement* ExportStatementBlock::clone( ) const
{
	return new ExportStatementBlock(*this);
}



returnValue ExportStatementBlock::addStatement(	const ExportStatement& _statement
												)
{
	statements.push_back( statementPtr( _statement.clone() ) );
	
	return SUCCESSFUL_RETURN;
}


returnValue ExportStatementBlock::addStatement(	const String& _statementString
												)
{
	ExportStatementString tmp( _statementString );
	return addStatement( tmp );
}


returnValue ExportStatementBlock::addFunction(	const ExportFunction& _function
												)
{
	return addStatement( _function );
}


returnValue ExportStatementBlock::addFunctionCall(	const String& _fName,
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
	ExportFunctionCall tmp(	_fName,
							_argument1,_argument2,_argument3,
							_argument4,_argument5,_argument6,
							_argument7,_argument8,_argument9 );

	return addStatement( tmp );
}


returnValue ExportStatementBlock::addFunctionCall(	const ExportFunction& _f,
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
	ExportFunctionCall tmp(	_f,
							_argument1,_argument2,_argument3,
							_argument4,_argument5,_argument6,
							_argument7,_argument8,_argument9 );

	return addStatement( tmp );
}



returnValue ExportStatementBlock::addDeclaration(	const ExportVariable& _data,
													ExportStruct _dataStruct
													)
{
	// do not declare empty variables
	if ( _data.getDim() == 0 )
		return SUCCESSFUL_RETURN;

	if ( ( _dataStruct == ACADO_ANY ) ||
		 ( _dataStruct == _data.getDataStruct() ) )
	{
		ExportDataDeclaration tmp( _data );
		return addStatement( tmp );
	}
	
	return SUCCESSFUL_RETURN;
}


returnValue ExportStatementBlock::addDeclaration(	const ExportIndex& _data,
													ExportStruct _dataStruct
													)
{
	if ( ( _dataStruct == ACADO_ANY ) ||
		 ( _dataStruct == _data.getDataStruct() ) )
	{
		ExportDataDeclaration tmp( _data );
		return addStatement( tmp );
	}
	
	return SUCCESSFUL_RETURN;
}


returnValue ExportStatementBlock::addDeclaration(	const ExportFunction& _f
													)
{
	ExportFunctionDeclaration tmp( _f );
	return addStatement( tmp );
}


returnValue ExportStatementBlock::addDeclaration(	const ExportODEfunction& _f
													)
{
	ExportFunctionDeclaration tmp( _f );
	return addStatement( tmp );
}



returnValue ExportStatementBlock::addLinebreak(	uint num
												)
{
	if ( num < 1 )
		num = 1;
	
	if ( num > 10 )
		num = 10;

	String tmp = "\n";

	for( uint i=1; i<num; ++i )
		tmp << "\n";

	return addStatement( tmp );
}


returnValue ExportStatementBlock::addComment(	const String& _comment
												)
{
	String tmp = "/* ";
	tmp << _comment << " */\n";
	return addStatement( tmp );
}


returnValue ExportStatementBlock::addComment(	uint _nBlanks,
												const String& _comment
												)
{
	char* blanks = new char[_nBlanks+1];
	for( uint i=0; i<_nBlanks; ++i )
		blanks[i] = ' ';
	blanks[_nBlanks] = '\0';
	
	String tmp = blanks;
	tmp << "/* " << _comment << " */\n";
	
	delete[] blanks;

	return addStatement( tmp );
}



uint ExportStatementBlock::getNumStatements( ) const
{
	return statements.size();
}



returnValue ExportStatementBlock::exportDataDeclaration(	FILE *file,
															const String& _realString,
															const String& _intString,
															int _precision
															) const
{
	statementPtrArray::const_iterator it = statements.begin();
	for(; it != statements.end(); ++it)
		if ((*it)->exportDataDeclaration(file, _realString, _intString, _precision) != SUCCESSFUL_RETURN)
			return ACADOERROR( RET_UNABLE_TO_EXPORT_STATEMENT );

	return SUCCESSFUL_RETURN;
}



returnValue ExportStatementBlock::exportCode(	FILE* file,
												const String& _realString,
												const String& _intString,
												int _precision
												) const
{
	statementPtrArray::const_iterator it = statements.begin();
	for(; it != statements.end(); ++it)
		if ((*it)->exportCode(file, _realString, _intString, _precision) != SUCCESSFUL_RETURN)
			return ACADOERROR( RET_UNABLE_TO_EXPORT_STATEMENT );

	return SUCCESSFUL_RETURN;
}



returnValue ExportStatementBlock::clear( )
{
	return SUCCESSFUL_RETURN;
}



//
// PROTECTED MEMBER FUNCTIONS:
//



CLOSE_NAMESPACE_ACADO

// end of file.
