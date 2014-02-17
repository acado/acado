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
 *    \file src/code_generation/export_statement_block.cpp
 *    \authors Hans Joachim Ferreau, Boris Houska, Milan Vukov
 *    \date 2010-2013
 */


#include <acado/code_generation/export_statement_block.hpp>
#include <acado/code_generation/export_function.hpp>
#include <acado/code_generation/export_acado_function.hpp>
#include <acado/code_generation/export_function_call.hpp>
#include <acado/code_generation/export_statement_string.hpp>
#include <acado/code_generation/export_function_declaration.hpp>

#include <acado/code_generation/export_variable.hpp>
#include <acado/code_generation/export_index.hpp>
#include <acado/code_generation/export_data_declaration.hpp>

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
	statements.push_back( StatementPtr( _statement.clone() ) );
	
	return SUCCESSFUL_RETURN;
}


returnValue ExportStatementBlock::addStatement(	const std::string& _statementString
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


returnValue ExportStatementBlock::addFunctionCall(	const std::string& _fName,
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


returnValue ExportStatementBlock::addDeclaration(	const ExportAcadoFunction& _f
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

	std::stringstream ss;
	ss << std::endl;

	for (uint i = 1; i < num; ++i)
		ss << std::endl;

	return addStatement( ss.str() );
}


returnValue ExportStatementBlock::addComment(	const std::string& _comment
												)
{
	std::stringstream ss; ss << "/* " << _comment << " */\n";
	return addStatement( ss.str() );
}


returnValue ExportStatementBlock::addComment(	uint _nBlanks,
												const std::string& _comment
												)
{
	std::stringstream ss;
	for(unsigned i = 0; i < _nBlanks; ++i)
		ss << " ";
	ss << "/* " << _comment << " */\n";

	return addStatement( ss.str() );
}



uint ExportStatementBlock::getNumStatements( ) const
{
	return statements.size();
}



returnValue ExportStatementBlock::exportDataDeclaration(	std::ostream& stream,
															const std::string& _realString,
															const std::string& _intString,
															int _precision
															) const
{
	StatementPtrArray::const_iterator it = statements.begin();
	for(; it != statements.end(); ++it)
		if ((*it)->exportDataDeclaration(stream, _realString, _intString, _precision) != SUCCESSFUL_RETURN)
			return ACADOERROR( RET_UNABLE_TO_EXPORT_STATEMENT );

	return SUCCESSFUL_RETURN;
}



returnValue ExportStatementBlock::exportCode(	std::ostream& stream,
												const std::string& _realString,
												const std::string& _intString,
												int _precision
												) const
{
	StatementPtrArray::const_iterator it = statements.begin();
	for(; it != statements.end(); ++it)
		if ((*it)->exportCode(stream, _realString, _intString, _precision) != SUCCESSFUL_RETURN)
			return ACADOERROR( RET_UNABLE_TO_EXPORT_STATEMENT );

	return SUCCESSFUL_RETURN;
}



returnValue ExportStatementBlock::clear( )
{
	return SUCCESSFUL_RETURN;
}

ExportStatementBlock& operator<<(ExportStatementBlock& _block, const ExportStatement& _statement)
{
	returnValue status = _block.addStatement( _statement );
	if (status != SUCCESSFUL_RETURN)
		ACADOERROR( status );

	return _block;
}

ExportStatementBlock& operator<<(ExportStatementBlock& _block, const std::string& _statement)
{
	returnValue status = _block.addStatement( _statement );
	if (status != SUCCESSFUL_RETURN)
		ACADOERROR( status );

	return _block;
}

//
// PROTECTED MEMBER FUNCTIONS:
//



CLOSE_NAMESPACE_ACADO

// end of file.
