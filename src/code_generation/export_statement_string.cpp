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
 *    \file src/code_generation/export_statement_string.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 2010-2011
 */

#include <acado/code_generation/export_statement_string.hpp>


BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

ExportStatementString::ExportStatementString(	const String& _statementString
												) : ExportStatement( )
{
	statementString = _statementString;
}


ExportStatementString::ExportStatementString( const ExportStatementString& arg ) : ExportStatement( arg )
{
	statementString = arg.statementString;
}


ExportStatementString::~ExportStatementString( )
{
}


ExportStatementString& ExportStatementString::operator=( const ExportStatementString& arg )
{
	if( this != &arg )
	{
		// empty
		ExportStatement::operator=( arg );
		statementString = arg.statementString;
	}

	return *this;
}


ExportStatement* ExportStatementString::clone( ) const
{
	return new ExportStatementString(*this);
}


returnValue ExportStatementString::exportCode(	FILE *file,
												const String& _realString,
												const String& _intString,
												int _precision
												) const
{
	acadoFPrintf( file,"%s", statementString.getName() );

	return SUCCESSFUL_RETURN;
}


//
// PROTECTED MEMBER FUNCTIONS:
//



CLOSE_NAMESPACE_ACADO

// end of file.
