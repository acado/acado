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

ExportStatementstd::string::ExportStatementstd::string(	const std::string& _statementstd::string
												) : ExportStatement( )
{
	statementstd::string = _statementstd::string;
}


ExportStatementstd::string::ExportStatementstd::string( const ExportStatementstd::string& arg ) : ExportStatement( arg )
{
	statementstd::string = arg.statementstd::string;
}


ExportStatementstd::string::~ExportStatementstd::string( )
{
}


ExportStatementstd::string& ExportStatementstd::string::operator=( const ExportStatementstd::string& arg )
{
	if( this != &arg )
	{
		// empty
		ExportStatement::operator=( arg );
		statementstd::string = arg.statementstd::string;
	}

	return *this;
}


ExportStatement* ExportStatementstd::string::clone( ) const
{
	return new ExportStatementstd::string(*this);
}


returnValue ExportStatementstd::string::exportCode(	FILE *file,
												const std::string& _realstd::string,
												const std::string& _intstd::string,
												int _precision
												) const
{
	acadoFPrintf( file,"%s", statementstd::string.getName() );

	return SUCCESSFUL_RETURN;
}


//
// PROTECTED MEMBER FUNCTIONS:
//



CLOSE_NAMESPACE_ACADO

// end of file.
