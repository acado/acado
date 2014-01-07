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
 *    \file src/code_generation/export_function_declaration.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 2010-2011
 */

#include <acado/code_generation/export_function_declaration.hpp>


BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

ExportFunctionDeclaration::ExportFunctionDeclaration( ) : ExportStatement( ), f( ExportFunction() )
{}


ExportFunctionDeclaration::ExportFunctionDeclaration(	const ExportFunction& _f
														) : ExportStatement( ), f( _f )
{}


ExportFunctionDeclaration::ExportFunctionDeclaration(	const ExportAcadoFunction& _f
														) : ExportStatement( ), f( _f )
{}

ExportFunctionDeclaration::~ExportFunctionDeclaration( )
{}

ExportStatement* ExportFunctionDeclaration::clone( ) const
{
	return new ExportFunctionDeclaration(*this);
}


returnValue ExportFunctionDeclaration::exportCode(	std::ostream& stream,
													const std::string& _realString,
													const std::string& _intString,
													int _precision
													) const
{
	return f.exportForwardDeclaration(stream, _realString, _intString, _precision);
}

CLOSE_NAMESPACE_ACADO
