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
 *    \file src/code_generation/export_auxiliary_functions.cpp
 *    \author Milan Vukov
 *    \date 2013
 */

#include <acado/code_generation/export_auxiliary_functions.hpp>
#include <acado/code_generation/templates/templates.hpp>

#include <sstream>

using namespace std;

BEGIN_NAMESPACE_ACADO


ExportAuxiliaryFunctions::ExportAuxiliaryFunctions(	const String& _headerFileName,
													const String& _sourceFileName,
													const String& _moduleName,
													const String& _commonHeaderName,
													const String& _realString,
													const String& _intString,
													int _precision,
													const String& _commentString
													)
	: source(AUXILIARY_FUNCTIONS_SOURCE, _sourceFileName, _commonHeaderName, _realString, _intString, _precision, _commentString),
	  header(AUXILIARY_FUNCTIONS_HEADER, _headerFileName, _commonHeaderName, _realString, _intString, _precision, _commentString),
	  moduleName( _moduleName )
{}


returnValue ExportAuxiliaryFunctions::configure( )
{
	//
	// Source file configuration
	//

	source.dictionary[ "@MODULE_NAME@" ] = string( moduleName.getName() );

	source.fillTemplate();

	//
	// Header file configuration
	//
	header.dictionary[ "@MODULE_NAME@" ] = string( moduleName.getName() );

	header.fillTemplate();

	return SUCCESSFUL_RETURN;
}

returnValue ExportAuxiliaryFunctions::exportCode()
{
	source.exportCode();
	header.exportCode();

	return SUCCESSFUL_RETURN;
}


CLOSE_NAMESPACE_ACADO
