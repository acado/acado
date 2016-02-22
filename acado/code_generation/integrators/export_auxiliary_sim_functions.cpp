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
 *    \file src/code_generation/integrators/export_auxiliary_sim_functions.cpp
 *    \author Rien Quirynen
 *    \date 2013
 */

#include <acado/code_generation/integrators/export_auxiliary_sim_functions.hpp>
#include <acado/code_generation/templates/templates.hpp>

using namespace std;

BEGIN_NAMESPACE_ACADO


ExportAuxiliarySimFunctions::ExportAuxiliarySimFunctions(	const std::string& _headerFileName,
                                                            const std::string& _sourceFileName,
                                                            const std::string& _moduleName,
                                                            const std::string& _modulePrefix,
                                                            const std::string& _commonHeaderName,
                                                            const std::string& _realString,
                                                            const std::string& _intString,
                                                            int _precision,
                                                            const std::string& _commentString
                                                            )
	: source(AUXILIARY_SIM_FUNCTIONS_SOURCE, _sourceFileName, _commonHeaderName, _realString, _intString, _precision, _commentString),
	  header(AUXILIARY_SIM_FUNCTIONS_HEADER, _headerFileName, _commonHeaderName, _realString, _intString, _precision, _commentString),
	  moduleName( _moduleName ),
      modulePrefix( _modulePrefix )
{}


returnValue ExportAuxiliarySimFunctions::configure( )
{
	//
	// Source file configuration
	//

	source.dictionary[ "@MODULE_NAME@" ] = moduleName;
    source.dictionary[ "@MODULE_PREFIX@" ] = modulePrefix;

	source.fillTemplate();

	//
	// Header file configuration
	//
	header.dictionary[ "@MODULE_NAME@" ] = moduleName;
    header.dictionary[ "@MODULE_PREFIX@" ] = modulePrefix;

	header.fillTemplate();

	return SUCCESSFUL_RETURN;
}

returnValue ExportAuxiliarySimFunctions::exportCode()
{
	source.exportCode();
	header.exportCode();

	return SUCCESSFUL_RETURN;
}


CLOSE_NAMESPACE_ACADO
