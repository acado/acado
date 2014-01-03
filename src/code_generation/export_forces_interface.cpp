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
 *    \file src/code_generation/export_forces_interface.cpp
 *    \author Milan Vukov
 *    \date 2012
 */

#include <acado/code_generation/export_forces_interface.hpp>

using namespace std;

BEGIN_NAMESPACE_ACADO

ExportForcesInterface::ExportForcesInterface(	const std::string& _templateName,
												const std::string& _fileName,
												const std::string& _commonHeaderName,
												const std::string& _realString,
												const std::string& _intString,
												int _precision,
												const std::string& _commentString
						) : ExportTemplatedFile(_templateName, _fileName, _commonHeaderName, _realString, _intString, _precision, _commentString)
{}

returnValue ExportForcesInterface::configure(	const std::string& _forcesHeader,
												const std::string& _forcesParams,
												const std::string& _forcesParamsObj,
												const std::string& _forcesOutput,
												const std::string& _forcesOutputObj,
												const std::string& _forcesInfo,
												const std::string& _forcesInfoObj
												)
{	
	// Configure the dictionary
	dictionary[ "@FORCES_HEADER@" ] =  _forcesHeader;
	dictionary[ "@FORCES_PARAMS@" ] = _forcesParams;
	dictionary[ "@FORCES_PARAMS_OBJ@" ] =  _forcesParamsObj;
	dictionary[ "@FORCES_OUTPUT@" ] =  _forcesOutput;
	dictionary[ "@FORCES_OUTPUT_OBJ@" ] =  _forcesOutputObj;
	dictionary[ "@FORCES_INFO@" ] =  _forcesInfo;
	dictionary[ "@FORCES_INFO_OBJ@" ] =  _forcesInfoObj;
	
	// And then fill a template file
	fillTemplate();

	return SUCCESSFUL_RETURN;
}

CLOSE_NAMESPACE_ACADO
