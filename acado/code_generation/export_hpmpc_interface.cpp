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
 *    \file src/code_generation/export_hpmpc_interface.cpp
 *    \author Milan Vukov
 *    \date 2014
 */

#include <acado/code_generation/export_hpmpc_interface.hpp>
#include <acado/code_generation/templates/templates.hpp>

#include <sstream>

using namespace std;

BEGIN_NAMESPACE_ACADO

ExportHpmpcInterface::ExportHpmpcInterface(	const std::string& _fileName,
											const std::string& _commonHeaderName,
											const std::string& _realString,
											const std::string& _intString,
											int _precision,
											const std::string& _commentString
						) : ExportTemplatedFile(HPMPC_INTERFACE, _fileName, _commonHeaderName, _realString, _intString, _precision, _commentString)
{}

returnValue ExportHpmpcInterface::configure(	const unsigned _maxIter,
												const unsigned _printLevel,
												bool _useSinglePrecision,
												bool _warmStart
												)
{
	// Configure the dictionary
	dictionary[ "@MAX_ITER@" ] =  toString( _maxIter );
	dictionary[ "@PRINT_LEVEL@" ] =  _printLevel == 0 ? toString( 0 ) : toString( 1 );
	dictionary[ "@PRECISION@" ] =  _useSinglePrecision == true ? "1" : "0";
	dictionary[ "@WARM_START@" ] =  _warmStart == true ? "1" : "0";

	// And then fill a template file
	fillTemplate();

	return SUCCESSFUL_RETURN;
}

CLOSE_NAMESPACE_ACADO
