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
 *    \file src/code_generation/export_hessian_regularization.cpp
 *    \author Rien Quirynen
 *    \date 2014
 */

#include <acado/code_generation/export_hessian_regularization.hpp>
#include <acado/code_generation/templates/templates.hpp>

using namespace std;

BEGIN_NAMESPACE_ACADO


ExportHessianRegularization::ExportHessianRegularization(	const std::string& _sourceFileName,
                                                            const std::string& _moduleName,
                                                            const std::string& _modulePrefix,
                                                            const std::string& _commonHeaderName,
                                                            const std::string& _realString,
                                                            const std::string& _intString,
                                                            int _precision,
                                                            const std::string& _commentString
                                                            )
	: ExportTemplatedFile(HESSIAN_REG_SOURCE, _sourceFileName, _commonHeaderName, _realString, _intString, _precision, _commentString)
{
	moduleName = _moduleName;
    modulePrefix = _modulePrefix;
}


returnValue ExportHessianRegularization::configure( uint DIM, double eps )
{
	//
	// Source file configuration
	//
	stringstream ss;

	dictionary[ "@MODULE_NAME@" ] = moduleName;
    dictionary[ "@MODULE_PREFIX@" ] = modulePrefix;
    
	ss << DIM;
	dictionary[ "@MODULE_DIM@" ] = ss.str();

	ss.str( string() );
	ss << eps;
	dictionary[ "@MODULE_EPS@" ] = ss.str();

	fillTemplate();

	return SUCCESSFUL_RETURN;
}


CLOSE_NAMESPACE_ACADO
