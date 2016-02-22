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
 *    \file src/code_generation/export_common_header.cpp
 *    \author Milan Vukov
 *    \date 2013
 */

#include <acado/code_generation/export_common_header.hpp>
#include <acado/code_generation/templates/templates.hpp>

#include <algorithm>

using namespace std;

BEGIN_NAMESPACE_ACADO

ExportCommonHeader::ExportCommonHeader(	const std::string& _fileName,
										const std::string& _commonHeaderName,
										const std::string& _realString,
										const std::string& _intString,
										int _precision,
										const std::string& _commentString
										) : ExportTemplatedFile(COMMON_HEADER_TEMPLATE, _fileName, _commonHeaderName, _realString, _intString, _precision, _commentString)
{}

returnValue ExportCommonHeader::configure(	const std::string& _moduleName,
                                            const std::string& _modulePrefix,
											bool _useSinglePrecision,
											bool _useComplexArithmetic,
											QPSolverName _qpSolver,
											const std::map<std::string, std::pair<std::string, std::string> >& _options,
											const std::string& _variables,
											const std::string& _workspace,
											const std::string& _functions
											)
{
	// Configure the template
	dictionary[ "@MODULE_NAME@" ] = _moduleName;
    dictionary[ "@MODULE_PREFIX@" ] = _modulePrefix;

	stringstream ss;
	if( _useComplexArithmetic ) ss << "\n#include <complex.h>\n" << endl;

	ss 	<< "/** qpOASES QP solver indicator. */" << endl
		<< "#define " << _modulePrefix << "_QPOASES  0" << endl
        << "#define " << _modulePrefix << "_QPOASES3 1" << endl
		<< "/** FORCES QP solver indicator.*/" << endl
		<< "#define " << _modulePrefix << "_FORCES   2" << endl
		<< "/** qpDUNES QP solver indicator.*/" << endl
		<< "#define " << _modulePrefix << "_QPDUNES  3" << endl
		<< "/** HPMPC QP solver indicator. */" << endl
		<< "#define " << _modulePrefix << "_HPMPC    4" << endl << endl
		<< "/** Indicator for determining the QP solver used by the ACADO solver code. */" << endl;

	switch ( _qpSolver )
	{
	case QP_QPOASES:
		ss << "#define " << _modulePrefix << "_QP_SOLVER " << _modulePrefix << "_QPOASES\n" << endl;
		ss << "#include \"" << _moduleName << "_qpoases_interface.hpp\"\n";

		break;

	case QP_QPOASES3:
		ss << "#define " << _modulePrefix << "_QP_SOLVER " << _modulePrefix << "_QPOASES3\n" << endl;
		ss << "#include \"" << _moduleName << "_qpoases3_interface.h\"\n";

		break;

	case QP_FORCES:
	case QP_HPMPC:
	case QP_QPDUNES:
		if (_qpSolver == QP_FORCES)
			ss << "#define " << _modulePrefix << "_QP_SOLVER " << _modulePrefix << "_FORCES\n" << endl;
		else if (_qpSolver == QP_HPMPC)
			ss << "#define " << _modulePrefix << "_QP_SOLVER " << _modulePrefix << "_HPMPC\n" << endl;
		else
			ss << "#define " << _modulePrefix << "_QP_SOLVER " << _modulePrefix << "_QPDUNES\n" << endl;

		ss << "\n#include <string.h>\n\n" << endl;

		ss << "/** Definition of the floating point data type. */\n";
		if (_useSinglePrecision == true)
			ss << "typedef float real_t;\n";
		else
			ss << "typedef double real_t;\n";

		break;
	
	case QP_NONE:
		ss << "/** Definition of the floating point data type. */\n";
		if (_useSinglePrecision == true)
			ss << "typedef float real_t;\n";
		else
			ss << "typedef double real_t;\n";

		break;

	default:
		return ACADOERROR( RET_INVALID_OPTION );

	}
	dictionary[ "@QP_SOLVER_INTERFACE@" ] = ss.str();

	ss.str( string() );
	// Key: define name
	// Value.first: value
	// Value.second: comment
	std::map<std::string, std::pair<std::string, std::string> >::const_iterator it;
	for (it = _options.begin(); it != _options.end(); ++it)
	{
		ss << "/** " << it->second.second << " */" << endl;
		ss << "#define " << it->first << " " << it->second.first << endl;
	}

	dictionary[ "@COMMON_DEFINITIONS@" ] = ss.str();

	dictionary[ "@VARIABLES_DECLARATION@" ] = _variables;

	dictionary[ "@WORKSPACE_DECLARATION@" ] = _workspace;

	dictionary[ "@FUNCTION_DECLARATIONS@" ] = _functions;

	// And then fill a template file
	fillTemplate();

	return SUCCESSFUL_RETURN;
}

CLOSE_NAMESPACE_ACADO
