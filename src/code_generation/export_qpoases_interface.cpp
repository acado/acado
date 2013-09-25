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
 *    \file src/code_generation/export_qpoases_interface.cpp
 *    \author Milan Vukov
 *    \date 2012 - 2013
 */

#include <acado/code_generation/export_qpoases_interface.hpp>
#include <acado/code_generation/templates/templates.hpp>

#include <sstream>

using namespace std;

BEGIN_NAMESPACE_ACADO


ExportQpOasesInterface::ExportQpOasesInterface(		const String& _headerFileName,
													const String& _sourceFileName,
													const String& _commonHeaderName,
													const String& _realString,
													const String& _intString,
													int _precision,
													const String& _commentString
													)
	: qpoHeader(QPOASES_HEADER, _headerFileName, _commonHeaderName, _realString, _intString, _precision, _commentString),
	  qpoSource(QPOASES_SOURCE, _sourceFileName, _commonHeaderName, _realString, _intString, _precision, _commentString)
{}


returnValue ExportQpOasesInterface::configure(	const std::string& _prefix,
												const std::string& _solverDefine,
												const int nvmax,
												const int ncmax,
												const int nwsrmax,
												const std::string& _printLevel,
												const double _eps,
												const std::string& _reat_t,

												const std::string& _commonHeader,
												const std::string& _solverName,
												const std::string& _namespace,
												const std::string& _callSolver,
												const std::string& _primalSolution,
												const std::string& _dualSolution,
												const std::string& _ctor,
												const std::string& _sigma
												)
{
	//
	// Source file configuration
	//

	qpoSource.dictionary[ "@ACADO_COMMON_HEADER@" ] =  _commonHeader;
	qpoSource.dictionary[ "@SOLVER_NAME@" ] =  _solverName;
	qpoSource.dictionary[ "@PREFIX@" ] =  _prefix;
	qpoSource.dictionary[ "@USE_NAMESPACE@" ] =  _namespace;
	qpoSource.dictionary[ "@CALL_SOLVER@" ] =  _callSolver;
	qpoSource.dictionary[ "@PRIMAL_SOLUTION@" ] =  _primalSolution;
	qpoSource.dictionary[ "@DUAL_SOLUTION@" ] =  _dualSolution;
	qpoSource.dictionary[ "@CTOR@" ] =  _ctor;
	qpoSource.dictionary[ "@SIGMA@" ] =  _sigma;

	// And then fill a template file
	qpoSource.fillTemplate();

	//
	// Header file configuration
	//

	stringstream s;

	// Configure the dictionary
	qpoHeader.dictionary[ "@PREFIX@" ] =  _prefix;
	qpoHeader.dictionary[ "@SOLVER_DEFINE@" ] =  _solverDefine;

	s.str(std::string());
	s << nvmax;
	qpoHeader.dictionary[ "@NVMAX@" ] = s.str();

	s.str(std::string());
	s << ncmax;
	qpoHeader.dictionary[ "@NCMAX@" ] =  s.str();

	s.str(std::string());
	s << nwsrmax;
	qpoHeader.dictionary[ "@NWSRMAX@" ] =  s.str();
	qpoHeader.dictionary[ "@PRINT_LEVEL@" ] =  _printLevel;

	s.str(std::string());
	s << _eps;
	qpoHeader.dictionary[ "@EPS@" ] =  s.str();
	qpoHeader.dictionary[ "@REAL_T@" ] =  _reat_t;

	// And then fill a template file
	qpoHeader.fillTemplate();

	return SUCCESSFUL_RETURN;
}

returnValue ExportQpOasesInterface::exportCode()
{
	qpoHeader.exportCode();
	qpoSource.exportCode();

	return SUCCESSFUL_RETURN;
}


CLOSE_NAMESPACE_ACADO
