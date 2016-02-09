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
 *    \file src/code_generation/export_qpoases_interface.cpp
 *    \author Milan Vukov
 *    \date 2012 - 2014
 */

#include <acado/code_generation/export_qpoases_interface.hpp>
#include <acado/code_generation/templates/templates.hpp>

using namespace std;

BEGIN_NAMESPACE_ACADO


ExportQpOasesInterface::ExportQpOasesInterface(		const std::string& _headerFileName,
													const std::string& _sourceFileName,
													const std::string& _commonHeaderName,
													const std::string& _realString,
													const std::string& _intString,
													int _precision,
													const std::string& _commentString
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
												bool _useSinglePrecision,

												const std::string& _commonHeader,
												const std::string& _namespace,
												const std::string& _primalSolution,
												const std::string& _dualSolution,
												const std::string& _sigma,
												bool _hotstartQP,
												bool _externalCholesky,
												const std::string& _qpH,
												const std::string& _qpR,
												const std::string& _qpg,
												const std::string& _qpA,
												const std::string& _qplb,
												const std::string& _qpub,
												const std::string& _qplbA,
												const std::string& _qpubA
												)
{
	//
	// Source file configuration
	//

	stringstream s, ctor;
	string solverName;
	if (ncmax > 0)
	{
		solverName = "QProblem";

		s	<< _qpH << ", ";
		if (_externalCholesky == false)
			s << _qpR << ", ";
		s	<< _qpg << ", " << _qpA << ", " << _qplb << ", " << _qpub << ", "
			<< _qplbA << ", " << _qpubA << ", " << ExportStatement::fcnPrefix << _prefix << "_nWSR";

		if ( (bool)_hotstartQP == true )
			s << ", " << _dualSolution;

		ctor << solverName << " qp(" << nvmax << ", " << ncmax << ")";
	}
	else
	{
		solverName = "QProblemB";

		s	<< _qpH << ", ";
		if (_externalCholesky == false)
			s << _qpR << ", ";
		s	<< _qpg << ", " << _qplb << ", " << _qpub << ", " << ExportStatement::fcnPrefix << _prefix << "_nWSR";

		if ( (bool)_hotstartQP == true )
			s << ", " << _dualSolution;

		ctor << solverName << " qp( " << nvmax << " )";
	}

	qpoSource.dictionary[ "@ACADO_COMMON_HEADER@" ] =  _commonHeader;
	qpoSource.dictionary[ "@SOLVER_NAME@" ] =  solverName;
	qpoSource.dictionary[ "@PREFIX@" ] =  _prefix;
	qpoSource.dictionary[ "@USE_NAMESPACE@" ] =  _namespace;
	qpoSource.dictionary[ "@CALL_SOLVER@" ] =  s.str();
	qpoSource.dictionary[ "@PRIMAL_SOLUTION@" ] =  _primalSolution;
	qpoSource.dictionary[ "@DUAL_SOLUTION@" ] =  _dualSolution;
	qpoSource.dictionary[ "@CTOR@" ] =  ctor.str();
	qpoSource.dictionary[ "@SIGMA@" ] =  _sigma;
    qpoSource.dictionary[ "@MODULE_NAME@" ] = ExportStatement::fcnPrefix;
    qpoSource.dictionary[ "@MODULE_PREFIX@" ] = ExportStatement::varPrefix;

	// And then fill a template file
	qpoSource.fillTemplate();

	//
	// Header file configuration
	//

	// Configure the dictionary
    qpoHeader.dictionary[ "@MODULE_NAME@" ] = ExportStatement::fcnPrefix;
    qpoHeader.dictionary[ "@MODULE_PREFIX@" ] = ExportStatement::varPrefix;
    
	qpoHeader.dictionary[ "@PREFIX@" ] =  _prefix;
	qpoHeader.dictionary[ "@SOLVER_DEFINE@" ] =  _solverDefine;

	qpoHeader.dictionary[ "@NVMAX@" ] = toString( nvmax );

	qpoHeader.dictionary[ "@NCMAX@" ] = toString( ncmax );

	qpoHeader.dictionary[ "@NWSRMAX@" ] =  toString(nwsrmax > 0 ? nwsrmax : 3 * (nvmax + ncmax));

	qpoHeader.dictionary[ "@PRINT_LEVEL@" ] =  _printLevel;

	double eps;
	string realT;
	if ( _useSinglePrecision )
	{
		eps = 1.193e-07;
		realT = "float";
	}
	else
	{
		eps = 2.221e-16;
		realT = "double";
	}
	qpoHeader.dictionary[ "@EPS@" ] =  toString( eps );
	qpoHeader.dictionary[ "@REAL_T@" ] =  toString( realT );

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
