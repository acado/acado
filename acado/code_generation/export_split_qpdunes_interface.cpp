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
 *    \file src/code_generation/export_split_qpdunes_interface.cpp
 *    \author Milan Vukov, Rien Quirynen
 *    \date 2013 - 2014
 */

#include <acado/code_generation/export_split_qpdunes_interface.hpp>
#include <acado/code_generation/templates/templates.hpp>

using namespace std;

BEGIN_NAMESPACE_ACADO

ExportSplitQpDunesInterface::ExportSplitQpDunesInterface(	const std::string& _fileName,
												const std::string& _commonHeaderName,
												const std::string& _realString,
												const std::string& _intString,
												int _precision,
												const std::string& _commentString
						) : ExportTemplatedFile(QPDUNES_SPLIT_TEMPLATE, _fileName, _commonHeaderName, _realString, _intString, _precision, _commentString)
{}

returnValue ExportSplitQpDunesInterface::configure(	const unsigned _maxIter,
												const unsigned _printLevel,
												const std::string& _HH,
												const std::string& _g,
												const std::string& _gN,
												const std::string& _CC,
												const std::string& _c,
												const std::string& _DD,
												const std::string& _lb0,
												const std::string& _ub0,
												const std::string& _lb,
												const std::string& _ub,
												const std::string& _lbA,
												const std::string& _ubA,
												const std::string& _primal,
												const std::string& _lambda,
												const std::string& _mu,
												const std::vector< unsigned >& conDim,
												const std::string& _initialStateFixed,
												const std::string& _diagH,
												const std::string& _diagHN,
												const unsigned _NI,
												const unsigned _NX,
												const unsigned _NU
)
{
	stringstream ss;

	// Configure the dictionary

	ss << _maxIter;
	dictionary[ "@MAX_ITER@" ] =  ss.str();

	ss.str( string() );
	ss << _printLevel;
	dictionary[ "@PRINT_LEVEL@" ] =  ss.str();

	ss.str( string() );
	ss << _NI;
	dictionary[ "@ACADO_N@" ] =  ss.str();

	ss.str( string() );
	ss << _NX;
	dictionary[ "@ACADO_NX@" ] =  ss.str();

	ss.str( string() );
	ss << _NU;
	dictionary[ "@ACADO_NU@" ] =  ss.str();

	dictionary[ "@QP_H@" ] =  _HH;
	dictionary[ "@QP_G@" ] =  _g;
	dictionary[ "@QP_G_N@" ] =  _gN;
	dictionary[ "@QP_C@" ] =  _CC;
	dictionary[ "@QP_c@" ] =  _c;

	dictionary[ "@QP_LB@" ] =  _lb;
	dictionary[ "@QP_UB@" ] =  _ub;

	dictionary[ "@QP_LB_0@" ] =  _lb0;
	dictionary[ "@QP_UB_0@" ] =  _ub0;

	if (conDim.size() > 0)
	{
		dictionary[ "@QP_D@" ]   =  _DD;
		dictionary[ "@QP_LBA@" ] =  _lbA;
		dictionary[ "@QP_UBA@" ] =  _ubA;

		ss.str( string() );
		ss << "unsigned int nD[";
		ss << _NI;
		ss << " + 1] = {";
		for (unsigned i = 0; i < conDim.size(); ++i)
		{
			ss << conDim[ i ];
			if (i < conDim.size() - 1)
				ss << ", ";
		}
		ss << "};";
		dictionary[ "@QP_ND_ARRAY@" ] = ss.str();
	}
	else
	{
		dictionary[ "@QP_D@" ] = dictionary[ "@QP_LBA@" ] = dictionary[ "@QP_UBA@" ] = "0";
		ss.str( string() );
		ss << "unsigned int nD[";
		ss << _NI;
		ss << " + 1]; for (kk = 0; kk < ";
		ss << _NI;
		ss << " + 1; nD[ kk++ ] = 0);";
		dictionary[ "@QP_ND_ARRAY@" ] = ss.str();
	}

	dictionary[ "@QP_PRIMAL@" ] =  _primal;
	dictionary[ "@QP_LAMBDA@" ] =  _lambda;
	dictionary[ "@QP_MU@" ] =  _mu;

	dictionary[ "@INITIAL_STATE_FIXED@" ] = _initialStateFixed;

	dictionary[ "@DIAG_H@" ] = _diagH;
	dictionary[ "@DIAG_HN@" ] = _diagHN;

	// And then fill a template file
	fillTemplate();

	return SUCCESSFUL_RETURN;
}

CLOSE_NAMESPACE_ACADO
