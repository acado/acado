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

returnValue ExportHpmpcInterface::configure( const unsigned _maxIter,
												const unsigned _printLevel,
												bool _useSinglePrecision,
												bool _warmStart,
												const std::string& _Hx,
												const std::string& _Hu,
												const std::string& _lbA,
												const std::string& _ubA,
												const unsigned _DimH,
												const std::vector< unsigned >& _conDim,
												const unsigned _NI,
												const unsigned _NX,
												const unsigned _NU
												)
{
	stringstream ss;
	// Configure the dictionary
	dictionary[ "@MAX_ITER@" ] =  toString( _maxIter );
	dictionary[ "@PRINT_LEVEL@" ] =  _printLevel == 0 ? toString( 0 ) : toString( 1 );
	dictionary[ "@PRECISION@" ] =  _useSinglePrecision == true ? "1" : "0";
	dictionary[ "@WARM_START@" ] =  _warmStart == true ? "1" : "0";
	dictionary[ "@MODULE_NAME@" ] = ExportStatement::fcnPrefix;
	dictionary[ "@MODULE_PREFIX@" ] = ExportStatement::varPrefix;
	dictionary[ "@QP_DIMMU@"] = toString( 2*(_NX+_NU)*(_NI+1) + 2*_DimH );
	// Single precision is not supported yet!
	if(_useSinglePrecision) return ACADOERROR( RET_NOT_YET_IMPLEMENTED );

	if (_conDim.size() > 0)
	{
		dictionary[ "@QP_Hx@" ]   =  _Hx;
		dictionary[ "@QP_Hu@" ]   =  _Hu;
		dictionary[ "@QP_LBA@" ] =  _lbA;
		dictionary[ "@QP_UBA@" ] =  _ubA;

		ss.str( string() );
		ss << "unsigned int nD[";
		ss << _NI;
		ss << " + 1] = {";
		for (unsigned i = 0; i < _conDim.size(); ++i)
		{
			ss << _conDim[ i ];
			if (i < _conDim.size() - 1)
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

	// And then fill a template file
	fillTemplate();

	return SUCCESSFUL_RETURN;
}

CLOSE_NAMESPACE_ACADO
