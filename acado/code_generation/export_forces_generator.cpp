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
 *    \file src/code_generation/export_forces_generator.cpp
 *    \author Milan Vukov
 *    \date 2012 - 2014
 */

#include <acado/code_generation/export_forces_generator.hpp>

using namespace std;

BEGIN_NAMESPACE_ACADO

ExportForcesGenerator::ExportForcesGenerator(	const std::string& _templateName,
												const std::string& _fileName,
												const std::string& _commonHeaderName,
												const std::string& _realString,
												const std::string& _intString,
												int _precision,
												const std::string& _commentString
						) : ExportTemplatedFile(_templateName, _fileName, _commonHeaderName, _realString, _intString, _precision, _commentString)
{}

returnValue ExportForcesGenerator::configure(	const unsigned _nx,
												const unsigned _nu,
												const unsigned _NN,
												const std::vector< std::vector< unsigned > >& _lbIdx,
												const std::vector< std::vector< unsigned > >& _ubIdx,
												const std::vector< unsigned >& _AbDim,
												const bool _constHessian,
												const bool _diagHessian,
												const bool _diagHessianN,
												const bool _fixedInitialState,
												const std::string& _solverName,
												const unsigned _printLevel,
												const unsigned _maxIterations,
												const unsigned _parallel,
												bool matlabGenerator,
												bool warmStart
												)
{
	stringstream s;

	// Configure the dictionary

	s.str(std::string());
	s << _nx;
	dictionary[ "@NX@" ] =  s.str();

	s.str(std::string());
	s << _nu;
	dictionary[ "@NU@" ] =  s.str();

	// NOTE FORCES needs number of shooting nodes...
	s.str(std::string());
	s << _NN + 1;
	dictionary[ "@N@" ] =  s.str();

	s.str(std::string());
	if (matlabGenerator == true)
		for (unsigned i = 0; i < _lbIdx.size(); ++i)
		{
			if ( i )
				s << ", ..." << endl << "\t";

			s << "{";
			for (unsigned j = 0; j < _lbIdx[ i ].size(); ++j)
			{
				if ( j )
					s << ", ";
				s << _lbIdx[ i ][ j ] + 1;
			}
			s << "}";
		}
	else
		// Python generator
		for (unsigned i = 0; i < _lbIdx.size(); ++i)
		{
			if ( i )
				s << ", " << endl << "\t";

			s << "[";
			for (unsigned j = 0; j < _lbIdx[ i ].size(); ++j)
			{
				if ( j )
					s << ", ";
				s << _lbIdx[ i ][ j ] + 1;
			}
			s << "]";
		}

	dictionary[ "@LB_IDX@" ] =  s.str();

	s.str(std::string());
	if (matlabGenerator == true)
		for (unsigned i = 0; i < _ubIdx.size(); ++i)
		{
			if ( i )
				s << ", ..." << endl << "\t";

			s << "{";
			for (unsigned j = 0; j < _ubIdx[ i ].size(); ++j)
			{
				if ( j )
					s << ", ";
				s << _ubIdx[ i ][ j ] + 1;
			}
			s << "}";
		}
	else
		// Python generator
		for (unsigned i = 0; i < _ubIdx.size(); ++i)
		{
			if ( i )
				s << ", " << endl << "\t";

			s << "[";
			for (unsigned j = 0; j < _ubIdx[ i ].size(); ++j)
			{
				if ( j )
					s << ", ";
				s << _ubIdx[ i ][ j ] + 1;
			}
			s << "]";
		}

	dictionary[ "@UB_IDX@" ] =  s.str();

	s.str(std::string());
	if (matlabGenerator == true)
		for (unsigned i = 0; i < _NN + 1; ++i)
		{
			if ( i )
				s << ", ..." << endl << "\t";

			if( i >= _AbDim.size() ) {
				s << 0;
			}
			else {
				s << _AbDim[ i ];
			}
		}
//	else
//		// Python generator
//		return ACADOERROR( RET_NOT_YET_IMPLEMENTED );

	dictionary[ "@AB_DIM@" ] =  s.str();

	s.str(std::string());
	s << (_constHessian == true ? 1 : 0);
	dictionary[ "@CONST_HESSIAN@" ] =  s.str();

	s.str(std::string());
	s << (_diagHessian == true ? 1 : 0);
	dictionary[ "@DIAG_HESSIAN@" ] =  s.str();

	s.str(std::string());
	s << (_diagHessianN == true ? 1 : 0);
	dictionary[ "@DIAG_HESSIAN_N@" ] =  s.str();

	s.str(std::string());
	s << (_fixedInitialState == true ? 1 : 0);
	dictionary[ "@FIXED_INITIAL_STATE@" ] =  s.str();

	dictionary[ "@SOLVER_NAME@" ] =  _solverName;

	s.str(std::string());
	s << _printLevel;
	dictionary[ "@PRINT_LEVEL@" ] =  s.str();

	s.str(std::string());
	s << _maxIterations;
	dictionary[ "@MAX_ITERATIONS@" ] =  s.str();

	s.str(std::string());
	s << _parallel;
	dictionary[ "@PARALLEL@" ] =  s.str();

	dictionary[ "@WARM_START@" ] =  warmStart ? "2" : "0";

	// And then fill a template file
	fillTemplate();

	return SUCCESSFUL_RETURN;
}

CLOSE_NAMESPACE_ACADO
