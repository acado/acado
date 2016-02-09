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
 *    \file src/code_generation/export_simulink_interface.cpp
 *    \author Milan Vukov
 *    \date 2013 - 2014
 */

#include <acado/code_generation/export_simulink_interface.hpp>
#include <acado/code_generation/templates/templates.hpp>

#include <sstream>

using namespace std;

BEGIN_NAMESPACE_ACADO


ExportSimulinkInterface::ExportSimulinkInterface(	const std::string& _makefileName,
													const std::string& _wrapperHeaderFileName,
													const std::string& _wrapperSourceFileName,
													const std::string& _moduleName,
                                                    const std::string& _modulePrefix,
													const std::string& _commonHeaderName,
													const std::string& _realString,
													const std::string& _intString,
													int _precision,
													const std::string& _commentString
													)
	: makefile(MAKEFILE_SFUN_QPOASES, _makefileName, "", _realString, _intString, _precision, "%"),
	  wrapperSource(SOLVER_SFUN_SOURCE, _wrapperSourceFileName, _commonHeaderName, _realString, _intString, _precision, _commentString),
	  wrapperHeader(SOLVER_SFUN_HEADER, _wrapperHeaderFileName, _commonHeaderName, _realString, _intString, _precision, _commentString),
	  moduleName( _moduleName ),
      modulePrefix( _modulePrefix )
{}


returnValue ExportSimulinkInterface::configure(	unsigned N,
												unsigned NX,
												unsigned NDX,
												unsigned NXA,
												unsigned NU,
												unsigned NOD,
												unsigned NY,
												unsigned NYN,
												bool _initialStateFixed,
												unsigned _wMatrixType,
												bool _hardcodedConstraints,
												bool _useArrivalCost,
												bool _compCovMatrix,
												std::string _qpSolver
												)
{
	//
	// Source file configuration
	//

	wrapperSource.dictionary[ "@MODULE_NAME@" ] = moduleName;
    wrapperSource.dictionary[ "@MODULE_PREFIX@" ] = modulePrefix;

	wrapperSource.fillTemplate();

	//
	// Header file configuration
	//
	wrapperHeader.dictionary[ "@MODULE_NAME@" ] = moduleName;
    wrapperHeader.dictionary[ "@MODULE_PREFIX@" ] = modulePrefix;

	wrapperHeader.fillTemplate();

	//
	// Makefile configuration
	//

	makefile.dictionary[ "@MODULE_NAME@" ] = moduleName;
    makefile.dictionary[ "@MODULE_PREFIX@" ] = modulePrefix;
    
	makefile.dictionary[ "@REAL_TYPE@" ] = makefile.realString;

	makefile.dictionary[ "@N@" ] = toString( N );
	makefile.dictionary[ "@NX@" ] = toString( NX );
	makefile.dictionary[ "@NXA@" ] = toString( NXA );
	makefile.dictionary[ "@NDX@" ] = toString( NDX );
	makefile.dictionary[ "@NU@" ] = toString( NU );
	makefile.dictionary[ "@NOD@" ] = toString( NOD );
	makefile.dictionary[ "@NY@" ] = toString( NY );
	makefile.dictionary[ "@NYN@" ] = toString( NYN );

	makefile.dictionary[ "@INIT_STATE_FIXED@" ] = _initialStateFixed == true ? "1" : "0";

	makefile.dictionary[ "@WEIGHT_MATRIX_TYPE@" ] = toString( _wMatrixType );

	makefile.dictionary[ "@HARCODED_CONSTRAINTS@" ] = _hardcodedConstraints == true ? "1" : "0";

	makefile.dictionary[ "@ARRIVAL_COST@" ] = _useArrivalCost == true ? "1" : "0";

	makefile.dictionary[ "@COV_MATRIX@" ] = _compCovMatrix == true ? "1" : "0";

	makefile.dictionary[ "@QP_SOLVER@" ] = _qpSolver;

	makefile.fillTemplate();

	return SUCCESSFUL_RETURN;
}

returnValue ExportSimulinkInterface::exportCode()
{
	makefile.exportCode();
	wrapperSource.exportCode();
	wrapperHeader.exportCode();

	return SUCCESSFUL_RETURN;
}


CLOSE_NAMESPACE_ACADO
