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
 *    \file ...
 *    \author Rien Quirynen
 *    \date 2012
 */

#include <acado/code_generation/integrators/export_matlab_integrator.hpp>


BEGIN_NAMESPACE_ACADO

using namespace std;


//
// PUBLIC MEMBER FUNCTIONS:
//

ExportMatlabIntegrator::ExportMatlabIntegrator(	const std::string& _templateName,
												const std::string& _fileName,
												const std::string& _commonHeaderName,
												const std::string& _realString,
												const std::string& _intString,
												int _precision,
												const std::string& _commentString
						) : ExportTemplatedFile(_templateName, _fileName, _commonHeaderName, _realString, _intString, _precision, _commentString)
{}


ExportMatlabIntegrator::ExportMatlabIntegrator(	const ExportMatlabIntegrator& arg
						) : ExportTemplatedFile( arg )
{}


ExportMatlabIntegrator::~ExportMatlabIntegrator( )
{}


ExportMatlabIntegrator& ExportMatlabIntegrator::operator=(	const ExportMatlabIntegrator& arg
														)
{
	if( this != &arg )
	{
		ExportTemplatedFile::operator=( arg );
	}
	
	return *this;
}

returnValue ExportMatlabIntegrator::configure(	const uint sensProp, const uint online, const uint debugMode, const uint timingCalls, const uint numStages )
{	
	// Configure the dictionary
	dictionary[ "@SENS_PROP@" ] =  toString(sensProp);
	dictionary[ "@ONLINE_GRID@" ] =  toString(online);
	dictionary[ "@DEBUG_MODE@" ] =  toString(debugMode);
	if( timingCalls > 1 ) 	dictionary[ "@CALLS_TIMING@" ] =  toString(timingCalls);
	else 					dictionary[ "@CALLS_TIMING@" ] =  toString(1);
	dictionary[ "@NUM_STAGES@" ] =  toString(numStages);
	dictionary[ "@MODULE_NAME@" ] = ExportStatement::fcnPrefix;
	dictionary[ "@MODULE_PREFIX@" ] = ExportStatement::varPrefix;
	
	// And then fill a template file
	fillTemplate();

	return SUCCESSFUL_RETURN;
}

//
// PROTECTED MEMBER FUNCTIONS:
//

CLOSE_NAMESPACE_ACADO

// end of file.
