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
 *    \file ...
 *    \author Rien Quirynen
 *    \date 2012
 */

#include <acado/code_generation/integrators/export_matlab_rhs.hpp>


BEGIN_NAMESPACE_ACADO

using namespace std;


//
// PUBLIC MEMBER FUNCTIONS:
//

ExportMatlabRhs::ExportMatlabRhs(	const String& _templateName,
												const String& _fileName,
												const String& _commonHeaderName,
												const String& _realString,
												const String& _intString,
												int _precision,
												const String& _commentString
						) : ExportTemplatedFile(_templateName, _fileName, _commonHeaderName, _realString, _intString, _precision, _commentString)
{}


ExportMatlabRhs::ExportMatlabRhs(	const ExportMatlabRhs& arg
						) : ExportTemplatedFile( arg )
{}


ExportMatlabRhs::~ExportMatlabRhs( )
{}


ExportMatlabRhs& ExportMatlabRhs::operator=(	const ExportMatlabRhs& arg
														)
{
	if( this != &arg )
	{
		ExportTemplatedFile::operator=( arg );
	}
	
	return *this;
}

returnValue ExportMatlabRhs::configure(	const String& nameFunction )
{	
	// Configure the dictionary
	dictionary[ "@ACADO_RHS@" ] =  std::string(nameFunction.getName());
	
	// And then fill a template file
	fillTemplate();

	return SUCCESSFUL_RETURN;
}

//
// PROTECTED MEMBER FUNCTIONS:
//

CLOSE_NAMESPACE_ACADO

// end of file.
