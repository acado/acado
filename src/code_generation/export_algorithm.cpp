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
 *    \file src/code_generation/export_algorithm.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 2010-2011
 */

#include <acado/code_generation/export_algorithm.hpp>



BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

ExportAlgorithm::ExportAlgorithm(	UserInteraction* _userInteraction,
									const String& _commonHeaderName
									) : AlgorithmicBase( _userInteraction )
{
	setDimensions( 0,0,0,0 );
	commonHeaderName = _commonHeaderName;
}


ExportAlgorithm::ExportAlgorithm(	const ExportAlgorithm& arg
									) : AlgorithmicBase( arg )
{
	copy( arg );
}


ExportAlgorithm::~ExportAlgorithm( )
{
}


ExportAlgorithm& ExportAlgorithm::operator=(	const ExportAlgorithm& arg
												)
{
	if( this != &arg )
	{
		AlgorithmicBase::operator=( arg );
		copy( arg );
	}
	
	return *this;
}



returnValue ExportAlgorithm::setup( )
{
	return SUCCESSFUL_RETURN;
}



returnValue ExportAlgorithm::setDimensions(	uint _NX,
											uint _NU,
											uint _NP,
											uint _NI
											)
{
	return setDimensions( _NX, 0, 0, _NU, _NP, _NI );
}



returnValue ExportAlgorithm::setDimensions(	uint _NX,
											uint _NDX,
											uint _NXA,
											uint _NU,
											uint _NP,
											uint _NI
											)
{
	NX = _NX;
	NDX = _NDX;
	NXA = _NXA;
	NU = _NU;
	NP = _NP;
	N  = _NI;

	return SUCCESSFUL_RETURN;
}



uint ExportAlgorithm::getNX( ) const
{
	return NX;
}


uint ExportAlgorithm::getNXA( ) const
{
	return NXA;
}


uint ExportAlgorithm::getNU( ) const
{
	return NU;
}


uint ExportAlgorithm::getNP( ) const
{
	return NP;
}


uint ExportAlgorithm::getN( ) const
{
	return N;
}

void ExportAlgorithm::setNY( uint NY_ )
{
	NY = NY_;
}

uint ExportAlgorithm::getNY( ) const
{
	return NY;
}

void ExportAlgorithm::setNYN( uint NYN_ )
{
	NYN = NYN_;
}

uint ExportAlgorithm::getNYN( ) const
{
	return NYN;
}



FILE* ExportAlgorithm::openFile(	const String& dirName,
									const String& fileName
									) const
{
	String fullFileName( dirName );
    fullFileName << "/" << fileName;

    FILE* file = acadoFOpen( fullFileName.getName(), "w" );
	if ( file == 0 )
		return 0;

	acadoPrintAutoGenerationNotice( file );

	if ( commonHeaderName.isEmpty() == BT_FALSE )
		acadoFPrintf( file, "#include \"%s\"\n\n\n",commonHeaderName.getName() );
	
	return file;
}



//
// PROTECTED MEMBER FUNCTIONS:
//


returnValue ExportAlgorithm::copy(	const ExportAlgorithm& arg
									)
{
	NX = arg.NX;
	NXA = arg.NXA;
	NU = arg.NU;
	NP = arg.NP;
	N  = arg.N;
	
	NY = arg.NY;

	commonHeaderName = arg.commonHeaderName;

	return SUCCESSFUL_RETURN;
}


CLOSE_NAMESPACE_ACADO

// end of file.
