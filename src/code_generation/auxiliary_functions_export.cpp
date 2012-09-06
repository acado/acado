/*
 *    This file is part of ACADO Toolkit.
 *
 *    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
 *    Copyright (C) 2008-2009 by Boris Houska and Hans Joachim Ferreau, K.U.Leuven.
 *    Developed within the Optimization in Engineering Center (OPTEC) under
 *    supervision of Moritz Diehl. All rights reserved.
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
 *    \file src/code_generation/auxiliary_functions_export.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 2010-2011
 */

#include <acado/code_generation/auxiliary_functions_export.hpp>



BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

AuxiliaryFunctionsExport::AuxiliaryFunctionsExport(	UserInteraction* _userInteraction,
													const String& _commonHeaderName
													) : ExportAlgorithm( _userInteraction,_commonHeaderName )
{
}


AuxiliaryFunctionsExport::AuxiliaryFunctionsExport(	const AuxiliaryFunctionsExport& arg
													) : ExportAlgorithm( arg )
{
	copy( arg );
}


AuxiliaryFunctionsExport::~AuxiliaryFunctionsExport( )
{
	clear( );
}


AuxiliaryFunctionsExport& AuxiliaryFunctionsExport::operator=(	const AuxiliaryFunctionsExport& arg
																)
{
	if( this != &arg )
	{
		clear( );
		ExportAlgorithm::operator=( arg );
		copy( arg );
	}

	return *this;
}



returnValue AuxiliaryFunctionsExport::setup( )
{
	String fileName( "auxiliary_functions.c" );

	int printLevel;
	get( PRINTLEVEL,printLevel );

	if ( (PrintLevel)printLevel >= HIGH ) 
		acadoPrintf( "--> Preparing to export %s... ",fileName.getName() );


	ExportVariable x   ( "x",    N+1,NX, REAL,ACADO_VARIABLES );
	ExportVariable u   ( "u",    N,NU,   REAL,ACADO_VARIABLES );
	ExportVariable p   ( "p",    1,NP,   REAL,ACADO_VARIABLES );
	ExportVariable xRef( "xRef", N+1,NX, REAL,ACADO_VARIABLES );
	ExportVariable uRef( "uRef", N,NU,   REAL,ACADO_VARIABLES );


	getAcadoVariablesX.setup( "getAcadoVariablesX" );
	getAcadoVariablesX.setReturnValue( x,BT_TRUE );

	getAcadoVariablesU.setup( "getAcadoVariablesU" );
	getAcadoVariablesU.setReturnValue( u,BT_TRUE );

	getAcadoVariablesXRef.setup( "getAcadoVariablesXRef" );
	getAcadoVariablesXRef.setReturnValue( xRef,BT_TRUE );

	getAcadoVariablesURef.setup( "getAcadoVariablesURef" );
	getAcadoVariablesURef.setReturnValue( uRef,BT_TRUE );


	printStates.setup( "printStates" );
	printStates.addStatement( ExportPrintf(x.accessTransposed()) );

	printControls.setup( "printControls" );
	printControls.addStatement( ExportPrintf(u.accessTransposed()) );


	int operatingSystem;
	get( OPERATING_SYSTEM,operatingSystem );

	getTime.setup( "getTime" );
	ExportVariable currentTime( "current_time",(Matrix)0.0 );
	currentTime.callByValue();

	getTime.addDeclaration( currentTime );

	if ( (OperatingSystem)operatingSystem == OS_WINDOWS )
	{
		getTime.addStatement( "LARGE_INTEGER counter, frequency;\n" );
		getTime.addStatement( "QueryPerformanceFrequency(&frequency);\n" );
		getTime.addStatement( "QueryPerformanceCounter(&counter);\n" );
		getTime.addStatement( "current_time = ((real_t) counter.QuadPart) / ((real_t) frequency.QuadPart);\n" );
	}
	else
	{
		// OS_UNIX
		getTime.addStatement( "struct timeval theclock;\n" );
		getTime.addStatement( "gettimeofday( &theclock,0 );\n" );
		getTime.addStatement( "current_time = 1.0*theclock.tv_sec + 1.0e-6*theclock.tv_usec;\n" );
	}

	getTime.setReturnValue( currentTime );


	printHeader.setup( "printHeader" );
	printHeader.addStatement( "    printf(\"\\nACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.\\nCopyright (C) 2008-2011 by Boris Houska and Hans Joachim Ferreau, K.U.Leuven.\\nDeveloped within the Optimization in Engineering Center (OPTEC) under\\nsupervision of Moritz Diehl. All rights reserved.\\n\\nACADO Toolkit is distributed under the terms of the GNU Lesser\\nGeneral Public License 3 in the hope that it will be useful,\\nbut WITHOUT ANY WARRANTY; without even the implied warranty of\\nMERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the\\nGNU Lesser General Public License for more details.\\n\\n\" );\n" );

	if ( (PrintLevel)printLevel >= HIGH ) 
		acadoPrintf( "done.\n" );

	return SUCCESSFUL_RETURN;
}



returnValue AuxiliaryFunctionsExport::getDataDeclarations(	ExportStatementBlock& declarations,
															ExportStruct dataStruct
															) const
{
	return SUCCESSFUL_RETURN;
}


returnValue AuxiliaryFunctionsExport::getFunctionDeclarations(	ExportStatementBlock& declarations
																) const
{
	declarations.addDeclaration( getAcadoVariablesX );
	declarations.addDeclaration( getAcadoVariablesU );
	declarations.addDeclaration( getAcadoVariablesXRef );
	declarations.addDeclaration( getAcadoVariablesURef );

	declarations.addDeclaration( printStates );
	declarations.addDeclaration( printControls );
	declarations.addDeclaration( getTime );
	declarations.addDeclaration( printHeader );

    return SUCCESSFUL_RETURN;
}


returnValue AuxiliaryFunctionsExport::getCode(	ExportStatementBlock& code
												)
{
// 	int printLevel;
// 	get( PRINTLEVEL,printLevel );
// 
// 	if ( (PrintLevel)printLevel >= HIGH ) 
// 		acadoPrintf( "--> Exporting %s... ",fileName.getName() );

	code.addFunction( getAcadoVariablesX );
	code.addFunction( getAcadoVariablesU );
	code.addFunction( getAcadoVariablesXRef );
	code.addFunction( getAcadoVariablesURef );
	
	code.addFunction( printStates );
	code.addFunction( printControls );
	code.addFunction( getTime );
	code.addFunction( printHeader );

// 	if ( (PrintLevel)printLevel >= HIGH ) 
// 		acadoPrintf( "done.\n" );

	return SUCCESSFUL_RETURN;
}



//
// PROTECTED MEMBER FUNCTIONS:
//


returnValue AuxiliaryFunctionsExport::copy(	const AuxiliaryFunctionsExport& arg
											)
{
	getAcadoVariablesX    = arg.getAcadoVariablesX;
	getAcadoVariablesU    = arg.getAcadoVariablesU;
	getAcadoVariablesXRef = arg.getAcadoVariablesXRef;
	getAcadoVariablesURef = arg.getAcadoVariablesURef;
	
	printStates           = arg.printStates;
	printControls         = arg.printControls;
	getTime               = arg.getTime;
	printHeader           = arg.printHeader;

	return SUCCESSFUL_RETURN;
}


returnValue AuxiliaryFunctionsExport::clear( )
{
	return SUCCESSFUL_RETURN;
}


CLOSE_NAMESPACE_ACADO

// end of file.
