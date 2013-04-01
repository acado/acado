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
 *    \file src/code_generation/integrators/narx_export.cpp
 *    \author Rien Quirynen
 *    \date 2013
 */

#include <acado/code_generation/integrators/narx_export.hpp>

#include <sstream>
using namespace std;

BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

NARXExport::NARXExport(	UserInteraction* _userInteraction,
									const String& _commonHeaderName
									) : DiscreteTimeExport( _userInteraction,_commonHeaderName )
{
}


NARXExport::NARXExport(	const NARXExport& arg
									) : DiscreteTimeExport( arg )
{
	copy( arg );
}


NARXExport::~NARXExport( )
{
	clear( );
}


returnValue NARXExport::setup( )
{
	// non equidistant integration grids not implemented for NARX integrators
	if( !equidistant ) return ACADOERROR( RET_INVALID_OPTION );

	String fileName( "integrator.c" );

	int printLevel;
	get( PRINTLEVEL,printLevel );

	if ( (PrintLevel)printLevel >= HIGH ) 
		acadoPrintf( "--> Preparing to export %s... ",fileName.getName() );

	// TODO: ADD NARX STUFF
	
	if ( (PrintLevel)printLevel >= HIGH ) 
		acadoPrintf( "done.\n" );	

	return SUCCESSFUL_RETURN;
}



returnValue NARXExport::setDifferentialEquation(	const Expression& rhs_ )
{
	// TODO: ADD NARX STUFF

	return SUCCESSFUL_RETURN;
}


returnValue NARXExport::setLinearInput( const Matrix& M1, const Matrix& A1, const Matrix& B1 ) {

	// TODO: ADD NARX STUFF

	return ACADOERROR( RET_INVALID_OPTION );
}


returnValue NARXExport::setLinearOutput( const Matrix& M3, const Matrix& A3, const Expression& rhs ) {

	return ACADOERROR( RET_INVALID_OPTION );
}


returnValue NARXExport::setModel(	const String& _rhs, const String& _diffs_rhs ) {

	// You can't use this feature yet with NARX integrators !
	return ACADOERROR( RET_INVALID_OPTION );
}


returnValue NARXExport::getDataDeclarations(	ExportStatementBlock& declarations,
													ExportStruct dataStruct
													) const
{
	ExportVariable max = rhs.getGlobalExportVariable();
	if( max.getDim() >= diffs_rhs.getGlobalExportVariable().getDim() ) {
		declarations.addDeclaration( max,dataStruct );
	}
	else {
		declarations.addDeclaration( diffs_rhs.getGlobalExportVariable(),dataStruct );
	}
	declarations.addDeclaration( rk_xxx,dataStruct );
	declarations.addDeclaration( reset_int,dataStruct );

	return SUCCESSFUL_RETURN;
}


returnValue NARXExport::getFunctionDeclarations(	ExportStatementBlock& declarations
														) const
{
	declarations.addDeclaration( integrate );
	declarations.addDeclaration( rhs );
	declarations.addDeclaration( diffs_rhs );

	return SUCCESSFUL_RETURN;
}


returnValue NARXExport::getCode(	ExportStatementBlock& code
										)
{
	code.addFunction( rhs );
	code.addFunction( diffs_rhs );
	code.addFunction( integrate );

	return SUCCESSFUL_RETURN;
}


returnValue NARXExport::setupOutput( const std::vector<Grid> outputGrids_, const std::vector<Expression> rhs ) {
	
	return ACADOERROR( RET_INVALID_OPTION );
}


returnValue NARXExport::setupOutput(  const std::vector<Grid> outputGrids_,
									  	  	  	  	const std::vector<String> _outputNames,
									  	  	  	  	const std::vector<String> _diffs_outputNames,
									  	  	  	  	const std::vector<uint> _dims_output ) {

	return ACADOERROR( RET_INVALID_OPTION );
}


returnValue NARXExport::setupOutput(  const std::vector<Grid> outputGrids_,
									  	  	  	  	const std::vector<String> _outputNames,
									  	  	  	  	const std::vector<String> _diffs_outputNames,
									  	  	  	  	const std::vector<uint> _dims_output,
									  	  	  	  	const std::vector<Matrix> _outputDependencies ) {

	return ACADOERROR( RET_INVALID_OPTION );
}



// PROTECTED:



CLOSE_NAMESPACE_ACADO

// end of file.
