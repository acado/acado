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
 *    \file src/code_generation/integrator_export.cpp
 *    \author Hans Joachim Ferreau, Boris Houska, Rien Quirynen
 *    \date 2010-2011
 */

#include <acado/code_generation/integrators/integrator_export.hpp>



BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

IntegratorExport::IntegratorExport(	UserInteraction* _userInteraction,
									const String& _commonHeaderName
									) : ExportAlgorithm( _userInteraction,_commonHeaderName )
{
	EXPORT_RHS = BT_TRUE;
	EQUIDISTANT = BT_TRUE;
	CRS_FORMAT = BT_FALSE;

	reset_int = ExportVariable( "resetIntegrator", 1, 1, INT, ACADO_VARIABLES, BT_TRUE );
}


IntegratorExport::IntegratorExport(	const IntegratorExport& arg
									) : ExportAlgorithm( arg )
{
	EXPORT_RHS = BT_TRUE;
	EQUIDISTANT = BT_TRUE;
	CRS_FORMAT = BT_FALSE;
}


IntegratorExport::~IntegratorExport( )
{
	clear( );
}


IntegratorExport& IntegratorExport::operator=(	const IntegratorExport& arg
												)
{
	if( this != &arg && &arg != 0 )
	{
		clear( );
		ExportAlgorithm::operator=( arg );
	}
    return *this;
}


returnValue IntegratorExport::setGrid(	const Grid& _grid )
{
	grid = _grid;

	return SUCCESSFUL_RETURN;
}


returnValue IntegratorExport::setModel(	const String& _name_ODE, const String& _name_diffs_ODE ) {

	if( RHS.getFunctionDim() == 0 ) {
		name_RHS = String(_name_ODE);
		name_diffs_RHS = String(_name_diffs_ODE);

		EXPORT_RHS = BT_FALSE;
	}
	else {
		return ACADOERROR( RET_INVALID_OPTION );
	}

	return SUCCESSFUL_RETURN;
}


returnValue IntegratorExport::setModelData( const ModelData& data ) {

	setDimensions( data.getNX(),data.getNDX(),data.getNXA(),data.getNU(),data.getNP(),data.getN() );
	EXPORT_RHS = data.exportRhs();

	if( EXPORT_RHS ) {
		DifferentialEquation f;
		data.getModel(f);
		Expression rhs;
		f.getExpression( rhs );
		if ( setDifferentialEquation( rhs ) != SUCCESSFUL_RETURN )
			return RET_UNABLE_TO_EXPORT_CODE;
	}
	else {
		if ( setModel( data.getNameRhs(), data.getNameDiffsRhs() ) != SUCCESSFUL_RETURN )
			return RET_UNABLE_TO_EXPORT_CODE;
	}
	Grid integrationGrid;
	data.getIntegrationGrid(integrationGrid);
	grid = integrationGrid;
	EQUIDISTANT = data.hasEquidistantIntegrationGrid();

	setup( );

	if( data.hasOutputs() ) {
		uint i;

		std::vector<Grid> newGrids_;
		std::vector<Grid> outputGrids_;
		data.getOutputGrids(outputGrids_);
		std::vector<Expression> outputExpressions_;
		data.getOutputExpressions(outputExpressions_);
		for( i = 0; i < outputGrids_.size(); i++ ) {
			uint numOuts = (int) ceil((double)outputGrids_[i].getNumIntervals()/((double) data.getN()) - 10.0*EPS);
			Grid nextGrid( 0.0, 1.0, numOuts + 1 );
			newGrids_.push_back( nextGrid );
		}

		if( outputExpressions_.size() > 0 ) {
			setupOutput( newGrids_, outputExpressions_ );
		}
		else {
			std::vector<String> outputNames;
			std::vector<String> diffs_outputNames;
			std::vector<uint> dim_outputs;
			std::vector<Matrix> outputDependencies_ = data.getOutputDependencies();
			data.getNameOutputs(outputNames);
			data.getNameDiffsOutputs(diffs_outputNames);
			data.getDimOutputs(dim_outputs);
			if( !data.hasCompressedStorage() ) {
				setupOutput( newGrids_, outputNames, diffs_outputNames, dim_outputs );
			}
			else {
				setupOutput( newGrids_, outputNames, diffs_outputNames, dim_outputs, outputDependencies_ );
			}
		}
	}

	return SUCCESSFUL_RETURN;
}



// PROTECTED:


returnValue IntegratorExport::copy(	const IntegratorExport& arg
									)
{
	EXPORT_RHS = arg.EXPORT_RHS;
	EQUIDISTANT = arg.EQUIDISTANT;
	CRS_FORMAT = arg.CRS_FORMAT;
	grid = arg.grid;
	numSteps = arg.numSteps;

	// ExportFunctions
	integrate = arg.integrate;
	
	return SUCCESSFUL_RETURN;
}


returnValue IntegratorExport::clear( )
{
	return SUCCESSFUL_RETURN;
}


uint IntegratorExport::getIntegrationInterval( double time ) {
	uint index = 0;
	double scale = 1.0/(grid.getLastTime() - grid.getFirstTime());
	while( index < (grid.getNumIntervals()-1) && time > scale*grid.getTime( index+1 ) ) {
		index++;
	}
	return index;
}


returnValue IntegratorExport::getGrid( Grid& grid_ ) const{

    grid_ = grid;
    return SUCCESSFUL_RETURN;
}


returnValue IntegratorExport::getNumSteps( Vector& _numSteps ) const{

    _numSteps = numSteps;
    return SUCCESSFUL_RETURN;
}


returnValue IntegratorExport::getOutputExpressions( std::vector<Expression>& outputExpressions_ ) const{

    outputExpressions_ = outputExpressions;
    return SUCCESSFUL_RETURN;
}


returnValue IntegratorExport::getOutputGrids( std::vector<Grid>& outputGrids_ ) const{

    outputGrids_ = outputGrids;
    return SUCCESSFUL_RETURN;
}

BooleanType IntegratorExport::equidistantControlGrid( ) const{
	
	return numSteps.isEmpty();
}

const String IntegratorExport::getNameRHS() const{
	if( EXPORT_RHS ) {
		return RHS.getName();
	}
	else {
		return name_RHS;
	}
}

const String IntegratorExport::getNameOUTPUT( uint index ) const{
	if( EXPORT_RHS ) {
		return OUTPUTS[index].getName();
	}
	else {
		return name_OUTPUTS[index];
	}
}

uint IntegratorExport::getDimOUTPUT( uint index ) const{
	if( EXPORT_RHS ) {
		return outputExpressions[index].getDim();
	}
	else {
		return num_OUTPUTS[index];
	}
}


const String IntegratorExport::getNameDiffsRHS() const{
	if( EXPORT_RHS ) {
		return diffs_RHS.getName();
	}
	else {
		return name_diffs_RHS;
	}
}

const String IntegratorExport::getNameDiffsOUTPUT( uint index ) const{
	if( EXPORT_RHS ) {
		return diffs_OUTPUTS[index].getName();
	}
	else {
		return name_diffs_OUTPUTS[index];
	}
}


CLOSE_NAMESPACE_ACADO

// end of file.
