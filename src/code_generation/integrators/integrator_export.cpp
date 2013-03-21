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
	exportRhs = BT_TRUE;
	equidistant = BT_TRUE;
	crsFormat = BT_FALSE;

	reset_int = ExportVariable( "resetIntegrator", 1, 1, INT, ACADO_LOCAL, BT_TRUE );
	error_code = ExportVariable( "error", 1, 1, INT, ACADO_LOCAL, BT_TRUE );
}


IntegratorExport::IntegratorExport(	const IntegratorExport& arg
									) : ExportAlgorithm( arg )
{
	exportRhs = BT_TRUE;
	equidistant = BT_TRUE;
	crsFormat = BT_FALSE;
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

	if( rhs.getFunctionDim() == 0 ) {
		name_rhs = String(_name_ODE);
		name_diffs_rhs = String(_name_diffs_ODE);

		exportRhs = BT_FALSE;
	}
	else {
		return ACADOERROR( RET_INVALID_OPTION );
	}

	return SUCCESSFUL_RETURN;
}


returnValue IntegratorExport::setModelData( const ModelData& data ) {

	setDimensions( data.getNX(),data.getNDX(),data.getNXA(),data.getNU(),data.getNP(),data.getN() );
	exportRhs = data.exportRhs();

	if( exportRhs ) {
		DifferentialEquation f;
		data.getModel(f);
		Matrix M1, A1, B1;
		data.getLinearInput( M1, A1, B1 );
		Matrix M3, A3;
		OutputFcn f3;
		data.getLinearOutput( M3, A3, f3 );

		Expression rhs_, rhs3_;
		f.getExpression( rhs_ );
		f3.getExpression( rhs3_ );

		if ( M1.getNumRows() > 0 && setLinearInput( M1, A1, B1 ) != SUCCESSFUL_RETURN )
			return RET_UNABLE_TO_EXPORT_CODE;
		if ( f.getDim() > 0 && setDifferentialEquation( rhs_ ) != SUCCESSFUL_RETURN )
			return RET_UNABLE_TO_EXPORT_CODE;
		if ( M3.getNumRows() > 0 && setLinearOutput( M3, A3, rhs3_ ) != SUCCESSFUL_RETURN )
			return RET_UNABLE_TO_EXPORT_CODE;
	}
	else {
		if ( setModel( data.getNameRhs(), data.getNameDiffsRhs() ) != SUCCESSFUL_RETURN )
			return RET_UNABLE_TO_EXPORT_CODE;
	}
	Grid integrationGrid;
	data.getIntegrationGrid(integrationGrid);
	grid = integrationGrid;
	data.getNumSteps( numSteps );
	equidistant = data.hasEquidistantIntegrationGrid();

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

	setup( );

	return SUCCESSFUL_RETURN;
}



// PROTECTED:


returnValue IntegratorExport::copy(	const IntegratorExport& arg
									)
{
	exportRhs = arg.exportRhs;
	equidistant = arg.equidistant;
	crsFormat = arg.crsFormat;
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
	if( exportRhs ) {
		return rhs.getName();
	}
	else {
		return name_rhs;
	}
}

const String IntegratorExport::getNameFullRHS() const{
	return getNameRHS();
}

const String IntegratorExport::getNameOUTPUT( uint index ) const{
	if( exportRhs ) {
		return outputs[index].getName();
	}
	else {
		return name_outputs[index];
	}
}

uint IntegratorExport::getDimOUTPUT( uint index ) const{
	if( exportRhs ) {
		return outputExpressions[index].getDim();
	}
	else {
		return num_outputs[index];
	}
}


const String IntegratorExport::getNameDiffsRHS() const{
	if( exportRhs ) {
		return diffs_rhs.getName();
	}
	else {
		return name_diffs_rhs;
	}
}

const String IntegratorExport::getNameDiffsOUTPUT( uint index ) const{
	if( exportRhs ) {
		return diffs_outputs[index].getName();
	}
	else {
		return name_diffs_outputs[index];
	}
}


CLOSE_NAMESPACE_ACADO

// end of file.
