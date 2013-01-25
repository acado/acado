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
 *    \file src/code_generation/modelData.cpp
 *    \author Rien Quirynen
 *    \date 2012
 */

#include <acado/code_generation/model_data.hpp>


BEGIN_NAMESPACE_ACADO



//
// PUBLIC MEMBER FUNCTIONS:
//

ModelData::ModelData() {

	NX = 0;
	NDX = 0;
	NXA = 0;
	NU = 0;
	NP = 0;
	N  = 0;
	model_dimensions_set = BT_FALSE;
	equidistant = BT_TRUE;
}


returnValue ModelData::setDimensions( uint _NX, uint _NDX, uint _NXA, uint _NU )
{
	NX = _NX;
	NDX = _NDX;
	NXA = _NXA;
	NU = _NU;
	model_dimensions_set = BT_TRUE;
	return SUCCESSFUL_RETURN;
}


returnValue ModelData::setDimensions( uint _NX, uint _NU )
{
	setDimensions( _NX, 0, 0, _NU );
	return SUCCESSFUL_RETURN;
}


returnValue ModelData::addOutput( const OutputFcn& outputEquation_ ){

	if( rhs_name.isEmpty() && outputNames.size() == 0 ) {
		Expression next;
		outputEquation_.getExpression( next );
		outputExpressions.push_back( next );
		dim_outputs.push_back( next.getDim() );
	}
	else {
		return ACADOERROR( RET_INVALID_OPTION );
	}

    return SUCCESSFUL_RETURN;
}


returnValue ModelData::addOutput( const String& output, const String& diffs_output, const uint dim ){

	if( outputExpressions.size() == 0 && differentialEquation.getNumDynamicEquations() == 0 ) {
		outputNames.push_back( output );
		diffs_outputNames.push_back( diffs_output );
		dim_outputs.push_back( dim );
	}
	else {
		return ACADOERROR( RET_INVALID_OPTION );
	}

    return SUCCESSFUL_RETURN;
}


returnValue ModelData::addOutput( 	const String& output, const String& diffs_output, const uint dim,
									const String& colInd, const String& rowPtr	){


	Vector colIndV = readFromFile( colInd.getName() );
	Vector rowPtrV = readFromFile( rowPtr.getName() );
	if( rowPtrV.getDim() != (dim+1) ) {
		return ACADOERROR( RET_INVALID_OPTION );
	}
	colInd_outputs.push_back( colIndV );
	rowPtr_outputs.push_back( rowPtrV );

	addOutput( output, diffs_output, dim );

    return SUCCESSFUL_RETURN;
}


BooleanType ModelData::hasOutputs() const{

	if( outputExpressions.size() == 0 && outputNames.size() == 0 ) return BT_FALSE;
	return BT_TRUE;
}


returnValue ModelData::setMeasurements( const Vector& numberMeasurements ){

	int i;
	if( outputExpressions.size() != numberMeasurements.getDim() && outputNames.size() != numberMeasurements.getDim() ) {
		return ACADOERROR( RET_INVALID_OPTION );
	}
	outputGrids.clear();
	num_meas.clear();
	for( i = 0; i < (int)numberMeasurements.getDim(); i++ ) {
		Grid nextGrid( 0.0, 1.0, (int)numberMeasurements(i) + 1 );
		outputGrids.push_back( nextGrid );

		uint numOuts = (int) ceil((double)outputGrids[i].getNumIntervals()/((double) N) - 10.0*EPS);
		num_meas.push_back( numOuts );
	}

    return SUCCESSFUL_RETURN;
}


returnValue ModelData::getNumSteps( Vector& _numSteps ) const {

    _numSteps = numSteps;
    return SUCCESSFUL_RETURN;
}


returnValue ModelData::setNumSteps( const Vector& _numSteps ) {

    numSteps = _numSteps;
    return SUCCESSFUL_RETURN;
}


returnValue ModelData::getOutputExpressions( std::vector<Expression>& outputExpressions_ ) const {

    outputExpressions_ = outputExpressions;
    return SUCCESSFUL_RETURN;
}


std::vector<Matrix> ModelData::getOutputDependencies( ) const {
	std::vector<Matrix> outputDependencies;
	if( hasCompressedStorage() ) {
		for( int i = 0; i < outputNames.size(); i++ ) {
			Vector colIndV = colInd_outputs[i];
			Vector rowPtrV = rowPtr_outputs[i];

			Matrix dependencyMat = zeros( dim_outputs[i],NX+NXA+NU+NDX );
			int index = 1;
			for( int j = 0; j < dim_outputs[i]; j++ ) {
				int upper = rowPtrV(j+1);
				for( int k = rowPtrV(j); k < upper; k++ ) {
					dependencyMat(j,colIndV(k-1)-1) = index++;
				}
			}

			outputDependencies.push_back( dependencyMat );
		}
	}
	return outputDependencies;
}


returnValue ModelData::getOutputGrids( std::vector<Grid>& outputGrids_ ) const {

    outputGrids_ = outputGrids;
    return SUCCESSFUL_RETURN;
}


returnValue ModelData::getModel( DifferentialEquation& _f ) const{

    _f = differentialEquation;
    return SUCCESSFUL_RETURN;
}


returnValue ModelData::setModel( const DifferentialEquation& _f )
{
	if( rhs_name.isEmpty() ) {
		differentialEquation = _f;
		Expression rhs;
		differentialEquation.getExpression( rhs );

		NX = rhs.getDim() - differentialEquation.getNXA();
		NDX = differentialEquation.getNDX();
		NXA = differentialEquation.getNXA();
		NU = differentialEquation.getNU();
		NP = differentialEquation.getNP();

		model_dimensions_set = BT_TRUE;

		export_rhs = BT_TRUE;
	}
	else {
		return ACADOERROR( RET_INVALID_OPTION );
	}
	return SUCCESSFUL_RETURN;
}


returnValue ModelData::setModel( const String& fileName, const String& _rhs_ODE, const String& _diffs_ODE )
{
	if( differentialEquation.getNumDynamicEquations() == 0 ) {
		externModel = String(fileName);
		rhs_name = String(_rhs_ODE);
		diffs_name = String(_diffs_ODE);

		export_rhs = BT_FALSE;
	}
	else {
		return ACADOERROR( RET_INVALID_OPTION );
	}

	return SUCCESSFUL_RETURN;
}


returnValue ModelData::getIntegrationGrid( Grid& integrationGrid_ ) const
{
	integrationGrid_ = integrationGrid;
	return SUCCESSFUL_RETURN;
}


returnValue ModelData::setIntegrationGrid( const Vector& gridPoints )
{
	integrationGrid = Grid(gridPoints);
	equidistant = BT_FALSE;
	return SUCCESSFUL_RETURN;
}


returnValue ModelData::setIntegrationGrid(	const Grid& _ocpGrid, const uint _numSteps
										)
{
	uint i;
	N = _ocpGrid.getNumIntervals();
	BooleanType equidistant = _ocpGrid.isEquidistant();
	double T = _ocpGrid.getLastTime() - _ocpGrid.getFirstTime();
	double h = T/((double)_numSteps);
	Vector stepsVector( N );

	for( i = 0; i < stepsVector.getDim(); i++ )
	{
		stepsVector(i) = (int) ceil((_ocpGrid.getTime(i+1)-_ocpGrid.getTime(i))/h - 10.0*EPS);
	}

	if( equidistant )
	{
		// Setup fixed integrator grid for equidistant control grid
		integrationGrid = Grid( 0.0, ((double) T)/((double) N), (int) ceil((double)_numSteps/((double) N) - 10.0*EPS) + 1 );
	}
	else
	{
		// Setup for non equidistant control grid
		// NOTE: This grid defines only one integration step because the control
		// grid is non equidistant.
		integrationGrid = Grid( 0.0, h, 2 );
		numSteps = stepsVector;
	}
	return SUCCESSFUL_RETURN;
}


BooleanType ModelData::hasEquidistantIntegrationGrid(  ) const
{
	return equidistant;
}


BooleanType ModelData::hasOutputFunctions() const {

    if( outputExpressions.size() == 0 ) return BT_FALSE;
    return BT_TRUE;
}


BooleanType ModelData::hasDifferentialEquation() const {

    if( differentialEquation.getDim() == 0 ) return BT_FALSE;
    return BT_TRUE;
}


BooleanType ModelData::modelDimensionsSet() const {

    return model_dimensions_set;
}


BooleanType ModelData::exportRhs() const {

    return export_rhs;
}


BooleanType ModelData::hasCompressedStorage() const {

	if( colInd_outputs.size() == 0 ) return BT_FALSE;
    return BT_TRUE;
}


uint ModelData::getNX( ) const
{
	return NX;
}


uint ModelData::getNDX( ) const
{
	return NDX;
}


uint ModelData::getNXA( ) const
{
	return NXA;
}


uint ModelData::getNU( ) const
{
	return NU;
}


uint ModelData::getNP( ) const
{
	return NP;
}


uint ModelData::getN( ) const
{
	return N;
}


returnValue ModelData::setN( const uint N_ )
{
	N = N_;
	return SUCCESSFUL_RETURN;
}


Vector ModelData::getDimOutputs( ) const
{
	Vector nOutV( dim_outputs.size() );
	for( uint i = 0; i < (int)dim_outputs.size(); i++ ) {
		nOutV(i) = dim_outputs[i];
	}
	return nOutV;
}


uint ModelData::getNumOutputs( ) const
{
	return outputGrids.size();
}


returnValue ModelData::getDimOutputs( std::vector<uint>& dims ) const
{
	dims = dim_outputs;
	return SUCCESSFUL_RETURN;
}


Vector ModelData::getNumMeas( ) const
{
	Vector nMeasV( num_meas.size() );
	for( uint i = 0; i < (int)num_meas.size(); i++ ) {
		nMeasV(i) = num_meas[i];
	}
	return nMeasV;
}


const String ModelData::getFileNameModel() const {
	return externModel;
}


const String ModelData::getNameRhs() const {
	return rhs_name;
}


const String ModelData::getNameDiffsRhs() const {
	return diffs_name;
}


returnValue ModelData::getNameOutputs( std::vector<String>& names ) const {
	names = outputNames;
	return SUCCESSFUL_RETURN;
}


returnValue ModelData::getNameDiffsOutputs( std::vector<String>& names ) const {
	names = diffs_outputNames;
	return SUCCESSFUL_RETURN;
}



// PROTECTED:




CLOSE_NAMESPACE_ACADO

// end of file.
