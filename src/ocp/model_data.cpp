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
 *    \file src/ocp/modelData.cpp
 *    \author Rien Quirynen
 *    \date 2012
 */

#include <acado/ocp/model_data.hpp>


BEGIN_NAMESPACE_ACADO



//
// PUBLIC MEMBER FUNCTIONS:
//

ModelData::ModelData() {

	NX1 = 0;
	NX2 = 0;
	NX3 = 0;
	NDX = 0;
	NDX3 = 0;
	NXA = 0;
	NXA3 = 0;
	NU = 0;
	NP = 0;
	N  = 0;
	model_dimensions_set = BT_FALSE;
	export_rhs = BT_TRUE;
	delay = 1;
}


returnValue ModelData::setDimensions( uint _NX1, uint _NX2, uint _NX3, uint _NDX, uint _NDX3, uint _NXA, uint _NXA3, uint _NU, uint _NP )
{
	NX1 = _NX1;
	NX2 = _NX2;
	NX3 = _NX3;
	NDX = _NDX;
	NDX3 = _NDX3;
	NXA = _NXA;
	NXA3 = _NXA3;
	NU = _NU;
	NP = _NP;
	model_dimensions_set = BT_TRUE;
	return SUCCESSFUL_RETURN;
}


uint ModelData::addOutput( const OutputFcn& outputEquation_, const Grid& grid ){

	if( rhs_name.isEmpty() && outputNames.size() == 0 ) {
		Expression next;
		outputEquation_.getExpression( next );
		outputExpressions.push_back( next );
		dim_outputs.push_back( next.getDim() );

		outputGrids.push_back( grid );

		uint numOuts = (int) ceil((double)grid.getNumIntervals());
		num_meas.push_back( numOuts );
	}
	else {
		return ACADOERROR( RET_INVALID_OPTION );
	}

	return dim_outputs.size();
}


uint ModelData::addOutput( const String& output, const String& diffs_output, const uint dim, const Grid& grid ){

	if( outputExpressions.size() == 0 && differentialEquation.getNumDynamicEquations() == 0 ) {
		outputNames.push_back( output );
		diffs_outputNames.push_back( diffs_output );
		dim_outputs.push_back( dim );

		outputGrids.push_back( grid );

		uint numOuts = (int) ceil((double)grid.getNumIntervals());
		num_meas.push_back( numOuts );
	}
	else {
		return ACADOERROR( RET_INVALID_OPTION );
	}

	return dim_outputs.size();
}


uint ModelData::addOutput( 	const String& output, const String& diffs_output, const uint dim,
							const Grid& grid, const String& colInd, const String& rowPtr	){


	Vector colIndV = readFromFile( colInd.getName() );
	Vector rowPtrV = readFromFile( rowPtr.getName() );
	if( rowPtrV.getDim() != (dim+1) ) {
		return ACADOERROR( RET_INVALID_OPTION );
	}
	colInd_outputs.push_back( colIndV );
	rowPtr_outputs.push_back( rowPtrV );

    return addOutput( output, diffs_output, dim, grid );
}


BooleanType ModelData::hasOutputs() const{

	if( outputExpressions.size() == 0 && outputNames.size() == 0 ) return BT_FALSE;
	return BT_TRUE;
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
		for( uint i = 0; i < outputNames.size(); i++ ) {
			Vector colIndV = colInd_outputs[i];
			Vector rowPtrV = rowPtr_outputs[i];

			Matrix dependencyMat = zeros( dim_outputs[i],getNX()+NXA+NU+NDX );
			int index = 1;
			for( uint j = 0; j < dim_outputs[i]; j++ ) {
				uint upper = (uint)rowPtrV(j+1);
				for( uint k = (uint)rowPtrV(j); k < upper; k++ ) {
					dependencyMat(j,(uint)colIndV(k-1)-1) = index++;
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


returnValue ModelData::getNARXmodel( uint& _delay, Matrix& _parms ) const{

    _delay = delay;
    _parms = parms;

    return SUCCESSFUL_RETURN;
}


returnValue ModelData::getLinearInput( Matrix& M1_, Matrix& A1_, Matrix& B1_ ) const {
	M1_ = M1;
	A1_ = A1;
	B1_ = B1;

	return SUCCESSFUL_RETURN;
}


returnValue ModelData::getLinearOutput( Matrix& M3_, Matrix& A3_, OutputFcn& rhs_ ) const {
	M3_ = M3;
	A3_ = A3;
	rhs_ = rhs3;

	return SUCCESSFUL_RETURN;
}


returnValue ModelData::getLinearOutput( Matrix& M3_, Matrix& A3_ ) const {
	M3_ = M3;
	A3_ = A3;

	return SUCCESSFUL_RETURN;
}


returnValue ModelData::setModel( const DifferentialEquation& _f )
{
	if( rhs_name.isEmpty() && NX2 == 0 ) {
		differentialEquation = _f;
		Expression rhs;
		differentialEquation.getExpression( rhs );

		NX2 = rhs.getDim() - differentialEquation.getNXA();
		if( NDX == 0 ) NDX = differentialEquation.getNDX();
		NXA = differentialEquation.getNXA();
		if( NU == 0 ) NU = differentialEquation.getNU();
		if( NP == 0 ) NP = differentialEquation.getNP();

		model_dimensions_set = BT_TRUE;
		export_rhs = BT_TRUE;
	}
	else {
		return ACADOERROR( RET_INVALID_OPTION );
	}
	return SUCCESSFUL_RETURN;
}


returnValue ModelData::setNARXmodel( const uint _delay, const Matrix& _parms ) {

	if( rhs_name.isEmpty() && NX2 == 0 && NX3 == 0 ) {
		NX2 = _parms.getNumRows();
		delay = _delay;
		uint numParms = 1;
		uint n = delay*getNX();
		if( delay >= 1 ) {
			numParms = numParms + n;
		}
		for( uint i = 1; i < delay; i++ ) {
			numParms = numParms + (n+1)*(uint)pow((double)n,(int)i)/2;
		}
		if( _parms.getNumCols() == numParms ) {
			parms = _parms;
		}
		else {
			return ACADOERROR( RET_UNABLE_TO_EXPORT_CODE );
		}

		model_dimensions_set = BT_TRUE;
		export_rhs = BT_TRUE;
	}
	else {
		return ACADOERROR( RET_INVALID_OPTION );
	}
	return SUCCESSFUL_RETURN;
}


returnValue ModelData::setLinearInput( const Matrix& M1_, const Matrix& A1_, const Matrix& B1_ )
{
	M1 = M1_;
	A1 = A1_;
	B1 = B1_;
	NX1 = A1.getNumCols();
	if( !model_dimensions_set ) {
		NU = B1.getNumCols();
		model_dimensions_set = BT_TRUE;
	}
	export_rhs = BT_TRUE;

	return SUCCESSFUL_RETURN;
}


returnValue ModelData::setLinearOutput( const Matrix& M3_, const Matrix& A3_, const OutputFcn& rhs3_ )
{
	M3 = M3_;
	A3 = A3_;
	rhs3 = rhs3_;
	NX3 = A3.getNumCols();
	if( !model_dimensions_set ) {
		if( NU == 0 ) NU = rhs3_.getNU();
		model_dimensions_set = BT_TRUE;
	}
	export_rhs = BT_TRUE;

	return SUCCESSFUL_RETURN;
}


returnValue ModelData::setLinearOutput( const Matrix& M3_, const Matrix& A3_, const String& rhs3_, const String& diffs3_ )
{
	if( !export_rhs ) {
		M3 = M3_;
		A3 = A3_;
		NX3 = A3.getNumCols();

		rhs3_name = String(rhs3_);
		diffs3_name = String(diffs3_);
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


returnValue ModelData::setIntegrationGrid(	const Grid& _ocpGrid, const uint _numSteps
										)
{
	uint i;
	N = _ocpGrid.getNumIntervals();
	BooleanType equidistantControl = _ocpGrid.isEquidistant();
	double T = _ocpGrid.getLastTime() - _ocpGrid.getFirstTime();
	double h = T/((double)_numSteps);
	Vector stepsVector( N );

	if (integrationGrid.isEmpty() == BT_TRUE)
	{
		for( i = 0; i < stepsVector.getDim(); i++ )
		{
			stepsVector(i) = (int) acadoRound((_ocpGrid.getTime(i+1)-_ocpGrid.getTime(i))/h);
		}

		if( equidistantControl )
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
	}
	return SUCCESSFUL_RETURN;
}


returnValue ModelData::clearIntegrationGrid( )
{
	integrationGrid = Grid();

	return SUCCESSFUL_RETURN;
}


BooleanType ModelData::hasEquidistantControlGrid(  ) const
{
	return numSteps.isEmpty();
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
	if( parms.isEmpty() ) {
		return NX1+NX2+NX3;
	}
	else {
		return delay*(NX1+NX2)+NX3;		// IMPORTANT for NARX models where the state space is increased because of the delay
	}
}


uint ModelData::getNX1( ) const
{
	return NX1;
}


uint ModelData::getNX2( ) const
{
	return NX2;
}


uint ModelData::getNX3( ) const
{
	return NX3;
}


uint ModelData::getNDX( ) const
{
	if( NDX > 0 ) {
		return getNX();
	}
	return NDX;
}


uint ModelData::getNDX3( ) const
{
	if( NDX3 > 0 ) {
		return NX1+NX2;
	}
	return NDX3;
}


uint ModelData::getNXA( ) const
{
	return NXA;
}


uint ModelData::getNXA3( ) const
{
	if( NXA3 > 0 ) {
		return NXA;
	}
	return NXA3;
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
	Vector nOutV( (uint)dim_outputs.size() );
	for( uint i = 0; i < dim_outputs.size(); i++ ) {
		nOutV(i) = dim_outputs[i];
	}
	return nOutV;
}


uint ModelData::getNumOutputs( ) const
{
	return (uint)outputGrids.size();
}


returnValue ModelData::getDimOutputs( std::vector<uint>& dims ) const
{
	dims = dim_outputs;
	return SUCCESSFUL_RETURN;
}


Vector ModelData::getNumMeas( ) const
{
	Vector nMeasV( (uint)num_meas.size() );
	for( uint i = 0; i < num_meas.size(); i++ ) {
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


const String ModelData::getNameOutput() const {
	return rhs3_name;
}


const String ModelData::getNameDiffsOutput() const {
	return diffs3_name;
}


returnValue ModelData::getNameOutputs( std::vector<String>& names ) const {
	names = outputNames;
	return SUCCESSFUL_RETURN;
}


returnValue ModelData::getNameDiffsOutputs( std::vector<String>& names ) const {
	names = diffs_outputNames;
	return SUCCESSFUL_RETURN;
}


BooleanType ModelData::checkConsistency( ) const
{
	// Number of differential state derivatives must be either zero or equal to the number of differential states:
	if( NDX == 0 || NX3 > 0 || NX1 > 0 || NDX == NX2 ) {
		return BT_TRUE;
	}
	return BT_FALSE;
}



// PROTECTED:




CLOSE_NAMESPACE_ACADO

// end of file.
