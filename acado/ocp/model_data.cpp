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
	NOD = 0;
	model_dimensions_set = BT_FALSE;
	export_rhs = BT_TRUE;
	delay = 1;
}


returnValue ModelData::setDimensions( uint _NX1, uint _NX2, uint _NX3, uint _NDX, uint _NDX3, uint _NXA, uint _NXA3, uint _NU, uint _NOD, uint _NP )
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
	NOD = _NOD;
	model_dimensions_set = BT_TRUE;
	return SUCCESSFUL_RETURN;
}


uint ModelData::addOutput( const OutputFcn& outputEquation_, const Grid& grid ){

	if( rhs_name.empty() && outputNames.size() == 0 ) {
		Expression next;
		outputEquation_.getExpression( next );
		outputExpressions.push_back( next );
		dim_outputs.push_back( next.getDim() );

		if( NDX == 0 ) NDX = outputEquation_.getNDX();
		if( NU == 0 ) NU = outputEquation_.getNU();
		if( NP == 0 ) NP = outputEquation_.getNP();
		if( NOD == 0 ) NOD = outputEquation_.getNOD();

		outputGrids.push_back( grid );

		uint numOuts = (int) ceil((double)grid.getNumIntervals());
		num_meas.push_back( numOuts );
	}
	else {
		return ACADOERROR( RET_INVALID_OPTION );
	}

	return dim_outputs.size();
}


uint ModelData::addOutput( const std::string& output, const std::string& diffs_output, const uint dim, const Grid& grid ){

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


uint ModelData::addOutput( 	const std::string& output, const std::string& diffs_output, const uint dim,
							const Grid& grid, const std::string& colInd, const std::string& rowPtr	){


	DVector colIndV, rowPtrV;

	colIndV.read( colInd.c_str() );
	rowPtrV.read( rowPtr.c_str() );

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


returnValue ModelData::getNumSteps( DVector& _numSteps ) const {

    _numSteps = numSteps;
    return SUCCESSFUL_RETURN;
}


returnValue ModelData::setNumSteps( const DVector& _numSteps ) {

    numSteps = _numSteps;
    return SUCCESSFUL_RETURN;
}


returnValue ModelData::getOutputExpressions( std::vector<Expression>& outputExpressions_ ) const {

    outputExpressions_ = outputExpressions;
    return SUCCESSFUL_RETURN;
}


std::vector<DMatrix> ModelData::getOutputDependencies( ) const {
	std::vector<DMatrix> outputDependencies;
	if( hasCompressedStorage() ) {
		for( uint i = 0; i < outputNames.size(); i++ ) {
			DVector colIndV = colInd_outputs[i];
			DVector rowPtrV = rowPtr_outputs[i];

			DMatrix dependencyMat = zeros<double>( dim_outputs[i],getNX()+NXA+NU+NDX );
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


returnValue ModelData::getNARXmodel( uint& _delay, DMatrix& _parms ) const{

    _delay = delay;
    _parms = parms;

    return SUCCESSFUL_RETURN;
}


returnValue ModelData::getLinearInput( DMatrix& M1_, DMatrix& A1_, DMatrix& B1_ ) const {
	M1_ = M1;
	A1_ = A1;
	B1_ = B1;

	return SUCCESSFUL_RETURN;
}


returnValue ModelData::getLinearOutput( DMatrix& M3_, DMatrix& A3_, OutputFcn& rhs_ ) const {
	M3_ = M3;
	A3_ = A3;
	rhs_ = rhs3;

	return SUCCESSFUL_RETURN;
}


returnValue ModelData::getNonlinearFeedback( DMatrix& C_, OutputFcn& feedb_ ) const {
	C_ = C;
	feedb_ = feedb;

	return SUCCESSFUL_RETURN;
}


returnValue ModelData::getLinearOutput( DMatrix& M3_, DMatrix& A3_ ) const {
	M3_ = M3;
	A3_ = A3;

	return SUCCESSFUL_RETURN;
}


returnValue ModelData::setModel( const DifferentialEquation& _f )
{
	if( rhs_name.empty() && NX2 == 0 ) {
		differentialEquation = _f;
		Expression rhs;
		differentialEquation.getExpression( rhs );

		NXA = differentialEquation.getNXA();
		NX2 = rhs.getDim() - NXA;
		if( NDX == 0 ) NDX = _f.getNDX();
//		if( _f.getNDX() > 0 && _f.getNDX() != (int)NX2 ) { // TODO: this test returns an error for well-defined models when using a linear input subsystem!
//			std::cout << "nonlinear model of size " << NX2 << " depends on " << _f.getNDX() << " differential state derivatives!" << std::endl;
//			return ACADOERROR( RET_INVALID_OPTION );
//		}
		if( NU == 0 ) NU = _f.getNU();
		if( NP == 0 ) NP = _f.getNP();
		if( NOD == 0 ) NOD = _f.getNOD();

		export_rhs = BT_TRUE;
	}
	else {
		return ACADOERROR( RET_INVALID_OPTION );
	}
	return SUCCESSFUL_RETURN;
}


returnValue ModelData::setNARXmodel( const uint _delay, const DMatrix& _parms ) {

	if( rhs_name.empty() && NX2 == 0 && NX3 == 0 ) {
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

		export_rhs = BT_TRUE;
	}
	else {
		return ACADOERROR( RET_INVALID_OPTION );
	}
	return SUCCESSFUL_RETURN;
}


returnValue ModelData::setLinearInput( const DMatrix& M1_, const DMatrix& A1_, const DMatrix& B1_ )
{
	M1 = M1_;
	A1 = A1_;
	B1 = B1_;
	NX1 = A1.getNumCols();
	NU = B1.getNumCols();
	export_rhs = BT_TRUE;

	return SUCCESSFUL_RETURN;
}


returnValue ModelData::setLinearOutput( const DMatrix& M3_, const DMatrix& A3_, const OutputFcn& rhs3_ )
{
	M3 = M3_;
	A3 = A3_;
	NX3 = A3.getNumCols();

	rhs3 = rhs3_;
	if( NDX == 0 ) NDX = rhs3_.getNDX();
	if( NU == 0 ) NU = rhs3_.getNU();
	if( NP == 0 ) NP = rhs3_.getNP();
	if( NOD == 0 ) NOD = rhs3_.getNOD();

	export_rhs = BT_TRUE;

	return SUCCESSFUL_RETURN;
}


returnValue ModelData::setNonlinearFeedback( const DMatrix& C_, const OutputFcn& feedb_ )
{
	C = C_;
	feedb = feedb_;
	if( feedb_.getNDX() > 0 || feedb_.getNU() > 0 || feedb_.getNP() > 0 || feedb_.getNOD() > 0 ) return RET_NOT_IMPLEMENTED_YET;

	export_rhs = BT_TRUE;

	return SUCCESSFUL_RETURN;
}


returnValue ModelData::setLinearOutput( const DMatrix& M3_, const DMatrix& A3_, const std::string& rhs3_, const std::string& diffs3_ )
{
	if( !export_rhs ) {
		M3 = M3_;
		A3 = A3_;
		NX3 = A3.getNumCols();

		rhs3_name = std::string(rhs3_);
		diffs3_name = std::string(diffs3_);
	}
	else {
		return ACADOERROR( RET_INVALID_OPTION );
	}

	return SUCCESSFUL_RETURN;
}


returnValue ModelData::setModel( const std::string& fileName, const std::string& _rhs_ODE, const std::string& _diffs_ODE )
{
	if( differentialEquation.getNumDynamicEquations() == 0 )
	{
		externModel = fileName;
		rhs_name = _rhs_ODE;
		diffs_name = _diffs_ODE;

		export_rhs = BT_FALSE;
	}
	else
	{
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
	DVector stepsVector( N );

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

uint ModelData::getNOD( ) const
{
	return NOD;
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


returnValue ModelData::setNU( const uint NU_ )
{
	NU = NU_;
	return SUCCESSFUL_RETURN;
}


returnValue ModelData::setNP( const uint NP_ )
{
	NP = NP_;
	return SUCCESSFUL_RETURN;
}


returnValue ModelData::setNOD( const uint NOD_ )
{
	NOD = NOD_;
	return SUCCESSFUL_RETURN;
}


DVector ModelData::getDimOutputs( ) const
{
	DVector nOutV( (uint)dim_outputs.size() );
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


DVector ModelData::getNumMeas( ) const
{
	DVector nMeasV( (uint)num_meas.size() );
	for( uint i = 0; i < num_meas.size(); i++ ) {
		nMeasV(i) = num_meas[i];
	}
	return nMeasV;
}


const std::string ModelData::getFileNameModel() const {
	return externModel;
}


const std::string ModelData::getNameRhs() const {
	return rhs_name;
}


const std::string ModelData::getNameDiffsRhs() const {
	return diffs_name;
}


const std::string ModelData::getNameOutput() const {
	return rhs3_name;
}


const std::string ModelData::getNameDiffsOutput() const {
	return diffs3_name;
}


returnValue ModelData::getNameOutputs( std::vector<std::string>& names ) const {
	names = outputNames;
	return SUCCESSFUL_RETURN;
}


returnValue ModelData::getNameDiffsOutputs( std::vector<std::string>& names ) const {
	names = diffs_outputNames;
	return SUCCESSFUL_RETURN;
}



// PROTECTED:




CLOSE_NAMESPACE_ACADO

// end of file.
