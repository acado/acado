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
 *    \file src/ocp/Model_container.cpp
 *    \author Rien Quirynen
 *    \date 2012
 */

#include <acado/ocp/model_container.hpp>


BEGIN_NAMESPACE_ACADO



//
// PUBLIC MEMBER FUNCTIONS:
//


ModelContainer::ModelContainer() {
	NU = 0;
	NP = 0;
	NOD = 0;
}


returnValue ModelContainer::setDimensions( uint _NX1, uint _NX2, uint _NX3, uint _NDX, uint _NDX3, uint _NXA, uint _NXA3, uint _NU, uint _NOD, uint _NP ) {
	return modelData.setDimensions( _NX1, _NX2, _NX3, _NDX, _NDX3, _NXA, _NXA3, _NU, _NOD, _NP );
}


returnValue ModelContainer::setDimensions( uint _NX1, uint _NX2, uint _NX3, uint _NDX, uint _NXA, uint _NU, uint _NOD, uint _NP ) {
	return setDimensions( _NX1, _NX2, _NX3, _NDX, 0, _NXA, 0, _NU, _NOD, _NP );
}


returnValue ModelContainer::setDimensions( uint _NX, uint _NDX, uint _NXA, uint _NU, uint _NOD, uint _NP ) {
	return setDimensions( 0, _NX, 0, _NDX, _NXA, _NU, _NOD, _NP );
}


returnValue ModelContainer::setDimensions( uint _NX, uint _NU, uint _NOD, uint _NP ) {
	return setDimensions( _NX, 0, 0, _NU, _NOD, _NP );
}

returnValue ModelContainer::setModel( const DifferentialEquation& _f ) {
	return modelData.setModel( _f );
}


returnValue ModelContainer::setNARXmodel( const uint _delay, const DMatrix& _parms ) {
	return modelData.setNARXmodel( _delay, _parms );
}


returnValue ModelContainer::setModel( 	const std::string& fileName, const std::string& _rhs_ODE, const std::string& _diffs_rhs_ODE ) {
	return modelData.setModel( fileName, _rhs_ODE, _diffs_rhs_ODE );
}


returnValue ModelContainer::setLinearInput( const DMatrix& M1_, const DMatrix& A1_, const DMatrix& B1_ )
{
	return modelData.setLinearInput( M1_, A1_, B1_ );
}


returnValue ModelContainer::setLinearInput( const DMatrix& A1_, const DMatrix& B1_ )
{
	DMatrix M1_ = eye<double>(A1_.getNumRows());
	return modelData.setLinearInput( M1_, A1_, B1_ );
}


returnValue ModelContainer::setLinearOutput( const DMatrix& M3_, const DMatrix& A3_, const OutputFcn& rhs3_ )
{
	return modelData.setLinearOutput( M3_, A3_, rhs3_ );
}


returnValue ModelContainer::setLinearOutput( const DMatrix& A3_, const OutputFcn& rhs3_ )
{
	DMatrix M3_ = eye<double>(A3_.getNumRows());
	return modelData.setLinearOutput( M3_, A3_, rhs3_ );
}


returnValue ModelContainer::setLinearOutput( const DMatrix& M3_, const DMatrix& A3_, const std::string& rhs3_, const std::string& diffs_rhs3_ )
{
	return modelData.setLinearOutput( M3_, A3_, rhs3_, diffs_rhs3_ );
}


returnValue ModelContainer::setLinearOutput( const DMatrix& A3_, const std::string& rhs3_, const std::string& diffs_rhs3_ )
{
	DMatrix M3_ = eye<double>(A3_.getNumRows());
	return modelData.setLinearOutput( M3_, A3_, rhs3_, diffs_rhs3_ );
}


returnValue ModelContainer::setNonlinearFeedback( const DMatrix& C_, const OutputFcn& feedb_ )
{
	return modelData.setNonlinearFeedback( C_, feedb_ );
}


uint ModelContainer::addOutput( const OutputFcn& outputEquation_, const DVector& measurements ) {
	DVector newMeas(measurements);
	newMeas.append( 1.0 );
	Grid grid( newMeas );
	return modelData.addOutput( outputEquation_, grid );
}


uint ModelContainer::addOutput( const OutputFcn& outputEquation_, const uint numberMeasurements ) {
	Grid grid( 0.0, 1.0, (int)numberMeasurements + 1 );
	return modelData.addOutput( outputEquation_, grid );
}


uint ModelContainer::addOutput( const std::string& output, const std::string& diffs_output, const uint dim, const DVector& measurements ) {
	DVector newMeas(measurements);
	newMeas.append( 1.0 );
	Grid grid( newMeas );
	return modelData.addOutput( output, diffs_output, dim, grid );
}


uint ModelContainer::addOutput( const std::string& output, const std::string& diffs_output, const uint dim, const uint numberMeasurements ) {
	Grid grid( 0.0, 1.0, (int)numberMeasurements + 1 );
	return modelData.addOutput( output, diffs_output, dim, grid );
}


uint ModelContainer::addOutput( const std::string& output, const std::string& diffs_output, const uint dim,
								const DVector& measurements, const std::string& colInd, const std::string& rowPtr	) {
	DVector newMeas(measurements);
	newMeas.append( 1.0 );
	Grid grid( newMeas );
	return modelData.addOutput( output, diffs_output, dim, grid, colInd, rowPtr );
}


uint ModelContainer::addOutput( const std::string& output, const std::string& diffs_output, const uint dim,
								const uint numberMeasurements, const std::string& colInd, const std::string& rowPtr	) {
	Grid grid( 0.0, 1.0, (int)numberMeasurements + 1 );
	return modelData.addOutput( output, diffs_output, dim, grid, colInd, rowPtr );
}


returnValue ModelContainer::getIntegrationGrid( Grid& _grid ) const {
	return modelData.getIntegrationGrid( _grid );
}


returnValue ModelContainer::setIntegrationGrid( const Grid& _ocpGrid, const uint _numSteps ) {
	return modelData.setIntegrationGrid( _ocpGrid, _numSteps );
}


returnValue ModelContainer::getModel( DifferentialEquation &differentialEquation_ ) const {

    modelData.getModel(differentialEquation_);
    return SUCCESSFUL_RETURN;
}


BooleanType ModelContainer::hasOutputs() const {

	std::vector<Expression> outputExpressions_;
	modelData.getOutputExpressions(outputExpressions_);
    if( outputExpressions_.size() == 0 ) return BT_FALSE;
    return BT_TRUE;
}


BooleanType ModelContainer::hasDifferentialEquation() const {

	DifferentialEquation differentialEquation_;
	modelData.getModel(differentialEquation_);
    if( differentialEquation_.getDim() == 0 ) return BT_FALSE;
    return BT_TRUE;
}


BooleanType ModelContainer::modelDimensionsSet() const {

    return modelData.modelDimensionsSet();
}


BooleanType ModelContainer::hasEquidistantControlGrid() const {

    return modelData.hasEquidistantControlGrid();
}


BooleanType ModelContainer::exportRhs() const {

    return modelData.exportRhs();
}


uint ModelContainer::getNX( ) const
{
	return modelData.getNX();
}


uint ModelContainer::getNDX( ) const
{
	return modelData.getNDX();
}


uint ModelContainer::getNXA( ) const
{
	return modelData.getNXA();
}


uint ModelContainer::getNU( ) const
{
	if( NU > 0 ) return NU;
	return modelData.getNU();
}


uint ModelContainer::getNP( ) const
{
	if( NP > 0 ) return NP;
	return modelData.getNP();
}

uint ModelContainer::getNOD( ) const
{
	if( NOD > 0 ) return NOD;
	return modelData.getNOD();
}


uint ModelContainer::getN( ) const
{
	return modelData.getN();
}


returnValue ModelContainer::setN( const uint N_ )
{
	modelData.setN( N_ );
	return SUCCESSFUL_RETURN;
}

returnValue ModelContainer::setNU( const uint NU_ )
{
	NU = NU_;
	modelData.setNU( NU );
	return SUCCESSFUL_RETURN;
}

returnValue ModelContainer::setNP( const uint NP_ )
{
	NP = NP_;
	modelData.setNP( NP );
	return SUCCESSFUL_RETURN;
}

returnValue ModelContainer::setNOD( const uint NOD_ )
{
	NOD = NOD_;
	modelData.setNOD( NOD );
	return SUCCESSFUL_RETURN;
}


DVector ModelContainer::getDimOutputs( ) const
{
	return modelData.getDimOutputs();
}


DVector ModelContainer::getNumMeas( ) const
{
	return modelData.getNumMeas();
}


const std::string ModelContainer::getFileNameModel( ) const
{
	return modelData.getFileNameModel();
}


ModelData& ModelContainer::getModelData( ) {
	return modelData;
}


returnValue ModelContainer::setModelData( const ModelData& data ) {
	modelData = data;
	return SUCCESSFUL_RETURN;
}


// PROTECTED:



CLOSE_NAMESPACE_ACADO

// end of file.
