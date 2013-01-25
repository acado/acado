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
 *    \file src/code_generation/Model_container.cpp
 *    \author Rien Quirynen
 *    \date 2012
 */

#include <acado/code_generation/model_container.hpp>


BEGIN_NAMESPACE_ACADO



//
// PUBLIC MEMBER FUNCTIONS:
//


ModelContainer::ModelContainer() {
}


returnValue ModelContainer::setDimensions( uint _NX, uint _NDX, uint _NXA, uint _NU ) {
	return modelData.setDimensions( _NX, _NDX, _NXA, _NU );
}


returnValue ModelContainer::setDimensions( uint _NX, uint _NU ) {
	return modelData.setDimensions( _NX, _NU );
}


returnValue ModelContainer::setModel( const DifferentialEquation& _f ) {
	return modelData.setModel( _f );
}


returnValue ModelContainer::setModel( 	const String& fileName, const String& _rhs_ODE, const String& _diffs_rhs_ODE ) {
	return modelData.setModel( fileName, _rhs_ODE, _diffs_rhs_ODE );
}


returnValue ModelContainer::addOutput( const OutputFcn& outputEquation_ ) {
	return modelData.addOutput( outputEquation_ );
}


returnValue ModelContainer::addOutput( const String& output, const String& diffs_output, const uint dim ) {
	return modelData.addOutput( output, diffs_output, dim );
}



returnValue ModelContainer::addOutput( 	const String& output, const String& diffs_output, const uint dim,
						const String& colInd, const String& rowPtr	) {
	return modelData.addOutput( output, diffs_output, dim, colInd, rowPtr );
}


returnValue ModelContainer::setMeasurements( const Vector& numberMeasurements ) {
	return modelData.setMeasurements( numberMeasurements );
}


returnValue ModelContainer::setIntegrationGrid( const Vector& gridPoints ) {
	return modelData.setIntegrationGrid( gridPoints );
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


BooleanType ModelContainer::hasEquidistantIntegrationGrid() const {

    return modelData.hasEquidistantIntegrationGrid();
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
	return modelData.getNU();
}


uint ModelContainer::getNP( ) const
{
	return modelData.getNP();
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


Vector ModelContainer::getDimOutputs( ) const
{
	return modelData.getDimOutputs();
}


Vector ModelContainer::getNumMeas( ) const
{
	return modelData.getNumMeas();
}


const String ModelContainer::getFileNameModel( ) const
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
