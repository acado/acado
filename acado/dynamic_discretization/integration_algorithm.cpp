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
 *    \file src/dynamic_discretization/integration_algorithm.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *
 */


#include <acado/dynamic_discretization/integration_algorithm.hpp>



BEGIN_NAMESPACE_ACADO



//
// PUBLIC MEMBER FUNCTIONS:
//


IntegrationAlgorithm::IntegrationAlgorithm() : UserInteraction( )
{
	setupOptions( );
	setupLogging( );

	integrationMethod = new ShootingMethod( this );
	
	setStatus( BS_NOT_INITIALIZED );
}


IntegrationAlgorithm::IntegrationAlgorithm ( const IntegrationAlgorithm& arg ) : UserInteraction( arg )
{
	// take care of userInteraction pointer!!!
	if ( arg.integrationMethod != 0 )
		integrationMethod = new ShootingMethod( *(arg.integrationMethod) );
	else
		integrationMethod = 0;
	
	iter = arg.iter;
}


IntegrationAlgorithm::~IntegrationAlgorithm( )
{
	if ( integrationMethod != 0 )
		delete integrationMethod;
}


IntegrationAlgorithm& IntegrationAlgorithm::operator=( const IntegrationAlgorithm& arg )
{
	if ( this != &arg )
	{
		if ( integrationMethod != 0 )
			delete integrationMethod;
		
		// take care of userInteraction pointer!!!
		if ( arg.integrationMethod != 0 )
			integrationMethod = new ShootingMethod( *(arg.integrationMethod) );
		else
			integrationMethod = 0;
		
		iter = arg.iter;
	}

	return *this;
}


returnValue IntegrationAlgorithm::addStage( const DynamicSystem  &dynamicSystem_,
                                            const Grid           &stageIntervals,
                                            const IntegratorType &integratorType_ ){

    return integrationMethod->addStage( dynamicSystem_, stageIntervals, integratorType_ );
}


returnValue IntegrationAlgorithm::addTransition( const Transition& transition_ ){

    return integrationMethod->addTransition( transition_ );
}


returnValue IntegrationAlgorithm::clear(){

    return integrationMethod->clear( );
}



returnValue IntegrationAlgorithm::evaluate( VariablesGrid  *x ,
                                            VariablesGrid  *xa,
                                            VariablesGrid  *p ,
                                            VariablesGrid  *u ,
                                            VariablesGrid  *w   ){

    iter.init( x,xa,p,u,w );
    return evaluate( iter );
}


returnValue IntegrationAlgorithm::evaluate( OCPiterate& _iter ){

	if ( integrationMethod->evaluate( _iter ) != SUCCESSFUL_RETURN )
		return ACADOERROR( RET_INTALG_INTEGRATION_FAILED );

	return plot( );
}



returnValue IntegrationAlgorithm::integrate(	VariablesGrid  *x ,
												VariablesGrid  *xa,
												VariablesGrid  *p ,
												VariablesGrid  *u ,
												VariablesGrid  *w
												){

    return evaluate( x,xa,p,u,w );
}


returnValue IntegrationAlgorithm::integrate(	double       t0,
												double       tend,
												const DVector &x0,
												const DVector &xa,
												const DVector &p,
												const DVector &u,
												const DVector &w
												)
{
	Grid grid( t0,tend,2 );

	VariablesGrid  xGrid( x0,grid );
	VariablesGrid xaGrid( xa,grid );
	VariablesGrid  pGrid(  p,grid );
	VariablesGrid  uGrid(  u,grid );
	VariablesGrid  wGrid(  w,grid );

	return integrate( &xGrid,&xaGrid,&pGrid,&uGrid,&wGrid );
}


returnValue IntegrationAlgorithm::integrate(	const Grid& t,
												const DVector &x0,
												const DVector &xa,
												const DVector &p,
												const DVector &u,
												const DVector &w
												)
{
	//Grid grid( t.getFirstTime(),t.getLastTime(),2 );

	VariablesGrid  xGrid( x0,t );
	VariablesGrid xaGrid( xa,t );
	VariablesGrid  pGrid(  p,t );
	VariablesGrid  uGrid(  u,t );
	VariablesGrid  wGrid(  w,t );

	return integrate( &xGrid,&xaGrid,&pGrid,&uGrid,&wGrid );
}



returnValue IntegrationAlgorithm::setForwardSeed(	const BlockMatrix &xSeed_,
													const BlockMatrix &pSeed_,
													const BlockMatrix &uSeed_,
													const BlockMatrix &wSeed_
													)
{
    return integrationMethod->setForwardSeed( xSeed_,pSeed_,uSeed_,wSeed_ );
}


returnValue IntegrationAlgorithm::setForwardSeed(	const DVector &xSeed,
													const DVector &pSeed,
													const DVector &uSeed,
													const DVector &wSeed
													)
{
	BlockMatrix xSeedTmp( (DMatrix)xSeed );
	BlockMatrix pSeedTmp( (DMatrix)pSeed );
	BlockMatrix uSeedTmp( (DMatrix)uSeed );
	BlockMatrix wSeedTmp( (DMatrix)wSeed );

	return integrationMethod->setForwardSeed( xSeedTmp,pSeedTmp,uSeedTmp,wSeedTmp );
}


returnValue IntegrationAlgorithm::setUnitForwardSeed( )
{
    return integrationMethod->setUnitForwardSeed( );
}


returnValue IntegrationAlgorithm::setBackwardSeed(	const BlockMatrix &seed
													)
{
    return integrationMethod->setBackwardSeed( seed );
}


returnValue IntegrationAlgorithm::setBackwardSeed(	const DVector &seed
													)
{
	DMatrix seedTmpMatrix( seed );
	seedTmpMatrix.transposeInPlace();
	BlockMatrix seedTmp( seedTmpMatrix );

	return integrationMethod->setBackwardSeed( seedTmp );
}


returnValue IntegrationAlgorithm::setUnitBackwardSeed( )
{
    return integrationMethod->setUnitBackwardSeed( );
}



returnValue IntegrationAlgorithm::deleteAllSeeds( )
{
    return integrationMethod->deleteAllSeeds( );
}



returnValue IntegrationAlgorithm::evaluateSensitivities( )
{
    return integrationMethod->evaluateSensitivities( );
}


returnValue IntegrationAlgorithm::integrateSensitivities( )
{
    return evaluateSensitivities( );
}


returnValue IntegrationAlgorithm::evaluateSensitivities( const BlockMatrix &seed, BlockMatrix &hessian )
{
    return integrationMethod->evaluateSensitivities( seed,hessian );
}


returnValue IntegrationAlgorithm::integrateSensitivities( const BlockMatrix &seed, BlockMatrix &hessian )
{
    return evaluateSensitivities( seed,hessian );
}



returnValue IntegrationAlgorithm::unfreeze()
{
	return integrationMethod->unfreeze( );
}


BooleanType IntegrationAlgorithm::isAffine( ) const
{
	return integrationMethod->isAffine( );
}


returnValue IntegrationAlgorithm::getX(	DVector& xEnd
										) const
{
	if ( iter.x != 0 )
		xEnd = iter.x->getLastVector( );

	return SUCCESSFUL_RETURN;
}


returnValue IntegrationAlgorithm::getXA(	DVector& xaEnd
											) const
{
	if ( iter.xa != 0 )
		xaEnd = iter.xa->getLastVector( );

	return SUCCESSFUL_RETURN;
}


returnValue IntegrationAlgorithm::getX(	VariablesGrid& X
										) const
{
	if ( iter.x != 0 )
		X = *(iter.x);

	return SUCCESSFUL_RETURN;
}


returnValue IntegrationAlgorithm::getXA(	VariablesGrid& XA
											) const
							
{
	if ( iter.xa != 0 )
		XA = *(iter.xa);

	return SUCCESSFUL_RETURN;
}



returnValue IntegrationAlgorithm::getForwardSensitivities(	BlockMatrix &D
															) const
{
	return integrationMethod->getForwardSensitivities( D );
}


returnValue IntegrationAlgorithm::getForwardSensitivities(	DVector &Dx
															) const
{
	BlockMatrix DxTmp;
	
	returnValue returnvalue = integrationMethod->getForwardSensitivities( DxTmp );
	if ( returnvalue != SUCCESSFUL_RETURN )
		return returnvalue;

	if ( DxTmp.isEmpty( ) == BT_FALSE )
	{
		DMatrix DxTmpMatrix;
		DxTmp.getSubBlock( 0,0,DxTmpMatrix );
		
		if ( DxTmpMatrix.getNumCols( ) > 0 )
			Dx = DxTmpMatrix.getCol( 0 );
	}

	return SUCCESSFUL_RETURN;
}



returnValue IntegrationAlgorithm::getBackwardSensitivities(	BlockMatrix &D
															) const
{
	return integrationMethod->getBackwardSensitivities( D );
}


returnValue IntegrationAlgorithm::getBackwardSensitivities(	DVector &Dx_x0,
															DVector &Dx_p ,
															DVector &Dx_u ,
															DVector &Dx_w
															) const
{
	BlockMatrix DxTmp;
	
	returnValue returnvalue = integrationMethod->getBackwardSensitivities( DxTmp );
	if ( returnvalue != SUCCESSFUL_RETURN )
		return returnvalue;

	if ( DxTmp.isEmpty( ) == BT_FALSE )
	{
		DMatrix DxTmpMatrix;
		
		DxTmp.getSubBlock( 0,0,DxTmpMatrix );
		if ( DxTmpMatrix.getNumRows( ) > 0 )
			Dx_x0 = DxTmpMatrix.getRow( 0 );
		else
			Dx_x0.init( );
		
		DxTmp.getSubBlock( 0,2,DxTmpMatrix );
		if ( DxTmpMatrix.getNumRows( ) > 0 )
			Dx_p = DxTmpMatrix.getRow( 0 );
		else
			Dx_p.init( );

		DxTmp.getSubBlock( 0,3,DxTmpMatrix );
		if ( DxTmpMatrix.getNumRows( ) > 0 )
			Dx_u = DxTmpMatrix.getRow( 0 );
		else
			Dx_u.init( );

		DxTmp.getSubBlock( 0,4,DxTmpMatrix );
		if ( DxTmpMatrix.getNumRows( ) > 0 )
			Dx_w = DxTmpMatrix.getRow( 0 );
		else
			Dx_w.init( );
	}

	return SUCCESSFUL_RETURN;
}



//
// PROTECTED MEMBER FUNCTIONS:
//

returnValue IntegrationAlgorithm::setupOptions( )
{
	//printf( "IntegrationAlgorithm::setupOptions( ) called.\n" );
	
	// add integration options
	addOption( FREEZE_INTEGRATOR           , BT_FALSE                       );
	addOption( INTEGRATOR_TYPE             , INT_BDF                        );
	addOption( FEASIBILITY_CHECK           , defaultFeasibilityCheck        );
	addOption( PLOT_RESOLUTION             , defaultPlotResoltion           );
	
	// add integrator options
	addOption( MAX_NUM_INTEGRATOR_STEPS    , defaultMaxNumSteps             );
	addOption( INTEGRATOR_TOLERANCE        , defaultIntegratorTolerance     );
	addOption( ABSOLUTE_TOLERANCE          , defaultAbsoluteTolerance       );
	addOption( INITIAL_INTEGRATOR_STEPSIZE , defaultInitialStepsize         );
	addOption( MIN_INTEGRATOR_STEPSIZE     , defaultMinStepsize             );
	addOption( MAX_INTEGRATOR_STEPSIZE     , defaultMaxStepsize             );
	addOption( STEPSIZE_TUNING             , defaultStepsizeTuning          );
	addOption( CORRECTOR_TOLERANCE         , defaultCorrectorTolerance      );
	addOption( INTEGRATOR_PRINTLEVEL       , defaultIntegratorPrintlevel    );
	addOption( LINEAR_ALGEBRA_SOLVER       , defaultLinearAlgebraSolver     );
	addOption( ALGEBRAIC_RELAXATION        , defaultAlgebraicRelaxation     );
	addOption( RELAXATION_PARAMETER        , defaultRelaxationParameter     );
	addOption( PRINT_INTEGRATOR_PROFILE    , defaultprintIntegratorProfile  );

	return SUCCESSFUL_RETURN;
}


returnValue IntegrationAlgorithm::setupLogging( )
{
    LogRecord tmp( LOG_AT_END );

    tmp.addItem( LOG_DIFFERENTIAL_STATES      );
    tmp.addItem( LOG_ALGEBRAIC_STATES         );
    tmp.addItem( LOG_PARAMETERS               );
    tmp.addItem( LOG_CONTROLS                 );
    tmp.addItem( LOG_DISTURBANCES             );
    tmp.addItem( LOG_INTERMEDIATE_STATES      );

    tmp.addItem( LOG_DISCRETIZATION_INTERVALS );
    tmp.addItem( LOG_STAGE_BREAK_POINTS       );

    logIdx = addLogRecord( tmp );

	return SUCCESSFUL_RETURN;
}


CLOSE_NAMESPACE_ACADO

// end of file.
