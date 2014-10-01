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
 *    \file src/controller/controller.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 */


#include <acado/controller/controller.hpp>


// #define SIM_DEBUG


BEGIN_NAMESPACE_ACADO

//
// PUBLIC MEMBER FUNCTIONS:
//

Controller::Controller( ) : SimulationBlock( BN_CONTROLLER )
{
	setupOptions( );
	setupLogging( );

	controlLaw = 0;
	estimator   = 0;
	referenceTrajectory = 0;
	
	isEnabled = BT_TRUE;
	
	setStatus( BS_NOT_INITIALIZED );
}


Controller::Controller(	ControlLaw& _controlLaw,
						Estimator& _estimator,
						ReferenceTrajectory& _referenceTrajectory
						) : SimulationBlock( BN_CONTROLLER )
{
	setupOptions( );
	setupLogging( );

	if ( _controlLaw.isDefined( ) == BT_TRUE )
	{
		controlLaw = &_controlLaw;
		setStatus( BS_NOT_INITIALIZED );
	}
	else
		controlLaw = 0;

	if ( _estimator.isDefined( ) == BT_TRUE )
		estimator = &_estimator;
	else
		estimator = 0;

	if ( _referenceTrajectory.isDefined( ) == BT_TRUE )
		referenceTrajectory = &_referenceTrajectory;
	else
		referenceTrajectory = 0;
	
	isEnabled = BT_TRUE;
	
	setStatus( BS_NOT_INITIALIZED );
}


Controller::Controller(	ControlLaw& _controlLaw,
						ReferenceTrajectory& _referenceTrajectory
						) : SimulationBlock( BN_CONTROLLER )
{
	setupOptions( );
	setupLogging( );

	if ( _controlLaw.isDefined( ) == BT_TRUE )
	{
		controlLaw = &_controlLaw;
		setStatus( BS_NOT_INITIALIZED );
	}
	else
		controlLaw = 0;

	estimator = 0;

	if ( _referenceTrajectory.isDefined( ) == BT_TRUE )
		referenceTrajectory = &_referenceTrajectory;
	else
		referenceTrajectory = 0;
	
	isEnabled = BT_TRUE;
	
	setStatus( BS_NOT_INITIALIZED );
}


Controller::Controller( const Controller& rhs ) : SimulationBlock( rhs )
{
	if ( rhs.controlLaw != 0 )
		controlLaw = rhs.controlLaw;
	else
		controlLaw = 0;

	if ( rhs.estimator != 0 )
		estimator = rhs.estimator;
	else
		estimator = 0;

	if ( rhs.referenceTrajectory != 0 )
		referenceTrajectory = rhs.referenceTrajectory;
	else
		referenceTrajectory = 0;
	
	isEnabled = rhs.isEnabled;
}


Controller::~Controller( )
{
// 	if ( controlLaw != 0 )
// 	 	delete controlLaw;
// 
// 	if ( estimator != 0 )
// 		delete estimator;
// 
// 	if ( referenceTrajectory != 0 )
// 		delete referenceTrajectory;
}


Controller& Controller::operator=( const Controller& rhs )
{
	if ( this != &rhs )
	{
// 		if ( controlLaw != 0 )
// 			delete controlLaw;
// 
// 		if ( estimator != 0 )
// 			delete estimator;
// 
// 		if ( referenceTrajectory != 0 )
// 			delete referenceTrajectory;

		SimulationBlock::operator=( rhs );

		if ( rhs.controlLaw != 0 )
			controlLaw = rhs.controlLaw;
		else
			controlLaw = 0;

		if ( rhs.estimator != 0 )
			estimator = rhs.estimator;
		else
			estimator = 0;

		if ( rhs.referenceTrajectory != 0 )
			referenceTrajectory = rhs.referenceTrajectory;
		else
			referenceTrajectory = 0;
		
		isEnabled = rhs.isEnabled;
	}

    return *this;
}



returnValue Controller::setControlLaw(	ControlLaw& _controlLaw
										)
{
	if ( _controlLaw.isDefined( ) == BT_TRUE )
	{
		if ( controlLaw == 0 )
			controlLaw = &_controlLaw;
		else
			*controlLaw = _controlLaw;

		setStatus( BS_NOT_INITIALIZED );
	}

	return SUCCESSFUL_RETURN;
}


returnValue Controller::setEstimator(	Estimator& _estimator
										)
{
	if ( _estimator.isDefined( ) == BT_TRUE )
	{
		if ( estimator == 0 )
			estimator = &_estimator;
		else
			*estimator = _estimator;

		if ( getStatus( ) > BS_NOT_INITIALIZED )
			setStatus( BS_NOT_INITIALIZED );
	}
	
	return SUCCESSFUL_RETURN;
}


returnValue Controller::setReferenceTrajectory(	ReferenceTrajectory& _referenceTrajectory
												)
{
	if ( _referenceTrajectory.isDefined( ) == BT_TRUE )
	{
		if ( referenceTrajectory == 0 )
			referenceTrajectory = &_referenceTrajectory;
		else
			*referenceTrajectory = _referenceTrajectory;

		if ( getStatus( ) > BS_NOT_INITIALIZED )
			setStatus( BS_NOT_INITIALIZED );
	}

	return SUCCESSFUL_RETURN;
}



returnValue Controller::initializeAlgebraicStates(	const VariablesGrid& _xa_init
													)
{
	if ( controlLaw != 0 )
		controlLaw->initializeAlgebraicStates( _xa_init.getVector(0) );

	return SUCCESSFUL_RETURN;
}


returnValue Controller::initializeAlgebraicStates( 	const char* fileName
													)
{
	VariablesGrid tmp;
	tmp.read( fileName );

	if ( tmp.isEmpty( ) == BT_TRUE )
		return ACADOERROR( RET_FILE_CAN_NOT_BE_OPENED );

	return initializeAlgebraicStates( tmp );
}



returnValue Controller::init(	double startTime,
								const DVector& _x0,
								const DVector& _p,
								const VariablesGrid& _yRef
								)
{
	if ( controlLaw == 0 )
		return ACADOERROR( RET_NO_CONTROLLAW_SPECIFIED );

	/* 1) Initialize all sub-blocks. */
	/* a) Estimator */
	DVector xEst( _x0 );
	DVector pEst( _p );
	DVector xaEst, uEst, wEst;

	if ( estimator != 0 )
	{
		if ( estimator->init( startTime,_x0,_p ) != SUCCESSFUL_RETURN )
			return ACADOERROR( RET_CONTROLLER_INIT_FAILED );

		if ( estimator->getOutputs( xEst,xaEst,uEst,pEst,wEst ) != SUCCESSFUL_RETURN )
			return ACADOERROR( RET_CONTROLLER_INIT_FAILED );
	}

	/* b) Reference trajectory */
	if ( ( referenceTrajectory != 0 ) && ( _yRef.isEmpty( ) == BT_TRUE ) )
		if ( referenceTrajectory->init( startTime,xEst,xaEst,uEst,pEst,wEst ) != SUCCESSFUL_RETURN )
			return ACADOERROR( RET_CONTROLLER_INIT_FAILED );
		
	VariablesGrid yRef( _yRef );
	getCurrentReference( startTime,yRef );
	#ifdef SIM_DEBUG
	yRef.print( "yRef init" );
	#endif

	/* c) Control law */
// 	_x0.print( "x0 init" );
	if ( controlLaw->init( startTime,_x0,_p,yRef ) != SUCCESSFUL_RETURN )
		return ACADOERROR( RET_CONTROLLER_INIT_FAILED );


	/* 2) Consistency checks. */
	if ( estimator != 0 )
	{
		if ( estimator->getNX( ) != controlLaw->getNX( ) )
			return ACADOERROR( RET_BLOCK_DIMENSION_MISMATCH );

		if ( estimator->getNXA( ) != controlLaw->getNXA( ) )
			return ACADOERROR( RET_BLOCK_DIMENSION_MISMATCH );

		if ( estimator->getNU( ) != controlLaw->getNU( ) )
			return ACADOERROR( RET_BLOCK_DIMENSION_MISMATCH );

		if ( estimator->getNP( ) != controlLaw->getNP( ) )
			return ACADOERROR( RET_BLOCK_DIMENSION_MISMATCH );

		if ( estimator->getNW( ) != controlLaw->getNW( ) )
			return ACADOERROR( RET_BLOCK_DIMENSION_MISMATCH );
	}

	realClock.reset( );
	setStatus( BS_READY );

	return SUCCESSFUL_RETURN;
}



returnValue Controller::step(	double currentTime,
								const DVector& _y,
								const VariablesGrid& _yRef
								)
{
	/* Consistency check. */
	if ( getStatus( ) != BS_READY )
		return ACADOERROR( RET_BLOCK_NOT_READY );

	if ( controlLaw == 0 )
		return ACADOERROR( RET_NO_CONTROLLAW_SPECIFIED );

	/* Do nothing if controller is disabled */
	if ( isEnabled == BT_FALSE )
	{
		setLast( LOG_TIME_CONTROLLER,0.0 );
		setLast( LOG_TIME_CONTROL_LAW,0.0 );
		setLast( LOG_TIME_ESTIMATOR,0.0 );
		return SUCCESSFUL_RETURN;
	}

	if ( feedbackStep( currentTime,_y,_yRef ) != SUCCESSFUL_RETURN )
		return ACADOERROR( RET_CONTROLLER_STEP_FAILED );

	double nextTime = currentTime + controlLaw->getSamplingTime( );
		
	if ( preparationStep( nextTime,_yRef ) != SUCCESSFUL_RETURN )
		return ACADOERROR( RET_CONTROLLER_STEP_FAILED );

	return SUCCESSFUL_RETURN;
}


returnValue Controller::step(	double currentTime,
								uint dim,
								const double* const _y,
								const VariablesGrid& _yRef
								)
{
	/* Convert double array to vector and call standard step routine. */
	DVector tmp( dim,_y );
	return step( currentTime,tmp,_yRef );
}


returnValue Controller::obtainEstimates(	double currentTime,
											const DVector& _y,
											DVector& xEst,
											DVector& pEst
											)
{
	/* 1) Call Estimator */
	RealClock clock;
	DVector xaEst, uEst, wEst;

	clock.reset();
	clock.start();
	
	if ( estimator != 0 )
	{
		if ( estimator->step( currentTime,_y ) != SUCCESSFUL_RETURN )
			return ACADOERROR( RET_CONTROLLER_STEP_FAILED );

		if ( estimator->getOutputs( xEst,xaEst,uEst,pEst,wEst ) != SUCCESSFUL_RETURN )
			return ACADOERROR( RET_CONTROLLER_STEP_FAILED );
	}
	else
	{
		/* If no estimator is specified, assume full state feedback. */
		xEst = _y;
	}

	clock.stop();
	setLast(LOG_TIME_ESTIMATOR, clock.getTime());

	// step internal reference trajectory
	if ( referenceTrajectory != 0 )
	{
		if ( referenceTrajectory->step( currentTime,_y,xEst,xaEst,uEst,pEst,wEst ) != SUCCESSFUL_RETURN )
			return ACADOERROR( RET_CONTROLLER_STEP_FAILED );
	}
	
	return SUCCESSFUL_RETURN;
}


returnValue Controller::feedbackStep(	double currentTime,
										const DVector& _y,
										const VariablesGrid& _yRef
										)
{
	realClock.reset( );
	
	if ( controlLaw == 0 )
		return ACADOERROR( RET_NO_CONTROLLAW_SPECIFIED );

	/* Do nothing if controller is disabled */
	if ( isEnabled == BT_FALSE )
		return SUCCESSFUL_RETURN;

	// start real runtime measurement
	realClock.start( );
	
	DVector xEst, pEst;

	/* 1) Call Estimator */
	if ( obtainEstimates( currentTime,_y,xEst,pEst ) != SUCCESSFUL_RETURN )
		return ACADOERROR( RET_CONTROLLER_STEP_FAILED );

	/* 2) Evaluate reference trajectory */
	VariablesGrid yRef( _yRef );
	getCurrentReference( currentTime,yRef );
	#ifdef SIM_DEBUG
	yRef.print( "yRef feedback" );
	#endif

	controlLawClock.reset();
	controlLawClock.start();
	
	/* 3) Perform feedback step of control law */
	if ( controlLaw->feedbackStep( currentTime,xEst,pEst,yRef ) != SUCCESSFUL_RETURN )
		return ACADOERROR( RET_CONTROLLER_STEP_FAILED );

	controlLawClock.stop();
	realClock.stop( );

	#ifdef SIM_DEBUG
	DVector uTmp;
	getU( uTmp );
	uTmp.print("u(0) after feedbackStep");
	#endif

	return SUCCESSFUL_RETURN;
}


returnValue Controller::preparationStep(	double nextTime,
											const VariablesGrid& _yRef
											)
{
	if ( controlLaw == 0 )
		return ACADOERROR( RET_NO_CONTROLLAW_SPECIFIED );

	/* Do nothing if controller is disabled */
	if ( isEnabled == BT_FALSE )
		return SUCCESSFUL_RETURN;

	realClock.start();
	controlLawClock.start();

	/* 1) Evaluate reference trajectory */
	VariablesGrid yRef( _yRef );
	getCurrentReference( nextTime,yRef );
	#ifdef SIM_DEBUG
	yRef.print( "yRef preparation" );
	#endif

	/* 2) Perform preparation step of control law */
	if ( controlLaw->preparationStep( nextTime,yRef ) != SUCCESSFUL_RETURN )
		return ACADOERROR( RET_CONTROLLER_STEP_FAILED );

	controlLawClock.stop();
	setLast(LOG_TIME_CONTROL_LAW, controlLawClock.getTime());
	
	// stop real runtime measurement
	realClock.stop();
	setLast(LOG_TIME_CONTROLLER, realClock.getTime());


	#ifdef SIM_DEBUG
	DVector uTmp;
	getU( uTmp );
	uTmp.print("u(0) after preparationStep");
	#endif

	return SUCCESSFUL_RETURN;
}



double Controller::getNextSamplingInstant(	double currentTime
											)
{
	double nextControlLaw = INFTY;
	double nextEstimator  = INFTY;

	if ( controlLaw != 0 )
	{
		if ( acadoIsInteger( (currentTime-0.0) / getSamplingTimeControlLaw( ) ) == BT_TRUE )
			nextControlLaw = currentTime + getSamplingTimeControlLaw( );
		else
			nextControlLaw = ceil( (currentTime-0.0) / getSamplingTimeControlLaw( ) ) * getSamplingTimeControlLaw( );
	}

	if ( estimator != 0 )
	{
		if ( acadoIsInteger( (currentTime-0.0) / getSamplingTimeEstimator( ) ) == BT_TRUE )
			nextEstimator = currentTime + getSamplingTimeEstimator( );
		else
			nextEstimator = ceil( (currentTime-0.0) / getSamplingTimeEstimator( ) ) * getSamplingTimeEstimator( );
	}

	return acadoMin( nextControlLaw,nextEstimator );
}



//
// PROTECTED MEMBER FUNCTIONS:
//


returnValue Controller::setupOptions( )
{
	addOption( USE_REFERENCE_PREDICTION,defaultUseReferencePrediction );

	return SUCCESSFUL_RETURN;
}


returnValue Controller::setupLogging( )
{
	LogRecord tmp(LOG_AT_EACH_ITERATION, PS_DEFAULT);

	tmp.addItem( LOG_FEEDBACK_CONTROL );
	tmp.addItem( LOG_TIME_CONTROLLER );
	tmp.addItem( LOG_TIME_ESTIMATOR );
	tmp.addItem( LOG_TIME_CONTROL_LAW );

	addLogRecord( tmp );

	return SUCCESSFUL_RETURN;
}


returnValue Controller::getCurrentReference(	double tStart,
												VariablesGrid& _yRef
												) const
{
	double tEnd = tStart + controlLaw->getLengthPredictionHorizon( );

	// if no external reference trajectory is given, evaluate internal one
	if ( _yRef.isEmpty( ) == BT_TRUE )
	{
		if ( referenceTrajectory != 0 )
			referenceTrajectory->getReference( tStart,tEnd, _yRef );
	}

	// if prediction shall not be used, only use first value
	int useReferencePrediction = 0;
	get( USE_REFERENCE_PREDICTION,useReferencePrediction );
	
	if ( (BooleanType)useReferencePrediction == BT_FALSE )
	{
		DVector firstVector = _yRef.getFirstVector( );
		Grid predictionGrid( tStart,tEnd );
		_yRef.init( firstVector,predictionGrid );
	}

	return SUCCESSFUL_RETURN;
}


CLOSE_NAMESPACE_ACADO

// end of file.
