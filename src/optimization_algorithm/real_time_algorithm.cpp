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
 *    \file src/optimization_algorithm/real_time_algorithm.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date 2009
 */



#include <acado/optimization_algorithm/real_time_algorithm.hpp>



// #define SIM_DEBUG



BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

RealTimeAlgorithm::RealTimeAlgorithm() : OptimizationAlgorithmBase( ), ControlLaw( )
{
	setupOptions( );
	setupLogging( );
	
	x0        = 0;
	p0        = 0;
    reference = 0;

	set( HESSIAN_APPROXIMATION, GAUSS_NEWTON );
	set( USE_REALTIME_ITERATIONS,BT_TRUE );
	set( MAX_NUM_ITERATIONS,1 );
	set( GLOBALIZATION_STRATEGY,GS_FULLSTEP );
	set( TERMINATE_AT_CONVERGENCE,BT_FALSE );
	
	setStatus( BS_NOT_INITIALIZED );
}


RealTimeAlgorithm::RealTimeAlgorithm(	const OCP& ocp_,
										double _samplingTime
										) : OptimizationAlgorithmBase( ocp_ ), ControlLaw( _samplingTime )
{
	setupOptions( );
	setupLogging( );

	x0 = 0;
	p0 = 0;
    reference = 0;

	set( HESSIAN_APPROXIMATION, GAUSS_NEWTON );
	set( USE_REALTIME_ITERATIONS,BT_TRUE );
	set( MAX_NUM_ITERATIONS,1 );
	set( GLOBALIZATION_STRATEGY,GS_FULLSTEP );
	set( TERMINATE_AT_CONVERGENCE,BT_FALSE );
	
	setStatus( BS_NOT_INITIALIZED );
}



RealTimeAlgorithm::RealTimeAlgorithm( const RealTimeAlgorithm& rhs ) : OptimizationAlgorithmBase( rhs ), ControlLaw( rhs )
{
    if( rhs.x0 != 0 ) x0 = new DVector(*rhs.x0);
    else              x0 = 0                  ;

    if( rhs.p0 != 0 ) p0 = new DVector(*rhs.p0);
    else              p0 = 0                  ;

    if( rhs.reference != 0 ) reference = new VariablesGrid(*rhs.reference);
    else                     reference = 0                         ;
}


RealTimeAlgorithm::~RealTimeAlgorithm( )
{
	clear( );
}



RealTimeAlgorithm& RealTimeAlgorithm::operator=( const RealTimeAlgorithm& rhs ){

    if( this != &rhs ){

		clear( );

		OptimizationAlgorithmBase::operator=( rhs );
		ControlLaw::operator=( rhs );

        if( rhs.x0 != 0 ) x0 = new DVector(*rhs.x0);
        else              x0 = 0                  ;

        if( rhs.p0 != 0 ) p0 = new DVector(*rhs.p0);
        else              p0 = 0                  ;

        if( rhs.reference != 0 ) reference = new VariablesGrid(*rhs.reference);
        else                     reference = 0                         ;
    }
    return *this;
}


ControlLaw* RealTimeAlgorithm::clone( ) const
{
	return new RealTimeAlgorithm( *this );
}



returnValue RealTimeAlgorithm::initializeAlgebraicStates(	const VariablesGrid& _xa_init
															)
{
	return OptimizationAlgorithmBase::initializeAlgebraicStates( _xa_init );
}


returnValue RealTimeAlgorithm::initializeAlgebraicStates(	const char* fileName
															)
{
	return OptimizationAlgorithmBase::initializeAlgebraicStates( fileName );
}


returnValue RealTimeAlgorithm::initializeControls(	const VariablesGrid& _u_init
													)
{
	return OptimizationAlgorithmBase::initializeControls( _u_init );
}


returnValue RealTimeAlgorithm::initializeControls(	const char* fileName
													)
{
	return OptimizationAlgorithmBase::initializeControls( fileName );
}



returnValue RealTimeAlgorithm::init( )
{
	if ( ( getStatus( ) == BS_READY ) && ( haveOptionsChanged( ) == BT_FALSE ) )
		return SUCCESSFUL_RETURN;

	returnValue returnvalue = OptimizationAlgorithmBase::init( this );

	setStatus( BS_READY );
	declareOptionsUnchanged( );

	return returnvalue;
}


returnValue RealTimeAlgorithm::init(	double startTime,
										const DVector& _x,
										const DVector& _p,
										const VariablesGrid& _yRef
										)
{
	/* 0) Consistency checks */
	int useImmediateFeedback = 0;
	get( USE_IMMEDIATE_FEEDBACK,useImmediateFeedback );

	int maxNumberOfSteps;
	get( MAX_NUM_ITERATIONS, maxNumberOfSteps );

// 	if ( ( (BooleanType)useImmediateFeedback == BT_TRUE ) && ( isInRealTimeMode( ) == BT_FALSE ) )
// 		return ACADOERROR( RET_NEED_TO_ACTIVATE_RTI );

	if ( ( (BooleanType)useImmediateFeedback == BT_TRUE ) && ( maxNumberOfSteps != 1 ) )
	{
		set( MAX_NUM_ITERATIONS,1 );
		ACADOWARNING( RET_IMMEDIATE_FEEDBACK_ONE_ITERATION );
	}


	/* 1) Initialize reference trajectory and real-time parameters. */
    clear( );

	if ( _yRef.isEmpty( ) == BT_FALSE )
		reference = new VariablesGrid( _yRef );

    x0 = new DVector(_x);

	if ( _p.isEmpty( ) == BT_FALSE )
		p0 = new DVector(_p);	


	/* 2) Initialize all sub-blocks. */
	if ( init( ) != SUCCESSFUL_RETURN )
		return ACADOERROR( RET_CONTROLLAW_INIT_FAILED );

	u.init( OptimizationAlgorithmBase::getNU( ) );
	u.setZero( );

	p.init( OptimizationAlgorithmBase::getNP( ) );
	p.setZero( );
	
	setStatus( BS_READY );

	return SUCCESSFUL_RETURN;
}



returnValue RealTimeAlgorithm::step(	double currentTime,
										const DVector& _x,
										const DVector& _p,
										const VariablesGrid& _yRef
										)
{
	if ( feedbackStep( currentTime,_x,_p ) != SUCCESSFUL_RETURN )
		return ACADOERROR( RET_CONTROLLAW_STEP_FAILED );

	if ( preparationStep( currentTime+getSamplingTime(),_yRef ) != SUCCESSFUL_RETURN )
		return ACADOERROR( RET_CONTROLLAW_STEP_FAILED );

	return SUCCESSFUL_RETURN;
}


returnValue RealTimeAlgorithm::feedbackStep(	double currentTime,
												const DVector &_x,
												const DVector &_p,
												const VariablesGrid& _yRef
												)
{
	if ( getStatus( ) != BS_READY )
		return ACADOERROR( RET_BLOCK_NOT_READY );

	if ( _x.getDim( ) != getNX() )
		return ACADOERROR( RET_INVALID_ARGUMENTS );

	if ( _p.getDim( ) > getNP() )
		return ACADOERROR( RET_INVALID_ARGUMENTS );

	nlpSolver->resetNumberOfSteps( );


	if ( isInRealTimeMode( ) == BT_TRUE )
	{
		if ( ( x0 != 0 ) && ( _x.isEmpty() == BT_FALSE ) )
			*x0 = _x;
		
		if ( ( p0 != 0 ) && ( _p.isEmpty() == BT_FALSE ) )
			*p0 = _p;
	}
	
	
	if ( performFeedbackStep( currentTime,_x,_p ) != SUCCESSFUL_RETURN )
		return ACADOERROR( RET_CONTROLLAW_STEP_FAILED );

	
	// if real-time mode is NOT enabled, perform usual step 
	// otherwise ignore current reference trajectory as it is
	// expected to be set during previous call to preparationStep
	if ( isInRealTimeMode( ) == BT_FALSE )
	{
		// solve at currentTime
// 		if ( solve( currentTime,_x,_p,_yRef ) != SUCCESSFUL_RETURN )
// 			return ACADOERROR( RET_CONTROLLAW_STEP_FAILED );
		returnValue returnvalue;

		int maxNumberOfSteps;
		get( MAX_NUM_ITERATIONS, maxNumberOfSteps );

		int terminateAtConvergence = 0;
		get( TERMINATE_AT_CONVERGENCE,terminateAtConvergence );

		while( nlpSolver->getNumberOfSteps( ) < maxNumberOfSteps )
		{
			returnvalue = performPreparationStep( _yRef,BT_FALSE );

			if ( ( returnvalue != CONVERGENCE_ACHIEVED ) && ( returnvalue != CONVERGENCE_NOT_YET_ACHIEVED ) )
				return ACADOERROR( RET_NLP_SOLUTION_FAILED );

			if ( ((BooleanType)terminateAtConvergence == BT_TRUE ) && ( returnvalue == CONVERGENCE_ACHIEVED ) )
				break;
			
			if ( performFeedbackStep( currentTime,_x,_p ) != SUCCESSFUL_RETURN )
				return ACADOERROR( RET_NLP_SOLUTION_FAILED );
		}
	}

	return SUCCESSFUL_RETURN;
}


returnValue RealTimeAlgorithm::preparationStep(	double nextTime,
												const VariablesGrid& _yRef
												)
{
	returnValue returnvalue = performPreparationStep( _yRef,BT_TRUE );
	if ( ( returnvalue != CONVERGENCE_ACHIEVED ) && ( returnvalue != CONVERGENCE_NOT_YET_ACHIEVED ) )
		return ACADOERROR( RET_CONTROLLAW_STEP_FAILED );

	return SUCCESSFUL_RETURN;
}



returnValue RealTimeAlgorithm::solve(	double startTime,
										const DVector &_x,
										const DVector &_p,
										const VariablesGrid& _yRef
										)
{
	if ( getStatus( ) == BS_NOT_INITIALIZED )
	{
		if ( init( startTime,_x ) != SUCCESSFUL_RETURN )
			return ACADOERROR( RET_OPTALG_INIT_FAILED );
	}

	if ( getStatus( ) != BS_READY )
    	return ACADOERROR( RET_OPTALG_INIT_FAILED );

	returnValue returnvalue;
	nlpSolver->resetNumberOfSteps( );

	int maxNumberOfSteps;
	get( MAX_NUM_ITERATIONS, maxNumberOfSteps );

	int terminateAtConvergence = 0;
	get( TERMINATE_AT_CONVERGENCE,terminateAtConvergence );

	while( nlpSolver->getNumberOfSteps( ) < maxNumberOfSteps )
	{
		if ( performFeedbackStep( startTime,_x,_p ) != SUCCESSFUL_RETURN )
			return ACADOERROR( RET_NLP_SOLUTION_FAILED );

		if ( nlpSolver->getNumberOfSteps( ) == maxNumberOfSteps )
			returnvalue = performPreparationStep( _yRef,BT_TRUE );
		else
			returnvalue = performPreparationStep( _yRef,BT_FALSE );

		if ( ( returnvalue != CONVERGENCE_ACHIEVED ) && ( returnvalue != CONVERGENCE_NOT_YET_ACHIEVED ) )
			return ACADOERROR( RET_NLP_SOLUTION_FAILED );

		if ( ((BooleanType)terminateAtConvergence == BT_TRUE ) && ( returnvalue == CONVERGENCE_ACHIEVED ) )
			break;
	}

	replot( PLOT_AT_END );

	return SUCCESSFUL_RETURN;
}



returnValue RealTimeAlgorithm::shift(	double timeShift
										)
{
	if ( acadoIsNegative( timeShift ) == BT_TRUE )
		timeShift = getSamplingTime( );
	
// 	printf( "shift!\n" );

	return nlpSolver->shiftVariables( timeShift );
}


returnValue RealTimeAlgorithm::setReference( const VariablesGrid &ref )
{
    if ( ( getStatus() != BS_READY ) && ( getStatus() != BS_RUNNING ) )
		return ACADOERROR( RET_OPTALG_INIT_FAILED );

// 	printf( "new ref!\n" );
// 	ref.print();
    return nlpSolver->setReference( ref );
}



uint RealTimeAlgorithm::getNX( ) const
{
	return OptimizationAlgorithmBase::getNX( );
}

uint RealTimeAlgorithm::getNXA( ) const
{
	return OptimizationAlgorithmBase::getNXA( );
}

uint RealTimeAlgorithm::getNU( ) const
{
	return OptimizationAlgorithmBase::getNU( );
}

uint RealTimeAlgorithm::getNP( ) const
{
	return OptimizationAlgorithmBase::getNP( );
}

uint RealTimeAlgorithm::getNW( ) const
{
	return OptimizationAlgorithmBase::getNW( );
}


uint RealTimeAlgorithm::getNY( ) const
{
	return OptimizationAlgorithmBase::getNX( );
}



double RealTimeAlgorithm::getLengthPredictionHorizon( ) const
{
	return getEndTime( ) - getStartTime( );
}


double RealTimeAlgorithm::getLengthControlHorizon( ) const
{
	return getLengthPredictionHorizon( );
}



BooleanType RealTimeAlgorithm::isDynamic( ) const
{
	return BT_TRUE;
}


BooleanType RealTimeAlgorithm::isStatic( ) const
{
	if ( isDynamic() == BT_TRUE )
		return BT_FALSE;
	else
		return BT_TRUE;
}


BooleanType RealTimeAlgorithm::isInRealTimeMode( ) const
{
	int maxNumIterations;
	get( MAX_NUM_ITERATIONS,maxNumIterations );
	
	if ( maxNumIterations > 1 )
		return BT_FALSE;

	int useRealtimeIterations;
	get( USE_REALTIME_ITERATIONS,useRealtimeIterations );

	return (BooleanType)useRealtimeIterations;
}



//
// PROTECTED MEMBER FUNCTIONS:
//

returnValue RealTimeAlgorithm::setupOptions( )
{
	// add NLP solver options
	addOption( MAX_NUM_ITERATIONS          , defaultMaxNumIterations        );
	addOption( KKT_TOLERANCE               , defaultKKTtolerance            );
	addOption( KKT_TOLERANCE_SAFEGUARD     , defaultKKTtoleranceSafeguard   );
	addOption( LEVENBERG_MARQUARDT         , defaultLevenbergMarguardt      );
	addOption( HESSIAN_PROJECTION_FACTOR   , defaultHessianProjectionFactor );
	addOption( PRINTLEVEL                  , defaultPrintlevel              );
	addOption( PRINT_COPYRIGHT             , defaultPrintCopyright          );
	addOption( HESSIAN_APPROXIMATION       , defaultHessianApproximation    );
	addOption( DYNAMIC_HESSIAN_APPROXIMATION, defaultDynamicHessianApproximation );
	addOption( DYNAMIC_SENSITIVITY         , defaultDynamicSensitivity      );
	addOption( OBJECTIVE_SENSITIVITY       , defaultObjectiveSensitivity    );
	addOption( CONSTRAINT_SENSITIVITY      , defaultConstraintSensitivity   );
	addOption( DISCRETIZATION_TYPE         , defaultDiscretizationType      );
	addOption( LINESEARCH_TOLERANCE        , defaultLinesearchTolerance     );
	addOption( MIN_LINESEARCH_PARAMETER    , defaultMinLinesearchParameter  );
	addOption( MAX_NUM_QP_ITERATIONS       , defaultMaxNumQPiterations      );
	addOption( HOTSTART_QP                 , defaultHotstartQP              );
	addOption( INFEASIBLE_QP_RELAXATION    , defaultInfeasibleQPrelaxation  );
	addOption( INFEASIBLE_QP_HANDLING      , defaultInfeasibleQPhandling    );
	addOption( USE_REALTIME_ITERATIONS     , defaultUseRealtimeIterations   );
	addOption( USE_REALTIME_SHIFTS         , defaultUseRealtimeShifts       );
	addOption( USE_IMMEDIATE_FEEDBACK      , defaultUseImmediateFeedback    );
	addOption( TERMINATE_AT_CONVERGENCE    , defaultTerminateAtConvergence  );
	addOption( SPARSE_QP_SOLUTION          , defaultSparseQPsolution        );
	addOption( GLOBALIZATION_STRATEGY      , defaultGlobalizationStrategy   );
	addOption( PRINT_SCP_METHOD_PROFILE    , defaultprintSCPmethodProfile   );

	// add integration options
	addOption( FREEZE_INTEGRATOR           , defaultFreezeIntegrator        );
	addOption( INTEGRATOR_TYPE             , defaultIntegratorType          );
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


returnValue RealTimeAlgorithm::setupLogging( )
{
// 	LogRecord tmp( LOG_AT_EACH_ITERATION,stdout,PS_DEFAULT,BT_FALSE );
// 
// 	tmp.addItem( LOG_NUM_NLP_ITERATIONS );
// 
// 	addLogRecord( tmp );
  
	return SUCCESSFUL_RETURN;
}


returnValue RealTimeAlgorithm::clear( )
{
	if( x0 != 0 )
	{
		delete x0;
		x0 = 0;
	}
	
    if( p0 != 0 )
	{
		delete p0;
		p0 = 0;
	}
	
    if( reference != 0 )
	{
		delete reference;
		reference = 0;
	}

	return SUCCESSFUL_RETURN;
}



returnValue RealTimeAlgorithm::allocateNlpSolver( Objective *F, DynamicDiscretization *G, Constraint *H )
{
	if( nlpSolver != 0 )
		delete nlpSolver;

	nlpSolver = new SCPmethod( this, F,G,H, isLinearQuadratic( F,G,H ) );

	return SUCCESSFUL_RETURN;
}


returnValue RealTimeAlgorithm::initializeNlpSolver( const OCPiterate& _userInit
													)
{
	returnValue returnvalue = SUCCESSFUL_RETURN;

	if( x0 != 0 ) returnvalue = _userInit.x->setVector( 0,x0[0] );
	if( p0 != 0 ) returnvalue = _userInit.p->setAllVectors( p0[0] );

	if( returnvalue != SUCCESSFUL_RETURN )
		return ACADOERROR(returnvalue);

	ACADO_TRY( nlpSolver->init( _userInit.x, _userInit.xa, _userInit.p, _userInit.u, _userInit.w ) );
    return SUCCESSFUL_RETURN;
}


returnValue RealTimeAlgorithm::initializeObjective( Objective *F )
{
	returnValue returnvalue = SUCCESSFUL_RETURN;

	if ( ( reference != 0 ) && ( F != 0 ) )
	{
// 		reference->print( "refer init" );
		returnvalue = F->setReference( *reference );
	}

	return returnvalue;
}



returnValue RealTimeAlgorithm::performFeedbackStep(	double currentTime,
													const DVector &_x,
													const DVector &_p
													)
{
	if ( getStatus( ) == BS_NOT_INITIALIZED )
	{
    	if ( init( currentTime,_x ) != SUCCESSFUL_RETURN )
			return ACADOERROR( RET_OPTALG_INIT_FAILED );
	}

	if ( getStatus( ) != BS_READY )
    	return ACADOERROR( RET_OPTALG_FEEDBACK_FAILED );


	if ( nlpSolver->feedbackStep( _x ) != SUCCESSFUL_RETURN ) //,_p
		return ACADOERROR( RET_OPTALG_FEEDBACK_FAILED );

// 	#ifdef SIM_DEBUG
	if ( nlpSolver->getFirstControl( u ) != SUCCESSFUL_RETURN )
		return ACADOERROR( RET_CONTROLLAW_STEP_FAILED );
// 	u.print("u after feedbackStep");
// 	#endif

	setStatus( BS_RUNNING );

	return SUCCESSFUL_RETURN;
}


returnValue RealTimeAlgorithm::performPreparationStep(	const VariablesGrid& _yRef,
														BooleanType isLastIteration
														)
{
// 	_yRef.print( "yRef" );
	
	if ( getStatus( ) != BS_RUNNING )
    	return ACADOERROR( RET_OPTALG_INIT_FAILED );

	int useRealTimeShifts = 0;
	get( USE_REALTIME_SHIFTS,useRealTimeShifts );

	// perform current step and check for convergence if desired
	returnValue returnvalueStep = nlpSolver->performCurrentStep( );
	
	if ( ( returnvalueStep != CONVERGENCE_ACHIEVED ) && ( returnvalueStep != CONVERGENCE_NOT_YET_ACHIEVED ) )
		return ACADOERROR( returnvalueStep );

	if ( isLastIteration == BT_TRUE )
	{
		if ( _yRef.isEmpty() == BT_FALSE )
			setReference( _yRef );

		if ( (BooleanType)useRealTimeShifts == BT_TRUE )
			shift( );
	}

	// prepare next step
	int terminateAtConvergence = 0;
	get( TERMINATE_AT_CONVERGENCE,terminateAtConvergence );

	if ( ((BooleanType)terminateAtConvergence == BT_TRUE ) && ( returnvalueStep == CONVERGENCE_ACHIEVED ) )
	{
		if ( _yRef.isEmpty() == BT_FALSE )
			setReference( _yRef );
		
		if ( (BooleanType)useRealTimeShifts == BT_TRUE )
			shift( );
	}

	returnValue returnvalue = nlpSolver->prepareNextStep( );
	if ( ( returnvalue != CONVERGENCE_ACHIEVED ) && ( returnvalue != CONVERGENCE_NOT_YET_ACHIEVED ) )
		return ACADOERROR( returnvalue );

	if ( ((BooleanType)terminateAtConvergence == BT_TRUE ) && ( returnvalueStep == CONVERGENCE_ACHIEVED ) )
		returnvalue = CONVERGENCE_ACHIEVED;

	setStatus( BS_READY );

	return returnvalue;
}



CLOSE_NAMESPACE_ACADO

// end of file.
