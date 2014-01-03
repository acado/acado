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
*    \file src/process/process.cpp
*    \author Boris Houska, Hans Joachim Ferreau
*    \date 24.08.2008
*/


#include <acado/process/process.hpp>

#ifdef WIN32
#define round( value ) floor( value + 0.5 )
#endif

BEGIN_NAMESPACE_ACADO


// #define SIM_DEBUG


//
// PUBLIC MEMBER FUNCTIONS:
//

Process::Process( ) : SimulationBlock( BN_PROCESS )
{
	setupOptions( );
	setupLogging( );

	nDynSys        = 0;
	dynamicSystems = 0;

	integrationMethod = 0;

	actuator = 0;
	sensor   = 0;

	processDisturbance = 0;

	lastTime = 0.0;

	setStatus( BS_NOT_INITIALIZED );
}


Process::Process(	const DynamicSystem& _dynamicSystem,
					IntegratorType _integratorType
					) : SimulationBlock( BN_PROCESS )
{
	setupOptions( );
	setupLogging( );

	nDynSys        = 0;
	dynamicSystems = 0;

	integrationMethod = 0;

	actuator = 0;
	sensor   = 0;

	processDisturbance = 0;

	if ( _dynamicSystem.getNumDynamicEquations( ) > 0 )
	{
		returnValue returnvalue = setDynamicSystem( _dynamicSystem,_integratorType );
		ASSERT( returnvalue == SUCCESSFUL_RETURN );
	}

	lastTime = 0.0;

	setStatus( BS_NOT_INITIALIZED );
}


Process::Process( const Process& rhs ) : SimulationBlock( rhs )
{
	x  = rhs.x;
	xa = rhs.xa;

	if ( rhs.dynamicSystems != 0 )
	{
		nDynSys = rhs.nDynSys;
		dynamicSystems = (DynamicSystem**) calloc( rhs.nDynSys,sizeof(DynamicSystem*) );
		for( uint i=0; i<rhs.nDynSys; ++i )
			dynamicSystems[i] = new DynamicSystem( *(rhs.dynamicSystems[i]) );
	}
	else
	{
		nDynSys        = 0;
		dynamicSystems = 0;
	}

	if ( rhs.integrationMethod != 0 )
		integrationMethod = new ShootingMethod( *(rhs.integrationMethod) );
	else
		integrationMethod = 0;

	if ( rhs.actuator != 0 )
		actuator = new Actuator( *(rhs.actuator) );
	else
		actuator = 0;

	if ( rhs.sensor != 0 )
		sensor = new Sensor( *(rhs.sensor) );
	else
		sensor = 0;

	if ( rhs.processDisturbance != 0 )
		processDisturbance = new Curve( *(rhs.processDisturbance) );
	else
		processDisturbance = 0;

	y = rhs.y;

	lastTime = rhs.lastTime;
}


Process::~Process( )
{
	clear( );
}


Process& Process::operator=( const Process& rhs )
{
    if ( this != &rhs )
    {
		clear( );

		SimulationBlock::operator=( rhs );

		x  = rhs.x;
		xa = rhs.xa;

		if ( rhs.dynamicSystems != 0 )
		{
			nDynSys = rhs.nDynSys;
			dynamicSystems = (DynamicSystem**) calloc( rhs.nDynSys,sizeof(DynamicSystem*) );
			for( uint i=0; i<rhs.nDynSys; ++i )
				dynamicSystems[i] = new DynamicSystem( *(rhs.dynamicSystems[i]) );
		}
		else
		{
			nDynSys        = 0;
			dynamicSystems = 0;
		}

		if ( rhs.integrationMethod != 0 )
			integrationMethod = new ShootingMethod( *(rhs.integrationMethod) );
		else
			integrationMethod = 0;
	
		if ( rhs.actuator != 0 )
			actuator = new Actuator( *(rhs.actuator) );
		else
			actuator = 0;

		if ( rhs.sensor != 0 )
			sensor = new Sensor( *(rhs.sensor) );
		else
			sensor = 0;

		if ( rhs.processDisturbance != 0 )
			processDisturbance = new Curve( *(rhs.processDisturbance) );
		else
			processDisturbance = 0;

		y = rhs.y;

		lastTime = rhs.lastTime;
    }

    return *this;
}



returnValue Process::setDynamicSystem(	const DynamicSystem& _dynamicSystem,
										IntegratorType _integratorType
										)
{
	if ( _dynamicSystem.hasImplicitSwitches( ) == BT_TRUE )
		return ACADOERROR( RET_NOT_YET_IMPLEMENTED );

	// setup simulation algorithm
	if ( integrationMethod != 0 )
		delete integrationMethod;

	integrationMethod = new ShootingMethod( this );

    Grid dummy( 0.0, 1.0 );   // bad idea ... QUICK HACK

	addStage( _dynamicSystem, dummy, _integratorType );


	integratorType = _integratorType;   // weird hack -- make clean later.

// NOTE: THE "integratorType" can
//          change during the use of the process, it might be better to load the integrator type from the options
//          every time the routine "simulate" is called. This enables the user to change options online.
// (SORRY, THIS IS NOT YET IMPLEMENTED - AT THE MOMENT THE  "integratorType"  GETS LOST.)

	y.init( _dynamicSystem.getNumOutputs( ),1 );
	y.setZero( );

	setStatus( BS_NOT_INITIALIZED );

	return SUCCESSFUL_RETURN;
}


returnValue Process::addDynamicSystemStage(	const DynamicSystem& _dynamicSystem,
											IntegratorType _integratorType
											)
{
	// setup simulation algorithm
	if ( integrationMethod == 0 )
		return setDynamicSystem( _dynamicSystem,_integratorType );

	if ( getNumStages( ) == 0 )
		return setDynamicSystem( _dynamicSystem,_integratorType );
	else
		return ACADOERROR( RET_NOT_YET_IMPLEMENTED );
}



returnValue Process::setActuator(	const Actuator& _actuator
									)
{
	if ( _actuator.isDefined( ) == BT_TRUE )
	{
		if ( actuator != 0 )
			*actuator = _actuator;
		else
			actuator = new Actuator( _actuator );

		setStatus( BS_NOT_INITIALIZED );
	}

	return SUCCESSFUL_RETURN;
}


returnValue Process::setSensor(	const Sensor& _sensor
								)
{
	if ( _sensor.isDefined( ) == BT_TRUE )
	{
		if ( sensor != 0 )
			*sensor = _sensor;
		else
			sensor = new Sensor( _sensor );

		setStatus( BS_NOT_INITIALIZED );
	}

	return SUCCESSFUL_RETURN;
}



returnValue Process::setProcessDisturbance(	const Curve& _processDisturbance
											)
{
	if ( processDisturbance != 0 )
		*processDisturbance = _processDisturbance;
	else
		processDisturbance = new Curve( _processDisturbance );

	setStatus( BS_NOT_INITIALIZED );

	return SUCCESSFUL_RETURN;
}


returnValue Process::setProcessDisturbance(	const VariablesGrid& _processDisturbance
											)
{
	if ( _processDisturbance.isEmpty( ) == BT_TRUE )
		return ACADOERROR( RET_INVALID_ARGUMENTS );

    Curve _processDisturbanceCurve;
    _processDisturbanceCurve.add( _processDisturbance );

	return setProcessDisturbance( _processDisturbanceCurve );
}


returnValue Process::setProcessDisturbance(	const char* _processDisturbance
											)
{
    VariablesGrid _processDisturbanceFromFile;
    _processDisturbanceFromFile.read( _processDisturbance );

	if ( _processDisturbanceFromFile.isEmpty( ) == BT_TRUE )
		return ACADOERROR( RET_FILE_CAN_NOT_BE_OPENED );

	return setProcessDisturbance( _processDisturbanceFromFile );
}



returnValue Process::initializeStartValues(	const DVector& _xStart,
											const DVector& _xaStart
											)
{
	x = _xStart;

	if ( _xaStart.getDim( ) > 0 )
		xa = _xaStart;

	setStatus( BS_NOT_INITIALIZED );

	return SUCCESSFUL_RETURN;
}


returnValue Process::initializeAlgebraicStates(	const DVector& _xaStart
												)
{
	return initializeStartValues( x,_xaStart );
}



returnValue Process::init(	double _startTime,
							const DVector& _xStart,
							const DVector& _uStart,
							const DVector& _pStart
							)
{
	DVector uStart;
	DVector pStart;

	/* 1) Assign values */
	lastTime = _startTime;

	if ( _xStart.getDim( ) > 0 )
		x = _xStart;
	else
	{
		if ( x.getDim() == 0 )
		{
			x.init( getNX() );
			x.setZero( );
		}
	}

	if ( ( xa.getDim( ) == 0 ) && ( getNXA( ) > 0 ) )
	{
		xa.init( getNXA() );
		xa.setZero( );
	}

	if ( _uStart.getDim( ) > 0 )
		uStart = _uStart;
	else
	{
		uStart.init( getNU() );
		uStart.setZero( );
	}

	if ( _pStart.getDim( ) > 0 )
		pStart = _pStart;
	else
	{
		pStart.init( getNP() );
		pStart.setZero( );
	}
	

	#ifdef SIM_DEBUG
	uStart.print("uStart(0)");
	#endif


	DynamicSystem*  dynSys  = dynamicSystems[0];
	OutputFcn     outputFcn = dynSys->getOutputFcn( );
	
	/* 2) Consistency checks. */
	if ( getNumStages( ) == 0 )
		return ACADOERROR( RET_NO_DYNAMICSYSTEM_SPECIFIED );

	if ( getNX( ) != x.getDim( ) )
		return ACADOERROR( RET_DIFFERENTIAL_STATE_DIMENSION_MISMATCH );

	if ( getNXA( ) != xa.getDim( ) )
		return ACADOERROR( RET_ALGEBRAIC_STATE_DIMENSION_MISMATCH );

	if ( hasActuator( ) == BT_TRUE )
	{
		if ( actuator->getNU( ) != getNU( ) )
			return ACADOERROR( RET_CONTROL_DIMENSION_MISMATCH );

		if ( actuator->getNP( ) != getNP( ) )
			return ACADOERROR( RET_PARAMETER_DIMENSION_MISMATCH );

		if ( dynSys->isDiscretized( ) == BT_TRUE )
		{
			if ( acadoIsInteger( actuator->getSamplingTime( ) / dynSys->getSampleTime( ) ) == BT_FALSE )
				return ACADOERROR( RET_INCOMPATIBLE_ACTUATOR_SAMPLING_TIME );
		}
	}

	if ( hasSensor( ) == BT_TRUE )
	{
		if ( sensor->getNY( ) != getNY( ) )
			return ACADOERROR( RET_OUTPUT_DIMENSION_MISMATCH );

		if ( dynSys->isDiscretized( ) == BT_TRUE )
		{
			if ( acadoIsInteger( sensor->getSamplingTime( ) / dynSys->getSampleTime( ) ) == BT_FALSE )
				return ACADOERROR( RET_INCOMPATIBLE_SENSOR_SAMPLING_TIME );
		}
	}

	if ( hasProcessDisturbance( ) == BT_TRUE )
	{
		for( uint i=0; i<getNumStages( ); ++i )
			if ( (int) getNW( i ) > processDisturbance->getDim( ) )
				return ACADOERROR( RET_DISTURBANCE_DIMENSION_MISMATCH );

		if ( processDisturbance->isInTimeDomain( _startTime ) == BT_FALSE )
			return ACADOERROR( RET_WRONG_DISTURBANCE_HORIZON );
	}
	else
	{
		if ( getNW( ) > 0 )
			return ACADOERROR( RET_DISTURBANCE_DIMENSION_MISMATCH );
	}


	/* 3) Initialize all sub-blocks. */
	if ( hasActuator( ) == BT_TRUE )
		if ( actuator->init( _startTime,_uStart,_pStart ) != SUCCESSFUL_RETURN )
			return ACADOERROR( RET_PROCESS_INIT_FAILED );

	if ( hasSensor( ) == BT_TRUE )
		if ( sensor->init( _startTime ) != SUCCESSFUL_RETURN )
			return ACADOERROR( RET_PROCESS_INIT_FAILED );


	/* 4) Evalute output function with initial values */
	Grid currentGrid( 1 );
	currentGrid.setTime( _startTime );

	// get process disturbances
	DVector _wStart;

	if ( hasProcessDisturbance( ) == BT_TRUE )
	{
		if ( processDisturbance->evaluate( _startTime,_wStart ) != SUCCESSFUL_RETURN )
			return ACADOERROR( RET_PROCESS_INIT_FAILED );
	}

	if ( outputFcn.isDefined( ) == BT_TRUE )
	{
		// y = outputFcn(x,xa,p,u,w)
		y.init( dynSys->getNumOutputs( ),currentGrid,VT_OUTPUT );

		DVector yTmp( dynSys->getNumOutputs( ) );
 //     yTmp = outputFcn.evaluate( 0, _startTime, x, xa, _pStart, _uStart, _wStart );

		y.setVector( 0,yTmp );
	}
	else
	{
		// y = x
		y.init( _xStart.getDim( ),currentGrid,VT_OUTPUT );
		y.setVector( 0,_xStart );
	}


	setStatus( BS_READY );

	return SUCCESSFUL_RETURN;
}



returnValue Process::step(	const VariablesGrid& _u,
							const VariablesGrid& _p
							)
{
	/* Consistency checks. */
	if ( getStatus( ) != BS_READY )
		return ACADOERROR( RET_BLOCK_NOT_READY );

	if ( checkInputConsistency( _u,_p ) != SUCCESSFUL_RETURN )
		return ACADOERROR( RET_PROCESS_STEP_FAILED );

	if ( acadoIsEqual( _u.getFirstTime( ),lastTime ) == BT_FALSE )
		return ACADOERROR( RET_INVALID_ARGUMENTS );


	/* 0) Log original process inputs */
	if ( _u.isEmpty( ) == BT_FALSE )
		setLast( LOG_NOMINAL_CONTROLS,_u );

	if ( _p.isEmpty( ) == BT_FALSE )
		setLast( LOG_NOMINAL_PARAMETERS,_p );


	/* 1) Get actuator outputs. */
	VariablesGrid uAct = _u;
	VariablesGrid pAct = _p;

	if ( hasActuator( ) == BT_TRUE )
	{
		if ( actuator->step( uAct,pAct ) != SUCCESSFUL_RETURN )
			return ACADOERROR( RET_PROCESS_STEP_FAILED );
	}

// 	printf("uAct:\n" );
// 	uAct.print();
// 	printf("pAct:\n" );
// 	pAct.print();

	/* 2) Get process disturbances. */
	VariablesGrid _w;

	if ( hasProcessDisturbance( ) == BT_TRUE )
	{
		if ( processDisturbance->evaluate( _u.getFirstTime( ),_u.getLastTime( ),_w ) != SUCCESSFUL_RETURN )
			return ACADOERROR( RET_PROCESS_STEP_FAILED_DISTURBANCE );
	}
// 	_w.print( "read w" );

	/* 3) Call SimulationByIntegration and evaluate output function. */
	Grid commonGrid, tmpGrid;

	uAct.getGrid( commonGrid );
	
	if ( hasProcessDisturbance( ) == BT_TRUE )
	{
		_w.getGrid( tmpGrid );
		commonGrid & tmpGrid;
	}
// 	commonGrid.print();

	uAct.refineGrid( commonGrid );
	pAct.refineGrid( commonGrid );

	if ( hasProcessDisturbance( ) == BT_TRUE )
		_w.refineGrid( commonGrid );

//	_w.print( "passed w" );
	if ( simulate( uAct,pAct,_w ) != SUCCESSFUL_RETURN )
		return ACADOERROR( RET_PROCESS_STEP_FAILED );

// 	y.print("ySim");
	
	/* 4) Get sensor output. */
	if ( hasSensor( ) == BT_TRUE )
	{
		if ( sensor->step( y ) != SUCCESSFUL_RETURN )
			return ACADOERROR( RET_PROCESS_STEP_FAILED );
	}

// 	y.print("ySens");
	setLast( LOG_PROCESS_OUTPUT,y );
	//logCollection.setLast( LOG_SAMPLED_PROCESS_OUTPUT,y );

	/* 5) Update lastTime. */
	lastTime = _u.getLastTime( );

	// plot results
	replot( PLOT_AT_END );
	
	return SUCCESSFUL_RETURN;
}


returnValue Process::step(	const VariablesGrid& _u,
							const DVector& _p
							)
{
	VariablesGrid pTmp( _p.getDim( ),_u.getFirstTime( ),_u.getLastTime( ),_u.getNumPoints(),VT_PARAMETER );
	for( uint i=0; i<_u.getNumPoints(); ++i )
		pTmp.setVector( i,_p );

	return step( _u,pTmp );
}


returnValue Process::step(	double startTime,
							double endTime,
							const DVector& _u,
							const DVector& _p
							)
{
	VariablesGrid uTmp( _u.getDim( ),startTime,endTime,2,VT_CONTROL );
	uTmp.setVector( 0,_u );
	uTmp.setVector( 1,_u );

	VariablesGrid pTmp( _p.getDim( ),startTime,endTime,2,VT_PARAMETER );
	pTmp.setVector( 0,_p );
	pTmp.setVector( 1,_p );

	return step( uTmp,pTmp );
}



returnValue Process::run(	const VariablesGrid& _u,
							const VariablesGrid& _p
							)
{
	// Ensure that process is initialised
	if ( getStatus( ) == BS_NOT_INITIALIZED )
	{
		if ( init( _u.getFirstTime( ),x,_u.getFirstVector( ),_p.getFirstVector( ) ) != SUCCESSFUL_RETURN )
			return ACADOERROR( RET_PROCESS_RUN_FAILED );
	}

	return step( _u,_p );
}


returnValue Process::run(	const VariablesGrid& _u,
							const DVector& _p
							)
{
	VariablesGrid pTmp( _p.getDim( ),_u.getFirstTime( ),_u.getLastTime( ),_u.getNumPoints(),VT_PARAMETER );
	for( uint i=0; i<_u.getNumPoints(); ++i )
		pTmp.setVector( i,_p );

	return run( _u,pTmp );
}


returnValue Process::run(	double startTime,
							double endTime,
							const DVector& _u,
							const DVector& _p
							)
{
	VariablesGrid uTmp( _u.getDim( ),startTime,endTime,2,VT_CONTROL );
	uTmp.setVector( 0,_u );
	uTmp.setVector( 1,_u );

	VariablesGrid pTmp( _p.getDim( ),startTime,endTime,2,VT_PARAMETER );
	pTmp.setVector( 0,_p );
	pTmp.setVector( 1,_p );

	return run( uTmp,pTmp );
}



returnValue Process::replot(	PlotFrequency _frequency
								)
{
	int controlPlotting, parameterPlotting, outputPlotting;

	get( CONTROL_PLOTTING,   controlPlotting   );
	get( PARAMETER_PLOTTING, parameterPlotting );
	get( OUTPUT_PLOTTING,    outputPlotting    );

	if ( (ProcessPlotName)controlPlotting == PLOT_NOMINAL )
		plotCollection.enableNominalControls( );
	else
		plotCollection.disableNominalControls( );
	
	if ( (ProcessPlotName)parameterPlotting == PLOT_NOMINAL )
		plotCollection.enableNominalParameters( );
	else
		plotCollection.disableNominalParameters( );
	
	if ( (ProcessPlotName)outputPlotting == PLOT_NOMINAL )
		plotCollection.enableNominalOutputs( );
	else
		plotCollection.disableNominalOutputs( );

	return Plotting::replot( _frequency );
}



//
// PROTECTED MEMBER FUNCTIONS:
//

returnValue Process::setupOptions( )
{
	//printf( "Process::setupOptions( ) called.\n" );

	addOption( SIMULATION_ALGORITHM        , defaultSimulationAlgorithm     );
	addOption( CONTROL_PLOTTING            , defaultControlPlotting         );
	addOption( PARAMETER_PLOTTING          , defaultParameterPlotting       );
	addOption( OUTPUT_PLOTTING             , defaultOutputPlotting          );
	
	// add integration options
	addOption( FREEZE_INTEGRATOR           , BT_FALSE                       );
	addOption( INTEGRATOR_TYPE             , INT_BDF                        );
	addOption( FEASIBILITY_CHECK           , defaultFeasibilityCheck        );
	addOption( PLOT_RESOLUTION             , defaultPlotResoltion           );
	
	// add integrator options
	addOption( MAX_NUM_INTEGRATOR_STEPS    , defaultMaxNumSteps             );
	addOption( INTEGRATOR_TOLERANCE        , defaultIntegratorTolerance     );
	addOption( ABSOLUTE_TOLERANCE          , 1.0e-12       );
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


returnValue Process::setupLogging( )
{
	LogRecord tmp(LOG_AT_EACH_ITERATION, PS_DEFAULT);

	tmp.addItem( LOG_DIFFERENTIAL_STATES      );
	tmp.addItem( LOG_ALGEBRAIC_STATES         );
	tmp.addItem( LOG_PARAMETERS               );
	tmp.addItem( LOG_CONTROLS                 );
	tmp.addItem( LOG_DISTURBANCES             );
	tmp.addItem( LOG_INTERMEDIATE_STATES      );

	tmp.addItem( LOG_DISCRETIZATION_INTERVALS );
	tmp.addItem( LOG_STAGE_BREAK_POINTS       );

	tmp.addItem( LOG_NOMINAL_CONTROLS );
	tmp.addItem( LOG_NOMINAL_PARAMETERS );

	tmp.addItem( LOG_SIMULATED_DIFFERENTIAL_STATES );
	tmp.addItem( LOG_SIMULATED_ALGEBRAIC_STATES );
	tmp.addItem( LOG_SIMULATED_CONTROLS );
	tmp.addItem( LOG_SIMULATED_PARAMETERS );
	tmp.addItem( LOG_SIMULATED_DISTURBANCES );
	tmp.addItem( LOG_SIMULATED_OUTPUT );

	tmp.addItem( LOG_PROCESS_OUTPUT );

	addLogRecord( tmp );

	return SUCCESSFUL_RETURN;
}



returnValue Process::addStage(	const DynamicSystem  &_dynamicSystem,
								const Grid           &stageIntervals,
								const IntegratorType &_integratorType
								)
{
	if ( _dynamicSystem.hasImplicitSwitches( ) == BT_TRUE )
		return ACADOERROR( RET_NOT_YET_IMPLEMENTED );

	// add dynamic system to list...
	++nDynSys;

	dynamicSystems = (DynamicSystem**) realloc( dynamicSystems,nDynSys*sizeof(DynamicSystem*) );
	dynamicSystems[ nDynSys-1 ] = new DynamicSystem( _dynamicSystem );

	// ... and add stage to integration algorithm
	return integrationMethod->addStage( _dynamicSystem, stageIntervals, _integratorType );
}


returnValue Process::clear( )
{
	if ( dynamicSystems != 0 )
	{
		for( uint i=0; i<nDynSys; ++i )
			delete dynamicSystems[i];

		free( dynamicSystems );
		dynamicSystems = 0;
	}

	nDynSys = 0;
	
	if ( integrationMethod != 0 )
		delete integrationMethod;

	if ( actuator != 0 )
	 	delete actuator;

	if ( sensor != 0 )
	 	delete sensor;

	if ( processDisturbance != 0 )
		delete processDisturbance;

	return SUCCESSFUL_RETURN;
}



returnValue Process::simulate(	const VariablesGrid& _u,
								const VariablesGrid& _p,
								const VariablesGrid& _w
								)
{
	DynamicSystem*       dynSys    = dynamicSystems[0];
	DifferentialEquation diffEqn   = dynSys->getDifferentialEquation( );
	OutputFcn            outputFcn = dynSys->getOutputFcn( );

	Grid currentGrid;
	_u.getGrid( currentGrid );

	OCPiterate iter;

	// allocate memory for call to simulation algorithm
	// and initialise states with start value
	DVector xComponents = diffEqn.getDifferentialStateComponents( );

	iter.x = new VariablesGrid( (int)round( xComponents.getMax( )+1.0 ),currentGrid,VT_DIFFERENTIAL_STATE );
	for( uint i=0; i<xComponents.getDim( ); ++i )
		iter.x->operator()( 0,(int)xComponents(i) ) = x(i);

//	DVector xaComponents;
	if ( getNXA() > 0 )
	{
		iter.xa = new VariablesGrid( getNXA(),currentGrid,VT_ALGEBRAIC_STATE );
		iter.xa->setVector( 0,xa );
	}

	if ( ( _u.getNumPoints( ) > 0 ) && ( _u.getNumValues( ) > 0 ) )
		iter.u = new VariablesGrid( _u );

	if ( ( _p.getNumPoints( ) > 0 ) && ( _p.getNumValues( ) > 0 ) )
		iter.p = new VariablesGrid( _p );

	if ( ( _w.getNumPoints( ) > 0 ) && ( _w.getNumValues( ) > 0 ) )
		iter.w = new VariablesGrid( _w );


	// simulate process...

// 	delete integrationMethod;
// 	integrationMethod = new ShootingMethod( this );
	ACADO_TRY( integrationMethod->clear() );
	ACADO_TRY( integrationMethod->addStage( *dynSys, iter.getUnionGrid(), integratorType ) );

// 	iter.u->print( "u" );
	
	#ifdef SIM_DEBUG
// 	printf("process step \n");
	(iter.x->getVector(0)).print("iter.x(0)");
	(iter.u->getVector(0)).print("iter.u(0)");
// 	(iter.x->getVector(1)).print("iter.x(1)");
// 	(iter.u->getVector(1)).print("iter.u(1)");
	#endif
	
	iter.enableSimulationMode();
	ACADO_TRY( integrationMethod->evaluate( iter ) );


	// calculate sampled output
// 	if ( calculateOutput( outputFcn,iter.x,xComponents,iter.xa,iter.p,iter.u,iter.w, &y ) != SUCCESSFUL_RETURN )
// 		return ACADOERROR( RET_PROCESS_STEP_FAILED );


	// log simulation results
	VariablesGrid xCont;
	VariablesGrid xContFull;
	VariablesGrid xaCont;
	VariablesGrid pCont;
	VariablesGrid uCont;
	VariablesGrid wCont;

	if ( iter.x != 0 )
	{
		integrationMethod->getLast( LOG_DIFFERENTIAL_STATES,xContFull );

		xCont.init( getNX( ),xContFull.getTimePoints( ) );
		projectToComponents( xContFull,xComponents, xCont );
		setLast( LOG_SIMULATED_DIFFERENTIAL_STATES,xCont );
	}

	if ( iter.xa != 0 )
	{
		integrationMethod->getLast( LOG_ALGEBRAIC_STATES,xaCont );
		setLast( LOG_SIMULATED_ALGEBRAIC_STATES,xaCont );
	}

	if ( iter.p != 0 )
	{
		integrationMethod->getLast( LOG_PARAMETERS,pCont );
		setLast(LOG_SIMULATED_PARAMETERS,
				VariablesGrid(pCont.getCoarsenedGrid(_p.getTimePoints())));
	}

	if ( iter.u != 0 )
	{
		integrationMethod->getLast( LOG_CONTROLS,uCont );
		setLast(LOG_SIMULATED_CONTROLS,
				VariablesGrid(uCont.getCoarsenedGrid(_u.getTimePoints())));
	}

	if ( iter.w != 0 )
	{
		integrationMethod->getLast( LOG_DISTURBANCES,wCont );
		setLast(LOG_SIMULATED_DISTURBANCES,
				VariablesGrid(wCont.getCoarsenedGrid(_w.getTimePoints())));
	}


	// calculate and log continuous output
// 	VariablesGrid yTmp;

	if ( calculateOutput( outputFcn,&xContFull,xComponents,&xaCont,&pCont,&uCont,&wCont, &y ) != SUCCESSFUL_RETURN )
		return ACADOERROR( RET_PROCESS_STEP_FAILED );
	setLast( LOG_SIMULATED_OUTPUT,y );


	for( uint i=0; i<xComponents.getDim( ); ++i )
		x(i) = iter.x->operator()( iter.x->getNumPoints( )-1,(int)xComponents(i) );

	if ( iter.xa != 0 )
		xa = iter.xa->getLastVector( );

	return SUCCESSFUL_RETURN;
}



/* identitical to same function within the class Actuator! */
returnValue Process::checkInputConsistency(	const VariablesGrid& _u,
											const VariablesGrid& _p
											) const
{
	if ( _u.getNumPoints( ) < 2 )
		return ACADOERROR( RET_INVALID_ARGUMENTS );

	if ( _u.getNumRows( ) != getNU( ) )
		return ACADOERROR( RET_CONTROL_DIMENSION_MISMATCH );

	if ( _p.isEmpty( ) == BT_TRUE )
	{
		if ( getNP( ) > 0 )
			return ACADOERROR( RET_PARAMETER_DIMENSION_MISMATCH );
	}
	else
	{
		if ( _p.getNumPoints( ) < 2 )
			return ACADOERROR( RET_INVALID_ARGUMENTS );

		if ( _p.getNumRows( ) < getNP( ) )
			return ACADOERROR( RET_PARAMETER_DIMENSION_MISMATCH );

		if ( acadoIsEqual( _u.getFirstTime( ),_p.getFirstTime( ) ) == BT_FALSE )
			return ACADOERROR( RET_INVALID_ARGUMENTS );

		if ( acadoIsEqual( _u.getLastTime( ),_p.getLastTime( ) ) == BT_FALSE )
			return ACADOERROR( RET_INVALID_ARGUMENTS );
	}

	return SUCCESSFUL_RETURN;
}



returnValue Process::calculateOutput(	OutputFcn& _outputFcn,
										const VariablesGrid* _x,
										const DVector& _xComponents,
										const VariablesGrid* _xa,
										const VariablesGrid* _p,
										const VariablesGrid* _u,
										const VariablesGrid* _w,
										VariablesGrid* _output
										) const
{
	if ( _output == 0 )
		return ACADOERROR( RET_INVALID_ARGUMENTS );

	if ( _outputFcn.isDefined( ) == BT_TRUE )
	{
		VariablesGrid pTmp = _p->getRefinedGrid( *_x );
		VariablesGrid uTmp = _u->getRefinedGrid( *_x );

		// y = outputFcn(x,xa,p,u,w)
		_outputFcn.evaluate( _x,_xa,&pTmp,&uTmp,_w, _output );
	}
	else
	{
		// output = x
		Grid outputGrid;
		_x->getGrid( outputGrid );
		_output->init( _xComponents.getDim( ),outputGrid );

		projectToComponents( *_x,_xComponents, *_output );
	}

	return SUCCESSFUL_RETURN;
}



returnValue Process::projectToComponents(	const VariablesGrid& _x,
											const DVector& _xComponents,
											VariablesGrid& _output
											) const
{
	for( uint j=0; j<_x.getNumPoints( ); ++j )
		for( uint i=0; i<_xComponents.getDim( ); ++i )
			_output(j,i) = _x( j,(int)_xComponents(i) );

	return SUCCESSFUL_RETURN;
}



CLOSE_NAMESPACE_ACADO

// end of file.
