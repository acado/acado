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
 *    \file src/transfer_device/sensor.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 13.06.2008
 */


#include <acado/transfer_device/sensor.hpp>



BEGIN_NAMESPACE_ACADO



//
// PUBLIC MEMBER FUNCTIONS:
//

Sensor::Sensor( ) : TransferDevice( BN_SENSOR )
{
}


Sensor::Sensor(	uint _nY,
				double _samplingTime
				) : TransferDevice( _nY,BN_SENSOR,_samplingTime )
{
}


Sensor::Sensor( const Sensor& rhs ) : TransferDevice( rhs )
{
}


Sensor::~Sensor( )
{
}


Sensor& Sensor::operator=( const Sensor& rhs )
{
    if ( this != &rhs )
    {
		TransferDevice::operator=( rhs );

		
    }

    return *this;
}


returnValue Sensor::setOutputNoise(	const Noise& _noise,
									double _noiseSamplingTime
									)
{
	if ( _noise.getDim( ) != getNY( ) )
		return ACADOERROR( RET_INVALID_ARGUMENTS );

	for( uint i=0; i<getNY( ); ++i )
	{
		if ( additiveNoise[i] != 0 )
			delete additiveNoise[i];

		additiveNoise[i] = _noise.clone( i );
	}

	noiseSamplingTimes.setAll( _noiseSamplingTime );

	return SUCCESSFUL_RETURN;
}


returnValue Sensor::setOutputNoise(	uint idx,
									const Noise& _noise,
									double _noiseSamplingTime
									)
{
	if ( ( idx >= getNY( ) ) || ( _noise.getDim( ) != 1 ) )
		return ACADOERROR( RET_INVALID_ARGUMENTS );

	if ( additiveNoise[idx] != 0 )
		delete additiveNoise[idx];

	additiveNoise[idx] = _noise.clone( );

	if ( ( idx > 0 ) && ( acadoIsEqual( _noiseSamplingTime, noiseSamplingTimes(0) ) == BT_FALSE ) )
		ACADOWARNING( RET_NO_DIFFERENT_NOISE_SAMPLING_FOR_DISCRETE );

	noiseSamplingTimes.setAll( _noiseSamplingTime ); // should be changed later

	return SUCCESSFUL_RETURN;
}



returnValue Sensor::setOutputDeadTimes(	const DVector& _deadTimes
										)
{
	if ( _deadTimes.getDim( ) != getNY( ) )
		return ACADOERROR( RET_INVALID_ARGUMENTS );

	if ( _deadTimes.getMin( ) < 0.0 )
		return ACADOERROR( RET_INVALID_ARGUMENTS );

	if ( deadTimes.getDim( ) == 0 )
		return ACADOERROR( RET_MEMBER_NOT_INITIALISED );

	for( uint i=0; i<getNY(); ++i )
		deadTimes( i ) = _deadTimes( i );

	return SUCCESSFUL_RETURN;
}


returnValue Sensor::setOutputDeadTimes(	double _deadTime
										)
{
	if ( _deadTime < 0.0 )
		return ACADOERROR( RET_INVALID_ARGUMENTS );

	if ( deadTimes.getDim( ) == 0 )
		return ACADOERROR( RET_MEMBER_NOT_INITIALISED );

	for( uint i=0; i<getNY(); ++i )
		deadTimes( i ) = _deadTime;

	return SUCCESSFUL_RETURN;
}


returnValue Sensor::setOutputDeadTime(	uint idx,
										double _deadTime
										)
{
	if ( idx >= getNY( ) )
		return ACADOERROR( RET_INVALID_ARGUMENTS );

	if ( _deadTime < 0.0 )
		return ACADOERROR( RET_INVALID_ARGUMENTS );

	if ( deadTimes.getDim( ) == 0 )
		return ACADOERROR( RET_MEMBER_NOT_INITIALISED );

	deadTimes( idx ) = _deadTime;

	return SUCCESSFUL_RETURN;
}



returnValue Sensor::init(	double _startTime,
							const DVector& _startValue
							)
{
	if ( TransferDevice::init( _startTime,_startValue ) != SUCCESSFUL_RETURN )
		return ACADOERROR( RET_SENSOR_INIT_FAILED );

	return SUCCESSFUL_RETURN;
}



returnValue Sensor::step(	VariablesGrid& _y
							)
{
	// consistency checks
	if ( getStatus( ) != BS_READY )
		return ACADOERROR( RET_BLOCK_NOT_READY );

	if ( ( _y.getNumPoints( ) < 2 ) || ( _y.getNumRows( ) != getNY( ) ) )
		return ACADOERROR( RET_INVALID_ARGUMENTS );

	if ( acadoIsEqual( _y.getFirstTime( ),lastSignal.getLastTime( ) ) == BT_FALSE )
		return ACADOERROR( RET_INVALID_ARGUMENTS );


	// delay inputs and store last signal
	if ( delaySensorOutput( _y ) != SUCCESSFUL_RETURN )
		return ACADOERROR( RET_SENSOR_STEP_FAILED );
	
	// add sensor noise
	if ( addSensorNoise( _y ) != SUCCESSFUL_RETURN )
		return ACADOERROR( RET_SENSOR_STEP_FAILED );

	return SUCCESSFUL_RETURN;
}



//
// PROTECTED MEMBER FUNCTIONS:
//


returnValue Sensor::delaySensorOutput(	VariablesGrid& _y
										)
{
	if ( hasDeadTime( ) == BT_FALSE )
	{
		// store last signal
		DVector tmp = _y.getLastVector( );

		lastSignal.init( tmp );
		lastSignal.setTime( 0,_y.getLastTime( ) );
	
		return SUCCESSFUL_RETURN;
	}
	else
	{
		double startTime = _y.getFirstTime( );
		double endTime   = _y.getLastTime( );

		// determine variables grid of delayed input
		VariablesGrid yDelayed;
		if ( getDelayedOutputGrid( _y, yDelayed ) != SUCCESSFUL_RETURN )
			return ACADOERROR( RET_DELAYING_OUTPUTS_FAILED );

		// store last signal
		lastSignal = yDelayed.getTimeSubGrid( yDelayed.getFloorIndex( endTime ),yDelayed.getLastIndex( ) );

		// crop delayed signal to current horizon
		_y = yDelayed.getTimeSubGrid( yDelayed.getFloorIndex( startTime ),yDelayed.getFloorIndex( endTime ) );

		return SUCCESSFUL_RETURN;
	}
}


returnValue Sensor::getDelayedOutputGrid(	const VariablesGrid& _y,
											VariablesGrid& _yDelayed
											) const
{
	// determine common time grid for delayed outputs:
	Grid delayedOutputTimeGrid = lastSignal.getTimePoints( );

	// make sure that last time instant of horizon lies within the grid
	if ( acadoIsEqual( lastSignal.getLastTime(),_y.getLastTime( ) ) == BT_FALSE )
		delayedOutputTimeGrid.addTime( _y.getLastTime( ) );

	// add grids of all delayed output components
	for( uint i=0; i<getNY( ); ++i )
		delayedOutputTimeGrid.merge( _y.getTimePoints( ).shiftTimes( deadTimes(i) ),MM_REPLACE );

	VariablesGrid tmp;

	// setup common variables grid for delayed inputs
	_yDelayed.init( );

	for( uint i=0; i<getNY( ); ++i )
	{
		tmp = lastSignal( i );
		tmp.merge( _y( i ).shiftTimes( deadTimes(i) ),MM_REPLACE,BT_FALSE );
		tmp.refineGrid( delayedOutputTimeGrid );

		_yDelayed.appendValues( tmp );
	}

	return SUCCESSFUL_RETURN;
}



returnValue Sensor::addSensorNoise(	VariablesGrid& _y
									) const
{
	if ( hasNoise( ) == BT_FALSE )
		return SUCCESSFUL_RETURN;

	// generate current noise
	VariablesGrid currentNoise;

	if ( generateNoise( _y.getFirstTime(),_y.getLastTime(),currentNoise ) != SUCCESSFUL_RETURN )
		return ACADOERROR( RET_GENERATING_NOISE_FAILED );

	// determine common grid
	Grid commonGrid, tmpGrid;
	_y.getGrid( commonGrid );
	currentNoise.getGrid( tmpGrid );
	commonGrid.merge( tmpGrid,MM_KEEP );
	
	// adapt input grids and add noise
	_y.refineGrid( commonGrid );
	currentNoise.refineGrid( commonGrid );
	_y += currentNoise.getValuesSubGrid( 0,getNY()-1 );

	return SUCCESSFUL_RETURN;
}



CLOSE_NAMESPACE_ACADO

/*
 *	end of file
 */
