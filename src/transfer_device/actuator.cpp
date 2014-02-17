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
 *    \file src/transfer_device/actuator.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 25.08.2008
 */


#include <acado/transfer_device/actuator.hpp>



BEGIN_NAMESPACE_ACADO



//
// PUBLIC MEMBER FUNCTIONS:
//

Actuator::Actuator( ) : TransferDevice( BN_ACTUATOR )
{
	nU = 0;
	nP = 0;
}


Actuator::Actuator(	uint _nU,
					uint _nP,
					double _samplingTime
					) : TransferDevice( _nU+_nP,BN_ACTUATOR,_samplingTime )
{
	nU = _nU;
	nP = _nP;
}


Actuator::Actuator( const Actuator& rhs ) : TransferDevice( rhs )
{
	nU = rhs.nU;
	nP = rhs.nP;
}


Actuator::~Actuator( )
{
}


Actuator& Actuator::operator=( const Actuator& rhs )
{
    if ( this != &rhs )
    {
		TransferDevice::operator=( rhs );

		nU = rhs.nU;
		nP = rhs.nP;
    }

    return *this;
}


returnValue Actuator::setControlNoise(	const Noise& _noise,
										double _noiseSamplingTime
										)
{
	if ( _noise.getDim( ) != getNU( ) )
		return ACADOERROR( RET_INVALID_ARGUMENTS );

	for( uint i=0; i<getNU( ); ++i )
	{
		if ( additiveNoise[i] != 0 )
			delete additiveNoise[i];

		additiveNoise[i] = _noise.clone( i );
	}
	
	noiseSamplingTimes.setAll( _noiseSamplingTime );

	return SUCCESSFUL_RETURN;
}


returnValue Actuator::setControlNoise(	uint idx,
										const Noise& _noise,
										double _noiseSamplingTime
										)
{
	if ( ( idx >= getNU( ) ) || ( _noise.getDim( ) != 1 ) )
		return ACADOERROR( RET_INVALID_ARGUMENTS );

	if ( additiveNoise[idx] != 0 )
		delete additiveNoise[idx];

	additiveNoise[idx] = _noise.clone( );

	if ( ( idx > 0 ) && ( acadoIsEqual( _noiseSamplingTime, noiseSamplingTimes(0) ) == BT_FALSE ) )
		ACADOWARNING( RET_NO_DIFFERENT_NOISE_SAMPLING_FOR_DISCRETE );

	noiseSamplingTimes.setAll( _noiseSamplingTime ); // should be changed later

	return SUCCESSFUL_RETURN;
}


returnValue Actuator::setParameterNoise(	const Noise& _noise,
											double _noiseSamplingTime
											)
{
	if ( _noise.getDim( ) != getNP( ) )
		return ACADOERROR( RET_INVALID_ARGUMENTS );

	for( uint i=0; i<getNP( ); ++i )
	{
		if ( additiveNoise[ getNU()+i ] != 0 )
			delete additiveNoise[ getNU()+i ];

		additiveNoise[ getNU()+i ] = _noise.clone( i );
	}
	
	noiseSamplingTimes.setAll( _noiseSamplingTime );

	return SUCCESSFUL_RETURN;
}


returnValue Actuator::setParameterNoise(	uint idx,
											const Noise& _noise,
											double _noiseSamplingTime
											)
{
	if ( ( idx >= getNP( ) ) || ( _noise.getDim( ) != 1 ) )
		return ACADOERROR( RET_INVALID_ARGUMENTS );

	if ( additiveNoise[ getNU()+idx ] != 0 )
		delete additiveNoise[ getNU()+idx ];

	additiveNoise[ getNU()+idx ] = _noise.clone( );

	if ( ( idx > 0 ) && ( acadoIsEqual( _noiseSamplingTime, noiseSamplingTimes(0) ) == BT_FALSE ) )
		ACADOWARNING( RET_NO_DIFFERENT_NOISE_SAMPLING_FOR_DISCRETE );

	noiseSamplingTimes.setAll( _noiseSamplingTime ); // should be changed later

	return SUCCESSFUL_RETURN;
}



returnValue Actuator::setControlDeadTimes(	const DVector& _deadTimes
											)
{
	if ( _deadTimes.getDim( ) != getNU( ) )
		return ACADOERROR( RET_INVALID_ARGUMENTS );

	if ( _deadTimes.getMin( ) < 0.0 )
		return ACADOERROR( RET_INVALID_ARGUMENTS );

	if ( deadTimes.getDim( ) == 0 )
		return ACADOERROR( RET_MEMBER_NOT_INITIALISED );

	for( uint i=0; i<getNU(); ++i )
		deadTimes( i ) = _deadTimes( i );

	return SUCCESSFUL_RETURN;
}


returnValue Actuator::setControlDeadTimes(	double _deadTime
											)
{
	if ( _deadTime < 0.0 )
		return ACADOERROR( RET_INVALID_ARGUMENTS );

	if ( deadTimes.getDim( ) == 0 )
		return ACADOERROR( RET_MEMBER_NOT_INITIALISED );

	for( uint i=0; i<getNU(); ++i )
		deadTimes( i ) = _deadTime;

	return SUCCESSFUL_RETURN;
}


returnValue Actuator::setControlDeadTime(	uint idx,
											double _deadTime
											)
{
	if ( idx >= getNU( ) )
		return ACADOERROR( RET_INVALID_ARGUMENTS );

	if ( _deadTime < 0.0 )
		return ACADOERROR( RET_INVALID_ARGUMENTS );

	if ( deadTimes.getDim( ) == 0 )
		return ACADOERROR( RET_MEMBER_NOT_INITIALISED );

	deadTimes( idx ) = _deadTime;

	return SUCCESSFUL_RETURN;
}



returnValue Actuator::setParameterDeadTimes(	const DVector& _deadTimes
												)
{
	if ( _deadTimes.getDim( ) != getNP( ) )
		return ACADOERROR( RET_INVALID_ARGUMENTS );

	if ( _deadTimes.getMin( ) < 0.0 )
		return ACADOERROR( RET_INVALID_ARGUMENTS );

	if ( deadTimes.getDim( ) == 0 )
		return ACADOERROR( RET_MEMBER_NOT_INITIALISED );

	for( uint i=0; i<getNP(); ++i )
		deadTimes( getNU()+i ) = _deadTimes( i );

	return SUCCESSFUL_RETURN;
}


returnValue Actuator::setParameterDeadTimes(	double _deadTime
												)
{
	if ( _deadTime < 0.0 )
		return ACADOERROR( RET_INVALID_ARGUMENTS );

	if ( deadTimes.getDim( ) == 0 )
		return ACADOERROR( RET_MEMBER_NOT_INITIALISED );

	for( uint i=0; i<getNP(); ++i )
		deadTimes( getNU()+i ) = _deadTime;

	return SUCCESSFUL_RETURN;
}


returnValue Actuator::setParameterDeadTime(	uint idx,
											double _deadTime
											)
{
	if ( idx >= getNP( ) )
		return ACADOERROR( RET_INVALID_ARGUMENTS );

	if ( _deadTime < 0.0 )
		return ACADOERROR( RET_INVALID_ARGUMENTS );

	if ( deadTimes.getDim( ) == 0 )
		return ACADOERROR( RET_MEMBER_NOT_INITIALISED );

	deadTimes( getNU()+idx ) = _deadTime;

	return SUCCESSFUL_RETURN;
}



returnValue Actuator::init(	double _startTime,
							const DVector& _startValueU,
							const DVector& _startValueP
							)
{
	DVector tmp;

	if ( _startValueU.isEmpty( ) == BT_FALSE )
		tmp.append( _startValueU );

	if ( _startValueP.isEmpty( ) == BT_FALSE )
		tmp.append( _startValueP );

	if ( TransferDevice::init( _startTime,tmp ) != SUCCESSFUL_RETURN )
		return ACADOERROR( RET_ACTUATOR_INIT_FAILED );

	return SUCCESSFUL_RETURN;
}



returnValue Actuator::step(	VariablesGrid& _u,
							VariablesGrid& _p
							)
{
	// consistency checks
	if ( getStatus( ) != BS_READY )
		return ACADOERROR( RET_BLOCK_NOT_READY );

	if ( checkInputConsistency( _u,_p ) != SUCCESSFUL_RETURN )
		return ACADOERROR( RET_ACTUATOR_STEP_FAILED );

	if ( acadoIsEqual( _u.getFirstTime( ),lastSignal.getLastTime( ) ) == BT_FALSE )
		return ACADOERROR( RET_INVALID_ARGUMENTS );

	// delay inputs and store last signal
	if ( delayActuatorInput( _u,_p ) != SUCCESSFUL_RETURN )
		return ACADOERROR( RET_ACTUATOR_STEP_FAILED );

	// add actuator noise
	if ( addActuatorNoise( _u,_p ) != SUCCESSFUL_RETURN )
		return ACADOERROR( RET_ACTUATOR_STEP_FAILED );

	return SUCCESSFUL_RETURN;
}



//
// PROTECTED MEMBER FUNCTIONS:
//


/* identitical to same function within the class Process! */
returnValue Actuator::checkInputConsistency(	const VariablesGrid& _u,
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

		if ( _p.getNumRows( ) != getNP( ) )
			return ACADOERROR( RET_PARAMETER_DIMENSION_MISMATCH );
	
		if ( acadoIsEqual( _u.getFirstTime( ),_p.getFirstTime( ) ) == BT_FALSE )
			return ACADOERROR( RET_INVALID_ARGUMENTS );
	
		if ( acadoIsEqual( _u.getLastTime( ),_p.getLastTime( ) ) == BT_FALSE )
			return ACADOERROR( RET_INVALID_ARGUMENTS );
	}

	return SUCCESSFUL_RETURN;
}



returnValue Actuator::delayActuatorInput(	VariablesGrid& _u,
											VariablesGrid& _p
											)
{
	if ( hasDeadTime( ) == BT_FALSE )
	{
		// store last signal
		DVector tmp = _u.getLastVector( );
		if ( _p.isEmpty( ) == BT_FALSE )
			tmp.append( _p.getLastVector( ) );

		lastSignal.init( tmp );
		lastSignal.setTime( 0,_u.getLastTime( ) );
	
		return SUCCESSFUL_RETURN;
	}
	else
	{
		double startTime = _u.getFirstTime( );
		double endTime   = _u.getLastTime( );

		// determine variables grid of delayed input
		VariablesGrid uDelayed, pDelayed;
		if ( getDelayedInputGrids( _u,_p, uDelayed,pDelayed ) != SUCCESSFUL_RETURN )
			return ACADOERROR( RET_DELAYING_INPUTS_FAILED );

		// store last signal
		lastSignal = uDelayed.getTimeSubGrid( uDelayed.getFloorIndex( endTime ),uDelayed.getLastIndex( ) );
		if ( _p.isEmpty( ) == BT_FALSE )
			lastSignal.appendValues( pDelayed.getTimeSubGrid( pDelayed.getFloorIndex( endTime ),pDelayed.getLastIndex( ) ) );

/*		printf("u:\n");
		_u.print();
		printf("p:\n");
		_p.print();
		
		printf("uDelayed:\n");
		uDelayed.print();
		printf("pDelayed:\n");
		pDelayed.print();*/
// 
// 		printf("last:\n");
// 		lastSignal.print();

		// crop delayed signal to current horizon
		_u = uDelayed.getTimeSubGrid( uDelayed.getFloorIndex( startTime ),uDelayed.getFloorIndex( endTime ) );
		if ( _p.isEmpty( ) == BT_FALSE )
			_p = pDelayed.getTimeSubGrid( pDelayed.getFloorIndex( startTime ),pDelayed.getFloorIndex( endTime ) );

// 		printf("u:\n");
// 		_u.print();
// 		printf("p:\n");
// 		_p.print();

		return SUCCESSFUL_RETURN;
	}
}


returnValue Actuator::getDelayedInputGrids(	const VariablesGrid& _u,
											const VariablesGrid& _p,
											VariablesGrid& _uDelayed,
											VariablesGrid& _pDelayed
											) const
{
	// determine common time grid for delayed inputs:
	Grid delayedInputTimeGrid = lastSignal.getTimePoints( );

	// make sure that last time instant of horizon lies within the grid
	if ( acadoIsEqual( lastSignal.getLastTime(),_u.getLastTime( ) ) == BT_FALSE )
		delayedInputTimeGrid.addTime( _u.getLastTime( ) );

//	delayedInputTimeGrid.print();
	
	// add grids of all delayed input components
	for( uint i=0; i<getNU( ); ++i )
		delayedInputTimeGrid = delayedInputTimeGrid & ( _u.getTimePoints( ).shiftTimes( deadTimes(i) ) );

// 	_u.getTimePoints( ).print();
// 	_u.getTimePoints( ).shiftTimes( deadTimes(0) ).print();
// 	delayedInputTimeGrid.print();
	
	if ( _p.isEmpty( ) == BT_FALSE )
	{
		for( uint i=0; i<getNP( ); ++i )
			delayedInputTimeGrid = delayedInputTimeGrid & ( _p.getTimePoints( ).shiftTimes( deadTimes(getNU()+i) ) );
	}

	VariablesGrid tmp;

	// setup common variables grid for delayed inputs
	_uDelayed.init( );
	_pDelayed.init( );

	for( uint i=0; i<getNU( ); ++i )
	{
// 		tmp.print("tmp");
		tmp = lastSignal( i );
// 		tmp.print("tmp");
		tmp.merge( _u( i ).shiftTimes( deadTimes(i) ),MM_REPLACE,BT_FALSE );
// 		tmp.print("tmp");
		tmp.refineGrid( delayedInputTimeGrid );
// 		tmp.print("tmp");
		
		_uDelayed.appendValues( tmp );
	}

	if ( _p.isEmpty( ) == BT_FALSE )
	{
		for( uint i=0; i<getNP( ); ++i )
		{
			tmp = lastSignal( getNU()+i );
			tmp.merge( _p( i ).shiftTimes( deadTimes(getNU()+i) ),MM_REPLACE,BT_FALSE );
			tmp.refineGrid( delayedInputTimeGrid );

			_pDelayed.appendValues( tmp );
		}
	}

	return SUCCESSFUL_RETURN;
}


returnValue Actuator::addActuatorNoise(	VariablesGrid& _u,
										VariablesGrid& _p
										) const
{
	if ( hasNoise( ) == BT_FALSE )
		return SUCCESSFUL_RETURN;

	// generate current noise
	VariablesGrid currentNoise;

	if ( generateNoise( _u.getFirstTime(),_u.getLastTime(),currentNoise ) != SUCCESSFUL_RETURN )
		return ACADOERROR( RET_GENERATING_NOISE_FAILED );

	// determine common grid
	Grid commonGrid, tmpGrid;
	currentNoise.getGrid( tmpGrid );
	_u.getGrid( commonGrid );
	commonGrid & tmpGrid;


	// adapt input grids and add noise
	_u.refineGrid( commonGrid );
	currentNoise.refineGrid( commonGrid );
	_u += currentNoise.getValuesSubGrid( 0,getNU()-1 );

	if ( _p.isEmpty( ) == BT_FALSE )
	{
		_p.refineGrid( commonGrid );
		_p += currentNoise.getValuesSubGrid( getNU(),getNU()+getNP()-1 );
	}

	return SUCCESSFUL_RETURN;
}




CLOSE_NAMESPACE_ACADO

/*
 *	end of file
 */
