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
 *    \file src/transfer_device/transfer_device.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 13.06.2008
 */


#include <acado/transfer_device/transfer_device.hpp>



BEGIN_NAMESPACE_ACADO



//
// PUBLIC MEMBER FUNCTIONS:
//

TransferDevice::TransferDevice( ) : SimulationBlock( )
{
	additiveNoise = 0;

	setStatus( BS_NOT_INITIALIZED );
}


TransferDevice::TransferDevice(	uint _dim,
								BlockName _name,
								double _samplingTime
								) : SimulationBlock( _name,_samplingTime )
{
	lastSignal.init( _dim,1 );

	additiveNoise = new Noise*[_dim];
	for( uint i=0; i<_dim; ++i )
		additiveNoise[i] = 0;

	noiseSamplingTimes.init( _dim );
	noiseSamplingTimes.setAll( 0.0 );

	deadTimes.init( _dim );
	deadTimes.setAll( 0.0 );

	setStatus( BS_NOT_INITIALIZED );
}


TransferDevice::TransferDevice( const TransferDevice& rhs ) : SimulationBlock( rhs )
{
	lastSignal = rhs.lastSignal;

	if ( rhs.additiveNoise != 0 )
	{
		additiveNoise = new Noise*[rhs.getDim( )];

		for( uint i=0; i<getDim( ); ++i )
			if ( rhs.additiveNoise[i] != 0 )
				additiveNoise[i] = (rhs.additiveNoise[i])->clone( );
			else
				additiveNoise[i] = 0;
	}
	else
		additiveNoise = 0;

	noiseSamplingTimes = rhs.noiseSamplingTimes;
	
	deadTimes = rhs.deadTimes;
}


TransferDevice::~TransferDevice( )
{
	if ( additiveNoise != 0 )
	{
		for( uint i=0; i<getDim( ); ++i )
			if ( additiveNoise[i] != 0 )
				delete additiveNoise[i];

		delete[] additiveNoise;
	}
}


TransferDevice& TransferDevice::operator=( const TransferDevice& rhs )
{
	if ( this != &rhs )
	{
		SimulationBlock::operator=( rhs );

		if ( additiveNoise != 0 )
		{
			for( uint i=0; i<getDim( ); ++i )
				if ( additiveNoise[i] != 0 )
					delete additiveNoise[i];
	
			delete[] additiveNoise;
		}


		lastSignal = rhs.lastSignal;

		if ( rhs.additiveNoise != 0 )
		{
			additiveNoise = new Noise*[rhs.getDim( )];
	
			for( uint i=0; i<getDim( ); ++i )
				if ( rhs.additiveNoise[i] != 0 )
					additiveNoise[i] = (rhs.additiveNoise[i])->clone( );
				else
					additiveNoise[i] = 0;
		}
		else
			additiveNoise = 0;

		noiseSamplingTimes = rhs.noiseSamplingTimes;

		deadTimes = rhs.deadTimes;
	}

	return *this;
}




//
// PROTECTED MEMBER FUNCTIONS:
//


returnValue TransferDevice::init(	double _startTime,
									const DVector& _startValue
									)
{
	// initialise lastSignal
	lastSignal.init( getDim( ),1 );

	lastSignal.setTime( 0,_startTime );

	if ( _startValue.getDim( ) == getDim( ) )
	{
		lastSignal.setVector( 0,_startValue );
	}
	else
	{
		DVector tmp( getDim( ) );
		tmp.setAll( 0.0 );
		lastSignal.setVector( 0,tmp );
	}

	// initialise additive noise
	if ( additiveNoise != 0 )
	{
		for( uint i=0; i<getDim( ); ++i )
		{
			if ( additiveNoise[i] != 0 )
				additiveNoise[i]->init( );
		}
	}

	setStatus( BS_READY );

	return SUCCESSFUL_RETURN;
}


returnValue TransferDevice::generateNoise(	double startTime,
											double endTime,
											VariablesGrid& currentNoise
											) const
{
	double horizonLength = endTime - startTime;
	if ( horizonLength <= 0.0 )
		return ACADOERROR( RET_INVALID_ARGUMENTS );

	// generate noise grid
	Grid noiseGrid;

	if ( acadoIsPositive( noiseSamplingTimes(0) ) == BT_TRUE )
	{
		for( uint i=0; i<=floor( horizonLength/noiseSamplingTimes(0) ); ++i )
			noiseGrid.addTime( ((double)i) * noiseSamplingTimes(0) );

		if ( acadoIsInteger( horizonLength/noiseSamplingTimes(0) ) == BT_FALSE )
			noiseGrid.addTime( endTime );

		noiseGrid.shiftTimes( startTime );
	}
	else
	{
		noiseGrid.init( startTime,endTime );
	}

	// generate current noise
	DVector noiseVector( 1 );

	currentNoise.init( getDim( ),noiseGrid );
	currentNoise.setZero( );

	if ( additiveNoise != 0 )
	{
		// generate noise
		for( uint i=0; i<getDim( ); ++i )
		{
			if ( additiveNoise[i] != 0 )
			{
				for( uint j=0; j<currentNoise.getNumPoints( )-1; ++j )
				{
					additiveNoise[i]->step( noiseVector );
					currentNoise( j,i ) = noiseVector( 0 );
				}
				currentNoise( currentNoise.getNumPoints( )-1,i ) = noiseVector( 0 );
			}
		}
	}

	return SUCCESSFUL_RETURN;
}



CLOSE_NAMESPACE_ACADO

/*
 *	end of file
 */
