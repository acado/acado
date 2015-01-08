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
*    \file src/clock/real_clock.cpp
*    \author Boris Houska, Hans Joachim Ferreau
*    \date 13.06.2008
*/


#include <acado/clock/real_clock.hpp>



BEGIN_NAMESPACE_ACADO



RealClock::RealClock( ) : Clock( )
{
	lastTimeInstant = 0.0;
}


RealClock::RealClock( const RealClock &rhs ) : Clock( rhs )
{
	lastTimeInstant = rhs.lastTimeInstant;
}


RealClock::~RealClock( )
{
}


RealClock& RealClock::operator=( const RealClock &rhs )
{
	if( this != &rhs )
	{
		Clock::operator=( rhs );

		lastTimeInstant = rhs.lastTimeInstant;
	}

	return *this;
}


Clock* RealClock::clone( ) const
{
	return new RealClock( *this );
}


returnValue RealClock::start( )
{
	if ( status != CS_STOPPED )
		return ACADOERROR( RET_CLOCK_NOT_READY );

	lastTimeInstant = acadoGetTime( );

	if ( lastTimeInstant < 0.0 )
		return ACADOERROR( RET_NO_SYSTEM_TIME );

	status = CS_RUNNING;

	return SUCCESSFUL_RETURN;
}


returnValue RealClock::step(	double _timeShift
								)
{
	returnValue returnvalue = stop( );
	if ( returnvalue != SUCCESSFUL_RETURN )
		return returnvalue;

	elapsedTime += _timeShift;

	return start( );
}


returnValue RealClock::stop( )
{
	if ( status == CS_NOT_INITIALIZED )
		return ACADOERROR( RET_CLOCK_NOT_READY );

	if ( status == CS_STOPPED )
		return SUCCESSFUL_RETURN;

	// actually stop the clock
	double currentTimeInstant = acadoGetTime( );
	
	if ( currentTimeInstant < 0.0 )
		return ACADOERROR( RET_NO_SYSTEM_TIME );
	
	elapsedTime += currentTimeInstant - lastTimeInstant;

	status = CS_STOPPED;

	return SUCCESSFUL_RETURN;
}



CLOSE_NAMESPACE_ACADO


/*
 *	end of file
 */
