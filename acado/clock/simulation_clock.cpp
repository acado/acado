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
*    \file src/clock/simulation_clock.cpp
*    \author Boris Houska, Hans Joachim Ferreau
*    \date 13.06.2008
*/



#include <acado/clock/simulation_clock.hpp>



BEGIN_NAMESPACE_ACADO



SimulationClock::SimulationClock( ) : Clock( )
{
}


SimulationClock::SimulationClock( const SimulationClock &rhs ) : Clock( rhs )
{
}


SimulationClock::~SimulationClock( )
{
}


SimulationClock& SimulationClock::operator=( const SimulationClock &rhs )
{
	if( this != &rhs )
	{
		Clock::operator=( rhs );
	}

    return *this;
}


Clock* SimulationClock::clone( ) const
{
	return new SimulationClock( *this );
}


returnValue SimulationClock::start( )
{
	if ( status != CS_STOPPED )
		return ACADOERROR( RET_CLOCK_NOT_READY );

	status = CS_RUNNING;

	return SUCCESSFUL_RETURN;
}


returnValue SimulationClock::step(	double _timeShift
									)
{
	returnValue returnvalue = stop( );
	if ( returnvalue != SUCCESSFUL_RETURN )
		return returnvalue;

	elapsedTime += _timeShift;

	return start( );
}


returnValue SimulationClock::stop( )
{
	if ( status == CS_NOT_INITIALIZED )
		return ACADOERROR( RET_CLOCK_NOT_READY );

	if ( status == CS_STOPPED )
		return SUCCESSFUL_RETURN;

	status = CS_STOPPED;

	return SUCCESSFUL_RETURN;
}


CLOSE_NAMESPACE_ACADO


/*
 *	end of file
 */

