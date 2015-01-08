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
*    \file src/clock/clock.cpp
*    \author Boris Houska, Hans Joachim Ferreau
*    \date 13.06.2008
*/



#include <stdio.h>

#include <acado/clock/clock.hpp>


BEGIN_NAMESPACE_ACADO




Clock::Clock( )
{
	reset( );
}


Clock::Clock( const Clock &rhs )
{
	elapsedTime = rhs.elapsedTime;
	status = rhs.status;
}


Clock::~Clock( )
{
}


Clock& Clock::operator=( const Clock &rhs )
{
	if( this != &rhs )
	{
		elapsedTime = rhs.elapsedTime;
		status = rhs.status;
	}

	return *this;
}



returnValue Clock::init(	double _initialTime
							)
{
	elapsedTime = _initialTime;
	status = CS_STOPPED;

	return SUCCESSFUL_RETURN;
}


returnValue Clock::reset( )
{
	return init( 0.0 );
}


returnValue Clock::getTime(	double& _elapsedTime
							)
{
	switch( status )
	{
		case CS_NOT_INITIALIZED:
			return ACADOERROR( RET_BLOCK_NOT_READY );

		case CS_RUNNING:
			stop( );
			_elapsedTime = elapsedTime;
			start( );
			return SUCCESSFUL_RETURN;

		case CS_STOPPED:
			_elapsedTime = elapsedTime;
			return SUCCESSFUL_RETURN;

		default:
			return ACADOERROR( RET_UNKNOWN_BUG );
	}
}


double Clock::getTime( )
{
	double _elapsedTime;

	if ( getTime( _elapsedTime ) == SUCCESSFUL_RETURN )
		return _elapsedTime;
	else
		return -INFTY;
}


CLOSE_NAMESPACE_ACADO


/*
 *	end of file
 */
