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
 *    \file src/reference_trajectory/static_reference_trajectory.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 */


#include <acado/reference_trajectory/static_reference_trajectory.hpp>



BEGIN_NAMESPACE_ACADO



//
// PUBLIC MEMBER FUNCTIONS:
//

StaticReferenceTrajectory::StaticReferenceTrajectory( ) : ReferenceTrajectory( )
{
}


// StaticReferenceTrajectory::StaticReferenceTrajectory(	const Curve& _yRef
// 														) : ReferenceTrajectory( )
// {
// 	yRef = _yRef;
// 
// 	setStatus( BS_READY );
// }


StaticReferenceTrajectory::StaticReferenceTrajectory(	const VariablesGrid& _yRef
														)
{
	if ( _yRef.isEmpty( ) == BT_TRUE )
	{
		ACADOERROR( RET_INVALID_ARGUMENTS );
		ASSERT( 1==0 );
	}

// 	yRef.add( _yRef );
	yRef = _yRef;

	//setStatus( BS_READY );
}


StaticReferenceTrajectory::StaticReferenceTrajectory(	const char* const _yRefFileName
														)
{
    VariablesGrid _yRef;
    _yRef.read( _yRefFileName );

	if ( _yRef.isEmpty( ) == BT_TRUE )
	{
		ACADOERROR( RET_FILE_CAN_NOT_BE_OPENED );
		ASSERT( 1==0 );
	}

// 	yRef.add( _yRef );
	yRef = _yRef;

	//setStatus( BS_READY );
}



StaticReferenceTrajectory::StaticReferenceTrajectory( const StaticReferenceTrajectory& rhs ) : ReferenceTrajectory( rhs )
{
	yRef = rhs.yRef;
}


StaticReferenceTrajectory::~StaticReferenceTrajectory( )
{
}


StaticReferenceTrajectory& StaticReferenceTrajectory::operator=( const StaticReferenceTrajectory& rhs )
{
	if ( this != &rhs )
	{
		ReferenceTrajectory::operator=( rhs );

		yRef = rhs.yRef;
	}

    return *this;
}


ReferenceTrajectory* StaticReferenceTrajectory::clone( ) const
{
	return new StaticReferenceTrajectory( *this );
}


returnValue StaticReferenceTrajectory::init(	double startTime,
												const DVector& _x,
												const DVector& _xa,
												const DVector& _u,
												const DVector& _p,
												const DVector& _w
												)
{
	return SUCCESSFUL_RETURN;
}


returnValue StaticReferenceTrajectory::step(	double _currentTime,
												const DVector& _y,
												const DVector& _x,
												const DVector& _xa,
												const DVector& _u,
												const DVector& _p,
												const DVector& _w
												)
{
	return SUCCESSFUL_RETURN;
}


returnValue StaticReferenceTrajectory::step(	const DVector& _x,
												const VariablesGrid& _u,
												const VariablesGrid& _p,
												const VariablesGrid& _w
												)
{
	return SUCCESSFUL_RETURN;
}


returnValue StaticReferenceTrajectory::getReference(	double tStart,
														double tEnd,
														VariablesGrid& _yRef
														) const
{
	if ( acadoIsSmaller( tStart,tEnd ) == BT_FALSE )
		return ACADOERROR( RET_INVALID_ARGUMENTS );

	if ( acadoIsStrictlySmaller( tStart,yRef.getFirstTime() ) == BT_TRUE )
		return ACADOERROR( RET_INVALID_ARGUMENTS );
	
//     return yRef.evaluate( tStart,tEnd,_yRef );

	if ( acadoIsSmaller( tEnd,yRef.getLastTime() ) == BT_TRUE )
	{
		_yRef = yRef.getTimeSubGrid( tStart,tEnd );
	}
	else
	{
		// constant extrapolation beyond end of interval
		if ( acadoIsSmaller( yRef.getLastTime(),tStart ) == BT_TRUE )
		{
			Grid grid( tStart,tEnd );
			_yRef.init( yRef.getLastVector(),grid );
		}
		else
		{
			_yRef = yRef.getTimeSubGrid( tStart,yRef.getLastTime() );
			_yRef.setTime( _yRef.getLastIndex(),tEnd );
		}
	}

	return SUCCESSFUL_RETURN;
}



uint StaticReferenceTrajectory::getDim( ) const
{
// 	return yRef.getDim( );
 	return yRef.getNumValues( );
}


//
// PROTECTED MEMBER FUNCTIONS:
//




CLOSE_NAMESPACE_ACADO

// end of file.
