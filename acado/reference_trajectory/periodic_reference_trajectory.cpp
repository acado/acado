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
 *    \file src/reference_trajectory/periodic_reference_trajectory.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 13.06.2008
 */


#include <acado/reference_trajectory/periodic_reference_trajectory.hpp>



BEGIN_NAMESPACE_ACADO



//
// PUBLIC MEMBER FUNCTIONS:
//

PeriodicReferenceTrajectory::PeriodicReferenceTrajectory( ) : StaticReferenceTrajectory( )
{
}


// PeriodicReferenceTrajectory::PeriodicReferenceTrajectory(	const Curve& _yRef
// 															) : StaticReferenceTrajectory( _yRef )
// {
// }


PeriodicReferenceTrajectory::PeriodicReferenceTrajectory(	const VariablesGrid& _yRef
															) : StaticReferenceTrajectory( _yRef )
{
}


PeriodicReferenceTrajectory::PeriodicReferenceTrajectory(	const char* const _yRefFileName
															) : StaticReferenceTrajectory( _yRefFileName )
{
}


PeriodicReferenceTrajectory::PeriodicReferenceTrajectory( const PeriodicReferenceTrajectory& rhs )
                            :StaticReferenceTrajectory( rhs )
{
	//yRef = rhs.yRef;
}


PeriodicReferenceTrajectory::~PeriodicReferenceTrajectory( )
{
}


PeriodicReferenceTrajectory& PeriodicReferenceTrajectory::operator=( const PeriodicReferenceTrajectory& rhs )
{
	if ( this != &rhs )
	{
		StaticReferenceTrajectory::operator=( rhs );
		//yRef = rhs.yRef;
	}

	return *this;
}


ReferenceTrajectory* PeriodicReferenceTrajectory::clone( ) const
{
	return new PeriodicReferenceTrajectory( *this );
}


returnValue PeriodicReferenceTrajectory::getReference(	double tStart,
														double tEnd,
														VariablesGrid& _yRef
														) const
{
	if ( acadoIsStrictlyGreater( tStart,tEnd ) == BT_TRUE )
		return ACADOERROR( RET_INVALID_ARGUMENTS );

	double T = yRef.getLastTime() - yRef.getFirstTime();         // cycle duration
	int nStart = (int)floor( (double) (tStart/T+100.0*EPS) );  // cycle number at start
	int nEnd   = (int)floor( (double) (tEnd  /T-100.0*EPS) );  // cycle number at end

	if ( nStart == nEnd )
	{
		_yRef = (yRef.getTimeSubGrid( tStart-T*(double)nStart,tEnd-T*(double)nStart )).shiftTimes( T*(double)nStart );
	}
	else
	{
		_yRef = (yRef.getTimeSubGrid( tStart-T*(double)nStart,yRef.getLastTime() )).shiftTimes( T*(double)nStart );
		
		for( int i=nStart+1; i<nEnd; ++i )
			_yRef.appendTimes( VariablesGrid(yRef).shiftTimes( T*(double)i ),MM_KEEP );
		
		_yRef.appendTimes( (yRef.getTimeSubGrid( yRef.getFirstTime(),tEnd-T*(double)nEnd )).shiftTimes( T*(double)nEnd ) );
	}
	
	return SUCCESSFUL_RETURN;
}



//
// PROTECTED MEMBER FUNCTIONS:
//




CLOSE_NAMESPACE_ACADO

// end of file.
