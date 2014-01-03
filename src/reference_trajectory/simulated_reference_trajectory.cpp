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
 *    \file src/reference_trajectory/simulated_reference_trajectory.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 13.06.2008
 */


#include <acado/reference_trajectory/simulated_reference_trajectory.hpp>



BEGIN_NAMESPACE_ACADO



//
// PUBLIC MEMBER FUNCTIONS:
//

SimulatedReferenceTrajectory::SimulatedReferenceTrajectory( ) : AdaptiveReferenceTrajectory( )
{
	process = 0;
}


SimulatedReferenceTrajectory::SimulatedReferenceTrajectory(	const DynamicSystem& _dynamicSystem,
															IntegratorType _integratorType
															)
{
	process = new Process( _dynamicSystem,_integratorType );
}


SimulatedReferenceTrajectory::SimulatedReferenceTrajectory( const SimulatedReferenceTrajectory& rhs ) : AdaptiveReferenceTrajectory( rhs )
{
	if ( rhs.process != 0 )
		process = new Process( *(rhs.process) );
	else
		process = 0;
}


SimulatedReferenceTrajectory::~SimulatedReferenceTrajectory( )
{
	if ( process != 0 )
		delete process;
}


SimulatedReferenceTrajectory& SimulatedReferenceTrajectory::operator=( const SimulatedReferenceTrajectory& rhs )
{
	if ( this != &rhs )
	{
		if ( process != 0 )
			delete process;

		AdaptiveReferenceTrajectory::operator=( rhs );
		
	}

    return *this;
}


ReferenceTrajectory* SimulatedReferenceTrajectory::clone( ) const
{
	return new SimulatedReferenceTrajectory( *this );
}


returnValue SimulatedReferenceTrajectory::init(	double startTime,
												const DVector& _x,
												const DVector& _xa,
												const DVector& _u,
												const DVector& _p,
												const DVector& _w
												)
{
    return ACADOERROR( RET_NOT_YET_IMPLEMENTED );
}


returnValue SimulatedReferenceTrajectory::step(	double _currentTime,
												const DVector& _y,
												const DVector& _x,
												const DVector& _xa,
												const DVector& _u,
												const DVector& _p,
												const DVector& _w
												)
{
	return ACADOERROR( RET_NOT_YET_IMPLEMENTED );
}


returnValue SimulatedReferenceTrajectory::step(	const DVector& _x,
												const VariablesGrid& _u,
												const VariablesGrid& _p,
												const VariablesGrid& _w
												)
{
	

	return SUCCESSFUL_RETURN;
}


returnValue SimulatedReferenceTrajectory::getReference(	double tStart,
														double tEnd,
														VariablesGrid& _yRef
														) const
{
    return ACADOERROR( RET_NOT_YET_IMPLEMENTED );
}



uint SimulatedReferenceTrajectory::getDim( ) const
{
	if ( process != 0 )
		return process->getNY( );
	else
		return 0;
}


//
// PROTECTED MEMBER FUNCTIONS:
//




CLOSE_NAMESPACE_ACADO

// end of file.
