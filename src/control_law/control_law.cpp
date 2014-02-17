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
 *    \file src/control_law/control_law.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 13.06.2008
 */


#include <acado/control_law/control_law.hpp>


BEGIN_NAMESPACE_ACADO



//
// PUBLIC MEMBER FUNCTIONS:
//

ControlLaw::ControlLaw( ) : SimulationBlock( BN_CONTROL_LAW )
{
}


ControlLaw::ControlLaw(	double _samplingTime
						) : SimulationBlock( BN_CONTROL_LAW,_samplingTime )
{
}


ControlLaw::ControlLaw( const ControlLaw& rhs ) : SimulationBlock( rhs )
{
	u = rhs.u;
	p = rhs.u;
}


ControlLaw::~ControlLaw( )
{
}


ControlLaw& ControlLaw::operator=( const ControlLaw& rhs )
{
	if ( this != &rhs )
	{
		SimulationBlock::operator=( rhs );

		u = rhs.u;
		p = rhs.u;
	}

    return *this;
}



returnValue ControlLaw::initializeAlgebraicStates( const VariablesGrid& _xa_init )
{
	return ACADOERROR( RET_NOT_IMPLEMENTED_IN_BASE_CLASS );
}


returnValue ControlLaw::initializeAlgebraicStates( const char* fileName )
{
	return ACADOERROR( RET_NOT_IMPLEMENTED_IN_BASE_CLASS );
}



returnValue ControlLaw::initializeControls( const VariablesGrid& _u_init )
{
	return ACADOERROR( RET_NOT_IMPLEMENTED_IN_BASE_CLASS );
}


returnValue ControlLaw::initializeControls( const char* fileName )
{
	return ACADOERROR( RET_NOT_IMPLEMENTED_IN_BASE_CLASS );
}




returnValue ControlLaw::step(	const DVector& _x,
								const DVector& _p,
								const VariablesGrid& _yRef
								)
{
    return step( 0.0,_x,_p,_yRef );
}


returnValue ControlLaw::feedbackStep(	double currentTime,
										const DVector& _x,
										const DVector& _p,
										const VariablesGrid& _yRef
										)
{
	// perform standard step
	return step( currentTime,_x,_p,_yRef );
}


returnValue ControlLaw::preparationStep(	double nextTime,
											const VariablesGrid& _yRef
											)
{
	// do nothing by default
	return SUCCESSFUL_RETURN;
}



returnValue ControlLaw::shift(	double timeShift
								)
{
	return ACADOERROR( RET_NO_REALTIME_MODE_AVAILABLE );
}



uint ControlLaw::getNX( ) const
{
	return 0;
}


uint ControlLaw::getNXA( ) const
{
	return 0;
}


uint ControlLaw::getNU( ) const
{
	return 0;
}


uint ControlLaw::getNP( ) const
{
	return 0;
}


uint ControlLaw::getNW( ) const
{
	return 0;
}


uint ControlLaw::getNY( ) const
{
	return 0;
}


double ControlLaw::getLengthPredictionHorizon( ) const
{
	return samplingTime;
}


double ControlLaw::getLengthControlHorizon( ) const
{
	return samplingTime;
}


//
// PROTECTED MEMBER FUNCTIONS:
//

BooleanType ControlLaw::isInRealTimeMode( ) const
{
	return BT_FALSE;
}



CLOSE_NAMESPACE_ACADO

// end of file.
