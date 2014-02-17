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
 *    \file src/control_law/feedforward_law.cpp
 *    \author Joris Gillis, Hans Joachim Ferreau, Boris Houska
 *    \date 2009
 */


/**

The objective of this class is to be able to use precomputed controls to run a simulation.
If the precomputed controls are given as a Curve mycontrols,
use:
    FeedforwardLaw( numberofstates, mycontrols[,sampleTime])

**/

#include <acado/control_law/feedforward_law.hpp>


BEGIN_NAMESPACE_ACADO



//
// PUBLIC MEMBER FUNCTIONS:
//

FeedforwardLaw::FeedforwardLaw( ) : ControlLaw( )
{
	setStatus( BS_NOT_INITIALIZED );
}


FeedforwardLaw::FeedforwardLaw(	const uint _nx,
								const Curve& _u,
								double _samplingTime
								) : ControlLaw( _samplingTime )
{
	nx=_nx;
	uRef=_u;
	setStatus( BS_NOT_INITIALIZED );
}


FeedforwardLaw::FeedforwardLaw( const FeedforwardLaw& rhs ) : ControlLaw( rhs )
{
	nx=rhs.nx;
	uRef=rhs.uRef;
}


FeedforwardLaw::~FeedforwardLaw( )
{
}


FeedforwardLaw& FeedforwardLaw::operator=( const FeedforwardLaw& rhs )
{
	if ( this != &rhs )
	{
		ControlLaw::operator=( rhs );

	}

    return *this;
}


ControlLaw* FeedforwardLaw::clone( ) const
{
	return new FeedforwardLaw( *this );
}


returnValue FeedforwardLaw::init(	double startTime,
									const DVector &x0_,
									const DVector &p_,
									const VariablesGrid& _yRef
									)
{
	u.init(getNU());
	u.setZero();
	
	p.init( 0 );
	p.setZero( );

	setStatus( BS_READY );

	return SUCCESSFUL_RETURN;
}


returnValue FeedforwardLaw::step(	double currentTime,
									const DVector& _x,
									const DVector& _p,
									const VariablesGrid& _yRef
									)
{
	if ( getStatus( ) != BS_READY )
		return ACADOERROR( RET_BLOCK_NOT_READY );
	
	if ( _x.getDim( ) != getNX( ) )
		return ACADOERROR( RET_VECTOR_DIMENSION_MISMATCH );


	// use reference control as control
	DVector uEv;
	uRef.evaluate(currentTime,uEv);
	//printf("FeedforwardLaw controllor outputs %d #.\n",uRef.getDim());
	//printf(" -- yref: %d samples of size (%d,%d) -- \n",_yRef.getNumPoints(),_yRef.getNumRows(),_yRef.getNumCols());

	u = uEv;
	p = _p;

	//printf(" -- u: [%.3e,%.3e,%.3e] -- \n",u(0),u(1),u(2));

	return SUCCESSFUL_RETURN;	
}



uint FeedforwardLaw::getNX( ) const
{
	return nx;
}


uint FeedforwardLaw::getNXA( ) const
{
	return 0;
}


uint FeedforwardLaw::getNU( ) const
{
	return uRef.getDim( );
}


uint FeedforwardLaw::getNP( ) const
{
	return 0;
}


uint FeedforwardLaw::getNW( ) const
{
	return 0;
}


uint FeedforwardLaw::getNY( ) const
{
	return getNX( );
}


BooleanType FeedforwardLaw::isDynamic( ) const
{
	return BT_FALSE;
}


BooleanType FeedforwardLaw::isStatic( ) const
{
	if ( isDynamic() == BT_TRUE )
		return BT_FALSE;
	else
		return BT_TRUE;
}



//
// PROTECTED MEMBER FUNCTIONS:
//




CLOSE_NAMESPACE_ACADO

// end of file.
