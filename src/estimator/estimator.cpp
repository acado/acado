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
 *    \file src/estimator/estimator.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 13.06.2008
 */


#include <acado/estimator/estimator.hpp>



BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

Estimator::Estimator( ) : SimulationBlock( BN_ESTIMATOR,DEFAULT_SAMPLING_TIME )
{
}


Estimator::Estimator(	double _samplingTime
						) : SimulationBlock( BN_ESTIMATOR,_samplingTime )
{
	setStatus( BS_NOT_INITIALIZED );
}


Estimator::Estimator( const Estimator& rhs ) : SimulationBlock( rhs )
{
	x  = rhs.x;
	xa = rhs.xa;
	u  = rhs.u;
	p  = rhs.p;
	w  = rhs.w;
}


Estimator::~Estimator( )
{
}


Estimator& Estimator::operator=( const Estimator& rhs )
{
	if ( this != &rhs )
	{
		SimulationBlock::operator=( rhs );

		x  = rhs.x;
		xa = rhs.xa;
		u  = rhs.u;
		p  = rhs.p;
		w  = rhs.w;
	}

    return *this;
}


returnValue Estimator::init(	double startTime,
								const DVector &x0_,
								const DVector &p_
								)
{
	setStatus( BS_READY );

	return SUCCESSFUL_RETURN;
}




//
// PROTECTED MEMBER FUNCTIONS:
//




CLOSE_NAMESPACE_ACADO

// end of file.
