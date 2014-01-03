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
 *    \file src/estimator/kalman_filter.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 13.06.2008
 */


#include <acado/estimator/kalman_filter.hpp>



BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//


KalmanFilter::KalmanFilter(	double _samplingTime
							) : Estimator( _samplingTime )
{
	setStatus( BS_NOT_INITIALIZED );
}


KalmanFilter::KalmanFilter( const KalmanFilter& rhs ) : Estimator( rhs )
{
}


KalmanFilter::~KalmanFilter( )
{
}


KalmanFilter& KalmanFilter::operator=( const KalmanFilter& rhs )
{
	if ( this != &rhs )
	{

		Estimator::operator=( rhs );

	}

    return *this;
}


Estimator* KalmanFilter::clone( ) const
{
	return new KalmanFilter( *this );
}



returnValue KalmanFilter::init(	double startTime,
								const DVector &x0_,
								const DVector &p_
								)
{
	setStatus( BS_READY );

	return SUCCESSFUL_RETURN;
}


returnValue KalmanFilter::step(	double currentTime,
								const DVector& _y
								)
{
	return SUCCESSFUL_RETURN;
}



//
// PROTECTED MEMBER FUNCTIONS:
//




CLOSE_NAMESPACE_ACADO

// end of file.
