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
 *    \file src/estimator/dynamic_estimator.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 13.06.2008
 */


#include <acado/estimator/dynamic_estimator.hpp>



BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

DynamicEstimator::DynamicEstimator( ) : Estimator( )
{
}


DynamicEstimator::DynamicEstimator(	const RealTimeAlgorithm& _realTimeAlgorithm,
									double _samplingTime
									) : Estimator( _samplingTime )
{
	realTimeAlgorithm = new RealTimeAlgorithm( _realTimeAlgorithm );
	setStatus( BS_NOT_INITIALIZED );
}


DynamicEstimator::DynamicEstimator( const DynamicEstimator& rhs ) : Estimator( rhs )
{
	if ( rhs.realTimeAlgorithm != 0 )
		realTimeAlgorithm = new RealTimeAlgorithm( *(rhs.realTimeAlgorithm) );
	else
		realTimeAlgorithm = 0;
}


DynamicEstimator::~DynamicEstimator( )
{
	if ( realTimeAlgorithm != 0 )
		delete realTimeAlgorithm;
}


DynamicEstimator& DynamicEstimator::operator=( const DynamicEstimator& rhs )
{
	if ( this != &rhs )
	{
		if ( realTimeAlgorithm != 0 )
			delete realTimeAlgorithm;


		Estimator::operator=( rhs );


		if ( rhs.realTimeAlgorithm!= 0 )
			realTimeAlgorithm = new RealTimeAlgorithm( *(rhs.realTimeAlgorithm) );
		else
			realTimeAlgorithm = 0;
	}

    return *this;
}


Estimator* DynamicEstimator::clone( ) const
{
	return new DynamicEstimator( *this );
}



returnValue DynamicEstimator::init(	double startTime,
									const DVector &x0_,
									const DVector &p_
									)
{
	setStatus( BS_READY );

	return SUCCESSFUL_RETURN;
}


returnValue DynamicEstimator::step(	double currentTime,
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
