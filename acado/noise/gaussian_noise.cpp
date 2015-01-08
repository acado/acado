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
*    \file src/noise/gaussian_noise.cpp
*    \author Boris Houska, Hans Joachim Ferreau
*/


#include <acado/noise/gaussian_noise.hpp>

#include <stdlib.h>
#include <time.h>



BEGIN_NAMESPACE_ACADO




GaussianNoise::GaussianNoise( ) : Noise( )
{
}


GaussianNoise::GaussianNoise(	const DVector& _mean,
								const DVector& _variance
								) : Noise( )
{
	if ( _mean.getDim( ) == _variance.getDim( ) )
	{
		w.init( _mean.getDim( ),1 );
		mean = _mean;
		variance = _variance;
	}
	else
		ACADOERROR( RET_INVALID_NOISE_SETTINGS );
}


GaussianNoise::GaussianNoise(	uint _dim,
								double _mean,
								double _variance
								) : Noise( )
{
	w.init( _dim,1 );

	mean.init( _dim );
	variance.init( _dim );
	
	for( uint i=0; i<_dim; ++i )
	{
		mean(i)     = _mean;
		variance(i) = _variance;
	}
}


GaussianNoise::GaussianNoise( const GaussianNoise &rhs ) : Noise( rhs )
{
	mean = rhs.mean;
	variance = rhs.variance;
}


GaussianNoise::~GaussianNoise( )
{
}


GaussianNoise& GaussianNoise::operator=( const GaussianNoise &rhs )
{
	if( this != &rhs )
	{
		Noise::operator=( rhs );

		mean = rhs.mean;
		variance = rhs.variance;
	}

	return *this;
}


GaussianNoise* GaussianNoise::clone( ) const
{
    return ( new GaussianNoise( *this ) );
}


GaussianNoise* GaussianNoise::clone(	uint idx
										) const
{
	if ( idx >= getDim( ) )
		return 0;

	GaussianNoise tmp( DVector(1),DVector(1) );
	tmp.Noise::operator=( *this );
	tmp.w.init( 1,1 );
	tmp.mean(0)     = mean(idx);
	tmp.variance(0) = variance(idx);

    return ( new GaussianNoise( tmp ) );
}



returnValue GaussianNoise::init(	uint seed
									)
{
	if ( mean.getDim( ) != variance.getDim( ) )
		return ACADOERROR( RET_INVALID_NOISE_SETTINGS );

	if ( mean.getDim( ) == 0 )
		return ACADOERROR( RET_NO_NOISE_SETTINGS );

	/* initialize random seed: */
	if ( seed == 0 )
		srand( (unsigned int)time(0) );
	else
		srand( seed );

	setStatus( BS_READY );

	return SUCCESSFUL_RETURN;
}


returnValue GaussianNoise::step(	DVector& _w
									)
{
	if ( getStatus( ) != BS_READY )
		return ACADOERROR( RET_BLOCK_NOT_READY );

	if ( getDim( ) != _w.getDim( ) )
		return ACADOERROR( RET_VECTOR_DIMENSION_MISMATCH );

	if ( w.getNumPoints( ) != 1 )
		w.init( 1,getDim() );

	for( uint j=0; j<getDim( ); ++j )
		w(0,j) = getGaussianRandomNumber( mean(j),variance(j) );

	_w = w.getVector( 0 );

	return SUCCESSFUL_RETURN;
}


returnValue GaussianNoise::step(	VariablesGrid& _w
									)
{
	if ( getStatus( ) != BS_READY )
		return ACADOERROR( RET_BLOCK_NOT_READY );

	if ( getDim( ) != _w.getNumValues( ) )
		return ACADOERROR( RET_VECTOR_DIMENSION_MISMATCH );

	if ( w.getNumPoints( ) != _w.getNumPoints( ) )
		w.init( getDim(),_w.getNumPoints( ) );

	for( uint i=0; i<_w.getNumPoints( ); ++i )
		for( uint j=0; j<getDim( ); ++j )
			w(i,j) = getGaussianRandomNumber( mean(j),variance(j) );

	_w = w;

	return SUCCESSFUL_RETURN;
}


//
// PROTECTED MEMBER FUNCTIONS:
//

double GaussianNoise::getGaussianRandomNumber(	double _mean,
												double _variance
												) const
{
	// Box-Muller method
	double norm = 2.0;
	double uniformRandomNumber1, uniformRandomNumber2;

	while ( norm >= 1.0 )
	{
		uniformRandomNumber1 = getUniformRandomNumber( -1.0,1.0 );
		uniformRandomNumber2 = getUniformRandomNumber( -1.0,1.0 );
		norm = uniformRandomNumber1*uniformRandomNumber1 + uniformRandomNumber2*uniformRandomNumber2;
	}
   
	double gaussianRandomNumber1 = sqrt( -2.0 * log(norm)/norm ) * uniformRandomNumber1;
	double gaussianRandomNumber2 = sqrt( -2.0 * log(norm)/norm ) * uniformRandomNumber2;

	return _mean + sqrt( _variance ) * (gaussianRandomNumber1+gaussianRandomNumber2)/2.0;
}




CLOSE_NAMESPACE_ACADO

// end of file.
