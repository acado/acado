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
*    \file src/noise/uniform_noise.cpp
*    \author Boris Houska, Hans Joachim Ferreau
*/


#include <acado/noise/uniform_noise.hpp>

#include <stdlib.h>
#include <time.h>



BEGIN_NAMESPACE_ACADO




UniformNoise::UniformNoise( ) : Noise( )
{
}


UniformNoise::UniformNoise(	const DVector& _lowerLimit,
							const DVector& _upperLimit
							) : Noise( )
{
	if ( _lowerLimit.getDim( ) == _upperLimit.getDim( ) )
	{
		w.init( _lowerLimit.getDim( ),1 );
		lowerLimit = _lowerLimit;
		upperLimit = _upperLimit;
	}
	else
		ACADOERROR( RET_INVALID_NOISE_SETTINGS );
}


UniformNoise::UniformNoise(	uint _dim,
							double _lowerLimit,
							double _upperLimit
							) : Noise( )
{
	w.init( _dim,1 );
	lowerLimit.init(_dim);
	upperLimit.init(_dim);
	
	for( uint i=0; i<_dim; ++i )
	{
		lowerLimit(i) = _lowerLimit;
		upperLimit(i) = _upperLimit;
	}
}


UniformNoise::UniformNoise( const UniformNoise &rhs ) : Noise( rhs )
{
	lowerLimit = rhs.lowerLimit;
	upperLimit = rhs.upperLimit;
}


UniformNoise::~UniformNoise( )
{
}


UniformNoise& UniformNoise::operator=( const UniformNoise &rhs )
{
	if( this != &rhs )
	{
		Noise::operator=( rhs );

		lowerLimit = rhs.lowerLimit;
		upperLimit = rhs.upperLimit;
	}

	return *this;
}


UniformNoise* UniformNoise::clone( ) const
{
    return ( new UniformNoise( *this ) );
}


UniformNoise* UniformNoise::clone(	uint idx
									) const
{
	if ( idx >= getDim( ) )
		return 0;

	UniformNoise tmp( DVector(1),DVector(1) );
	tmp.Noise::operator=( *this );
	tmp.w.init( 1,1 );
	tmp.lowerLimit(0) = lowerLimit(idx);
	tmp.upperLimit(0) = upperLimit(idx);

    return ( new UniformNoise( tmp ) );
}



returnValue UniformNoise::setLimits(	const DVector& _lowerLimit,
										const DVector& _upperLimit
										)
{
	if ( lowerLimit.getDim( ) != _lowerLimit.getDim( ) )
		return ACADOERROR( RET_VECTOR_DIMENSION_MISMATCH );

	if ( upperLimit.getDim( ) != _upperLimit.getDim( ) )
		return ACADOERROR( RET_VECTOR_DIMENSION_MISMATCH );

	for( uint i=0; i<_lowerLimit.getDim( ); ++i )
		if ( _lowerLimit(i) > _upperLimit(i) )
			return ACADOERROR( RET_INVALID_ARGUMENTS );

	lowerLimit = _lowerLimit;
	upperLimit = _upperLimit;

	return SUCCESSFUL_RETURN;
}


returnValue UniformNoise::setLimits(	double _lowerLimit,
										double _upperLimit
										)
{
	if ( _lowerLimit > _upperLimit )
		return ACADOERROR( RET_INVALID_ARGUMENTS );

	for( uint i=0; i<getDim( ); ++i )
	{
		lowerLimit(i) = _lowerLimit;
		upperLimit(i) = _upperLimit;
	}

	return SUCCESSFUL_RETURN;
}


returnValue UniformNoise::setLimit(	uint idx,
									double _lowerLimit,
									double _upperLimit
									)
{
	if ( idx >= getDim( ) )
		return ACADOERROR( RET_INDEX_OUT_OF_BOUNDS );

	if ( _lowerLimit > _upperLimit )
		return ACADOERROR( RET_INVALID_ARGUMENTS );

	lowerLimit(idx) = _lowerLimit;
	upperLimit(idx) = _upperLimit;

	return SUCCESSFUL_RETURN;
}



returnValue UniformNoise::init(	uint seed
								)
{
	if ( lowerLimit.getDim( ) != upperLimit.getDim( ) )
		return ACADOERROR( RET_INVALID_NOISE_SETTINGS );

	if ( lowerLimit.getDim( ) == 0 )
		return ACADOERROR( RET_NO_NOISE_SETTINGS );

	/* initialize random seed: */
	if ( seed == 0 )
		srand( (unsigned int)time(0) );
	else
		srand( seed );

	setStatus( BS_READY );

	return SUCCESSFUL_RETURN;
}


returnValue UniformNoise::step(	DVector& _w
								)
{
	if ( getStatus( ) != BS_READY )
		return ACADOERROR( RET_BLOCK_NOT_READY );

	if ( getDim( ) != _w.getDim( ) )
		return ACADOERROR( RET_VECTOR_DIMENSION_MISMATCH );

	if ( w.getNumPoints( ) != 1 )
		w.init( 1,getDim() );

	for( uint j=0; j<getDim( ); ++j )
		w(0,j) = getUniformRandomNumber( lowerLimit(j),upperLimit(j) );

	_w = w.getVector( 0 );

	return SUCCESSFUL_RETURN;
}


returnValue UniformNoise::step(	VariablesGrid& _w
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
			w(i,j) = getUniformRandomNumber( lowerLimit(j),upperLimit(j) );

	_w = w;

	return SUCCESSFUL_RETURN;
}


//
// PROTECTED MEMBER FUNCTIONS:
//



CLOSE_NAMESPACE_ACADO

// end of file.
