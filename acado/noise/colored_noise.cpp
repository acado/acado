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
 *	\file src/noise/colored_noise.cpp
 *	\author Boris Houska, Hans Joachim Ferreau
 */


#include <acado/noise/colored_noise.hpp>



BEGIN_NAMESPACE_ACADO




ColoredNoise::ColoredNoise( ) : Noise( )
{
	dynamicSystem = 0;
}


ColoredNoise::ColoredNoise(	const DynamicSystem& _dynamicSystem
							) : Noise( )
{
	dynamicSystem = new DynamicSystem( _dynamicSystem );
}


ColoredNoise::ColoredNoise( const ColoredNoise &rhs ) : Noise( rhs )
{
	if( rhs.dynamicSystem != 0 )
		dynamicSystem  = new DynamicSystem( *(rhs.dynamicSystem) );
	else
		dynamicSystem  = 0;
}


ColoredNoise::~ColoredNoise( )
{
	if( dynamicSystem != 0 )
		delete dynamicSystem;
}


ColoredNoise& ColoredNoise::operator=( const ColoredNoise &rhs )
{
	if( this != &rhs )
	{
		if( dynamicSystem != 0 )
			delete dynamicSystem;

		Noise::operator=( rhs );

		if( rhs.dynamicSystem != 0 )
			dynamicSystem  = new DynamicSystem( *(rhs.dynamicSystem) );
		else
			dynamicSystem  = 0;
	}

	return *this;
}


ColoredNoise* ColoredNoise::clone( ) const
{
	return ( new ColoredNoise( *this ) );
}


ColoredNoise* ColoredNoise::clone(	uint idx
									) const
{
	if ( idx >= getDim( ) )
		return 0;

	return ( new ColoredNoise( *this ) );
}


returnValue ColoredNoise::setDynamicSystem(	const DynamicSystem& _dynamicSystem
											)
{
	if( dynamicSystem != 0 )
		*dynamicSystem = _dynamicSystem;
	else
		dynamicSystem  = new DynamicSystem( _dynamicSystem );

	return SUCCESSFUL_RETURN;
}



returnValue ColoredNoise::init(	uint seed
								)
{
	w.setZero( );

	setStatus( BS_READY );
	return SUCCESSFUL_RETURN;
}


returnValue ColoredNoise::step( DVector& _w
								)
{
	return ACADOERROR( RET_NOT_YET_IMPLEMENTED );
}



returnValue ColoredNoise::step( VariablesGrid& _w
								)
{
	return ACADOERROR( RET_NOT_YET_IMPLEMENTED );
}


CLOSE_NAMESPACE_ACADO

// end of file.
