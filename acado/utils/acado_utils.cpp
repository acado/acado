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
 *    \file src/utils/acado_utils.cpp
 *    \author Hans Joachim Ferreau, Boris Houska, Milan Vukov
 *    \date 2008 - 2013
 */

#include <acado/utils/acado_utils.hpp>

BEGIN_NAMESPACE_ACADO

BooleanType acadoIsInteger( double x )
{
	if ( fabs( x - floor( x + 0.5) ) < 1.0e-5 )
		return BT_TRUE;
	else
		return BT_FALSE;
}


double acadoDiv( double nom, double den )
{
	if ( acadoIsInteger( nom/den ) == BT_TRUE )
		return floor( nom/den + 0.5 );
	else
		return floor( nom/den );
}


double acadoMod( double nom, double den )
{
	if ( acadoIsInteger( nom/den ) == BT_TRUE )
		return 0.0;
	else
		return ( nom/den ) - floor( nom/den );
}


int acadoMax( const int x, const int y ){

    return (y<x)?x:y;
}


double acadoMax( const double x, const double y ){

    return (y<x)?x:y;
}


int acadoMin( const int x, const int y ){

    return (y>x)?x:y;
}


double acadoMin( const double x, const double y ){

    return (y>x)?x:y;
}


BooleanType acadoIsEqual( double x, double y, double TOL ){

  double maxabs= acadoMax(fabs(x),fabs(y));
	if(maxabs > 1)
	{
		// use relative error
		if ( fabs( x-y )/maxabs >= 10.0*TOL ) return BT_FALSE;
		else                      return BT_TRUE ;
	}
	else
	{
		// use absolute error
		if ( fabs( x-y ) >= 10.0*TOL ) return BT_FALSE;
		else                      return BT_TRUE ;
	}
}


BooleanType acadoIsGreater( double x, double y, double TOL ){

    if ( x >= y - TOL ) return BT_TRUE ;
    else                return BT_FALSE;
}


BooleanType acadoIsSmaller( double x, double y, double TOL ){

    if ( x <= y + TOL ) return BT_TRUE ;
    else                return BT_FALSE;
}


BooleanType acadoIsStrictlyGreater( double x, double y, double TOL ){

    if ( x > y + TOL ) return BT_TRUE ;
    else                return BT_FALSE;
}


BooleanType acadoIsStrictlySmaller( double x, double y, double TOL ){

    if ( x < y - TOL ) return BT_TRUE ;
    else                return BT_FALSE;
}


BooleanType acadoIsPositive( double x, double TOL ){

    if ( x >= TOL ) return BT_TRUE ;
    else            return BT_FALSE;
}


BooleanType acadoIsNegative( double x, double TOL ){

    if ( x <= -TOL ) return BT_TRUE ;
    else             return BT_FALSE;
}


BooleanType acadoIsZero( double x, double TOL ){

    return acadoIsEqual( x, 0.0, TOL );
}


BooleanType acadoIsInfty( double x, double TOL )
{
	if ( ( acadoIsGreater( x, INFTY, TOL ) == BT_TRUE ) ||
		 ( acadoIsSmaller( x,-INFTY, TOL ) == BT_TRUE ) )
		return BT_TRUE;
	else
		return BT_FALSE;
}


BooleanType acadoIsFinite( double x, double TOL )
{
	if ( ( acadoIsStrictlySmaller( x, INFTY, TOL ) == BT_TRUE ) &&
		 ( acadoIsStrictlyGreater( x,-INFTY, TOL ) == BT_TRUE ) )
		return BT_TRUE;
	else
		return BT_FALSE;
}


BooleanType acadoIsNaN(	double x
						)
{
	if ( ( x > -1.0 ) || ( x < 1.0 ) )
		return BT_FALSE;
	else
		return BT_TRUE;
}



int acadoRound (double x){

	if (x - floor(x) > 0.5)
		return (int)ceil(x);
	else
		return (int)floor(x);

}


int acadoFactorial( int n ){
  
	ASSERT( n>= 0 );
	int r = 1;
	for (int i = n; i > 1; --i)
		r *= i;
	return r;
}


int acadoRoundAway (double x) {

	return x < 0 ? floor(x) : ceil(x);
}

CLOSE_NAMESPACE_ACADO

/*
 *	end of file
 */
