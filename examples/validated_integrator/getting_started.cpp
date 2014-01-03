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
 *    \file examples/validated_integrator/getting_started.cpp
 *    \author Boris Houska, Mario Villanueva, Benoit Chachuat
 *    \date 2013
 */


#include <acado/validated_integrator/ellipsoidal_integrator.hpp>


USING_NAMESPACE_ACADO
    
typedef TaylorVariable<Interval> T;

/* >>> start tutorial code >>> */
int main( ){

    // DEFINE VARIABLES:
    // ----------------------
    DifferentialState      x;
    DifferentialEquation   f;

    f << dot(x) == -x*x;

	TaylorModel<Interval> Mod( 1, 1 );
	
	Tmatrix<T> x0(1);
	x0(0) = T( &Mod, 0, Interval(0.99,1.0));
	
	EllipsoidalIntegrator integrator( f, 4 );
	
	integrator.set(INTEGRATOR_PRINTLEVEL, HIGH );
	integrator.set(INTEGRATOR_TOLERANCE, 1e-4 );
	integrator.set(ABSOLUTE_TOLERANCE, 1e-4 );
	
	integrator.integrate( 0.0, 5.0, &x0 );

    return 0;
}
/* <<< end tutorial code <<< */


