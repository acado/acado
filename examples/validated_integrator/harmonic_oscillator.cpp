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
 *    \file examples/validated_integrator/harmonoic_oscillator.cpp
 *    \author Boris Houska, Mario Villanueva, Benoit Chachuat
 *    \date 2013
 */

#include <acado/validated_integrator/ellipsoidal_integrator.hpp>


USING_NAMESPACE_ACADO


/* >>> start tutorial code >>> */
int main( ){

    // DEFINE VARIABLES:
    // ----------------------
    DifferentialState      x,y;
    DifferentialEquation   f;

    f << dot(x) ==  y;
	f << dot(y) == -x;
	
	Tmatrix<Interval> x_init(2);
	x_init(0) = Interval(-1.0,1.0);
	x_init(1) = Interval(-1.0,1.0);
	
	EllipsoidalIntegrator integrator( f, 8 );

	integrator.set(INTEGRATOR_PRINTLEVEL, MEDIUM );
	integrator.set(INTEGRATOR_TOLERANCE, 1e-8 );
	integrator.set(ABSOLUTE_TOLERANCE, 1e-10 );
	
	integrator.integrate( 0.0, 2*M_PI, 1, x_init );
	
    return 0;
}
/* <<< end tutorial code <<< */


