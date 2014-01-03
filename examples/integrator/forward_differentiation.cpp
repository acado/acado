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
 *    \file examples/integrator/forward_differentiation.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date 2008
 */


#include <acado_integrators.hpp>


/* >>> start tutorial code >>> */
int main( ){

    USING_NAMESPACE_ACADO


    // Define a Right-Hand-Side:
    // -------------------------

    DifferentialState   x, y;
    DifferentialEquation f;

    f << dot(x) ==  y;
    f << dot(y) == -x;

    // Define an integrator:
    // ---------------------

	IntegratorRK45 integrator( f );
    integrator.set( INTEGRATOR_PRINTLEVEL, MEDIUM );


    // Define an initial value:
    // ------------------------
    double x_start[2] = { 0.0, 1.0 };
    double t_start    =  0.0        ;
    double t_end      =  2.0*M_PI   ;


    // START THE INTEGRATION
    // AND COPUTE FIRST ORDER
    // FORWARD SENSITIVITIES:
    // ----------------------
    DVector seed(2);
    seed(0) = 1.0;
    seed(1) = 0.0;

	integrator.freezeAll( );
    integrator.integrate( t_start, t_end, x_start );

	integrator.setForwardSeed( 1, seed );
	integrator.integrateSensitivities();


    // GET THE RESULTS
    // ---------------

	VariablesGrid differentialStates;
	integrator.getX( differentialStates );

	DVector Dx( 2 );
	integrator.getForwardSensitivities( Dx,1 );
	
	differentialStates.print( "x" );
	Dx.print( "Dx" );


    return 0;
}
/* <<< end tutorial code <<< */


