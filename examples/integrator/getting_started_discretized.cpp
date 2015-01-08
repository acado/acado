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
 *    \file examples/integrator/getting_started_discretized.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date 2008
 */


#include <acado_integrators.hpp>


/* >>> start tutorial code >>> */
int main( ){

    USING_NAMESPACE_ACADO

    DifferentialState   x;
    Control             u;

    const double h = 0.01;
    DiscretizedDifferentialEquation f(h);

    const double t_start =  0.0;
    const double t_end   =  1.0;


    // DEFINE A DIFFERENTIAL EQUATION:
    // -------------------------------
    f << next(x) == x - h*x + h*u*u;


    // Define an integrator:
    // ---------------------
    IntegratorDiscretizedODE integrator( f );

    integrator.set( INTEGRATOR_PRINTLEVEL, MEDIUM );


    // Define an initial value:
    // ------------------------

    double x_start[1] = { 1.0 };
    double u_start[1] = { 1.0 };


    // START THE INTEGRATION
    // ----------------------

    integrator.freezeAll();
    integrator.integrate( t_start, t_end, x_start, 0, 0, u_start );

    // ----------------------
    DVector seed(1);
    seed(0) = 1.0;

    integrator.setForwardSeed( 1, emptyVector, emptyVector, seed );
    integrator.integrateSensitivities();

    integrator.setForwardSeed( 2, emptyVector, emptyVector, seed );
    integrator.integrateSensitivities();


    // GET THE RESULTS
    // ---------------

	VariablesGrid differentialStates;
	integrator.getX( differentialStates );

	DVector Dx( 1 );
	integrator.getForwardSensitivities( Dx,1 );
	
	differentialStates.print( "x" );
	Dx.print( "Dx" );


    return 0;
}
/* <<< end tutorial code <<< */
