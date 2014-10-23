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
 *    \file examples/integrator/harmonic_oscillator.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date 2008
 */


#include <acado_integrators.hpp>
#include <acado_gnuplot.hpp>


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
	integrator.set( PRINT_INTEGRATOR_PROFILE, YES );
	
    // Define an initial value:
    // ------------------------
    double x_start[2] = { 0.0, 1.0 };
    Grid timeInterval( 0.0, 2.0*M_PI, 100 );

    integrator.freezeAll();
    integrator.integrate( timeInterval, x_start );


    // GET THE RESULTS
    // ---------------

    VariablesGrid differentialStates;
    integrator.getX( differentialStates );

    GnuplotWindow window;
        window.addSubplot( differentialStates(0) );
        window.addSubplot( differentialStates(1) );

    window.plot();


//     DVector seed(2);
// 
//     seed( 0 ) = 1.0;
//     seed( 1 ) = 0.0;
// 
//     integrator.setForwardSeed( 1, seed );
//     integrator.integrateSensitivities();
// 
//     VariablesGrid sens;
//     integrator.getForwardSensitivities( sens, 1 );
// 
//     GnuplotWindow window2;
//         window2.addSubplot( sens(0) );
//         window2.addSubplot( sens(1) );
//     window2.plot();


    return 0;
}



