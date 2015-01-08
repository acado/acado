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
 *    \file examples/curve/getting_started.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *
 */

 
#include <acado_integrators.hpp>
#include <acado_gnuplot.hpp>


/* >>> start tutorial code >>> */
int main( ){

    USING_NAMESPACE_ACADO


    // DEFINE SOME FUNCTIONS:
    // ----------------------

    TIME t;
    Function sine, cosine,
             ramp1, ramp2, ramp3,
             parabola;

    sine   << sin(t);
    cosine << cos(t);

    ramp1  <<  t      ;
    ramp2  <<  t - 1.0;
    ramp3  <<  t - 2.0;

    parabola << t*t/M_PI - 3.0*t + 2.0*M_PI;


    // DEFINE SOME CURVES:
    // -------------------

    Curve c1, c2, c3, c4;

    c1.add( 0.0, 2.0*M_PI, sine   );
    c2.add( 0.0, 2.0*M_PI, cosine );

    c3.add( 0.0, 1.0, ramp1 );
    c3.add( 1.0, 2.0, ramp2 );
    c3.add( 2.0, 3.0, ramp3 );

    c4.add( 0.0 ,     M_PI, sine     );
    c4.add( M_PI, 2.0*M_PI, parabola );



    // PLOT CURVES ON GIVEN GRID:
    // --------------------------
	GnuplotWindow window;
         window.addSubplot( c1, 0.0,2.0*M_PI, "Sampled sine function"   );
         window.addSubplot( c2, 0.0,2.0*M_PI, "Sampled cosine function" );
         window.addSubplot( c3, 0.0,3.0,      "Sampled ramp function"   );
         window.addSubplot( c4, 0.0,2.0*M_PI, "composed curve"          );
    window.plot();

    return 0;
}
/* <<< end tutorial code <<< */


