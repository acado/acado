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
  *    \file   examples/ocp/van_der_pol.cpp
  *    \author Joel Andersson, Boris Houska
  *    \date   2009
  */


#include <acado_optimal_control.hpp>
#include <acado_gnuplot.hpp>


/* >>> start tutorial code >>> */
int main( ){

    USING_NAMESPACE_ACADO;


    // INTRODUCE THE VARIABLES:
    // -------------------------

    DifferentialState         x1,x2;
    Control                   u;
    Parameter                 p;
    Parameter                 T;
    DifferentialEquation      f(0.0,T);

    const double t_start =  0.0;


    // DEFINE A DIFFERENTIAL EQUATION:
    // -------------------------------

    f << dot(x1) ==  (1.0-x2*x2)*x1 - x2 + p*u;
    f << dot(x2) ==  x1;


    // DEFINE AN OPTIMAL CONTROL PROBLEM:
    // ----------------------------------
    OCP ocp( t_start, T, 27 );


 //   ocp.minimizeMayerTerm( T );
    ocp.minimizeLagrangeTerm(10*x1*x1 + 10*x2*x2 + u*u);

    ocp.subjectTo( f );
    ocp.subjectTo( AT_START, x1 ==  0.0 );
    ocp.subjectTo( AT_START, x2 ==  1.0 );

    ocp.subjectTo( AT_END  , x1 ==  0.0 );
    ocp.subjectTo( AT_END  , x2 ==  0.0 );

    ocp.subjectTo( -0.5 <= u <= 1.0 );

    ocp.subjectTo( p == 1.0 );
    ocp.subjectTo( 0.0 <= T <= 20.0 );


    // VISUALIZE THE RESULTS IN A GNUPLOT WINDOW:
    // ------------------------------------------
    GnuplotWindow window;
        window << x1;
        window << x2;
        window << u;
        window << T;

    // DEFINE AN OPTIMIZATION ALGORITHM AND SOLVE THE OCP:
    // ---------------------------------------------------
    OptimizationAlgorithm algorithm(ocp);

    algorithm.initializeControls("van_der_pol_controls.txt");

    algorithm << window;
    algorithm.solve();

    algorithm.getControls("van_der_pol_controls2.txt");

    return 0;
}
/* <<< end tutorial code <<< */


