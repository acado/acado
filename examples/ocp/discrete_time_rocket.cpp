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
  *    \file   examples/ocp/discrete_time_rocket.cpp
  *    \author Boris Houska, Hans Joachim Ferreau
  *    \date   2009
  */


#include <acado_optimal_control.hpp>
#include <acado_gnuplot.hpp>


/* >>> start tutorial code >>> */
int main( ){

    USING_NAMESPACE_ACADO

    // INTRODUCE THE VARIABLES:
    // ------------------------------------
    DifferentialState                v,s,m;
    Control                          u    ;

    const double t_start =    0.0;
    const double t_end   =   10.0;
    const double h       =   0.01;

    DiscretizedDifferentialEquation  f(h) ;


    // DEFINE A DISCRETE-TIME SYTSEM:
    // -------------------------------
    f << next(s) == s + h*v;
    f << next(v) == v + h*(u-0.02*v*v)/m;
    f << next(m) == m - h*0.01*u*u;

	
	Function eta;
	eta << u;

    // DEFINE AN OPTIMAL CONTROL PROBLEM:
    // ----------------------------------
    OCP ocp( t_start, t_end, 50 );

    //ocp.minimizeLagrangeTerm( u*u );
	ocp.minimizeLSQ( eta );
    ocp.subjectTo( f );

    ocp.subjectTo( AT_START, s ==  0.0 );
    ocp.subjectTo( AT_START, v ==  0.0 );
    ocp.subjectTo( AT_START, m ==  1.0 );

    ocp.subjectTo( AT_END  , s == 10.0 );
    ocp.subjectTo( AT_END  , v ==  0.0 );

    ocp.subjectTo( -0.01 <= v <= 1.3 );


    // DEFINE A PLOT WINDOW:
    // ---------------------
    GnuplotWindow window;
        window.addSubplot( s,"DifferentialState s" );
        window.addSubplot( v,"DifferentialState v" );
        window.addSubplot( m,"DifferentialState m" );
        window.addSubplot( u,"Control u" );
        window.addSubplot( PLOT_KKT_TOLERANCE,"KKT Tolerance" );
        window.addSubplot( 0.5 * m * v*v,"Kinetic Energy" );


    // DEFINE AN OPTIMIZATION ALGORITHM AND SOLVE THE OCP:
    // ---------------------------------------------------
    OptimizationAlgorithm algorithm(ocp);
	algorithm.set( INTEGRATOR_TYPE, INT_DISCRETE );
    algorithm.set( HESSIAN_APPROXIMATION, EXACT_HESSIAN );
    algorithm.set( KKT_TOLERANCE, 1e-10 );

    algorithm << window;
    algorithm.solve();

    return 0;
}
/* <<< end tutorial code <<< */

