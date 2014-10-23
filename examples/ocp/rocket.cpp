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
  *    \file   examples/ocp/rocket.cpp
  *    \author Boris Houska, Hans Joachim Ferreau
  *    \date   2009
  */

#include <acado_optimal_control.hpp>
#include <acado_gnuplot.hpp>


/* >>> start tutorial code >>> */
int main( ){

    USING_NAMESPACE_ACADO

    // INTRODUCE THE VARIABLES:
    // -------------------------
    DifferentialState     v,s,m;
    Control               u    ;
    DifferentialEquation  f    ;

    const double t_start =  0.0;
    const double t_end   = 10.0;

    // DEFINE A DIFFERENTIAL EQUATION:
    // -------------------------------

    f << dot(s) == v;
    f << dot(v) == (u-0.02*v*v)/m;
    f << dot(m) == -0.01*u*u;


    // DEFINE AN OPTIMAL CONTROL PROBLEM:
    // ----------------------------------
    OCP ocp( t_start, t_end, 20 );
    ocp.minimizeLagrangeTerm( u*u );
    ocp.subjectTo( f );

	ocp.subjectTo( AT_START, s ==  0.0 );
    ocp.subjectTo( AT_START, v ==  0.0 );
    ocp.subjectTo( AT_START, m ==  1.0 );
    ocp.subjectTo( AT_END  , s == 10.0 );
    ocp.subjectTo( AT_END  , v ==  0.0 );

    ocp.subjectTo( -0.01 <= v <= 1.3 );
	
	ocp.subjectTo( u*u >= -1.0 );


    // DEFINE A PLOT WINDOW:
    // ---------------------
    GnuplotWindow window;
        window.addSubplot( s,"DifferentialState s" );
        window.addSubplot( v,"DifferentialState v" );
        window.addSubplot( m,"DifferentialState m" );
        window.addSubplot( u,"Control u" );
        window.addSubplot( PLOT_KKT_TOLERANCE,"KKT Tolerance" );
//         window.addSubplot( 0.5 * m * v*v,"Kinetic Energy" );


    // DEFINE AN OPTIMIZATION ALGORITHM AND SOLVE THE OCP:
    // ---------------------------------------------------
    OptimizationAlgorithm algorithm(ocp);

// 	algorithm.set( INTEGRATOR_TYPE, INT_BDF );

//     algorithm.set( INTEGRATOR_TOLERANCE, 1e-6 );
//     algorithm.set( KKT_TOLERANCE, 1e-3 );

  //algorithm.set( DYNAMIC_SENSITIVITY,  FORWARD_SENSITIVITY );



    algorithm.set( HESSIAN_APPROXIMATION, EXACT_HESSIAN );
    algorithm.set( MAX_NUM_ITERATIONS, 20 );
    algorithm.set( KKT_TOLERANCE, 1e-10 );
// 	algorithm.set( MAX_NUM_INTEGRATOR_STEPS, 4 );

    algorithm << window;
    algorithm.solve();

// 	BlockMatrix sens;
// 	algorithm.getSensitivitiesX( sens );
// 	sens.print();

    return 0;
}
/* <<< end tutorial code <<< */



//  algorithm.set( DISCRETIZATION_TYPE, MULTIPLE_SHOOTING );
//  algorithm.set( DISCRETIZATION_TYPE, SINGLE_SHOOTING   );
//
//  algorithm.set( DYNAMIC_SENSITIVITY, FORWARD_SENSITIVITY );
//  algorithm.set( DYNAMIC_SENSITIVITY, BACKWARD_SENSITIVITY );

//  algorithm.set( INTEGRATOR_TYPE, INT_RK45 );
//  algorithm.set( INTEGRATOR_TYPE, INT_RK78 );
//  algorithm.set( INTEGRATOR_TYPE, INT_BDF );
//
//  algorithm.set( KKT_TOLERANCE, 1e-4 );
//  algorithm.set( MAX_NUM_ITERATIONS, 20 );

//  algorithm.set( PRINT_SCP_METHOD_PROFILE, YES );
