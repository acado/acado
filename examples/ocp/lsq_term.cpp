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
 *    \file   examples/ocp/lsq_term.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date   2009
 */


#include <acado_optimal_control.hpp>
#include <acado_gnuplot.hpp>


int main( ){

    USING_NAMESPACE_ACADO


    // INTRODUCE THE VARIABLES:
    // -------------------------
    const int N = 2;

    DifferentialState        x, y("", N, 1);
    Control                   u;
    DifferentialEquation      f;

    const double t_start =  0.0;
    const double t_end   = 10.0;


    // DEFINE A DIFFERENTIAL EQUATION:
    // -------------------------------

    f << dot(x) == -x + 0.9*x*x + u;

    int i;
    for( i = 0; i < N; i++ )
        f << dot( y(i) ) == -y(i) + 0.5*y(i)*y(i) + u;


    // DEFINE LEAST SQUARE FUNCTION:
    // -----------------------------

    Function h,m;

    h <<     x;
    h << 2.0*u;

    m << 10.0*x  ;
    m <<  0.1*x*x;

    DMatrix S(2,2);
    DVector r(2);

    S.setIdentity();
    r.setAll( 0.1 );


    // DEFINE AN OPTIMAL CONTROL PROBLEM:
    // ----------------------------------
    OCP ocp( t_start, t_end, 5 );

    ocp.minimizeLSQ       ( S, h, r );
    ocp.minimizeLSQEndTerm( S, m, r );

    ocp.subjectTo( f );
    ocp.subjectTo( AT_START, x == 1.0 );

    for( i = 0; i < N; i++ )
        ocp.subjectTo( AT_START, y(i) == 1.0 );


    // Additionally, flush a plotting object
    GnuplotWindow window;
        window.addSubplot( x,"DifferentialState x" );
        window.addSubplot( u,"Control u" );


    // DEFINE AN OPTIMIZATION ALGORITHM AND SOLVE THE OCP:
    // ---------------------------------------------------
    OptimizationAlgorithm algorithm(ocp);
    algorithm << window;

//    algorithm.set( PRINT_SCP_METHOD_PROFILE, YES );
//    algorithm.set( DYNAMIC_SENSITIVITY,  FORWARD_SENSITIVITY_LIFTED );
//    algorithm.set( HESSIAN_APPROXIMATION, CONSTANT_HESSIAN );
//    algorithm.set( HESSIAN_APPROXIMATION, FULL_BFGS_UPDATE );
//    algorithm.set( HESSIAN_APPROXIMATION, BLOCK_BFGS_UPDATE );
    algorithm.set( HESSIAN_APPROXIMATION, GAUSS_NEWTON );
//    algorithm.set( HESSIAN_APPROXIMATION, GAUSS_NEWTON_WITH_BLOCK_BFGS );
//    algorithm.set( HESSIAN_APPROXIMATION, EXACT_HESSIAN );

    // Necessary to use with GN Hessian approximation.
    algorithm.set( LEVENBERG_MARQUARDT, 1e-10 );

    algorithm.solve();

    return 0;
}



