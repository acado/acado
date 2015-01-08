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
  *    \file   examples/ocp/getting_started.cpp
  *    \author Boris Houska, Hans Joachim Ferreau
  *    \date   2009
  */

#include <acado_optimal_control.hpp>
#include <acado_gnuplot.hpp>


int main( ){

    USING_NAMESPACE_ACADO


    // INTRODUCE THE VARIABLES:
    // -------------------------

    DifferentialState         x;
    Control                   u;
    Disturbance               w;
    Parameter               p,q;
    DifferentialEquation      f;

    const double t_start = 0.0;
    const double t_end   = 1.0;


    // DEFINE A DIFFERENTIAL EQUATION:
    // -------------------------------

    f << -dot(x) -x*x + p + u*u + w;


    // DEFINE AN OPTIMAL CONTROL PROBLEM:
    // ----------------------------------
    OCP ocp( t_start, t_end, 20 );

    ocp.minimizeMayerTerm( x + p*p + q*q );
    ocp.subjectTo( f );
    ocp.subjectTo( AT_START, x == 1.0 );
    ocp.subjectTo(  0.1 <= u <= 2.0 );
    ocp.subjectTo( -0.1 <= w <= 2.1 );


    // DEFINE AN OPTIMIZATION ALGORITHM AND SOLVE THE OCP:
    // ---------------------------------------------------
    OptimizationAlgorithm algorithm(ocp);

//    algorithm.set( HESSIAN_APPROXIMATION, EXACT_HESSIAN );
    algorithm.solve();


    return 0;
}



