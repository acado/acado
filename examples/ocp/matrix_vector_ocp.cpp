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
  *    \file   examples/ocp/matrix_vector_ocp.cpp
  *    \author Boris Houska, Hans Joachim Ferreau
  *    \date   2009
  *
  */


#include <acado_optimal_control.hpp>
#include <acado_gnuplot.hpp>


/* >>> start tutorial code >>> */
int main( ){

    USING_NAMESPACE_ACADO

    // INTRODUCE THE VARIABLES:
    // ----------------------------
    DifferentialState    x("", 10, 1);    // a differential state vector with dimension 10. (vector)
    DifferentialState    y    ;    // another differential state y                   (scalar)
    Control              u("", 2, 1);    // a control input with dimension 2.              (vector)
    Parameter            p    ;    // a parameter (here a scalar).                   (scalar)

    DifferentialEquation f    ;    // the differential equation

    const double t_start =  0.0;
    const double t_end   =  1.0;


    // READ A MATRIX "A" FROM A FILE:
    // ------------------------------
    DMatrix A; A.read( "matrix_vector_ocp_A.txt" );
    DMatrix B; B.read( "matrix_vector_ocp_B.txt" );


    // READ A VECTOR "x0" FROM A FILE:
    // -------------------------------
    DVector x0; x0.read( "matrix_vector_ocp_x0.txt" );


    // DEFINE A DIFFERENTIAL EQUATION:
    // -------------------------------
    f << dot(x) == -(A*x) + B*u;                           // matrix vector notation for a linear equation
    f << dot(y) == x.transpose()*x + 2.0*u.transpose()*u;  // matrix vector notation:  x^x  = scalar product = ||x||_2^2
                                                           //                          u^u  = scalar product = ||u||_2^2


    // DEFINE AN OPTIMAL CONTROL PROBLEM:
    // ----------------------------------
    OCP ocp( t_start, t_end, 20 );
    ocp.minimizeMayerTerm( y );
    ocp.subjectTo( f );

    ocp.subjectTo( AT_START, x == x0  );
    ocp.subjectTo( AT_START, y == 0.0 );


    GnuplotWindow window;
        window.addSubplot( x(0),"x0" );
        window.addSubplot( x(6),"x6" );
        window.addSubplot( u(0),"u0" );
        window.addSubplot( u(1),"u1" );


    // DEFINE AN OPTIMIZATION ALGORITHM AND SOLVE THE OCP:
    // ---------------------------------------------------
    OptimizationAlgorithm algorithm(ocp);

    algorithm.set( MAX_NUM_ITERATIONS, 20 );
    algorithm.set( KKT_TOLERANCE, 1e-10 );

    algorithm << window;
    algorithm.solve();

    return 0;
}
/* <<< end tutorial code <<< */
