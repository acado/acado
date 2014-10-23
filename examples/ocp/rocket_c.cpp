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
  *    \file   examples/ocp/rocket_c.cpp
  *    \author Boris Houska, Hans Joachim Ferreau
  *    \date   2010
  */


#include <acado_optimal_control.hpp>
#include <acado_gnuplot.hpp>


/* >>> start tutorial code >>> */


// -------------------------------------------------------------------------
//             C-STYLE DEFINITION OF THE PROBLEM DIMENSIONS:
// -------------------------------------------------------------------------


#define  NJ   1    // number of objective functions
#define  NX   4    // number of differential states
#define  NI   4    // number of initial value constraints
#define  NE   2    // number of end-point / terminal constraints
#define  NH   1    // number of inequality path constraints


// -----------------------------------------------------------------------------
//   UGLY C-STYLE DEFINITION OF THE OBJECTIVE, MODEL AND CONSTRAINT FUNCTIONS:
// -----------------------------------------------------------------------------


void myDifferentialEquation( double *x, double *f, void *user_data ){

    f[0] =  x[0];
    f[1] =  (x[2]-0.02*x[0]*x[0])/(1.0+x[1]);
    f[2] =  -0.01*x[2]*x[2];
    f[3] =  x[2]*x[2];
}

void myObjectiveFunction( double *x, double *f, void *user_data ){

    f[0] = x[3];
}


void myInitialValueConstraint( double *x, double *f, void *user_data ){

    f[0] =  x[4];
    f[1] =  x[0];
    f[2] =  x[1];
    f[3] =  x[3];
}


void myEndPointConstraint( double *x, double *f, void *user_data ){

    f[0] =  x[4] - 10.0;
    f[1] =  x[0];
}


void myInequalityPathConstraint( double *x, double *f, void *user_data ){

    f[0] =  x[0];
}


// -------------------------------------------------------------------------
//              USE THE ACADO TOOLKIT TO SOLVE THE PROBLEM:
// -------------------------------------------------------------------------


USING_NAMESPACE_ACADO


int main( ){


    // INTRODUCE THE VARIABLES:
    // --------------------------------------------------
    DifferentialState     s,v,m,L;
    Control               u      ;
    DifferentialEquation  f      ;


    // DEFINE THE DIMENSIONS OF THE C-FUNCTIONS:
    // --------------------------------------------------
    CFunction F( NX, myDifferentialEquation     );
    CFunction M( NJ, myObjectiveFunction        );
    CFunction I( NI, myInitialValueConstraint   );
    CFunction E( NE, myEndPointConstraint       );
    CFunction H( NH, myInequalityPathConstraint );


    // DEFINE THE OPTIMIZATION VARIABLES:
    // --------------------------------------------------

    IntermediateState x(5);

    x(0) = v; x(1) = m; x(2) = u; x(3) = L; x(4) = s;


    // DEFINE AN OPTIMAL CONTROL PROBLEM:
    // ----------------------------------
    OCP ocp( 0.0, 10.0 );

    ocp.minimizeMayerTerm( M(x) );

    ocp.subjectTo( f << F(x) );

    ocp.subjectTo( AT_START, I(x) ==  0.0 );
    ocp.subjectTo( AT_END  , E(x) ==  0.0 );
    ocp.subjectTo(           H(x) <=  1.3 );


    // VISUALIZE THE RESULTS IN A GNUPLOT WINDOW:
    // ------------------------------------------
    GnuplotWindow window1;
    window1.addSubplot( s,"DifferentialState s" );
    window1.addSubplot( v,"DifferentialState v" );
    window1.addSubplot( m,"DifferentialState m" );
    window1.addSubplot( u,"Control u" );


    // DEFINE AN OPTIMIZATION ALGORITHM AND SOLVE THE OCP:
    // ---------------------------------------------------
    OptimizationAlgorithm algorithm(ocp);
    algorithm.set( INTEGRATOR_TOLERANCE, 1e-6 );
    algorithm.set( KKT_TOLERANCE, 1e-3 );
    algorithm << window1;
    algorithm.solve();


    return 0;
}
/* <<< end tutorial code <<< */
