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
  *    \file   testing/Sternberg/lyapunov_test.cpp
  *    \author Julia Sternberg, Boris Houska
  *    \date   2009
  */

#include <acado_optimal_control.hpp>
#include <acado_gnuplot.hpp>


#define NX 2


USING_NAMESPACE_ACADO


Expression model( const Expression &x, const Expression &u, const Expression &w ){

    IntermediateState rhs(3);

    rhs(0) =   x(1);
    rhs(1) =  -x(0) - (1.0-u*u)*x(1) + w;
    rhs(2) =   x(0)*x(0) + u*u;


    return rhs;
}

int main( ){

    // INTRODUCE THE VARIABLES:
    // -------------------------
    DifferentialState     x1(3);
    Control               u(1) ;
    Parameter             ww   ;
    DifferentialEquation  f    ;

    IntermediateState rhs1(3);
    rhs1 = model( x1, u, ww );

    f << dot(x1(0)) == rhs1(0);
    f << dot(x1(1)) == rhs1(1);
    f << dot(x1(2)) == rhs1(2);

    DifferentialState P(3,3);
    IntermediateState A = forwardDerivative( rhs1, x1 );
    IntermediateState B = forwardDerivative( rhs1, ww );
   

    //f << dot(P) == A*P+P*A.transpose()+B*B.transpose();
    f << dot(P) == Lyapunov(rhs1,A,B,P,x1,u,ww);
 
    //simulate( f );

     // DEFINE AN OPTIMAL CONTROL PROBLEM:
    // ----------------------------------
    OCP ocp( 0.0, 2.0*M_PI, 20 );
    ocp.minimizeMayerTerm( x1(2) );
    //ocp.minimizeLagrangeTerm( x1(0)*x1(0) + u*u );


    ocp.subjectTo( f );

    ocp.subjectTo( AT_START, x1(0) ==  0.0 );
    ocp.subjectTo( AT_START, x1(1) ==  1.0 );
    ocp.subjectTo( AT_START, x1(2) ==  0.0 );
    ocp.subjectTo( AT_START, ww == 0.0 );
    ocp.subjectTo( AT_START, P == 0.0 );
    //ocp.subjectTo( 0.0, P     , -P     , 0.0 );
    ocp.subjectTo( x1(0) <=  0.6 );

    OptimizationAlgorithm algorithm(ocp);
    algorithm.set                  ( INTEGRATOR_TYPE , INT_LYAPUNOV45 );
    algorithm.set          (DYNAMIC_SENSITIVITY, FORWARD_SENSITIVITY);

    // DEFINE A PLOT WINDOW:
    // ---------------------
    GnuplotWindow window;
    window.addSubplot( x1(0),"state x(0)" );
    window.addSubplot( x1(1),"state x(1)" );
    window.addSubplot( P(0),"Lyapunov matrix P(0)" );
    window.addSubplot( P(1),"Lyapunov matrix P(1)" );
    window.addSubplot( P(2),"Lyapunov matrix P(2)" );
    window.addSubplot( P(3),"Lyapunov matrix P(3)" );
 
    window.addSubplot( u ,"Control u" );

    algorithm << window;

    algorithm.initializeControls("controls1.txt");
    algorithm.initializeDifferentialStates("states1.txt");
    algorithm.set( KKT_TOLERANCE, 1e-10 );
    algorithm.solve();

    algorithm.getDifferentialStates("states2.txt"    );
    algorithm.getControls          ("controls2.txt"  );
 
  return 0;
}



