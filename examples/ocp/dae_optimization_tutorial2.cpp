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
  *    \file   examples/ocp/dae_optimization_tutorial2.cpp
  *    \author Boris Houska, Hans Joachim Ferreau
  *    \date   2010
  */


#include <acado_optimal_control.hpp>
#include <acado_gnuplot.hpp>


int main( ){

    USING_NAMESPACE_ACADO

    DifferentialState         x;             // definition of a differential state
    AlgebraicState            z;             // definition of an algebraic state
    Control                   u;             // definition of a control
    Parameter                 p;             // definition of a parameter
    DifferentialEquation      f;             // a differential equation


    f << dot(x) == -0.5*x-z+u*u;             // an example for a differential-
    f <<      0 ==  z+exp(z)+x-1.0+u;        // algebraic equation.

    OCP ocp( 0.0, 4.0 );                     // define an OCP with t_0 = 0.0 and T = 4.0
    ocp.minimizeMayerTerm( x*x + p*p );      // a Mayer term to be minimized

    ocp.subjectTo( f );                      // OCP should regard the DAE
    ocp.subjectTo( AT_START, x     == 1.0 ); // an initial value constraint
    ocp.subjectTo( AT_END  , x + p == 1.0 ); // an end (or terminal) constraint

    ocp.subjectTo( -1.0 <= x*u <= 1.0 );     // a path constraint

    OptimizationAlgorithm algorithm(ocp);    // define an algorithm
    algorithm.set( KKT_TOLERANCE, 1e-5 );    // define a termination criterion
	algorithm.solve();                       // to solve the OCP.

    return 0;
}



