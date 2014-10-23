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
  *    \file   examples/ocp/dae_optimization_tutorial.cpp
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
    DifferentialState         l;
    AlgebraicState            z;
    Control                   u;
    DifferentialEquation      f;

    const double t_start =  0.0;
    const double t_end   = 10.0;

    // DEFINE A DIFFERENTIAL EQUATION:
    // -------------------------------
    f << dot(x) == -x + 0.5*x*x + u + 0.5*z;
    f << dot(l) ==  x*x + 3.0*u*u            ;
    f <<      0 ==  z + exp(z) - 1.0 + x     ;


    // DEFINE AN OPTIMAL CONTROL PROBLEM:
    // ----------------------------------
    OCP ocp( t_start, t_end, 10 );
    ocp.minimizeMayerTerm( l );

    ocp.subjectTo( f );
    ocp.subjectTo( AT_START, x == 1.0 );
    ocp.subjectTo( AT_START, l == 0.0 );

    GnuplotWindow window;
        window.addSubplot(x,"DIFFERENTIAL STATE  x");
        window.addSubplot(z,"ALGEBRAIC STATE  z"   );
        window.addSubplot(u,"CONTROL u"            );


    // DEFINE AN OPTIMIZATION ALGORITHM AND SOLVE THE OCP:
    // ----------------------------------------------------
    OptimizationAlgorithm algorithm(ocp);

	algorithm.set( ABSOLUTE_TOLERANCE   , 1.0e-7        );
	algorithm.set( INTEGRATOR_TOLERANCE , 1.0e-7        );
	algorithm.set( HESSIAN_APPROXIMATION, EXACT_HESSIAN );
	//algorithm.set( GLOBALIZATION_STRATEGY, GS_FULLSTEP );

	algorithm << window;
    algorithm.solve();

    return 0;
}



