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
  *    \file   examples/multi_objective/car_nbi.cpp
  *    \author Filip Logist, Boris Houska, Hans Joachim Ferreau
  *    \date   2009
  *
  *    Normal Boundary Intersection for a car example with conflicting time and energy cost
  *    J1 = int_0^T max(0.0,u)dt
  *    J2 = T
  *    due to symmetry u_min = -u_max only half the time interval is simulated 
  *    [O.0,t1] = [0.0,t1/2.0]
  *
  *    Reference:
  *    P. Van Erdeghem, F. Logist, I. Smets, and J. Van Impe 2008.
  *    Improved procedures for multiple objective optimal control problems.
  *    In: Proceedings of the 17th IFAC World Congress, 7802-7807, Seoul (Korea)
  *
  */


// IMPLEMENTATION:
// ---------------

#include <acado_optimal_control.hpp>
#include <acado_gnuplot.hpp>


/* >>> start tutorial code >>> */
int main( ){

    USING_NAMESPACE_ACADO


    // INTRODUCE THE VARIABLES:
    // ----------------------------
    DifferentialState        x1,x2;
    Control                  u    ;
    Parameter                t1   ;

    DifferentialEquation f(0.0,t1);


    // DEFINE A DIFFERENTIAL EQUATION:
    // -------------------------------
    f << dot(x1) ==  x2;
    f << dot(x2) ==   u;


    // DEFINE AN OPTIMAL CONTROL PROBLEM:
    // ----------------------------------
    OCP ocp(0.0,t1,25);
    ocp.minimizeMayerTerm( 0, x2         );
    ocp.minimizeMayerTerm( 1, 2.0*t1/20.0);

    ocp.subjectTo( f );

    ocp.subjectTo( AT_START, x1 ==   0.0 );
    ocp.subjectTo( AT_START, x2 ==   0.0 );
    ocp.subjectTo( AT_END  , x1 == 200.0 );

    ocp.subjectTo( 0.0 <= x1 <= 200.0001 );
    ocp.subjectTo( 0.0 <= x2 <=  40.0    );
    ocp.subjectTo( 0.0 <= u  <=   5.0    );
    ocp.subjectTo( 0.1 <= t1 <=  50.0    );


    // DEFINE A MULTI-OBJECTIVE ALGORITHM AND SOLVE THE OCP:
    // -----------------------------------------------------
    MultiObjectiveAlgorithm algorithm(ocp);

    algorithm.set( PARETO_FRONT_DISCRETIZATION, 11 );
    algorithm.set( PARETO_FRONT_GENERATION, PFG_NORMAL_BOUNDARY_INTERSECTION );
    algorithm.set( KKT_TOLERANCE, 1e-8 );

    // Minimize individual objective function
    algorithm.solveSingleObjective(0);

    // Minimize individual objective function
    algorithm.solveSingleObjective(1);

    // Generate Pareto set
    algorithm.solve();

    algorithm.getWeights("car_nbi_weights.txt");
    algorithm.getAllDifferentialStates("car_nbi_states.txt");
    algorithm.getAllControls("car_nbi_controls.txt");
    algorithm.getAllParameters("car_nbi_parameters.txt");


    // GET THE RESULT FOR THE PARETO FRONT AND PLOT IT:
    // ------------------------------------------------
    VariablesGrid paretoFront;
    algorithm.getParetoFront( paretoFront );

    GnuplotWindow window1;
    window1.addSubplot( paretoFront, "Pareto Front (time versus energy)", "ENERGY","TIME", PM_POINTS );
    window1.plot( );


    // PRINT INFORMATION ABOUT THE ALGORITHM:
    // --------------------------------------
    algorithm.printInfo();


    // SAVE INFORMATION:
    // -----------------
    paretoFront.print( "car_nbi_pareto.txt" );

    return 0;
}
/* <<< end tutorial code <<< */

