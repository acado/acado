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
  *    \file   examples/multi_objective/catalyst_mixing_nbi.cpp
  *    \author Filip Logist, Boris Houska, Hans Joachim Ferreau
  *    \date   2009
  *
  *     Objectives:
  *	Maximize desired product
  *     Minimize catalyst A 
  *
  *	Reference: 
  *     Adapted from 
  *     Gunn and W.J. Thomas, 1965.
  *     Mass transport and chemical reaction in multifunctional catalysts.
  *     Chem. Eng. Sci. 20, 89.
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
    // -------------------------
    DifferentialState     x1,x2,x3;
    Control               u;

    DifferentialEquation  f(0.0,1.0);


    // DEFINE A DIFFERENTIAL EQUATION:
    // -------------------------------
    f << dot(x1) == -u*(x1-10.0*x2);
    f << dot(x2) ==  u*(x1-10.0*x2)-(1.0-u)*x2;
    f << dot(x3) ==  u/10.0;


    // DEFINE AN OPTIMAL CONTROL PROBLEM:
    // ----------------------------------
    OCP ocp(0.0,1.0,25);
    ocp.minimizeMayerTerm( 0, -(1.0-x1-x2));
    ocp.minimizeMayerTerm( 1, x3          );

    ocp.subjectTo( f );

    ocp.subjectTo( AT_START, x1 == 1.0  );
    ocp.subjectTo( AT_START, x2 == 0.0  );
    ocp.subjectTo( AT_START, x3 == 0.0  );

    ocp.subjectTo(  0.0 <= x1 <= 1.0  );
    ocp.subjectTo(  0.0 <= x2 <= 1.0  );
    ocp.subjectTo(  0.0 <= x3 <= 1.0  );
    ocp.subjectTo(  0.0 <= u  <= 1.0  );


    // DEFINE A MULTI-OBJECTIVE ALGORITHM AND SOLVE THE OCP:
    // -----------------------------------------------------
    MultiObjectiveAlgorithm algorithm(ocp);

    algorithm.set( PARETO_FRONT_GENERATION    , PFG_NORMAL_BOUNDARY_INTERSECTION );
    algorithm.set( PARETO_FRONT_DISCRETIZATION, 11               );
    algorithm.set( HESSIAN_APPROXIMATION, EXACT_HESSIAN );
    //algorithm.set( PARETO_FRONT_HOTSTART, BT_FALSE        );
    //algorithm.set( DISCRETIZATION_TYPE, SINGLE_SHOOTING   );    

    // Minimize individual objective function
    algorithm.solveSingleObjective(0);

    // Minimize individual objective function
    algorithm.solveSingleObjective(1);

    // Generate Pareto set
    algorithm.solve();

    algorithm.getWeights("catatlyst_mixing_nbi_weights.txt");
    algorithm.getAllDifferentialStates("catalyst_mixing_nbi_states.txt");
    algorithm.getAllControls("catalyst_mixing_nbi_controls.txt");


    // GET THE RESULT FOR THE PARETO FRONT AND PLOT IT:
    // ------------------------------------------------
    VariablesGrid paretoFront;
    algorithm.getParetoFront( paretoFront );

    GnuplotWindow window1;
    window1.addSubplot( paretoFront, "Pareto Front", "Conversion","Catalyst", PM_POINTS );
    window1.plot( );


    // PRINT INFORMATION ABOUT THE ALGORITHM:
    // --------------------------------------
    algorithm.printInfo();


    // SAVE INFORMATION:
    // -----------------
    paretoFront.print( "catalyst_mixing_nbi_pareto.txt" );

    return 0;
}
/* <<< end tutorial code <<< */

