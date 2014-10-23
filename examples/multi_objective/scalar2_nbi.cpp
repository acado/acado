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
  *    \file   examples/multi_objective/scalar2_nbi.cpp
  *    \author Boris Houska, Filip Logist, Hans Joachim Ferreau
  *    \date   2009
  */


#include <acado_optimal_control.hpp>
#include <acado_gnuplot.hpp>


/* >>> start tutorial code >>> */
int main( ){

    USING_NAMESPACE_ACADO


    // INTRODUCE THE VARIABLES:
    // -------------------------
    Parameter y1,y2;


    // DEFINE AN OPTIMIZATION PROBLEM:
    // -------------------------------
    NLP nlp;
    nlp.minimize( 0, y1 );
    nlp.minimize( 1, y2 );

    nlp.subjectTo( 0.0 <= y1 <= 5.0 );
    nlp.subjectTo( 0.0 <= y2 <= 5.2 );
    nlp.subjectTo( 0.0 <= y2 - 5.0*exp(-y1) - 2.0*exp(-0.5*(y1-3.0)*(y1-3.0)) );


    // DEFINE A MULTI-OBJECTIVE ALGORITHM AND SOLVE THE NLP:
    // -----------------------------------------------------
    MultiObjectiveAlgorithm algorithm(nlp);

    algorithm.set( PARETO_FRONT_GENERATION, PFG_NORMAL_BOUNDARY_INTERSECTION );
    algorithm.set( PARETO_FRONT_DISCRETIZATION, 41 );
    algorithm.set( KKT_TOLERANCE, 1e-12 );

    // Minimize individual objective function  
    algorithm.initializeParameters("initial_scalar2_2.txt");
    algorithm.solveSingleObjective(1);

    // Minimize individual objective function
    algorithm.solveSingleObjective(0);

    // Generate Pareto set 
    algorithm.solve();


    // GET THE RESULT FOR THE PARETO FRONT AND PLOT IT:
    // ------------------------------------------------
    VariablesGrid paretoFront;
    algorithm.getParetoFront( paretoFront );
    algorithm.getWeights("scalar2_nbi_weights.txt");

    GnuplotWindow window1;
    window1.addSubplot( paretoFront, "Pareto Front y1 vs y2", "y1","y2", PM_POINTS );
    window1.plot( );

    paretoFront.print();


    // FILTER THE PARETO FRONT AND PLOT IT:
    // ------------------------------------
    algorithm.getParetoFrontWithFilter( paretoFront );
    algorithm.getWeightsWithFilter("scalar2_nbi_weights_filtered.txt");

    GnuplotWindow window2;
    window2.addSubplot( paretoFront, "Pareto Front (with filter) y1 vs y2", "y1","y2", PM_POINTS );
    window2.plot( );

    paretoFront.print();


    // PRINT INFORMATION ABOUT THE ALGORITHM:
    // --------------------------------------
    algorithm.printInfo();

    return 0;
}
/* <<< end tutorial code <<< */

