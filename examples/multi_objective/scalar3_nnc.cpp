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
  *    \file   examples/multi_objective/scalar3_ws.cpp
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
    Parameter y1,y2,y3;


    // DEFINE AN OPTIMIZATION PROBLEM:
    // -------------------------------
    NLP nlp;
    nlp.minimize( 0, y1 );
    nlp.minimize( 1, y2 );
    nlp.minimize( 2, y3 );


    nlp.subjectTo( -5.0 <= y1 <= 5.0 );
    nlp.subjectTo( -5.0 <= y2 <= 5.0 );
    nlp.subjectTo( -5.0 <= y3 <= 5.0 );

    nlp.subjectTo( y1*y1+y2*y2+y3*y3 <= 4.0 );

    // DEFINE A MULTI-OBJECTIVE ALGORITHM AND SOLVE THE NLP:
    // -----------------------------------------------------
    MultiObjectiveAlgorithm algorithm(nlp);

    algorithm.set( PARETO_FRONT_GENERATION, PFG_NORMALIZED_NORMAL_CONSTRAINT );
    algorithm.set( PARETO_FRONT_DISCRETIZATION, 11 );

    // Minimize individual objective function
    algorithm.solveSingleObjective(0);

    // Minimize individual objective function
    algorithm.solveSingleObjective(1);

    // Minimize individual objective function
    algorithm.solveSingleObjective(2);

    // Generate Pareto set 
    algorithm.solve();

    algorithm.getWeights("scalar3_nnc_weights.txt");


    // GET THE RESULT FOR THE PARETO FRONT AND PLOT IT:
    // ------------------------------------------------
    VariablesGrid paretoFront;
    // algorithm.getParetoFrontWithFilter( paretoFront );
    algorithm.getParetoFront( paretoFront );

    //GnuplotWindow window;
    //window.addSubplot3D( paretoFront, "Pareto Front y1 vs y2 vs y3","y1","y2", PM_POINTS );
    //window.plot( );

    paretoFront.print();


    // PRINT INFORMATION ABOUT THE ALGORITHM:
    // --------------------------------------
    algorithm.printInfo();

    return 0;
}
/* <<< end tutorial code <<< */
