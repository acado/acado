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
  *    \file   examples/multi_objective/crane_nbi.cpp
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
    DifferentialState     x  , dx  ;   // the position of the mounting point and its velocity
    DifferentialState     L  , dL  ;   // the length of the cable and its velocity
    DifferentialState     phi, dphi;   // the angle phi and its velocity
    DifferentialState     P0,P1,P2 ;   // the variance-covariance states
    Control               ddx, ddL ;   // the accelarations
    Parameter                    T ;   // duration of the maneuver
    Parameter                gamma ;   // the confidence level

    double const           g = 9.81;   // the gravitational constant
    double const           m = 10.0;   // the mass at the end of the crane
    double const           b = 0.1 ;   // a frictional constant

    DifferentialEquation   f(0.0,T);

    const double F2 = 50.0;


    // DEFINE A DIFFERENTIAL EQUATION:
    // -------------------------------
    f << dot(x   )  == dx   + 0.000001*gamma; // small regularization term
    f << dot(dx  )  == ddx ;
    f << dot(L   )  == dL  ;
    f << dot(dL  )  == ddL ;
    f << dot(phi )  == dphi;
    f << dot(dphi)  == -(g/L)*phi - ( b + 2.0*dL/L )*dphi - ddx/L;

    f << dot(P0)    == 2.0*P1;
    f << dot(P1)    == -(g/L)*P0 - ( b + 2.0*dL/L )*P1 + P2;
    f << dot(P2)    == -2.0*(g/L)*P1 - 2.0*( b + 2.0*dL/L )*P2 + F2/(m*m*L*L);


    // DEFINE AN OPTIMAL CONTROL PROBLEM:
    // ----------------------------------
    OCP ocp(  0.0, T, 20 );
    ocp.minimizeMayerTerm( 0, T       );
    ocp.minimizeMayerTerm( 1, -gamma  );

    ocp.subjectTo( f );

    ocp.subjectTo( AT_START, x    ==    0.0 );
    ocp.subjectTo( AT_START, dx   ==    0.0 );
    ocp.subjectTo( AT_START, L    ==   70.0 );
    ocp.subjectTo( AT_START, dL   ==    0.0 );
    ocp.subjectTo( AT_START, phi  ==    0.0 );
    ocp.subjectTo( AT_START, dphi ==    0.0 );

    ocp.subjectTo( AT_START, P0   ==    0.0 );
    ocp.subjectTo( AT_START, P1   ==    0.0 );
    ocp.subjectTo( AT_START, P2   ==    0.0 );
    ocp.subjectTo( AT_END  , x    ==   10.0 );
    ocp.subjectTo( AT_END  , dx   ==    0.0 );
    ocp.subjectTo( AT_END  , L    ==   70.0 );
    ocp.subjectTo( AT_END  , dL   ==    0.0 );

    ocp.subjectTo( AT_END  , -0.075 <= phi - gamma*sqrt(P0)           );
    ocp.subjectTo( AT_END  ,           phi + gamma*sqrt(P0) <= 0.075  );

    ocp.subjectTo( gamma >= 0.0 );

    ocp.subjectTo( 5.0 <=  T  <=  17.0 );

    ocp.subjectTo( -0.3 <= ddx <= 0.3 );
    ocp.subjectTo( -1.0 <= ddL <= 1.0 );

    ocp.subjectTo( -10.0 <= x   <=  50.0 );
    ocp.subjectTo( -20.0 <= dx  <=  20.0 );
    ocp.subjectTo(  30.0 <= L   <=  75.0 );
    ocp.subjectTo( -20.0 <= dL  <=  20.0 );

    // DEFINE A MULTI-OBJECTIVE ALGORITHM AND SOLVE THE OCP:
    // -----------------------------------------------------
    MultiObjectiveAlgorithm algorithm(ocp);

    algorithm.set( PARETO_FRONT_DISCRETIZATION, 31 );
    algorithm.set( PARETO_FRONT_GENERATION, PFG_NORMAL_BOUNDARY_INTERSECTION );
    //algorithm.set( DISCRETIZATION_TYPE        , SINGLE_SHOOTING                  );

    // Minimize individual objective function
    algorithm.solveSingleObjective(0);

    // Minimize individual objective function
    algorithm.solveSingleObjective(1);

    // Generate Pareto set
    //algorithm.set( PARETO_FRONT_HOTSTART      , BT_FALSE         		   );
    algorithm.solve();

    algorithm.getWeights("crane_nbi_weights.txt");
    algorithm.getAllDifferentialStates("crane_nbi_states.txt");
    algorithm.getAllControls("crane_nbi_controls.txt");
    algorithm.getAllParameters("crane_nbi_parameters.txt");


    // GET THE RESULT FOR THE PARETO FRONT AND PLOT IT:
    // ------------------------------------------------
    VariablesGrid paretoFront;
    algorithm.getParetoFront( paretoFront );

    GnuplotWindow window1;
    window1.addSubplot( paretoFront, "Pareto Front (robustness versus time)", "TIME","ROBUSTNESS", PM_POINTS );
    window1.plot( );


    // PRINT INFORMATION ABOUT THE ALGORITHM:
    // --------------------------------------
    algorithm.printInfo();


    // SAVE INFORMATION:
    // -----------------
    paretoFront.print( "crane_nbi_pareto.txt" );

    return 0;
}
/* <<< end tutorial code <<< */

