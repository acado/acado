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
  *    \file   examples/multi_objective/plug_flow_reactor_ws.cpp
  *    \author Filip Logist, Boris Houska, Hans Joachim Ferreau
  *    \date   2009
  *
  *    \  Weighted Sum for jacketed plug flow reactor with conflicting conversion and energy cost
  *    \  J = (1-W)*J1+W*J2		Weighted Sum
  *    \  J1 = Cin*(1.0-x1)		Conversion cost
  *    \  J2 = pow((Tin*x2),2.0)	Energy cost
  *
  *	Reference: 
  * 	F. Logist, P.M. Van Erdeghem, and J.F. Van Impe 2009. 
  *	Efficient deterministic multiple objective optimal control of (bio)chemical processes. 
  *	Chemical Engineering Science, 64, 2527-2538.
  *
  */


#include "acado_optimal_control.hpp"
#include <acado_gnuplot.hpp>


int main( ){

    USING_NAMESPACE_ACADO

    // INTRODUCE FIXED PARAMETERS:
    // ---------------------------
    #define  v		0.1
    #define  L		1.0
    #define  Beta	0.2
    #define  Delta	0.25
    #define  E		11250.0
    #define  k0		1E+06
    #define  R		1.986
    #define  K1		250000.0
    #define  Cin	0.02
    #define  Tin	340.0


    // INTRODUCE THE VARIABLES:
    // -------------------------
    DifferentialState     x1,x2;
    Control               u    ;
    DifferentialEquation  f( 0.0, L );


    // DEFINE A DIFFERENTIAL EQUATION:
    // -------------------------------
    double Alpha, Gamma;
    Alpha = k0*exp(-E/(R*Tin));
    Gamma = E/(R*Tin);

    f << dot(x1) ==  Alpha       /v * (1.0-x1) * exp((Gamma*x2)/(1.0+x2));
    f << dot(x2) == (Alpha*Delta)/v * (1.0-x1) * exp((Gamma*x2)/(1.0+x2)) + Beta/v * (u-x2);


    // DEFINE AN OPTIMAL CONTROL PROBLEM:
    // ----------------------------------
    OCP ocp( 0.0, L, 50 );
    ocp.minimizeMayerTerm( 0, Cin*(1.0-x1)            ); // Solve conversion optimal problem
    ocp.minimizeMayerTerm( 1, (pow((Tin*x2),2.0)/K1) + 0.005*Cin*(1.0-x1) ); // Solve energy optimal problem (perturbed by small conversion cost; 
									     // otherwise the problem is ill-defined.)

    ocp.subjectTo( f );

    ocp.subjectTo( AT_START, x1 ==  0.0 );
    ocp.subjectTo( AT_START, x2 ==  0.0 );

    ocp.subjectTo(  0.0            <= x1 <=  1.0             );
    ocp.subjectTo( (280.0-Tin)/Tin <= x2 <= (400.0-Tin)/Tin  );
    ocp.subjectTo( (280.0-Tin)/Tin <= u  <= (400.0-Tin)/Tin  );


    // DEFINE A MULTI-OBJECTIVE ALGORITHM AND SOLVE THE OCP:
    // -----------------------------------------------------
    MultiObjectiveAlgorithm algorithm(ocp);

    algorithm.set( INTEGRATOR_TYPE, INT_BDF );
    algorithm.set( KKT_TOLERANCE, 1e-8 );

    algorithm.set( PARETO_FRONT_GENERATION    , PFG_WEIGHTED_SUM );
    algorithm.set( PARETO_FRONT_DISCRETIZATION, 11               );

    // Generate Pareto set 
    algorithm.solve();

    algorithm.getWeights("plug_flow_reactor_ws_weights.txt");
    algorithm.getAllDifferentialStates("plug_flow_reactor_ws_states.txt");
    algorithm.getAllControls("plug_flow_reactor_ws_controls.txt");


    // VISUALIZE THE RESULTS IN A GNUPLOT WINDOW:
    // ------------------------------------------
    VariablesGrid paretoFront;
    algorithm.getParetoFront( paretoFront );

    GnuplotWindow window1;
    window1.addSubplot( paretoFront, "Pareto Front (conversion versus energy)", "OUTLET CONCENTRATION", "ENERGY",PM_POINTS );
    window1.plot( );


    // PRINT INFORMATION ABOUT THE ALGORITHM:
    // --------------------------------------
    algorithm.printInfo();


    // SAVE INFORMATION:
    // -----------------
    paretoFront.print();

    return 0;
}



