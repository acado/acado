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
 *    \file   examples/ocp/active_damping.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date   2009
 */


#include <acado_optimal_control.hpp>
#include <acado_gnuplot.hpp>



const double k10 =  1.287e12;
const double k20 =  1.287e12;
const double k30 =  9.043e09;
const double E1  =  -9758.3;
const double E2  =  -9758.3;
const double E3  =  -8560.0;
const double H1  =      4.2;
const double H2  =    -11.0;
const double H3  =    -41.85;
const double rho =      0.9342;
const double Cp  =      3.01;
const double kw  =   4032.0;
const double AR  =      0.215;
const double VR  =     10.0;
const double mK  =      5.0;
const double CPK =      2.0;

const double cA0    =    5.1;
const double theta0 =  104.9;

const double FFs    =    14.19;  /* Feed Flow normed by VR: (dV/dt  / VR)*/
const double QdotKs = -1113.50;

const double cAs     =  2.1402105301746182e00;
const double cBs     =  1.0903043613077321e00;
const double thetas  =  1.1419108442079495e02;
const double thetaKs =  1.1290659291045561e02;


const double TIMEUNITS_PER_HOUR = 3600.0;


const double P11 =   3278.78;   
const double P21 =   1677.31;
const double P31 =   681.02;
const double P41 =   271.50;

const double P12 =   1677.31;
const double P22 =   919.78;
const double P32 =   344.19;
const double P42 =   137.27;

const double P13 =   681.02;
const double P23 =   344.19;
const double P33 =   172.45;
const double P43 =   65.53;

const double P14 =   271.50;
const double P24 =   137.27;
const double P34 =   65.53;
const double P44 =   29.28;


const double R_OMEGA = 90.0;



int main( ){

    USING_NAMESPACE_ACADO


    // Define a Right-Hand-Side:
    // -------------------------

    DifferentialState cA, cB, theta, thetaK;
	Control u("", 2, 1);

    DifferentialEquation f;

    IntermediateState k1, k2, k3;

	k1 = k10*exp(E1/(273.15 +theta));
	k2 = k20*exp(E2/(273.15 +theta));
	k3 = k30*exp(E3/(273.15 +theta));

	f << dot(cA) == (1/TIMEUNITS_PER_HOUR)*(u(0)*(cA0-cA) - k1*cA - k3*cA*cA); 
	f << dot(cB) == (1/TIMEUNITS_PER_HOUR)* (- u(0)*cB + k1*cA - k2*cB); 
	f << dot(theta) == (1/TIMEUNITS_PER_HOUR)*(u(0)*(theta0-theta) - (1/(rho*Cp)) *(k1*cA*H1 + k2*cB*H2 + k3*cA*cA*H3)+(kw*AR/(rho*Cp*VR))*(thetaK -theta)); 
	f << dot(thetaK) == (1/TIMEUNITS_PER_HOUR)*((1/(mK*CPK))*(u(1) + kw*AR*(theta-thetaK)));



    // DEFINE LEAST SQUARE FUNCTION:
    // -----------------------------
    Function h;

    h << cA;
    h << cB;
	h << theta;
	h << thetaK;
	h << u(0);
	h << u(1);

    DMatrix S = eye<double>(6);
    DVector r = zeros<double>(6);

	S(0,0) = 0.2;
	S(1,1) = 1.0;
	S(2,2) = 0.5;
	S(3,3) = 0.2;

	S(4,4) = 0.5000;
	S(5,5) = 0.0000005;

	r(0) = 2.14;
	r(1) = 1.09;
	r(2) = 114.2;
	r(3) = 112.9;
	r(4) = 14.19;
	r(5) = -1113.5;
	

    // DEFINE AN OPTIMAL CONTROL PROBLEM:
    // ----------------------------------
//     const double t_start = 0.0   ;
//     const double t_end   = 1500;

	
 double times[11];
//     double times[10];
    int run1;
    for( run1 = 0; run1 < 10; run1++ )
        times[run1] = run1*80.0;

 times[10] = 1500.0;
 Grid grid( 11, times );
//     Grid grid( 10, times );


//     double times[23];
//     int run1;
//     for( run1 = 0; run1 < 22; run1++ )
//         times[run1] = run1*20.0;
// 
//     times[22] = 1500.0;
//     Grid grid( 23, times );


    OCP ocp( grid );
//     OCP ocp( t_start, t_end, 22 );
//        OCP ocp( t_start, t_end, 75 );

	ocp.minimizeLSQ( S, h, r );

	ocp.subjectTo( f );

	ocp.subjectTo( AT_START, cA     == 1.0 );
	ocp.subjectTo( AT_START, cB     == 0.5 );
	ocp.subjectTo( AT_START, theta  == 100.0 );
	ocp.subjectTo( AT_START, thetaK == 100.0 );

	ocp.subjectTo( 3.0     <= u(0) <= 35.0 );
	ocp.subjectTo( -9000.0 <= u(1) <= 0.0 );

	
	VariablesGrid cstr75states;
	VariablesGrid cstr75controls;

	cstr75states.read( "cstr75_states.txt" );
	cstr75controls.read( "cstr75_controls.txt" );

    // Additionally, flush a plotting object
    GnuplotWindow window1;
      window1.addSubplot( cA,     "cA [mol/l]","","",PM_LINES,0,1500 );
	  window1.addData( 0,cstr75states(0) );
	  
    GnuplotWindow window2;
	  window2.addSubplot( cB,     "cB [mol/l]","","",PM_LINES,0,1500 );
	  window2.addData( 0,cstr75states(1) );
	  
    GnuplotWindow window3;
	  window3.addSubplot( theta,  "theta [C]","","",PM_LINES,0,1500 );
	  window3.addData( 0,cstr75states(2) );
	  
	GnuplotWindow window4;
      window4.addSubplot( thetaK, "thetaK [C]","","",PM_LINES,0,1500 );
	  window4.addData( 0,cstr75states(3) );
	  
    GnuplotWindow window5;
	  window5.addSubplot( u(0), "u1","","",PM_LINES,0,1500 );
	  window5.addData( 0,cstr75controls(0) );
	  
    GnuplotWindow window6;
	  window6.addSubplot( u(1), "u2","","",PM_LINES,0,1500 );
	  window6.addData( 0,cstr75controls(1) );


    GnuplotWindow window;
      window.addSubplot( cA,     "cA [mol/l]","","",PM_LINES,0,1500 );
	  window.addSubplot( cB,     "cB [mol/l]","","",PM_LINES,0,1500 );
	  window.addSubplot( theta,  "theta [C]","","",PM_LINES,0,1500 );
      window.addSubplot( thetaK, "thetaK [C]","","",PM_LINES,0,1500 );
	  window.addSubplot( u(0), "u1","","",PM_LINES,0,1500 );
	  window.addSubplot( u(1), "u2","","",PM_LINES,0,1500 );

	window.addData( 0,cstr75states(0) );
	window.addData( 1,cstr75states(1) );
	window.addData( 2,cstr75states(2) );
	window.addData( 3,cstr75states(3) );
	window.addData( 4,cstr75controls(0) );
	window.addData( 5,cstr75controls(1) );


    // DEFINE AN OPTIMIZATION ALGORITHM AND SOLVE THE OCP:
    // ---------------------------------------------------
    OptimizationAlgorithm algorithm(ocp);

//     algorithm << window;

//  algorithm.set( HESSIAN_APPROXIMATION, EXACT_HESSIAN );
	algorithm.set( HESSIAN_APPROXIMATION, GAUSS_NEWTON );

	algorithm.set( INTEGRATOR_TOLERANCE, 1e-6 );
	algorithm.set( KKT_TOLERANCE, 1e-4 );
// 	algorithm.set( GLOBALIZATION_STRATEGY, GS_FULLSTEP );
// 	algorithm.set( MAX_NUM_ITERATIONS, 1 );
    algorithm.set( PRINT_SCP_METHOD_PROFILE, YES );
	
	VariablesGrid uStart( 2,0.0,2000.0,2 );
	uStart( 0,0 ) = 14.19;
	uStart( 0,1 ) = -1113.5;
	uStart( 1,0 ) = 14.19;
	uStart( 1,1 ) = -1113.5;

	algorithm.initializeControls( uStart );

	algorithm << window1;
	algorithm << window2;
	algorithm << window3;
	algorithm << window4;
	algorithm << window5;
	algorithm << window6;

	algorithm << window;
	algorithm.solve();
// 	algorithm.solve();

// 	algorithm.getDifferentialStates( "cstr75_states.txt" );
// 	algorithm.getControls( "cstr75_controls.txt" );
// 	algorithm.getDifferentialStates( "cstr10_states.txt" );
// 	algorithm.getControls( "cstr10_controls.txt" );

    return 0;
}



