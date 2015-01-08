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
 *    \file   examples/simulation_environment/getting_started.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 */


#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>


int main( )
{
    USING_NAMESPACE_ACADO


    // INTRODUCE THE VARIABLES:
    // -------------------------
	DifferentialState xB; //Body Position
	DifferentialState xW; //Wheel Position
	DifferentialState vB; //Body Velocity
	DifferentialState vW; //Wheel Velocity

	Disturbance R;
	Control F;

	double mB = 350.0;
	double mW = 50.0;
	double kS = 20000.0;
	double kT = 200000.0;


    // DEFINE A DIFFERENTIAL EQUATION:
    // -------------------------------
    DifferentialEquation f;

	f << dot(xB) == vB;
	f << dot(xW) == vW;
	f << dot(vB) == ( -kS*xB + kS*xW + F ) / mB;
	f << dot(vW) == (  kS*xB - (kT+kS)*xW + kT*R - F ) / mW;


    // SETTING UP THE (SIMULATED) PROCESS:
    // -----------------------------------
	OutputFcn identity;
	DynamicSystem dynamicSystem( f,identity );

	Process process( dynamicSystem,INT_RK45 );

	VariablesGrid disturbance; disturbance.read( "road.txt" );
	if (process.setProcessDisturbance( disturbance ) != SUCCESSFUL_RETURN)
		exit( EXIT_FAILURE );

    // DEFINE AN OPTIMAL CONTROL PROBLEM:
    // ----------------------------------
    Function h;

    h << xB;
    h << xW;
	h << vB;
    h << vW;
	h << F;

    DMatrix Q = zeros<double>(5,5); // LSQ coefficient matrix
	Q(0,0) = 10.0;
	Q(1,1) = 10.0;
	Q(2,2) = 1.0;
	Q(3,3) = 1.0;
	Q(4,4) = 1.0e-8;

    DVector r(5); // Reference
    r.setAll( 0.0 );


    const double tStart = 0.0;
    const double tEnd   = 1.0;

    OCP ocp( tStart, tEnd, 20 );

    ocp.minimizeLSQ( Q, h, r );

	ocp.subjectTo( f );

	ocp.subjectTo( -200.0 <= F <= 200.0 );
	ocp.subjectTo( R == 0.0 );


    // SETTING UP THE MPC CONTROLLER:
    // ------------------------------
	RealTimeAlgorithm alg( ocp,0.05 );
	alg.set( INTEGRATOR_TYPE, INT_RK78 );
	alg.set( DYNAMIC_SENSITIVITY,FORWARD_SENSITIVITY );
// 	alg.set( MAX_NUM_ITERATIONS, 2 );
//  	alg.set( USE_IMMEDIATE_FEEDBACK,YES );

	StaticReferenceTrajectory zeroReference;

	Controller controller( alg,zeroReference );


    // SETTING UP THE SIMULATION ENVIRONMENT,  RUN THE EXAMPLE...
    // ----------------------------------------------------------
	SimulationEnvironment sim( 0.0,2.5,process,controller );

	DVector x0(4);
	x0.setZero();

	if (sim.init( x0 ) != SUCCESSFUL_RETURN)
		exit( EXIT_FAILURE );
	if (sim.run( ) != SUCCESSFUL_RETURN)
		exit( EXIT_FAILURE );


    // ... AND PLOT THE RESULTS
    // ------------------------
 	VariablesGrid diffStates;
 	sim.getProcessDifferentialStates( diffStates );

 	VariablesGrid feedbackControl;
 	sim.getFeedbackControl( feedbackControl );

 	GnuplotWindow window;
 	window.addSubplot( diffStates(0),   "Body Position [m]" );
 	window.addSubplot( diffStates(1),   "Wheel Position [m]" );
 	window.addSubplot( diffStates(2),   "Body Velocity [m/s]" );
 	window.addSubplot( diffStates(3),   "Wheel Velocity [m/s]" );
 	window.addSubplot( feedbackControl, "Damping Force [N]" );
 	window.addSubplot( disturbance,     "Road Excitation [m]" );
 	window.plot( );

    return EXIT_SUCCESS;
}



