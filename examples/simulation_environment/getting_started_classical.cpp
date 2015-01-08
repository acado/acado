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
 *    \file   examples/simulation_environment/getting_started_classical.cpp
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


    // SETTING UP THE MPC CONTROLLER:
    // ------------------------------
	PIDcontroller pid( 4,1,0.01 );

	DVector pWeights( 4 );
	pWeights(0) = 1000.0;
	pWeights(1) = -1000.0;
	pWeights(2) = 1000.0;
	pWeights(3) = -1000.0;

	DVector dWeights( 4 );
	dWeights(0) = 0.0;
	dWeights(1) = 0.0;
	dWeights(2) = 20.0;
	dWeights(3) = -20.0;

	pid.setProportionalWeights( pWeights );
	pid.setDerivativeWeights( dWeights );

	pid.setControlLowerLimit( 0,-200.0 );
	pid.setControlUpperLimit( 0, 200.0 );


// 	DMatrix K( 1,4 );
// 	K(0,0) = -3.349222044080232e+04;
// 	K(0,1) = -3.806600292165519e+03;
// 	K(0,2) =  9.999999999999985e+02;
// 	K(0,3) = -1.040810121403324e+03;
// 
// 	LinearStateFeedback lqr( K,0.025 );
// 
// 	lqr.setControlLowerLimit( 0,-200.0 );
// 	lqr.setControlUpperLimit( 0, 200.0 );


	StaticReferenceTrajectory zeroReference;

	Controller controller( pid,zeroReference );
// 	Controller controller( lqr,zeroReference );


    // SETTING UP THE SIMULATION ENVIRONMENT,  RUN THE EXAMPLE...
    // ----------------------------------------------------------
	SimulationEnvironment sim( 0.0,2.5,process,controller );

	DVector x0(4);
	x0.setZero();

	if (sim.init( x0 ) != SUCCESSFUL_RETURN)
		exit( EXIT_FAILURE );
	if (sim.run( ) != SUCCESSFUL_RETURN)
		exit( EXIT_FAILURE );

    // ...AND PLOT THE RESULTS
    // ----------------------------------------------------------
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



