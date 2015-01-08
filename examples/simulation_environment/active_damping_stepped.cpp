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
 *    \file   examples/simulation_environment/active_damping.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date   2009
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


    // DEFINE LEAST SQUARE FUNCTION:
    // -----------------------------
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


    // DEFINE AN OPTIMAL CONTROL PROBLEM:
    // ----------------------------------
    const double t_start = 0.0;
    const double t_end   = 1.0;

    OCP ocp( t_start, t_end, 20 );

    ocp.minimizeLSQ( Q, h, r );

	ocp.subjectTo( f );

	ocp.subjectTo( -200.0 <= F <= 200.0 );
	ocp.subjectTo( R == 0.0 );



    // SETTING UP THE (SIMULATED) PROCESS:
    // -----------------------------------
	OutputFcn identity;
	DynamicSystem dynamicSystem( f,identity );

	Process process( dynamicSystem,INT_RK45 );

	VariablesGrid disturbance; disturbance.read( "road.txt" );
	if (process.setProcessDisturbance( disturbance ) != SUCCESSFUL_RETURN)
		return EXIT_FAILURE;

    // SETTING UP THE MPC CONTROLLER:
    // ------------------------------

	double samplingTime = 0.025;
	RealTimeAlgorithm alg( ocp,samplingTime );
	alg.set( INTEGRATOR_TYPE, INT_RK78 );
	//alg.set( MAX_NUM_ITERATIONS, 2 );
	alg.set( USE_IMMEDIATE_FEEDBACK,YES );

	StaticReferenceTrajectory zeroReference;

	Controller controller( alg,zeroReference );


	double startTime = 0.0;
	double endTime   = 2.5;
	
	DVector x0(4);
	x0.setZero();

	// 	hand-coding call to 
	//	sim.init( x0 )

	DVector uCon;
	VariablesGrid ySim;
	
	if (controller.init( startTime,x0 ) != SUCCESSFUL_RETURN)
		exit( EXIT_FAILURE );
	controller.getU( uCon );
	
	if (process.init( startTime,x0,uCon ) != SUCCESSFUL_RETURN)
		exit( EXIT_FAILURE );
	process.getY( ySim );


	// 	hand-coding call to 
	//	sim.run( )

	double currentTime = startTime;
	int nSteps = 0;

	while ( currentTime <= endTime )
	{
		printf( "\n*** Simulation Loop No. %d (starting at time %.3f) ***\n",nSteps,currentTime );

		double t = acadoGetTime();
		if (controller.feedbackStep( currentTime,ySim.getLastVector() ) != SUCCESSFUL_RETURN)
			exit( EXIT_FAILURE );
		printf( "t = %e\n", acadoGetTime()-t );
		controller.getU( uCon );
		if (controller.preparationStep( ) != SUCCESSFUL_RETURN)
			exit( EXIT_FAILURE );
		
		if (process.step( currentTime,currentTime+samplingTime,uCon ) != SUCCESSFUL_RETURN)
			exit( EXIT_FAILURE );
		process.getY( ySim );
		
		currentTime += samplingTime;
		++nSteps;
	}

//     // SETTING UP THE SIMULATION ENVIRONMENT,  RUN THE EXAMPLE...
//     // ----------------------------------------------------------
// 	SimulationEnvironment sim( 0.0,3.0,process,controller );
// 
// 	DVector x0(4);
// 	x0.setZero();
// 
// 	sim.init( x0 );
// 	sim.run( );
// 
// 
//     // ...AND PLOT THE RESULTS
//     // ----------------------------------------------------------
// 	VariablesGrid diffStates;
// 	sim.getProcessDifferentialStates( diffStates );
// 
// 	VariablesGrid feedbackControl;
// 	sim.getFeedbackControl( feedbackControl );
// 
// 	GnuplotWindow window;
// 	window.addSubplot( diffStates(0), "Body Position [m]" );
// 	window.addSubplot( diffStates(1), "Wheel Position [m]" );
// 	window.addSubplot( diffStates(2), "Body Velocity [m/s]" );
// 	window.addSubplot( diffStates(3), "Wheel Velocity [m/s]" );
// 	window.addSubplot( feedbackControl, "Damping Force [N]" );
// 	window.addSubplot( disturbance,     "Road Excitation [m]" );
// 	window.plot( );

    return EXIT_SUCCESS;
}



