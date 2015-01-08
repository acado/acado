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
  *    \file   examples/simulation_environment/dae_simulation.cpp
  *    \author Boris Houska, Hans Joachim Ferreau, David Ariens
  *    \date   2010
  */


#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>


int main( ){

    USING_NAMESPACE_ACADO

    // INTRODUCE THE VARIABLES:
    // -------------------------
    DifferentialState         x;
    DifferentialState         l;
    AlgebraicState            z;
    Control                   u;
    DifferentialEquation      f;
//     Disturbance R;


    // DEFINE A DIFFERENTIAL EQUATION:
    // -------------------------------
    f << dot(x) == -x + 0.5*x*x + u + 0.5*z  ;
    f << dot(l) ==  x*x + 3.0*u*u         ;
    f <<      0 ==  z + exp(z) - 1.0 + x     ;


    // DEFINE AN OPTIMAL CONTROL PROBLEM:
    // ----------------------------------
    OCP ocp( 0.0, 5.0, 10 );
    ocp.minimizeMayerTerm( l );

    ocp.subjectTo( f );
//     ocp.subjectTo( R == 0.0 );


    // SETTING UP THE (SIMULATED) PROCESS:
    // -----------------------------------
	OutputFcn identity;
	DynamicSystem dynamicSystem( f,identity );

	Process process( dynamicSystem,INT_BDF );

	//VariablesGrid disturbance = readFromFile( "dae_simulation_disturbance.txt" );
	//process.setProcessDisturbance( disturbance );


    // SETTING UP THE MPC CONTROLLER:
    // ------------------------------
	RealTimeAlgorithm alg( ocp,0.5 );

	StaticReferenceTrajectory zeroReference;
	Controller controller( alg,zeroReference );


    // SETTING UP THE SIMULATION ENVIRONMENT,  RUN THE EXAMPLE...
    // ----------------------------------------------------------
	SimulationEnvironment sim( 0.0,15.0,process,controller );

	DVector x0(2);
	x0(0) = 1;
	x0(1) = 0;

	sim.init( x0 );
	sim.run( );


    // ...AND PLOT THE RESULTS
    // ----------------------------------------------------------
	VariablesGrid diffStates;
	sim.getProcessDifferentialStates( diffStates );
	diffStates.printToFile( "diffStates.txt" );
	diffStates.printToFile( "diffStates.m","DIFFSTATES",PS_MATLAB );

	VariablesGrid sampledProcessOutput;
    sim.getSampledProcessOutput( sampledProcessOutput );
    sampledProcessOutput.printToFile( "sampledOut.txt" );
    sampledProcessOutput.printToFile( "sampledOut.m","OUT",PS_MATLAB );

    VariablesGrid feedbackControl;
    sim.getFeedbackControl( feedbackControl );
	feedbackControl.printToFile( "controls.txt" );
	feedbackControl.printToFile( "controls.m","CONTROL",PS_MATLAB );

	VariablesGrid algStates;
	sim.getProcessAlgebraicStates( algStates );
	algStates.printToFile( "algStates.txt" );
	algStates.printToFile( "algStates.m","ALGSTATES",PS_MATLAB );


    GnuplotWindow window;
		window.addSubplot( diffStates(0), "DIFFERENTIAL STATE: x" );
		window.addSubplot( diffStates(1), "DIFFERENTIAL STATE: l" );
		window.addSubplot( algStates(0),            "ALGEBRAIC STATE: z"    );
		window.addSubplot( feedbackControl(0),      "CONTRUL: u"            );
    window.plot( );


    return 0;
}



