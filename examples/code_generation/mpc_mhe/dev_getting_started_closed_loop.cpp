/*
 *    This file is part of ACADO Toolkit.
 *
 *    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
 *    Copyright (C) 2008-2013 by Boris Houska, Hans Joachim Ferreau,
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
 *    \file   cstr_mpc_simulation.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 */


#include <acado_toolkit.hpp>
#include <include/acado_gnuplot/gnuplot_window.hpp>


extern "C"
{
#include "./getting_started_export/acado.h"
#include "./getting_started_export/auxiliary_functions.c"
} // extern "C"


ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

Vars         vars;
Params       params;

#ifdef USE_CVXGEN
Workspace    work;
Settings     settings;
#endif


int main( )
{
	USING_NAMESPACE_ACADO

	// DEFINE THE VARIABLES:
	// ----------------------------------------------------------
	DifferentialState   p    ;  // the trolley position
	DifferentialState   v    ;  // the trolley velocity 
	DifferentialState   phi  ;  // the excitation angle
	DifferentialState   omega;  // the angular velocity
	Control             a    ;  // the acc. of the trolley

	const double     g = 9.81;  // the gravitational constant 
	const double     b = 0.20;  // the friction coefficient
	// ----------------------------------------------------------


	// DEFINE THE MODEL EQUATIONS:
	// ----------------------------------------------------------
	DifferentialEquation f; 

	f << dot( p     )  ==  v                                ;
	f << dot( v     )  ==  a                                ;
	f << dot( phi   )  ==  omega                            ;
	f << dot( omega )  == -g*sin(phi) - a*cos(phi) - b*omega;
	// ----------------------------------------------------------


    // SETTING UP THE (SIMULATED) PROCESS:
    // -----------------------------------
	OutputFcn identity;
	DynamicSystem dynamicSystem( f,identity );
	Process process( dynamicSystem,INT_RK45 );


    // SETTING UP THE MPC CONTROLLER:
    // ------------------------------
	ExportedRTIscheme rtiScheme(
			4, // number of states
			1, // number of controls
			10, // number of horizon intervals
			0.3, // sampling time

			/* Function handlers: */
			preparationStep,
			feedbackStep,
			shiftControls,
			shiftStates,
			getAcadoVariablesX,
			getAcadoVariablesU,
			getAcadoVariablesXRef,
			getAcadoVariablesURef );

	#ifdef USE_CVXGEN
	set_defaults( );
	#endif

	Vector xuRef(5);
	xuRef.setZero( );

	VariablesGrid reference;
	reference.addVector( xuRef,  0.0 );
	reference.addVector( xuRef, 10.0 );

	StaticReferenceTrajectory referenceTrajectory( reference );

	Controller controller( rtiScheme,referenceTrajectory );
	controller.set( USE_REFERENCE_PREDICTION,NO );


    // SETTING UP THE SIMULATION ENVIRONMENT,  RUN THE EXAMPLE...
    // ----------------------------------------------------------
	SimulationEnvironment sim( 0.0,10.0, process,controller );

	Vector x0(4);
	x0(0) = 1.0;
	x0(1) = 0.0;
	x0(2) = 0.0;
	x0(3) = 0.0;

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
        window.addSubplot( diffStates(0), "p" );
        window.addSubplot( diffStates(1), "v" );
        window.addSubplot( diffStates(2), "phi" );
        window.addSubplot( diffStates(3), "omega" );
		window.addSubplot( feedbackControl(0), "a" );
	window.plot( );

    return EXIT_SUCCESS;
}



