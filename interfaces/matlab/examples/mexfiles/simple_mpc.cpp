/**
 *    This file is part of ACADO Toolkit.
 *
 *    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
 *    Copyright (C) 2008-2009 by Boris Houska and Hans Joachim Ferreau, K.U.Leuven.
 *    Developed within the Optimization in Engineering Center (OPTEC) under
 *    supervision of Moritz Diehl. All rights reserved.
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
 *    Author: David Ariens  --  http://www.acadotoolkit.org/matlab 
 *    Date: 2010
 *    
 *    SIMPLE MPC EXAMPLE
 *
 *    Compilation:
 *     - Go to the folder <ACADOtoolkit-inst-dir>/interfaces/matlab/
 *     - Run: makemex('examples/mexfiles/simple_mpc.cpp', 'simple_mpc', 'examples/mexfiles/');
 *     - Run: cd ('examples/mexfiles/');
 *     - Run: simple_mpc();
 */
 
#include <acado_toolkit.hpp>                    // Include the ACADO toolkit
#include <acado/utils/matlab_acado_utils.hpp>   // Include specific Matlab utils

USING_NAMESPACE_ACADO                           // Open the namespace

void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] )  // Start the MEX function. Do NOT change the header of this function.
 { 
    clearAllStaticCounters( );                  // Clear software counters
 
    
    
    // INTRODUCE THE VARIABLES:
    // -------------------------
	DifferentialState xB;
	DifferentialState xW;
	DifferentialState vB;
	DifferentialState vW;

	Control R;
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
	f << dot(vW) == ( -kT*xB - (kT+kS)*xW + kT*R - F ) / mW;


    // DEFINE LEAST SQUARE FUNCTION:
    // -----------------------------
    Function h;

    h << xB;
    h << xW;
	h << vB;
    h << vW;

    Matrix Q(4,4);
    Q.setIdentity();
	Q(0,0) = 10.0;
	Q(1,1) = 10.0;

    Vector r(4);
    r.setAll( 0.0 );


    // DEFINE AN OPTIMAL CONTROL PROBLEM:
    // ----------------------------------
    const double t_start = 0.0;
    const double t_end   = 1.0;

    OCP ocp( t_start, t_end, 20 );

    ocp.minimizeLSQ( Q, h, r );

	ocp.subjectTo( f );

	ocp.subjectTo( -500.0 <= F <= 500.0 );
	ocp.subjectTo( R == 0.0 );



    // SETTING UP THE (SIMULATED) PROCESS:
    // -----------------------------------
	OutputFcn identity;
	DynamicSystem dynamicSystem( f,identity );

	Process process( dynamicSystem,INT_RK45 );

	VariablesGrid disturbance = readFromFile( "simple_mpc_road.txt" );
	process.setProcessDisturbance( disturbance );


    // SETTING UP THE MPC CONTROLLER:
    // ------------------------------
	RealTimeAlgorithm alg( ocp,0.05 );
	alg.set( MAX_NUM_ITERATIONS, 2 );
	
	StaticReferenceTrajectory zeroReference;

	Controller controller( alg,zeroReference );


    // SETTING UP THE SIMULATION ENVIRONMENT,  RUN THE EXAMPLE...
    // ----------------------------------------------------------
	SimulationEnvironment sim( 0.0,3.0,process,controller );

	Vector x0(4);
	x0(0) = 0.01;
	x0(1) = 0.0;
	x0(2) = 0.0;
	x0(3) = 0.0;

	sim.init( x0 );
	sim.run( );


    // ...AND PLOT THE RESULTS
    // ----------------------------------------------------------
	VariablesGrid sampledProcessOutput;
	sim.getSampledProcessOutput( sampledProcessOutput );
    acadoPlot(sampledProcessOutput);
    
	VariablesGrid feedbackControl;
	sim.getFeedbackControl( feedbackControl );
    acadoPlot(feedbackControl);
    

          
    clearAllStaticCounters( );                  // Clear software counters
} 

