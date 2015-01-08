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
  *    \file   examples/ocp/getting_started.cpp
  *    \author Boris Houska, Hans Joachim Ferreau
  *    \date   2009
  */

//#include <acado_optimal_control.hpp>
#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>

USING_NAMESPACE_ACADO

int main( )
{
    // INTRODUCE THE VARIABLES:
    // -------------------------

    DifferentialState     h1,h2,h3,h4;
    Control                   u1,u2;
    // Parameter 			T;
    // Disturbance               w;
    // Parameter               p,q;
    const double t_start = 0.0;
    const double samplingTime=30.0;
    double A1=380.1327, A2=380.1327, A3=380.1327, A4=380.1327, a1=1.2272, a2=1.2272, a3=1.2272, a4=1.2272,g=981.0,gamma1=0.45, gamma2=0.4;
    double x_ss1=20.0, x_ss2=15.0, u_max1=400.0, u_max2=400.0, x_max1=40.0, x_max2=40.0, x_max3=40.0, x_max4=40.0;
    DifferentialEquation      f;
    int T=10;
    // 
     const double t_end   = t_start+T*samplingTime;
   
    // DEFINE A DIFFERENTIAL EQUATION:
    // -------------------------------

    // f << -dot(x) -x*x + p + u*u + w;
    f << dot(h1) == (1.0/A1)*(gamma1*u1+a3*sqrt(2.0*g*h3)-a1*sqrt(2.0*g*h1));
    f << dot(h2)== (1.0/A2)*(gamma2*u2+a4*sqrt(2.0*g*h4)-a2*sqrt(2.0*g*h2));
    f << dot(h3)==(1.0/A3)*((1.0-gamma2)*u2-a3*sqrt(2.0*g*h3));
    f << dot(h4)==(1.0/A4)*((1.0-gamma1)*u1-a4*sqrt(2.0*g*h4));


    // DEFINE THE LEAST SQUARE FUNCTION

    Function h;
    h << h1;
    h << h2;
    DMatrix Q(2,2);
    Q.setIdentity();

    DVector r(2);
    r(0)=x_ss1;
    r(1)=x_ss2;

    // DEFINE AN OPTIMAL CONTROL PROBLEM:
    // ----------------------------------
    
    OCP ocp(t_start,t_end,T);
    ocp.minimizeLSQ(Q,h,r);

    ocp.subjectTo(f);
    //ocp.subjectTo( AT_START, x1 == 0.0 );
    //ocp.subjectTo( AT_START, x2 == 0.0 );
    //ocp.subjectTo( AT_START, x3 == 0.0 );
    //ocp.subjectTo( AT_START, x4 == 0.0 );
    //ocp.subjectTo( AT_END, x1 == 0.0 );
    //ocp.subjectTo( AT_END, x2 == 0.0 );
    //ocp.subjectTo( AT_END, x3 == 10.0 );
    //ocp.subjectTo( AT_END, x4 == 0.0 );
    ocp.subjectTo(  0.0 <= u1 <= u_max1 );
    ocp.subjectTo(  0.0 <= u2 <= u_max2 );
    ocp.subjectTo(  0.0 <= h1 <= x_max1 );
    ocp.subjectTo(  0.0 <= h2 <= x_max2 );
    ocp.subjectTo(  0.0 <= h3 <= x_max3 );
    ocp.subjectTo(  0.0 <= h4 <= x_max4 );

    // SETTING UP THE PROCESS

    OutputFcn identity;
    DynamicSystem dynamicSystem(f,identity);

    GaussianNoise noise( 4,0.0,0.1 );

    Sensor sensor( 4 );
    if (sensor.setOutputNoise( noise,samplingTime ) != SUCCESSFUL_RETURN)
    	exit( EXIT_FAILURE );

    Process process(dynamicSystem,INT_RK45);
    //process.setSensor( sensor );

    // SETTING UP THE MPC CONTROLLER

    RealTimeAlgorithm algorithm(ocp,samplingTime);
//     algorithm.set( USE_REALTIME_ITERATIONS, YES );
    algorithm.set( MAX_NUM_ITERATIONS, 2 );
	algorithm.set(LEVENBERG_MARQUARDT, 1e-5);

    StaticReferenceTrajectory zeroReference;

    Controller controller(algorithm,zeroReference);

    // DEFINE AN OPTIMIZATION ALGORITHM AND SOLVE THE OCP:
    // ---------------------------------------------------
    //OptimizationAlgorithm algorithm(ocp);

    double simStartTime = 0.0;
    double simEndTime = 1200.0;
    SimulationEnvironment sim(simStartTime,simEndTime,process,controller);

    DVector x0(4);
    x0.setZero( );
    x0(0)=0.1;
    x0(1)=0.1;
    x0(2)=0.1;
    x0(3)=0.1;
    if (sim.init( x0 ) != SUCCESSFUL_RETURN)
    	exit( EXIT_FAILURE );
    if (sim.run( ) != SUCCESSFUL_RETURN)
    	exit( EXIT_FAILURE );
   
    VariablesGrid diffStates;
    sim.getProcessDifferentialStates(diffStates);
    VariablesGrid feedbackControl;
    sim.getFeedbackControl (feedbackControl);

    GnuplotWindow window;
	window.addSubplot( diffStates(0), "h1");
	window.addSubplot( diffStates(1), "h2");
	window.addSubplot( diffStates(2), "h3");
	window.addSubplot( diffStates(3), "h4");
	window.addSubplot( feedbackControl(0), "F1");
	window.addSubplot( feedbackControl(1), "F2");
    window.plot();
    
	diffStates.print("result.txt","diffstates",PS_MATLAB);

    //algorithm.set( HESSIAN_APPROXIMATION, EXACT_HESSIAN );
    
    //algorithm << window;

    //double t1 = acadoGetTime();
    //algorithm.solve();
    //printf(" %.16e \n ", acadoGetTime() - t1  );

    return EXIT_SUCCESS;
}
