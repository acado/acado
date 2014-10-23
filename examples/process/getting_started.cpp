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
 *    \file examples/process/getting_started.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date 25.08.2008
 *
 *     Simple example for getting started with using the Process of the ACADO Toolkit.
 */


#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>


int main( )
{
	USING_NAMESPACE_ACADO


    // INTRODUCE THE VARIABLES:
    // -------------------------
	DifferentialState xB;
	DifferentialState xW;
	DifferentialState vB;
	DifferentialState vW;

	Disturbance R;
	Control F;

	Parameter mB;
	double mW = 50.0;
	double kS = 20000.0;
	double kT = 200000.0;


    // DEFINE THE DYNAMIC SYSTEM:
    // --------------------------
    DifferentialEquation f;

	f << dot(xB) == vB;
	f << dot(xW) == vW;
	f << dot(vB) == ( -kS*xB + kS*xW + F ) / mB;
	f << dot(vW) == (  kS*xB - (kT+kS)*xW + kT*R - F ) / mW;

	OutputFcn g;
	g << xB;
	g << 500.0*vB + F;

    DynamicSystem dynSys( f,g );


    // SETUP THE PROCESS:
    // ------------------
	Process myProcess;
	
	myProcess.setDynamicSystem( dynSys,INT_RK45 );
	myProcess.set( ABSOLUTE_TOLERANCE,1.0e-8 );
	
	DVector x0( 4 );
	x0.setZero( );
	x0( 0 ) = 0.01;

	myProcess.initializeStartValues( x0 );
	myProcess.setProcessDisturbance( "road.txt" );

	myProcess.set( PLOT_RESOLUTION,HIGH );

	GnuplotWindow window;
	  window.addSubplot( xB, "Body Position [m]" );
	  window.addSubplot( xW, "Wheel Position [m]" );
	  window.addSubplot( vB, "Body Velocity [m/s]" );
	  window.addSubplot( vW, "Wheel Velocity [m/s]" );

	  window.addSubplot( F,"Damping Force [N]" );
	  window.addSubplot( mB,"Body Mass [kg]" );
	  window.addSubplot( R, "Road Disturbance" );
	  window.addSubplot( g(0),"Output 1" );
	  window.addSubplot( g(1),"Output 2" );

	myProcess << window;


    // SIMULATE AND GET THE RESULTS:
    // -----------------------------
	VariablesGrid u( 1,0.0,1.0,6 );

	u( 0,0 ) = 10.0;
	u( 1,0 ) = -200.0;
	u( 2,0 ) = 200.0;
	u( 3,0 ) = 0.0;
	u( 4,0 ) = 0.0;
	u( 5,0 ) = 0.0;

	DVector p( 1 );
	p(0) = 350.0;

	myProcess.init( 0.0 );
	myProcess.run( u,p );


	VariablesGrid xSim, ySim;

	myProcess.getLast( LOG_SIMULATED_DIFFERENTIAL_STATES,xSim );
	xSim.print( "Simulated Differential States" );

	myProcess.getLast( LOG_PROCESS_OUTPUT,ySim );
	ySim.print( "Process Output" );


	return 0;
}



