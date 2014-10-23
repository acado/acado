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
 *     Very simple example for testing rtcTOOLKIT.
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

	Control F;


    // DEFINE A DIFFERENTIAL EQUATION:
    // -------------------------------

	double h = 0.05;
	DiscretizedDifferentialEquation f( h );

// 	f << next(xB) == (  9.523918456856767e-01*xB - 3.093442425036754e-03*xW          + 4.450257887258270e-04*vW - 2.380407715716160e-07*F );
// 	f << next(xW) == ( -1.780103154903307e+00*xB - 1.005721624707961e+00*xW          - 3.093442425036752e-03*vW - 8.900515774516536e-06*F );
// 	f << next(vB) == ( -5.536210379145256e+00*xB - 2.021981836435758e-01*xW + 1.0*vB + 2.474992857984263e-02*vW + 1.294618052471308e-04*F );
// 	f << next(vW) == (  1.237376970014700e+01*xB + 1.183104351525840e+01*xW          - 1.005721624707961e+00*vW + 6.186884850073496e-05*F );

   f << next(xB) == (  0.9335*xB + 0.0252*xW + 0.048860*vB + 0.000677*vW  + 3.324e-06*F );
   f << next(xW) == (  0.1764*xB - 0.9821*xW + 0.004739*vB - 0.002591*vW  - 8.822e-06*F );
   f << next(vB) == ( -2.5210*xB - 0.1867*xW + 0.933500*vB + 0.025200*vW  + 0.0001261*F );
   f << next(vW) == ( -1.3070*xB + 11.670*xW + 0.176400*vB - 0.982100*vW  + 6.536e-05*F );
   
	OutputFcn g;
	g << xB;
	g << 500.0*vB + F;

    DynamicSystem dynSys( f,g );


    // SETUP THE PROCESS:
    // ------------------
	DVector mean( 1 ), amplitude( 1 );
	mean.setZero( );
	amplitude.setAll( 50.0 );

	GaussianNoise myNoise( mean,amplitude );

	Actuator myActuator( 1 );

	myActuator.setControlNoise( myNoise,0.1 );
	myActuator.setControlDeadTimes( 0.1 );
	

	mean.setZero( );
	amplitude.setAll( 0.001 );
	UniformNoise myOutputNoise1( mean,amplitude );
	
	mean.setAll( 20.0 );
	amplitude.setAll( 10.0 );
	GaussianNoise myOutputNoise2( mean,amplitude );
	
	Sensor mySensor( 2 );
	mySensor.setOutputNoise( 0,myOutputNoise1,0.1 );
	mySensor.setOutputNoise( 1,myOutputNoise2,0.1 );
	mySensor.setOutputDeadTimes( 0.15 );


	Process myProcess;
	
	myProcess.setDynamicSystem( dynSys );
	myProcess.set( ABSOLUTE_TOLERANCE,1.0e-8 );
	
	myProcess.setActuator( myActuator );
	myProcess.setSensor( mySensor );

	DVector x0( 4 );
	x0.setZero( );
	x0( 0 ) = 0.01;

	myProcess.initializeStartValues( x0 );

	myProcess.set( PLOT_RESOLUTION,HIGH );
// 	myProcess.set( CONTROL_PLOTTING,PLOT_NOMINAL );
// 	myProcess.set( PARAMETER_PLOTTING,PLOT_NOMINAL );
	myProcess.set( OUTPUT_PLOTTING,PLOT_REAL );

	GnuplotWindow window;
	  window.addSubplot( xB, "Body Position [m]" );
	  window.addSubplot( xW, "Wheel Position [m]" );
	  window.addSubplot( vB, "Body Velocity [m/s]" );
	  window.addSubplot( vW, "Wheel Velocity [m/s]" );

	  window.addSubplot( F,"Damping Force [N]" );
	  window.addSubplot( g(0),"Output 1" );
	  window.addSubplot( g(1),"Output 2" );

	myProcess << window;


    // SIMULATE AND GET THE RESULTS:
    // -----------------------------
	VariablesGrid u( 1,0.0,1.0,6 );

	u( 0,0 ) = 10.0;
	u( 1,0 ) = -200.0;
	u( 2,0 ) = 200.0;
	u( 3,0 ) = 200.0;
	u( 4,0 ) = 0.0;
	u( 5,0 ) = 0.0;

	myProcess.init( 0.0,x0,u.getFirstVector() );
	myProcess.run( u );


	VariablesGrid uNom, uSim, ySim, ySens, xSim;

// 	myProcess.getLast( LOG_NOMINAL_CONTROLS,uNom ); uNom.print( "uNom" );
// 	myProcess.getLast( LOG_SIMULATED_CONTROLS,uSim ); uSim.print( "uSim" );
// 	myProcess.getLast( LOG_SIMULATED_OUTPUT,ySim ); ySim.print( "ySim" );
// 	myProcess.getLast( LOG_PROCESS_OUTPUT,ySens ); ySens.print( "ySens" );
// 
// 	myProcess.getLast( LOG_DIFFERENTIAL_STATES,xSim );
	


	return 0;
}



