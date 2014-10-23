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
 *    \file examples/integration_algorithm/harmonic_oscillator.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date 2008
 */


#include <acado_optimal_control.hpp>
#include <acado_gnuplot.hpp>

using namespace std;

USING_NAMESPACE_ACADO

int main( )
{
	// Parameters
	double h_hw = 10;    // water level
	double A_hw = 1.0;   // amplitude of the waves
	double T_hw = 5.0;   // duration of a wave
	double rho  = 1000;  // density of water
	double A    = 1.0;   // bottom area of the buoy
	double m    = 100;   // mass of the buoy
	double g    = 9.81;  // gravitational constant

	// Free varameter
	double a = 1.0;      // take to be constant here

	// Variables
	DifferentialState h; // Position of the buoy
	DifferentialState v; // Velocity of the buoy
	DifferentialState w; // Produced wave energy
	TIME t;

	// Differential equation
	DifferentialEquation f;

	// Height of the wave
	IntermediateState hw;
	hw = h_hw + A_hw*sin(2*M_PI*t/T_hw);
	f << dot(h) ==  v;
	f << dot(v) ==  rho*A*(hw-h)/m - g - a*v;
	f << dot(w) ==  a*v*v;

	// Define an initial value:
	// ------------------------
	DVector xStart( 3 );
	xStart(0) = h_hw - 2.0*A_hw;
	xStart(1) = 0.0;
	xStart(2) = 0.0;

	Grid timeHorizon( 0.0,25.0 );

    // DEFINE AN INTEGRATOR:
    // ---------------------

    IntegrationAlgorithm intAlg;

	intAlg.addStage( f, timeHorizon, INT_RK45 );

	intAlg.set( INTEGRATOR_PRINTLEVEL, MEDIUM );
	intAlg.set( PLOT_RESOLUTION, HIGH );
	intAlg.set( FREEZE_INTEGRATOR, NO );

	GnuplotWindow window;
	window.addSubplot( h );
	window.addSubplot( v );
	window.addSubplot( w );
	
	intAlg << window;

	// START THE INTEGRATION:
    // ----------------------

	intAlg.integrate( timeHorizon, xStart );

	return 0;
}



