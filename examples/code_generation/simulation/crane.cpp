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
*    Author Rien Quirynen
*    Date 2012
*    http://www.acadotoolkit.org
*/

#include <acado_code_generation.hpp>


using namespace std;
USING_NAMESPACE_ACADO

int main()
{
	//
	// DEFINE THE VARIABLES:
    //
    DifferentialState   xT;     // the trolley position
    DifferentialState   vT;     // the trolley velocity
    IntermediateState   aT;     // the trolley acceleration
    DifferentialState   xL;     // the cable length
    DifferentialState   vL;     // the cable velocity
    IntermediateState   aL;     // the cable acceleration
    DifferentialState   phi;    // the excitation angle
    DifferentialState   omega;  // the angular velocity
        
    DifferentialState   uT;     // trolley velocity control
    DifferentialState   uL;     // cable velocity control

    Control             duT;
    Control             duL;

	//
    // DEFINE THE PARAMETERS:
    //
    const double      tau1 = 0.012790605943772;
    const double      a1   = 0.047418203070092;
    const double      tau2 = 0.024695192379264;
    const double      a2   = 0.034087337273386;
    const double      g = 9.81;       		
    const double      c = 0.0;        		
    const double      m = 1318.0;     		

    //
    // DEFINE THE MODEL EQUATIONS:
    //
    DifferentialEquation   f;
    aT = -1.0 / tau1 * vT + a1 / tau1 * uT;
    aL = -1.0 / tau2 * vL + a2 / tau2 * uL;

    f << 0 == dot( xT ) - vT;
    f << 0 == dot( vT ) - aT;
    f << 0 == dot( xL ) - vL;
    f << 0 == dot( vL ) - aL;
    f << 0 == dot( phi ) - omega;
    f << 0 == dot( omega ) - 1.0/xL*(-g*sin(phi)-aT*cos(phi) 
						-2*vL*omega-c*omega/(m*xL));
    f << 0 == dot( uT ) - duT;
    f << 0 == dot( uL ) - duL;

    //
    // DEFINE THE OUTPUT MODEL:
    //
	OutputFcn	h;

	h << aT;
	h << aL;

	//
	// SET UP THE SIMULATION EXPORT MODULE:
	//
	
	cout << "-----------------------------------------\n  Using an equidistant grid:\n-----------------------------------------\n";
	
	SIMexport sim( 1, 0.1 );
	
	sim.setModel( f );
	sim.addOutput( h, 5 );
	
	sim.set( INTEGRATOR_TYPE, INT_IRK_RIIA5 );
	sim.set( NUM_INTEGRATOR_STEPS, 5 );
	sim.setTimingSteps( 10000 );
	
	sim.exportAndRun( "crane_export", "init_crane.txt", "controls_crane.txt" );
	
	
	cout << "-----------------------------------------\n  Using a provided grid:\n-----------------------------------------\n";
	
	DVector Meas(5);
	Meas(0) = 0.0;
	Meas(1) = 0.2;
	Meas(2) = 0.4;
	Meas(3) = 0.6;
	Meas(4) = 0.8;
	
	SIMexport sim2( 1, 0.1 );
	
	sim2.setModel( f );
	sim2.addOutput( h, Meas );
	
	sim2.set( INTEGRATOR_TYPE, INT_IRK_RIIA5 );
	sim2.set( NUM_INTEGRATOR_STEPS, 5 );
	sim2.setTimingSteps( 10000 );
	
	sim2.exportAndRun( "crane_export", "init_crane.txt", "controls_crane.txt" );

	return 0;
}

