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
	// Define variables, functions and constants:
	// ----------------------------------------------------------
    DifferentialState   x;      
    DifferentialState   y;      
    DifferentialState	alpha;	
    DifferentialState   dx;
    DifferentialState	dy;
    DifferentialState	dalpha;
    
    AlgebraicState		ddx;
    AlgebraicState		ddy;
	AlgebraicState		ddalpha;
	AlgebraicState		Fx;
	AlgebraicState		Fy;
	
	Control 			u;

    DifferentialEquation   f1, f2;  
    
    OutputFcn h;
    
    h << Fx;
    h << Fy;
    
    const double      m = 2;
    const double      M = 3.5;
    const double      I = 0.1;
    const double	  g = 9.81;

	// Pendulum DAE model in ACADO syntax (semi-explicit):
	// ----------------------------------------------------------
    f1 << dot( x )		==  dx;
    f1 << dot( y )		==  dy;
    f1 << dot( alpha )	==  dalpha;
    f1 << dot( dx )		==  ddx ;
    f1 << dot( dy )		==  ddy;
    f1 << dot( dalpha )	==  ddalpha;
    
    f1 << 0		    	== m*ddx - (Fx+u);
    f1 << 0				== m*ddy + m*g - (Fy+u);
    f1 << 0				== I*ddalpha - M - (Fx+u)*y + (Fy+u)*x;
    f1 << 0				== ddx + dy*dalpha + y*ddalpha;
    f1 << 0 			== ddy - dx*dalpha - x*ddalpha;


	// Pendulum DAE model in ACADO syntax (implicit):
	// ----------------------------------------------------------
    f2 << 0			== dot( x ) - dx;
    f2 << 0			== dot( y ) - dy;
    f2 << 0			== dot( alpha ) - dalpha;
    f2 << 0			== dot( dx ) - ddx ;
    f2 << 0			== dot( dy ) - ddy;
    f2 << 0			== dot( dalpha ) - ddalpha;
    
    f2 << 0		    == m*ddx - (Fx+u);
    f2 << 0			== m*ddy + m*g - (Fy+u);
    f2 << 0			== I*ddalpha - M - (Fx+u)*y + (Fy+u)*x;
    f2 << 0			== ddx + dy*dalpha + y*ddalpha;
    f2 << 0 		== ddy - dx*dalpha - x*ddalpha;
 
	// ----------------------------------------------------------
	// ----------------------------------------------------------
    SIMexport sim1( 1, 0.1 );
    
    sim1.set( INTEGRATOR_TYPE, INT_IRK_RIIA3 );
    sim1.set( NUM_INTEGRATOR_STEPS, 4 );
    sim1.set( MEASUREMENT_GRID, OFFLINE_GRID );
    
    sim1.setModel( f1 );
    sim1.addOutput( h, 5 );
	sim1.setTimingSteps( 10000 );
    
    cout << "-----------------------------------------------------------\n  Using a Pendulum DAE model in ACADO syntax (semi-explicit):\n-----------------------------------------------------------\n";
    sim1.exportAndRun( "pendulum_export", "init_externModel.txt", "controls_externModel.txt" );
 
 
	// ----------------------------------------------------------
	// ----------------------------------------------------------
    SIMexport sim2( 1, 0.1 );
    
    sim2.set( INTEGRATOR_TYPE, INT_IRK_RIIA3 );
    sim2.set( NUM_INTEGRATOR_STEPS, 4 );
    sim2.set( MEASUREMENT_GRID, OFFLINE_GRID );
    
    sim2.setModel( f2 );
    sim2.addOutput( h, 5 );
	sim2.setTimingSteps( 10000 );
    
    cout << "-----------------------------------------------------------\n  Using a Pendulum DAE model in ACADO syntax (implicit):\n-----------------------------------------------------------\n";
    sim2.exportAndRun( "pendulum_export", "init_externModel.txt", "controls_externModel.txt" );
    
    
	// ----------------------------------------------------------
	// ----------------------------------------------------------
    SIMexport sim3( 1, 0.1 );

    sim3.set( INTEGRATOR_TYPE, INT_IRK_RIIA3 );
    sim3.set( NUM_INTEGRATOR_STEPS, 4 );
    sim3.set( MEASUREMENT_GRID, OFFLINE_GRID );
    sim3.set( GENERATE_MAKE_FILE, NO );

    sim3.setModel( "model", "rhs", "rhs_jac" );
	sim3.setDimensions(6, 6, 5, 1, 0, 0);

    sim3.addOutput( "out", "out_jac", 2, 5 );
	sim3.setTimingSteps( 10000 );

    cout << "-----------------------------------------------------------\n  Using an externally defined Pendulum DAE model:\n-----------------------------------------------------------\n";
    sim3.exportAndRun( "externModel_export", "init_externModel.txt", "controls_externModel.txt" );
    
    
	return 0;
} 

