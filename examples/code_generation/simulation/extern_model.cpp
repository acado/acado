/*
 *    This file is part of ACADO Toolkit.
 *
 *    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
 *    Copyright (C) 2008-2013 by Boris Houska, Hans Joachim Ferreau,
 *    Milan Vukov, Rien Quirynen, KU Leuven.
 *    Developed within the Optimization in Engineering Center (OPTEC)
 * 	  under supervision of Moritz Diehl. All rights reserved.
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

int main() {
	USING_NAMESPACE_ACADO
	
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

    DifferentialEquation   f;  
    
    OutputFcn h;
    
    h << Fx;
    h << Fy;
    
    const double      m = 2;
    const double      M = 3.5;
    const double      I = 0.1;
    const double	  g = 9.81;

    f << 0			== dot( x ) - dx;
    f << 0			== dot( y ) - dy;
    f << 0			== dot( alpha ) - dalpha;
    f << 0			== dot( dx ) - ddx ;
    f << 0			== dot( dy ) - ddy;
    f << 0			== dot( dalpha ) - ddalpha;
    
    f << 0		    == m*ddx - (Fx+u);
    f << 0			== m*ddy + m*g - (Fy+u);
    f << 0			== I*ddalpha - M - (Fx+u)*y + (Fy+u)*x;
    f << 0			== ddx + dy*dalpha + y*ddalpha;
    f << 0 			== ddy - dx*dalpha - x*ddalpha;
	// ----------------------------------------------------------
	
	Vector Meas(1);
	Meas(0) = 5;
 
    SIMexport sim( 1, 0.1 );
    
    sim.set( INTEGRATOR_TYPE, INT_IRK_RIIA3 );
    sim.set( NUM_INTEGRATOR_STEPS, 4 );
    sim.set( MEASUREMENT_GRID, EQUIDISTANT_GRID );
    
    sim.setModel( f );
    sim.addOutput( h );
    sim.setMeasurements( Meas );
	sim.setTimingSteps( 10000 );
    
    acadoPrintf( "-----------------------------------------\n  Using a Pendulum DAE model in ACADO syntax:\n-----------------------------------------\n" );
    sim.exportAndRun( "externModel_export", "init_externModel.txt", "controls_externModel.txt" );
    
    
    SIMexport sim2( 1, 0.1 );
    
    sim2.set( INTEGRATOR_TYPE, INT_IRK_RIIA3 );
    sim2.set( NUM_INTEGRATOR_STEPS, 4 );
    sim2.set( MEASUREMENT_GRID, EQUIDISTANT_GRID );
    
    sim2.setModel( "model", "rhs", "rhs_jac" );
    sim2.setDimensions( 6, 6, 5, 1 );
    
    sim2.addOutput( "out", "out_jac", 2 );
    sim2.setMeasurements( Meas );
	sim2.setTimingSteps( 10000 );
    
    acadoPrintf( "-----------------------------------------\n  Using an externally defined Pendulum DAE model:\n-----------------------------------------\n" );
    sim2.exportAndRun( "externModel_export", "init_externModel.txt", "controls_externModel.txt" );
    
	return 0;
} 

