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

#include <acado_code_generation.hpp>

using namespace std;
USING_NAMESPACE_ACADO

int main( )
{
	// Define variables, functions and constants:
	// ----------------------------------------------------------
    DifferentialState   dT1;
    DifferentialState   dT2;
    DifferentialState   dT3;
    DifferentialState   dT4;
    
    DifferentialState   T1;
    DifferentialState   T2;
    DifferentialState   T3;
    DifferentialState   T4;
    
    DifferentialState   W1;
    DifferentialState   W2;
    DifferentialState   W3;
    DifferentialState   W4;
    
    DifferentialState   q1;
    DifferentialState   q2;
    DifferentialState   q3;
    DifferentialState   q4;
    
    DifferentialState   Omega1;
    DifferentialState   Omega2;
    DifferentialState   Omega3;
    
    DifferentialState   V1;
    DifferentialState   V2;
    DifferentialState   V3;
    
    DifferentialState   P1;		// x
    DifferentialState   P2;		// y
    DifferentialState   P3;		// z
    
    DifferentialState   IP1;
    DifferentialState   IP2;
    DifferentialState   IP3;

    Control             U1;
    Control             U2;
    Control             U3;
    Control             U4;

    DifferentialEquation   f1, f2;   
	
    const double rho = 1.23;
    const double A = 0.1;
    const double Cl = 0.25;
    const double Cd = 0.3*Cl;
    const double m = 10;
    const double g = 9.81;
    const double L  = 0.5;
    const double Jp = 1e-2;
    const double xi = 1e-2;
    const double J1 = 0.25;
    const double J2 = 0.25;
    const double J3 = 1;
    const double gain = 1e-4;

    const double alpha = 0.0;


	// Define the quadcopter ODE model in fully nonlinear form:
	// ----------------------------------------------------------
	f1 << dot(dT1) == U1*gain; 
	f1 << dot(dT2) == U2*gain; 
	f1 << dot(dT3) == U3*gain; 
	f1 << dot(dT4) == U4*gain; 
	f1 << dot(T1) == dT1; 
	f1 << dot(T2) == dT2; 
	f1 << dot(T3) == dT3; 
	f1 << dot(T4) == dT4; 
	f1 << dot(W1) == (T1 - W1*xi)/Jp; 
	f1 << dot(W2) == (T2 - W2*xi)/Jp; 
	f1 << dot(W3) == (T3 - W3*xi)/Jp; 
	f1 << dot(W4) == (T4 - W4*xi)/Jp; 
	f1 << dot(q1) == - (Omega1*q2)/2 - (Omega2*q3)/2 - (Omega3*q4)/2 - (alpha*q1*(q1*q1 + q2*q2 + q3*q3 + q4*q4 - 1))/(q1*q1 + q2*q2 + q3*q3 + q4*q4); 
	f1 << dot(q2) == (Omega1*q1)/2 - (Omega3*q3)/2 + (Omega2*q4)/2 - (alpha*q2*(q1*q1 + q2*q2 + q3*q3 + q4*q4 - 1))/(q1*q1 + q2*q2 + q3*q3 + q4*q4); 
	f1 << dot(q3) == (Omega2*q1)/2 + (Omega3*q2)/2 - (Omega1*q4)/2 - (alpha*q3*(q1*q1 + q2*q2 + q3*q3 + q4*q4 - 1))/(q1*q1 + q2*q2 + q3*q3 + q4*q4); 
	f1 << dot(q4) == (Omega3*q1)/2 - (Omega2*q2)/2 + (Omega1*q3)/2 - (alpha*q4*(q1*q1 + q2*q2 + q3*q3 + q4*q4 - 1))/(q1*q1 + q2*q2 + q3*q3 + q4*q4); 
	f1 << dot(Omega1) == (J3*Omega2*Omega3 - J2*Omega2*Omega3 + (A*Cl*L*rho*(W2*W2 - W4*W4))/2)/J1; 
	f1 << dot(Omega2) == -(J3*Omega1*Omega3 - J1*Omega1*Omega3 + (A*Cl*L*rho*(W1*W1 - W3*W3))/2)/J2; 
	f1 << dot(Omega3) == (J2*Omega1*Omega2 - J1*Omega1*Omega2 + (A*Cd*rho*(W1*W1 - W2*W2 + W3*W3 - W4*W4))/2)/J3; 
	f1 << dot(V1) == (A*Cl*rho*(2*q1*q3 + 2*q2*q4)*(W1*W1 + W2*W2 + W3*W3 + W4*W4))/(2*m); 
	f1 << dot(V2) == -(A*Cl*rho*(2*q1*q2 - 2*q3*q4)*(W1*W1 + W2*W2 + W3*W3 + W4*W4))/(2*m); 
	f1 << dot(V3) == (A*Cl*rho*(W1*W1 + W2*W2 + W3*W3 + W4*W4)*(q1*q1 - q2*q2 - q3*q3 + q4*q4))/(2*m) - g; 
	f1 << dot(P1) == V1; 
	f1 << dot(P2) == V2; 
	f1 << dot(P3) == V3; 
	f1 << dot(IP1) == P1; 
	f1 << dot(IP2) == P2; 
	f1 << dot(IP3) == P3; 

	// Define the quadcopter ODE model in 3-stage format:
	// ----------------------------------------------------------
	
	// LINEAR INPUT SYSTEM (STAGE 1):
	DMatrix M1, A1, B1;
	M1 = eye<double>(12);
	A1 = zeros<double>(12,12);
	B1 = zeros<double>(12,4);
	
	A1(4,0) = 1.0;
	A1(5,1) = 1.0;
	A1(6,2) = 1.0;
	A1(7,3) = 1.0;
	A1(8,4) = 1.0/Jp;	A1(8,8) = -xi/Jp;
	A1(9,5) = 1.0/Jp;	A1(9,9) = -xi/Jp;
	A1(10,6) = 1.0/Jp;	A1(10,10) = -xi/Jp;
	A1(11,7) = 1.0/Jp;	A1(11,11) = -xi/Jp;
	
	B1(0,0) = gain;
	B1(1,1) = gain;
	B1(2,2) = gain;
	B1(3,3) = gain;
	
	// NONLINEAR SYSTEM (STAGE 2):
	f2 << dot(q1) == - (Omega1*q2)/2 - (Omega2*q3)/2 - (Omega3*q4)/2 - (alpha*q1*(q1*q1 + q2*q2 + q3*q3 + q4*q4 - 1))/(q1*q1 + q2*q2 + q3*q3 + q4*q4); 
	f2 << dot(q2) == (Omega1*q1)/2 - (Omega3*q3)/2 + (Omega2*q4)/2 - (alpha*q2*(q1*q1 + q2*q2 + q3*q3 + q4*q4 - 1))/(q1*q1 + q2*q2 + q3*q3 + q4*q4); 
	f2 << dot(q3) == (Omega2*q1)/2 + (Omega3*q2)/2 - (Omega1*q4)/2 - (alpha*q3*(q1*q1 + q2*q2 + q3*q3 + q4*q4 - 1))/(q1*q1 + q2*q2 + q3*q3 + q4*q4); 
	f2 << dot(q4) == (Omega3*q1)/2 - (Omega2*q2)/2 + (Omega1*q3)/2 - (alpha*q4*(q1*q1 + q2*q2 + q3*q3 + q4*q4 - 1))/(q1*q1 + q2*q2 + q3*q3 + q4*q4); 
	f2 << dot(Omega1) == (J3*Omega2*Omega3 - J2*Omega2*Omega3 + (A*Cl*L*rho*(W2*W2 - W4*W4))/2)/J1; 
	f2 << dot(Omega2) == -(J3*Omega1*Omega3 - J1*Omega1*Omega3 + (A*Cl*L*rho*(W1*W1 - W3*W3))/2)/J2; 
	f2 << dot(Omega3) == (J2*Omega1*Omega2 - J1*Omega1*Omega2 + (A*Cd*rho*(W1*W1 - W2*W2 + W3*W3 - W4*W4))/2)/J3; 
	f2 << dot(V1) == (A*Cl*rho*(2*q1*q3 + 2*q2*q4)*(W1*W1 + W2*W2 + W3*W3 + W4*W4))/(2*m); 
	f2 << dot(V2) == -(A*Cl*rho*(2*q1*q2 - 2*q3*q4)*(W1*W1 + W2*W2 + W3*W3 + W4*W4))/(2*m); 
	f2 << dot(V3) == (A*Cl*rho*(W1*W1 + W2*W2 + W3*W3 + W4*W4)*(q1*q1 - q2*q2 - q3*q3 + q4*q4))/(2*m) - g; 
	
	// LINEAR OUTPUT SYSTEM (STAGE 3):
	DMatrix M3, A3;
	M3 = eye<double>(6);
	A3 = zeros<double>(6,6);
	
	A3(3,0) = 1.0;
	A3(4,1) = 1.0;
	A3(5,2) = 1.0;
    
    OutputFcn f3;
	
	f3 << V1;
	f3 << V2;
	f3 << V3;
	f3 << 0.0;
	f3 << 0.0;
	f3 << 0.0;


	// ----------------------------------------------------------
	// ----------------------------------------------------------
	SIMexport sim1( 10, 1.0 );
	
	sim1.setModel( f1 );
	sim1.set( INTEGRATOR_TYPE, INT_IRK_GL4 );
	
	sim1.set( NUM_INTEGRATOR_STEPS, 50 );
	sim1.setTimingSteps( 10000 );
	
	cout << "-----------------------------------------------------------\n  Using a QuadCopter ODE model in fully nonlinear form:\n-----------------------------------------------------------\n";
	sim1.exportAndRun( "quadcopter_export", "init_quadcopter.txt", "controls_quadcopter.txt" );


	// ----------------------------------------------------------
	// ----------------------------------------------------------
	SIMexport sim2( 10, 1.0 );
	
	sim2.setLinearInput( M1, A1, B1 );
	sim2.setModel( f2 );
	sim2.setLinearOutput( M3, A3, f3 );
	sim2.set( INTEGRATOR_TYPE, INT_IRK_GL4 );
	
	sim2.set( NUM_INTEGRATOR_STEPS, 50 );
	sim2.setTimingSteps( 10000 );
	
	cout << "-----------------------------------------------------------\n  Using a QuadCopter ODE model in 3-stage format:\n-----------------------------------------------------------\n";
	sim2.exportAndRun( "quadcopter_export", "init_quadcopter.txt", "controls_quadcopter.txt" );

	return 0;
}

