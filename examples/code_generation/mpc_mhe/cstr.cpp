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
 *    \file   examples/code_generation/cstr.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date   2010-2013
 */

#include <acado_code_generation.hpp>

const double k10 =  1.287e12;
const double k20 =  1.287e12;
const double k30 =  9.043e09;
const double E1  =  -9758.3;
const double E2  =  -9758.3;
const double E3  =  -8560.0;
const double H1  =      4.2;
const double H2  =    -11.0;
const double H3  =    -41.85;
const double rho =      0.9342;
const double Cp  =      3.01;
const double kw  =   4032.0;
const double AR  =      0.215;
const double VR  =     10.0;
const double mK  =      5.0;
const double CPK =      2.0;

const double cA0    =    5.1;
const double theta0 =  104.9;

const double FFs    =    14.19;  /* Feed Flow normed by VR: (dV/dt  / VR)*/
const double QdotKs = -1113.50;

const double cAs     =  2.1402105301746182e00;
const double cBs     =  1.0903043613077321e00;
const double thetas  =  1.1419108442079495e02;
const double thetaKs =  1.1290659291045561e02;


const double TIMEUNITS_PER_HOUR = 3600.0;


const double P11 =   3278.78;   
const double P21 =   1677.31;
const double P31 =   681.02;
const double P41 =   271.50;

const double P12 =   1677.31;
const double P22 =   919.78;
const double P32 =   344.19;
const double P42 =   137.27;

const double P13 =   681.02;
const double P23 =   344.19;
const double P33 =   172.45;
const double P43 =   65.53;

const double P14 =   271.50;
const double P24 =   137.27;
const double P34 =   65.53;
const double P44 =   29.28;


const double R_OMEGA = 90.0;


int main()
{

	USING_NAMESPACE_ACADO

	// Define a Right-Hand-Side:
	DifferentialState cA, cB, theta, thetaK;
	Control           u("", 2, 1);

	DifferentialEquation f;

	IntermediateState k1, k2, k3;

	k1 = k10*exp(E1/(273.15 +theta));
	k2 = k20*exp(E2/(273.15 +theta));
	k3 = k30*exp(E3/(273.15 +theta));

	f << dot(cA) == (1/TIMEUNITS_PER_HOUR)*(u(0)*(cA0-cA) - k1*cA - k3*cA*cA);
	f << dot(cB) == (1/TIMEUNITS_PER_HOUR)* (- u(0)*cB + k1*cA - k2*cB);
	f << dot(theta) == (1/TIMEUNITS_PER_HOUR)*(u(0)*(theta0-theta) - (1/(rho*Cp)) *(k1*cA*H1 + k2*cB*H2 + k3*cA*cA*H3)+(kw*AR/(rho*Cp*VR))*(thetaK -theta));
	f << dot(thetaK) == (1/TIMEUNITS_PER_HOUR)*((1/(mK*CPK))*(u(1) + kw*AR*(theta-thetaK)));

	// Reference functions and weighting matrices:
	Function h, hN;
	h << cA << cB << theta << thetaK << u;
	hN << cA << cB << theta << thetaK;

	DMatrix W = eye<double>( h.getDim() );
	DMatrix WN = eye<double>( hN.getDim() );

	W(0, 0) = WN(0, 0) = 0.2;
	W(1, 1) = WN(1, 1) = 1.0;
	W(2, 2) = WN(2, 2) = 0.5;
	W(3, 3) = WN(3, 3) = 0.2;

	W(4, 4) = 0.5000;
	W(5, 5) = 0.0000005;

	//
	// Optimal Control Problem
	//
	OCP ocp(0.0, 1500.0, 10);

	ocp.subjectTo( f );

	ocp.minimizeLSQ(W, h);
	ocp.minimizeLSQEndTerm(WN, hN);

	ocp.subjectTo( 3.0 <= u(0) <= 35.0 );
	ocp.subjectTo( -9000.0 <= u(1) <= 0.0 );

	ocp.subjectTo( cA <= 2.5 );
// 	ocp.subjectTo( cB <= 1.055 );

	// Export the code:
	OCPexport mpc( ocp );

	mpc.set( INTEGRATOR_TYPE , INT_RK4 );
	mpc.set( NUM_INTEGRATOR_STEPS , 20 );
	mpc.set( HESSIAN_APPROXIMATION, GAUSS_NEWTON );
	mpc.set( QP_SOLVER, QP_QPOASES );
	mpc.set( GENERATE_TEST_FILE,NO );

//	mpc.set( USE_SINGLE_PRECISION,YES );
//	mpc.set( GENERATE_SIMULINK_INTERFACE,YES );

	if (mpc.exportCode( "cstr_export" ) != SUCCESSFUL_RETURN)
		exit( EXIT_FAILURE );

	mpc.printDimensionsQP( );

	return EXIT_SUCCESS;
}
