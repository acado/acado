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

USING_NAMESPACE_ACADO

int main( void )
{
	//
	// OCP parameters
	//
	// Sampling time
	double Ts = 0.01;
	// Number of shootin' intervals
	int N = 20;
	// Number of integrator steps per shootin' interval
	int Ni = 1;

	//
	// DEFINE THE VARIABLES
	//
	DifferentialState	xT;	// the trolley position
	DifferentialState	vT;	// the trolley velocity
	IntermediateState	aT;	// the trolley acceleration

	DifferentialState	xL;	// the cable length
	DifferentialState	vL;	// the cable velocity
	IntermediateState	aL;	// the cable acceleration

	DifferentialState 	phi;	// the angle
	DifferentialState	omega;	// the angular velocity

	DifferentialState	uT;		// trolley velocity control
	DifferentialState	uL;		// cable velocity control

	Control				duT;	// trolley accel control
	Control				duL;	// cable accel control

	//
	// DEFINE THE MODEL PARAMETERS
	//
	const double		tau1	= 0.012790605943772;
	const double		a1		= 0.047418203070092;
	const double		tau2	= 0.024695192379264;
	const double		a2		= 0.034087337273386;
	const double		g		= 9.81;
//	const double		c		= 0.0;
//	const double		m		= 1.3180;

	//
	// DEFINE THE MODEL EQUATIONS
	//
	DifferentialEquation	f;
	aT = -1.0 / tau1 * vT + a1 / tau1 * uT;
	aL = -1.0 / tau2 * vL + a2 / tau2 * uL;

	f << dot(xT) == vT;
	f << dot(vT) == aT;
	f << dot(xL) == vL;
	f << dot(vL) == aL;
	f << dot(phi) == omega;
	f << dot(omega)
			== -(g * sin(phi) + a1 * duT * cos(phi) + 2 * vL * omega) / xL;
	f << dot(uT) == duT;
	f << dot(uL) == duL;

	//
	// MHE PROBLEM FORMULATION
	//
	OCP ocp(0.0, N * Ts, N);

	ocp.subjectTo( f );

	// Measurement function h(x, u) on first N nodes
	Function h;

	h << xT << xL << phi << uT << uL << duT << duL;

	// Weighting matrices and measurement functions
	DMatrix W = eye<double>( 7 );
//	W(0,0) = 16.5;
//	W(1,1) = 23.9;
//	W(2,2) = 25.1;
//	W(3,3) = 12.1;
//	W(4,4) = 119.4;
//	W(5,5) = 45.0;
//	W(6,6) = 1.2;
//	W(7,7) = 0.4;

	Function hN;
	hN << xT << xL << phi << uT << uL;

	DMatrix WN = eye<double>( 5 );
	WN(0, 0) = W(0, 0);
	WN(1, 1) = W(1, 1);
	WN(2, 2) = W(2, 2);
	WN(3, 3) = W(3, 3);
	WN(4, 4) = W(4, 4);

	ocp.minimizeLSQ(W, h);
	ocp.minimizeLSQEndTerm(WN, hN);

	OCPexport mhe( ocp );

	mhe.set(INTEGRATOR_TYPE, INT_RK4);
	mhe.set(NUM_INTEGRATOR_STEPS, N * Ni);

	mhe.set(HESSIAN_APPROXIMATION, GAUSS_NEWTON);
	mhe.set(DISCRETIZATION_TYPE, MULTIPLE_SHOOTING);

	mhe.set(HOTSTART_QP, YES);

	// NOTE: This is crucial for export of MHE!
	mhe.set(SPARSE_QP_SOLUTION, CONDENSING);
	mhe.set(FIX_INITIAL_STATE, NO);

//	mhe.set( LEVENBERG_MARQUARDT, 1e-10 );

	if (mhe.exportCode("crane_kul_mhe_export") != SUCCESSFUL_RETURN)
		exit( EXIT_FAILURE );

	mhe.printDimensionsQP( );

	return EXIT_SUCCESS;
}
