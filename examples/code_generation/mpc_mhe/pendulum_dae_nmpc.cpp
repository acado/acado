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

int main(int argc, char * const argv[ ])
{
	//
	// Variables
	//

	DifferentialState	x, y, w, dx, dy, dw;
	AlgebraicState		mu;
	Control				F;

	IntermediateState	c, dc;

	const double		m = 1.0;
	const double		mc = 1.0;
	const double		L = 1.0;
	const double		g = 9.81;
	const double		p = 5.0;

	c	= 0.5 * ((x - w) * (x - w) + y * y - L * L);
	dc	= dy * y + (dw - dx) * (w - x);

	//
	// Differential algebraic equation
	//

	DifferentialEquation f;

	f << 0 == dot( x ) - dx;
	f << 0 == dot( y ) - dy;
	f << 0 == dot( w ) - dw;

	f << 0 == m * dot( dx ) + (x - w) * mu;
	f << 0 == m * dot( dy ) + y * mu + m * g;
	f << 0 == mc * dot( dw ) + (w - x) * mu  - F;

	f << 0 == (x - w) * dot( dx ) + y * dot( dy ) + (w - x) * dot( dw )
					- (-p * p * c - 2 * p * dc - dy * dy - (dw - dx) * (dw - dx));

	//
	// Weighting matrices and reference functions
	//

	Function rf;
	Function rfN;

	rf << x << y << w << dx << dy << dw << F;
	rfN << x << y << w << dx << dy << dw;

	DMatrix W = eye<double>( rf.getDim() );
	DMatrix WN = eye<double>( rfN.getDim() ) * 10;

	//
	// Optimal Control Problem
	//

	const int N  = 10;
	const int Ni = 4;
	const double Ts = 0.1;

	OCP ocp(0, N * Ts, N);

	ocp.subjectTo( f );

	ocp.minimizeLSQ(W, rf);
	ocp.minimizeLSQEndTerm(WN, rfN);

	ocp.subjectTo(-20 <= F <= 20);
//	ocp.subjectTo( -5 <= x <= 5 );

	//
	// Export the code:
	//
	OCPexport mpc( ocp );

	mpc.set(HESSIAN_APPROXIMATION, GAUSS_NEWTON);
	mpc.set(DISCRETIZATION_TYPE, MULTIPLE_SHOOTING);

	mpc.set(INTEGRATOR_TYPE, INT_IRK_RIIA3);
	mpc.set(NUM_INTEGRATOR_STEPS, N * Ni);

	mpc.set(SPARSE_QP_SOLUTION, FULL_CONDENSING);
//	mpc.set(SPARSE_QP_SOLUTION, CONDENSING);
	mpc.set(QP_SOLVER, QP_QPOASES);
//	mpc.set(MAX_NUM_QP_ITERATIONS, 20);
	mpc.set(HOTSTART_QP, YES);

//	mpc.set(SPARSE_QP_SOLUTION, SPARSE_SOLVER);
//	mpc.set(QP_SOLVER, QP_FORCES);

//	mpc.set(LEVENBERG_MARQUARDT, 1.0e-10);

	mpc.set(GENERATE_TEST_FILE, NO);
	mpc.set(GENERATE_MAKE_FILE, NO);
	mpc.set(GENERATE_MATLAB_INTERFACE, YES);

//	mpc.set(USE_SINGLE_PRECISION, YES);
//	mpc.set(CG_USE_OPENMP, YES);

	if (mpc.exportCode( "pendulum_dae_nmpc_export" ) != SUCCESSFUL_RETURN)
		exit( EXIT_FAILURE );

	mpc.printDimensionsQP( );

	return EXIT_SUCCESS;
}
