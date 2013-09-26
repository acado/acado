/*
 *    This file is part of ACADO Toolkit.
 *
 *    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
 *    Copyright (C) 2008-2013 by Boris Houska, Hans Joachim Ferreau,
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
 *    \file   examples/code_generation/crane.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date   2010 - 2013
 */

#include <acado_toolkit.hpp>

int main()
{
	USING_NAMESPACE_ACADO

	// Variables:
	DifferentialState x;     // the trolley position
	DifferentialState v;     // the trolley velocity
	DifferentialState phi;   // the excitation angle
	DifferentialState omega; // the angular velocity
	Control ax;              // the acc. of the trolley
	Parameter pp;

	const double g = 9.81;   // the gravitational constant
	const double b = 0.20;   // the friction coefficient

	// Model equations:
	DifferentialEquation f;

	f << dot( x ) == v+pp;
	f << dot( v ) == ax;
	f << dot( phi ) == omega;
	f << dot( omega ) == -g*sin(phi) - ax*cos(phi) - b*omega;

	// Reference functions and weighting matrices:
	Function h, hN;
	h << x << v << phi << omega << ax;
	hN << x << v << phi << omega;

	ExportVariable S = eye( h.getDim() );
	ExportVariable SN = eye( hN.getDim() );

	//
	// Optimal Control Problem
	//
	OCP ocp(0.0, 3.0, 10);

	ocp.subjectTo( f );

	ocp.minimizeLSQ(S, h);
	ocp.minimizeLSQEndTerm(SN, hN);

	// Input constraint
	ocp.subjectTo( -1.0 <= ax <= 1.0 );

	// Box-point constraint
//	ocp.subjectTo(AT_END, x == 0.0);
//	ocp.subjectTo(AT_END, v == 0.0);
//	ocp.subjectTo(AT_END, phi == 0.0);
//	ocp.subjectTo(AT_END, omega == 0.0);

	// State constraint
//	ocp.subjectTo( -0.5 <= v <= 1.5 );

	// Export the code:
	OCPexport mpc(ocp);

	mpc.set( INTEGRATOR_TYPE, INT_RK4 );
	mpc.set( NUM_INTEGRATOR_STEPS, 30 );
	mpc.set( HESSIAN_APPROXIMATION, GAUSS_NEWTON );
	mpc.set( DISCRETIZATION_TYPE, MULTIPLE_SHOOTING );

	mpc.set( MAX_NUM_QP_ITERATIONS, 40 );
	mpc.set( HOTSTART_QP, NO );
	mpc.set( QP_SOLVER, QP_QPOASES );
	mpc.set( SPARSE_QP_SOLUTION, FULL_CONDENSING );

	mpc.set( GENERATE_TEST_FILE, NO );
//	mpc.set( GENERATE_SIMULINK_INTERFACE, YES );
//	mpc.set( USE_SINGLE_PRECISION, YES );

	if (mpc.exportCode( "crane_export" ) != SUCCESSFUL_RETURN)
		exit( EXIT_FAILURE );

	mpc.printDimensionsQP( );

	return EXIT_SUCCESS;
}
