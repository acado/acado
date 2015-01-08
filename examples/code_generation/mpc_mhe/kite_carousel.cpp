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
 *    \file   examples/code_generation/kite_carousel.cpp
 *    \author Boris Houska, Hans Joachim Ferreau, Kurt Geebelen
 *    \date   2010-2013
 */

#include <acado_code_generation.hpp>

int main()
{
	USING_NAMESPACE_ACADO

	// Variables:
	DifferentialState phi;
	DifferentialState theta;
	DifferentialState dphi;
	DifferentialState dtheta;

	Control u1;
	Control u2;

	IntermediateState c;// c := rho * |we|^2 / (2*m)
	DifferentialEquation f;// the right-hand side of the model

	// Parameters:
	const double R = 1.00;// radius of the carousel arm
	const double Omega = 1.00;// constant rotational velocity of the carousel arm
	const double m = 0.80;// the mass of the plane
	const double r = 1.00;// length of the cable between arm and plane
	const double A = 0.15;// wing area of the plane

	const double rho = 1.20;// density of the air
	const double CL = 1.00;// nominal lift coefficient
	const double CD = 0.15;// nominal drag coefficient
	const double b = 15.00;// roll stabilization coefficient
	const double g = 9.81;// gravitational constant

	// COMPUTE THE CONSTANT c:
	// -------------------------
	c = ( (R*R*Omega*Omega)
			+ (r*r)*( (Omega+dphi)*(Omega+dphi) + dtheta*dtheta )
			+ (2.0*r*R*Omega)*( (Omega+dphi)*sin(theta)*cos(phi)
					+ dtheta*cos(theta)*sin(phi) ) ) * ( A*rho/( 2.0*m ) );

	f << dot( phi ) == dphi;
	f << dot( theta ) == dtheta;

	f << dot( dphi ) == ( 2.0*r*(Omega+dphi)*dtheta*cos(theta)
			+ (R*Omega*Omega)*sin(phi)
			+ c*(CD*(1.0+u2)+CL*(1.0+0.5*u2)*phi) ) / (-r*sin(theta));

	f << dot( dtheta ) == ( (R*Omega*Omega)*cos(theta)*cos(phi)
			+ r*(Omega+dphi)*sin(theta)*cos(theta)
			+ g*sin(theta) - c*( CL*u1 + b*dtheta ) ) / r;

	// Reference functions and weighting matrices:
	Function h, hN;
	h << phi << theta << dphi << dtheta << u1 << u2;
	hN << phi << theta << dphi << dtheta;

	DMatrix W = eye<double>( h.getDim() );
	DMatrix WN = eye<double>( hN.getDim() );

	W(0,0) = 5.000;
	W(1,1) = 1.000;
	W(2,2) = 10.000;
	W(3,3) = 10.000;
	W(4,4) = 0.1;
	W(5,5) = 0.1;

	WN(0,0) = 1.0584373059248833e+01;
	WN(0,1) = 1.2682415889087276e-01;
	WN(0,2) = 1.2922232012424681e+00;
	WN(0,3) = 3.7982078044271374e-02;
	WN(1,0) = 1.2682415889087265e-01;
	WN(1,1) = 3.2598407653299275e+00;
	WN(1,2) = -1.1779732282636639e-01;
	WN(1,3) = 9.8830655348904548e-02;
	WN(2,0) = 1.2922232012424684e+00;
	WN(2,1) = -1.1779732282636640e-01;
	WN(2,2) = 4.3662024133354898e+00;
	WN(2,3) = -5.9269992411260908e-02;
	WN(3,0) = 3.7982078044271367e-02;
	WN(3,1) = 9.8830655348904534e-02;
	WN(3,2) = -5.9269992411260901e-02;
	WN(3,3) = 3.6495564367004318e-01;

	//
	// Optimal Control Problem
	//

//	OCP ocp( 0.0, 12.0, 15 );
	OCP ocp( 0.0, 2.0 * M_PI, 10 );

	ocp.subjectTo( f );

	ocp.minimizeLSQ(W, h);
	ocp.minimizeLSQEndTerm(WN, hN);

	ocp.subjectTo( 18.0 <= u1 <= 22.0 );
	ocp.subjectTo( -0.2 <= u2 <= 0.2 );

	// Export the code:
	OCPexport mpc(ocp);

	mpc.set( INTEGRATOR_TYPE , INT_RK4 );
	mpc.set( NUM_INTEGRATOR_STEPS , 30 );
	mpc.set( HESSIAN_APPROXIMATION, GAUSS_NEWTON );
	mpc.set( HOTSTART_QP, YES );
	mpc.set( GENERATE_TEST_FILE, NO );

	if (mpc.exportCode("kite_carousel_export") != SUCCESSFUL_RETURN)
		exit( EXIT_FAILURE );

	mpc.printDimensionsQP( );

// 	DifferentialState Gx(4,4), Gu(4,2);
// 
// 	Expression rhs;
// 	f.getExpression( rhs );
// 
// 	Function Df;
// 	Df << rhs;
// 	Df << forwardDerivative( rhs, x ) * Gx;
// 	Df << forwardDerivative( rhs, x ) * Gu + forwardDerivative( rhs, u );
// 
// 
// 	EvaluationPoint z(Df);
// 	
// 	DVector xx(28);
// 	xx.setZero();
// 
//     xx(0) = -4.2155955213988627e-02;
// 	xx(1) =  1.8015724412870739e+00;
// 	xx(2) =  0.0;
// 	xx(3) =  0.0;
// 
// 	DVector uu(2);
//     uu(0) = 20.5;
// 	uu(1) = -0.1;
// 	
// 	z.setX( xx );
//     z.setU( uu );
// 
//     DVector result = Df.evaluate( z );
// 	result.print( "x" );

	return EXIT_SUCCESS;
}


