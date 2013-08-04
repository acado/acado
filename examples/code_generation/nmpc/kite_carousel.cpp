/*
 *    This file is part of ACADO Toolkit.
 *
 *    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
 *    Copyright (C) 2008-2009 by Boris Houska and Hans Joachim Ferreau, K.U.Leuven.
 *    Developed within the Optimization in Engineering Center (OPTEC) under
 *    supervision of Moritz Diehl. All rights reserved.
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
 *    \file   carousel_plane/steady_state_tracking.cpp
 *    \author Boris Houska, Hans Joachim Ferreau, Kurt Geebelen
 *    \date   2010
 */


#include <acado_code_generation.hpp>
#include <include/acado_gnuplot/gnuplot_window.hpp>


int main( ){

    USING_NAMESPACE_ACADO

    // DEFINE THE STATES OF THE MODEL:
    // -------------------------------
       DifferentialState          phi;
       DifferentialState        theta;
       DifferentialState         dphi;
       DifferentialState       dtheta;

       Control                     u1;
       Control                     u2;

       IntermediateState            c;  // c := rho * |we|^2 / (2*m) 
       DifferentialEquation         f;  // the right-hand side of the model


    // SETUP THE MODEL PARAMETERS:
    // -------------------------------
       const double          R =  1.00;  // radius of the carousel arm
       const double      Omega =  1.00;  // constant rotational velocity of the carousel arm
       const double          m =  0.80;  // the mass of the plane
       const double          r =  1.00;  // length of the cable between arm and plane
       const double          A =  0.15;  // wing area of the plane

       const double        rho =  1.20;  // density of the air
       const double         CL =  1.00;  // nominal lift coefficient
       const double         CD =  0.15;  // nominal drag coefficient
       const double          b = 15.00;  // roll stabilization coefficient
       const double          g =  9.81;  // gravitational constant

    // COMPUTE THE CONSTANT c:
    // -------------------------
       c = (   (R*R*Omega*Omega)
             + (r*r)*( (Omega+dphi)*(Omega+dphi) + dtheta*dtheta )
             + (2.0*r*R*Omega)*(  (Omega+dphi)*sin(theta)*cos(phi)
                                + dtheta*cos(theta)*sin(phi)       ) ) * ( A*rho/( 2.0*m ) );

       f << dot(    phi ) ==  dphi  ;
       f << dot(  theta ) ==  dtheta;

       f << dot(   dphi ) ==  (   2.0*r*(Omega+dphi)*dtheta*cos(theta)
                                + (R*Omega*Omega)*sin(phi)
                                + c*(CD*(1.0+u2)+CL*(1.0+0.5*u2)*phi)   ) / (-r*sin(theta));

       f << dot( dtheta ) ==  (   (R*Omega*Omega)*cos(theta)*cos(phi)
                                + r*(Omega+dphi)*sin(theta)*cos(theta)
                                + g*sin(theta) - c*( CL*u1 + b*dtheta ) ) / r;


    // DEFINE THE WEIGHTING MATRICES:
    // ----------------------------------------------------------
       Matrix QQ  = eye(4);
       Matrix PP  = eye(4);
       Matrix RR  = eye(2);
    // ----------------------------------------------------------

       QQ(0,0) = 5.000;
       QQ(1,1) = 1.000;
       QQ(2,2) = 10.000;
       QQ(3,3) = 10.000;
       RR(0,0) = 0.1;
       RR(1,1) = 0.1;

    PP(0,0) =  1.0584373059248833e+01;
    PP(0,1) =  1.2682415889087276e-01;
    PP(0,2) =  1.2922232012424681e+00;
    PP(0,3) =  3.7982078044271374e-02;
    PP(1,0) =  1.2682415889087265e-01;
    PP(1,1) =  3.2598407653299275e+00;
    PP(1,2) = -1.1779732282636639e-01;
    PP(1,3) =  9.8830655348904548e-02;
    PP(2,0) =  1.2922232012424684e+00;
    PP(2,1) = -1.1779732282636640e-01;
    PP(2,2) =  4.3662024133354898e+00;
    PP(2,3) = -5.9269992411260908e-02;
    PP(3,0) =  3.7982078044271367e-02;
    PP(3,1) =  9.8830655348904534e-02;
    PP(3,2) = -5.9269992411260901e-02;
    PP(3,3) =  3.6495564367004318e-01;
	   
// 		Vector ref = zeros(6);
// 		ref(0) = - 4.2155955213988627e-02;
// 		ref(1) =   1.8015724412870739e+00;
// 		ref(2) =   0.0                   ;
// 		ref(3) =   0.0                   ;
// 		ref(4) =  20.5                   ;
// 		ref(5) = - 0.1                   ;

	   
    // SET UP THE MPC - OPTIMAL CONTROL PROBLEM:
    // ----------------------------------------------------------
//        OCP ocp( 0.0, 12.0, 15 );
       OCP ocp( 0.0, 2.0*M_PI, 10 );

       ocp.minimizeLSQ       ( QQ, RR );
       ocp.minimizeLSQEndTerm( PP    );

       ocp.subjectTo( f );

       ocp.subjectTo(  18.0 <= u1 <= 22.0 );
       ocp.subjectTo( -0.2  <= u2 <= 0.2 );
    // ----------------------------------------------------------


    // DEFINE AN MPC EXPORT MODULE AND GENERATE THE CODE:
    // ----------------------------------------------------------
       MPCexport mpc(ocp);

       mpc.set( INTEGRATOR_TYPE      , INT_RK4      );
       mpc.set( NUM_INTEGRATOR_STEPS , 30           );
       mpc.set( HESSIAN_APPROXIMATION, GAUSS_NEWTON );
// 	   mpc.set( QP_SOLVER, QP_CVXGEN );
	   mpc.set( HOTSTART_QP, YES );
	   mpc.set( GENERATE_TEST_FILE,NO );
// 	   mpc.set( USE_SINGLE_PRECISION,YES );

       if (mpc.exportCode("kite_carousel_export") != SUCCESSFUL_RETURN)
    	   exit( EXIT_FAILURE );
    // ----------------------------------------------------------


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
// 	Vector xx(28);
// 	xx.setZero();
// 
//     xx(0) = -4.2155955213988627e-02;
// 	xx(1) =  1.8015724412870739e+00;
// 	xx(2) =  0.0;
// 	xx(3) =  0.0;
// 
// 	Vector uu(2);
//     uu(0) = 20.5;
// 	uu(1) = -0.1;
// 	
// 	z.setX( xx );
//     z.setU( uu );
// 
//     Vector result = Df.evaluate( z );
// 	result.print( "x" );


    return EXIT_SUCCESS;
}


