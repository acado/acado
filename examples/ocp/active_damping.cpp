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
 *    \file   examples/ocp/active_damping.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date   2009
 */


#include <acado_optimal_control.hpp>
#include <acado_gnuplot.hpp>


int main( ){

    USING_NAMESPACE_ACADO


    // INTRODUCE THE VARIABLES:
    // -------------------------
	DifferentialState xBody;
	DifferentialState xWheel;
	DifferentialState vBody;
	DifferentialState vWheel;

	Disturbance roadExcitation;

	Control dampingForce;

	double mBody   = 350.0;
	double mWheel  = 50.0;
	double kSpring = 20000.0;
	double kTire   = 200000.0;


    // DEFINE A DIFFERENTIAL EQUATION:
    // -------------------------------
    DifferentialEquation f;

	f << dot(xBody)  == vBody;
	f << dot(xWheel) == vWheel;
	f << dot(vBody)  == ( -kSpring*xBody + kSpring*xWheel + dampingForce ) / mBody;
	f << dot(vWheel) == ( -kTire*xBody - (kTire+kSpring)*xWheel + kTire*roadExcitation - dampingForce ) / mWheel;


    // DEFINE LEAST SQUARE FUNCTION:
    // -----------------------------
    Function h;

    h << xBody;
    h << xWheel;
	h << vBody;
    h << vWheel;

    DMatrix S(4,4);
    DVector r(4);

    S.setIdentity();
	S(0,0) = 10.0;
	S(1,1) = 10.0;

    r.setAll( 0.0 );


    // DEFINE AN OPTIMAL CONTROL PROBLEM:
    // ----------------------------------
    const double t_start = 0.0;
    const double t_end   = 1.0;

    OCP ocp( t_start, t_end, 20 );

    ocp.minimizeLSQ( S, h, r );
	//ocp.minimizeLagrangeTerm( 0.5*(10.0*xBody*xBody + 10.0*xWheel*xWheel + vBody*vBody + vWheel*vWheel ) );

	ocp.subjectTo( f );

	ocp.subjectTo( AT_START, xBody  == 0.01 );
	ocp.subjectTo( AT_START, xWheel == 0.0 );
	ocp.subjectTo( AT_START, vBody  == 0.0 );
	ocp.subjectTo( AT_START, vWheel == 0.0 );

	ocp.subjectTo( -500.0 <= dampingForce <= 500.0 );
	ocp.subjectTo( roadExcitation == 0.0 );


    // Additionally, flush a plotting object
    GnuplotWindow window1;//( PLOT_AT_EACH_ITERATION );
	window1.addSubplot( xBody, "Body Position [m]" );
	window1.addSubplot( xWheel,"Wheel Position [m]" );
	window1.addSubplot( vBody, "Body Velocity [m/s]" );
	window1.addSubplot( vWheel,"Wheel Velocity [m/s]" );

	window1.addSubplot( dampingForce,"Damping Force [N]" );
	window1.addSubplot( roadExcitation,"Road Excitation [m]" );


    // DEFINE AN OPTIMIZATION ALGORITHM AND SOLVE THE OCP:
    // ---------------------------------------------------
    OptimizationAlgorithm algorithm(ocp);

    algorithm << window1;

 // algorithm.set( HESSIAN_APPROXIMATION, EXACT_HESSIAN );
 //  algorithm.set( HESSIAN_APPROXIMATION, GAUSS_NEWTON );
	algorithm.set( HESSIAN_APPROXIMATION, GAUSS_NEWTON_WITH_BLOCK_BFGS );

	//algorithm.set( INTEGRATOR_TOLERANCE, 1e-8 );
	algorithm.set( KKT_TOLERANCE, 1e-6 );
	//algorithm.set( GLOBALIZATION_STRATEGY, GS_FULLSTEP );
	//algorithm.set( MAX_NUM_ITERATIONS, 1 );

	algorithm.solve();


    return 0;
}



