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
 *    \file   examples/ocp/parameter_estimation_tutorial.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date   2009
 */


#include <acado_optimal_control.hpp>
#include <acado_gnuplot.hpp>


int main( ){

    USING_NAMESPACE_ACADO


    // INTRODUCE THE VARIABLES:
    // -------------------------

    DifferentialState      phi;    // the angle phi
    DifferentialState    omega;    // the first derivative of phi w.r.t. time

    Parameter                l;    // the length of the pendulum
    Parameter            alpha;    // frictional constant
    Parameter                g;    // the gravitational constant


    // DEFINE A DIFFERENTIAL EQUATION:
    // -------------------------------

    DifferentialEquation f;

    f << dot(phi ) == omega;
    f << dot(omega) == -(g/l)*sin(phi) - alpha*omega;

    // REMARK: Note that the parameters g, and l are not independent.
    //         Only one of these parameters can be estimated from
    //         the measurements of the dynamic motion.
    // -----------------------------------------------------------------


    // DEFINE A MEASUREMENT FUNCTION:
    // ------------------------------

    Function h;
    h <<   phi;  // The state phi is being measured.


   // DEFINE THE INVERSE OF THE VARIANCE-COVARIANCE MATRIX OF THE MEASUREMENTS:
   // -------------------------------------------------------------------------
    DMatrix S(1,1);
    S(0,0) = 1.0/pow(0.1,2);  // (1 over the variance of the measurement)  HERE: the standard deviation of the measurement is assumed to be 0.1, thus S = 1/(0.1)^2.


    // READ THE MEASUREMENT FROM A DATA FILE:
    // --------------------------------------

    VariablesGrid measurements;
    measurements = readFromFile( "parameter_estimation_data.txt" );

    if( measurements.isEmpty() == BT_TRUE )
        printf("The file \"parameter_estimation_data.txt\" can't be opened.");


    // DEFINE A PARAMETER ESTIMATION PROBLEM:
    // --------------------------------------
    OCP ocp( measurements.getTimePoints() );

    ocp.minimizeLSQ( h, measurements );

    ocp.subjectTo( f );

    ocp.subjectTo( 0.0 <= alpha <= 4.0  );
    ocp.subjectTo( 0.0 <=   l   <= 2.0  );

    ocp.subjectTo( g == 9.81 );


    // SETUP AN PLOT WINDOW:
    // ---------------------------------------------------
	GnuplotWindow window( PLOT_NEVER );

	window.addSubplot( phi,   "The angle phi", "time [s]", "angle [rad]" );
	window.addSubplot( omega, "The angular velocity omega" );
	window.addSubplot( l,     "The length of the pendulum l" );
	window.addSubplot( alpha, "Frictional constant alpha" );


    // DEFINE AN OPTIMIZATION ALGORITHM AND SOLVE THE OCP:
    // ---------------------------------------------------
    ParameterEstimationAlgorithm algorithm(ocp);

	algorithm << window;
//     algorithm.initializeDifferentialStates( "parameter_estimation_data2.txt" );

    algorithm.solve();


    // GET THE OPTIMAL PARAMETERS:
    // -----------------------------------
    VariablesGrid parameters;
    algorithm.getParameters( parameters );


// 	return 0;

	// GET THE VARIANCE COVARIANCE IN THE SOLUTION:
	// ---------------------------------------------
	DMatrix var;
	algorithm.getParameterVarianceCovariance( var );

// 	return 0;

	// PRINT THE RESULT ON THE TERMINAL:
	// -----------------------------------------------------------------------

	printf("\n\nResults for the parameters: \n");
	printf("-----------------------------------------------\n");
	printf("   l      =  %.3e  +/-  %.3e \n", parameters(0,0), sqrt( var(0,0) )  );
	printf("   alpha  =  %.3e  +/-  %.3e \n", parameters(0,1), sqrt( var(1,1) )  );
	printf("   g      =  %.3e  +/-  %.3e \n", parameters(0,2), sqrt( var(2,2) )  );
	printf("-----------------------------------------------\n\n\n");


    // PLOT THE RESULT:
    // ---------------------------------------------------
    algorithm.getPlotWindow( window );

	window.addData( 0, measurements(0) );
	window.addLine( 2, parameters(0,0) + sqrt( var(0,0) ) );
	window.addLine( 2, parameters(0,0) - sqrt( var(0,0) ) );
	window.addLine( 3, parameters(0,1) + sqrt( var(1,1) ) );
	window.addLine( 3, parameters(0,1) - sqrt( var(1,1) ) );

	window.plot( );

    return 0;
}
