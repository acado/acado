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
 *    \file   examples/parameters_estimation/michaelis_menten.cpp
 *    \author Boris Houska, Filip Logist
 *    \date   2010
 *
 *    This example originates from
 *    http://en.wikipedia.org/wiki/Gauss-Newton_algorithm.
 */


#include <acado_optimal_control.hpp>
#include <acado_gnuplot.hpp>


int main( ){

    USING_NAMESPACE_ACADO

    // INTRODUCE THE VARIABLES:
    // -------------------------
    Parameter               V ;
    Parameter               km;


    // READ THE MEASUREMENT FROM A DATA FILE:
    // --------------------------------------
    DMatrix m; m.read( "michaelis_menten_data.txt" );


    // DEFINE A MEASUREMENT FUNCTION:
    // ------------------------------
    Function h;  // the measurement function

    int i;
    for( i = 0; i < (int) m.getNumRows(); i++ )
        h << V*m(i,0)/(km + m(i,0)) - m(i,1);


    // DEFINE A PARAMETER ESTIMATION PROBLEM:
    // --------------------------------------
    NLP nlp;
    nlp.minimizeLSQ( h );

    nlp.subjectTo( 0.0 <= V  <= 2.0 );
    nlp.subjectTo( 0.0 <= km <= 2.0 );


    // DEFINE AN OPTIMIZATION ALGORITHM AND SOLVE THE ESTIMATION PROBLEM:
    // ------------------------------------------------------------------
    ParameterEstimationAlgorithm algorithm(nlp);
    algorithm.solve();


    VariablesGrid parameters;
    algorithm.getParameters( parameters );

	return 0;
	
    // GET THE VARIANCE COVARIANCE IN THE SOLUTION:
    // ---------------------------------------------
    DMatrix var;
    algorithm.getParameterVarianceCovariance( var );

    double LSSE = 2.0*algorithm.getObjectiveValue();
    double MSE  = LSSE/( m.getNumRows() - 2.0 );   // m.getNumRows() == number of measurements
                                                   // 2              == number of parameters

    var *=  MSE;  // rescale the variance-covariance with the MSE factor.


    // PRINT THE RESULT ON THE TERMINAL:
    // -----------------------------------------------------------------------
	printf("\n\nResults for the parameters: \n");
	printf("-----------------------------------------------\n");
	printf("   V   =  %.3e  +/-  %.3e \n", parameters(0,0), sqrt( var(0,0) ) );
	printf("   km  =  %.3e  +/-  %.3e \n", parameters(0,1), sqrt( var(1,1) ) );
	printf("-----------------------------------------------\n\n\n");

    return 0;
}



