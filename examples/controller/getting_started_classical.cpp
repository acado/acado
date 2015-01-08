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
 *    \file   examples/controller/getting_started.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date   2010
 */


#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>


int main( )
{
    USING_NAMESPACE_ACADO


    // SETTING UP THE FEEDBACK CONTROLLER:
    // -----------------------------------
	PIDcontroller pid( 4,1,0.01 );

	DVector pWeights( 4 );
	pWeights(0) = 1000.0;
	pWeights(1) = -1000.0;
	pWeights(2) = 1000.0;
	pWeights(3) = -1000.0;

	DVector dWeights( 4 );
	dWeights(0) = 0.0;
	dWeights(1) = 0.0;
	dWeights(2) = 20.0;
	dWeights(3) = -20.0;

	pid.setProportionalWeights( pWeights );
	pid.setDerivativeWeights( dWeights );

	pid.setControlLowerLimit( 0,-200.0 );
	pid.setControlUpperLimit( 0, 200.0 );


// 	DMatrix K( 1,4 );
// 	K(0,0) = -3.349222044080232e+04;
// 	K(0,1) = -3.806600292165519e+03;
// 	K(0,2) =  9.999999999999985e+02;
// 	K(0,3) = -1.040810121403324e+03;
// 
// 	LinearStateFeedback lqr( K,0.025 );
// 
// 	lqr.setControlLowerLimit( 0,-200.0 );
// 	lqr.setControlUpperLimit( 0, 200.0 );


	StaticReferenceTrajectory zeroReference;

	Controller controller( pid,zeroReference );
// 	Controller controller( lqr,zeroReference );


	// INITIALIZE CONTROLLER AND PERFORM A STEP:
	// -----------------------------------------
	DVector y( 4 );
	y.setZero( );
	y(0) = 0.01;

	controller.init( 0.0,y );
	controller.step( 0.0,y );


	DVector u;
	controller.getU( u );
	u.print( "Feedback control" );

    return 0;
}



