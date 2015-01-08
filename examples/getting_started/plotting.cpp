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
 *    \file   examples/getting_started/simple_ocp.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date   2009
 */


#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>


int main( ){

    USING_NAMESPACE_ACADO


	// DEFINE SAMPLE OCP
	// -----------------
	DifferentialState x, y;
	Control           u;

	DifferentialEquation f;
	
	f << dot(x) == y;
    f << dot(y) == u;

    OCP ocp( 0.0, 5.0 );
    ocp.minimizeLagrangeTerm( x*x + 0.01*u*u );

    ocp.subjectTo( f );
    ocp.subjectTo( AT_START, x ==  1.0 );
    ocp.subjectTo( AT_START, y ==  0.0 );

	ocp.subjectTo( -1.0 <= u <=  1.0 );


	// SETUP OPTIMIZATION ALGORITHM AND DEFINE VARIOUS PLOT WINDOWS
	// ------------------------------------------------------------
	OptimizationAlgorithm algorithm( ocp );

	GnuplotWindow window1;
	window1.addSubplot( x, "1st state" );
	window1.addSubplot( y, "2nd state","x label","y label",PM_POINTS );
	window1.addSubplot( u, "control input","x label","y label",PM_LINES,0,5,-2,2 );
	window1.addSubplot( 2.0*sin(x)+y*u, "Why not plotting fancy stuff!?" );
// 	window1.addSubplot( PLOT_KKT_TOLERANCE, "","iteration","KKT tolerance" );
	
	GnuplotWindow window2( PLOT_AT_EACH_ITERATION );
	window2.addSubplot( x, "1st state evolving..." );
	
	algorithm << window1;
	algorithm << window2;
	
	algorithm.solve( );


	// For illustration, an alternative way to plot differential states
	// in form of a VariablesGrid
	VariablesGrid states;
	algorithm.getDifferentialStates( states );
	
	GnuplotWindow window3;
	window3.addSubplot( states(0), "1st state obtained as VariablesGrid" );
	window3.addSubplot( states(1), "2nd state obtained as VariablesGrid" );
	window3.plot();

	VariablesGrid data( 1, 0.0,10.0,11 );
	data.setZero();
	data( 2,0 ) = 1.0;
	data( 5,0 ) = -1.0;
	data( 8,0 ) = 2.0;
	data( 9,0 ) = 2.0;
	
// 	data.setType( VT_CONTROL );
	
	GnuplotWindow window4;
	window4.addSubplot( data, "Any data can be plotted" );
	window4.plot();


    return 0;
}
