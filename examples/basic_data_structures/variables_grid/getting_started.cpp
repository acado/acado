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
 *    \file examples/basic_data_structures/variables_grid/getting_started.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *
 */

 
#include <acado_optimal_control.hpp>


/* >>> start tutorial code >>> */
int main( )
{
	USING_NAMESPACE_ACADO

	// Setup an equidistant VariablesGrid with 5 grid points
	// and vectors of dimension 2 at each grid point and assign values
	double tStart =  0.0;
	double tEnd   =  2.0;

	VariablesGrid equidistantGrid( 2,tStart,tEnd,5 );

	equidistantGrid.setZero();

	Vector v( 2 );
	v(0) = 1.0;
	v(1) = 2.0;
	equidistantGrid.setVector( 1,v );
	equidistantGrid.setVector( 2,v );
	
	v.setAll( 5.0 );
	equidistantGrid.setVector( 3,v );
	equidistantGrid.setVector( 4,v );

	printf( "The grid consists of the following grid points:\n" );
	equidistantGrid.print();
	printf( "Its number of grid points is:  %d\n",equidistantGrid.getNumPoints() );
	printf( "Each vector has dimension:     %d\n",equidistantGrid.getNumValues() );

	// Construct another VariablesGrid from file
	VariablesGrid gridFromFile( "./data.txt" );

	printf( "\nThe second grid consists of the following grid points:\n" );
	gridFromFile.print();


	// Append (in time) grid points of second VariablesGrid to first one
	equidistantGrid.appendTimes( gridFromFile );

	printf( "\nNow, the grid consists of the following grid points:\n" );
	equidistantGrid.print();
	printf( "Its number of grid points is:  %d\n",equidistantGrid.getNumPoints() );
	printf( "Each vector has dimension:     %d\n",equidistantGrid.getNumValues() );


	// Setup a third grid with identical grid points as modified first one,
	// and comprising Vectors of dimension 1...
	VariablesGrid thirdGrid( 1,equidistantGrid );

	// ... and initialise these vectors
	for( unsigned int i=0; i<thirdGrid.getNumPoints( ); ++i )
		thirdGrid( i,0 ) = (double)i;
	
	printf( "\nThe third grid consists of the following grid points:\n" );
	thirdGrid.print();


	// Append values of all grid points of third VariablesGrid to ones of first VariablesGrid
	equidistantGrid.appendValues( thirdGrid );

	printf( "\nNow, the grid consists of the following grid points:\n" );
	equidistantGrid.print();
	printf( "Its number of grid points is:  %d\n",equidistantGrid.getNumPoints() );
	printf( "Each vector has dimension:     %d\n",equidistantGrid.getNumValues() );


    return 0;
}
/* <<< end tutorial code <<< */


