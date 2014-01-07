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
 *    \file examples/basic_data_structures/variables_grid/getting_started_grid.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *
 */

#include <acado_integrators.hpp>

using namespace std;

USING_NAMESPACE_ACADO

/* >>> start tutorial code >>> */
int main( )
{
	// Setup an equidistant Grid with 5 grid points
	double tStart =  0.0;
	double tEnd   =  2.0;

	Grid firstGrid( tStart,tEnd,5 );

	cout << "The grid consists of the following grid points:\n";
	firstGrid.print();
	cout << "Its third grid point has time:  " << firstGrid.getTime( 2 ) << endl;


	// Add additional grid point at time 7
	// (time must be greater than time at last grid point)
	firstGrid.addTime( 7.0 );

	cout << "\nNow, the grid consists of the following grid points:\n";
	firstGrid.print();
	cout << "Its interval length is:  " <<  firstGrid.getIntervalLength() << endl;


	// Setup an arbitrary grid with 3 grid points
	Grid secondGrid( 3 );
	
	secondGrid.setTime( 0,-1.0 );
	secondGrid.setTime( 1,1.0 );
	secondGrid.setTime( 2,5.0 );

	cout << "\nThe second grid consists of the following grid points:\n";
	secondGrid.print();

	// Add grid points of first grid to that of the second one
	secondGrid& firstGrid;

	cout << "\nNow, the second grid consists of the following grid points:\n";
	secondGrid.print();
	cout << "Its interval length is:         " << secondGrid.getIntervalLength() << endl
		 << "Its third grid point has time:  " << secondGrid.getTime( 2 ) << endl;

    return 0;
}
/* <<< end tutorial code <<< */


