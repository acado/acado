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
 *    \file examples/basic_data_structures/variables_grid/getting_started.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *
 */

 
#include <acado_optimal_control.hpp>


/* >>> start tutorial code >>> */
int main( )
{
	USING_NAMESPACE_ACADO

	VariablesGrid grid1( 1,0.0,5.0,6 );
	VariablesGrid grid2( 1,0.0,8.0,9 );

	for( int i=0; i<6; ++i )
		grid1( i,0 ) = (double)i;

	for( int i=0; i<9; ++i )
		grid2( i,0 ) = (double)i*2.0;

	grid1.print("grid1");
	grid2.print("grid2");
	
	grid2.merge( grid1, MM_REPLACE, BT_TRUE );
	grid2.print("modified grid1"); 
	
    return 0;
}
/* <<< end tutorial code <<< */


