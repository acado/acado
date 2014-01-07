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
 *    \file examples/basic_data_structures/function/taylor_model_tutorial.cpp
 *    \author Boris Houska
 *    \date 2013
 */

#include <acado/set_arithmetics/set_arithmetics.hpp>

USING_NAMESPACE_ACADO

typedef TaylorModel<Interval> MyTM;
typedef TaylorVariable<Interval> MyTV;


/* >>> start tutorial code >>> */
int main( ){
	
  // TEST Taylor model arithmetics:
  // ---------------------------------------------
    MyTM Mod( 2, 3 );

    Interval XI = Interval(  0.5, 1.0 );
    Interval YI = Interval( -0.2, 0.2 );

    MyTV X( &Mod, 0, XI );
	MyTV Y( &Mod, 1, YI );

    MyTV Z1 = pow( pow(X,2) , 2 );
	MyTV Z2 = log(X) + Y;// + X*Y + sin(X);

	std::cout << Z1;
	std::cout << Z2;

	return 0;
}
/* <<< end tutorial code <<< */


