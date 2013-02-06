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
 *    \file examples/basic_data_structures/function/taylor_model_tutorial.cpp
 *    \author Boris Houska
 *    \date 2013
 */

#include <acado/set_arithmetics/interval.hpp>
#include <acado/set_arithmetics/taylor_model.hpp>

USING_NAMESPACE_ACADO

typedef Interval I;
typedef TaylorModel<I> TM;
typedef TaylorVariable<I> TV;

/* >>> start tutorial code >>> */
int main( ){
	
  // TEST Taylor model arithmetics:
  // ---------------------------------------------
    TM Mod( 2, 1 );

    I XI = I(  0.01, 0.02 );
    I YI = I( -0.2, 0.2 );

    TV X( &Mod, 0, XI );
	TV Y( &Mod, 1, YI );

    TV Z1 = 1.0/X;
	TV Z2 = X*Y + sin(X);

	std::cout << Z1;
	std::cout << Z2;

	return 0;
}
/* <<< end tutorial code <<< */


