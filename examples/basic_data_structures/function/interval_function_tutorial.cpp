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
 *    \file examples/basic_data_structures/interval_function_tutorial.cpp
 *    \author Boris Houska
 *    \date 2013
 */

#include <time.h>

#include <acado/utils/acado_utils.hpp>
#include <acado/symbolic_expression/symbolic_expression.hpp>
#include <acado/function/function.hpp>
#include <acado/set_arithmetics/set_arithmetics.hpp>


/* >>> start tutorial code >>> */
int main( ){

    USING_NAMESPACE_ACADO

    // DEFINE VARIABLES:
    // ----------------------
    Parameter  x,y,z;
    Function       f;

	f << x+y*z;
	f << x*x+y;


    // TEST THE FUNCTION f:
    // ---------------------------------------
    TevaluationPoint<Interval> e(f);

    Tmatrix<Interval> p(3);
    p(0) = Interval(-1.0, 1.0 );
	p(1) = Interval( 0.0, 2.0 );
	p(2) = Interval( 0.0, 0.1 );

    e.setP( p );

    Tmatrix<Interval> result = f.evaluate( e );

    std::cout << "result:\n" << result << "\n";

    return 0;
}
/* <<< end tutorial code <<< */


