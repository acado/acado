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
 *    \file examples/function/symbolic_differentiation3.cpp
 *    \author Boris Houska, Hans Joachim Ferreau, Joris Gillis
 *    \date 2008
 */

#include <time.h>

#include <acado/utils/acado_utils.hpp>
#include <acado/user_interaction/user_interaction.hpp>
#include <acado/symbolic_expression/symbolic_expression.hpp>
#include <acado/function/function.hpp>


/* >>> start tutorial code >>> */
int main( ){

    USING_NAMESPACE_ACADO

    // DEFINE VALRIABLES:
    // ---------------------------
    DifferentialState x(2);
    DifferentialState y(2);

    Function             f;

    // DEFINE A TEST FUNCTION:
    // -----------------------

    IntermediateState ff;
    ff = sin(x(0)*x(1));

    f <<     forwardDerivative(  ff + x(0)   , x(0) );
    f <<     forwardDerivative(  x(1) + ff  , x(1) );
    f <<     forwardDerivative(  ff + ff  , x );
    f <<     forwardDerivative(  ff + ff  , x, y );
    f <<     forwardDerivative(  ff + ff  , x, dot(x) );
    //f << dot(sin(x(0)*x(1)));

    FILE *file = fopen("symbolic_differentiation3_output.txt", "w" );
    file << f;
    fclose(file);

    return 0;
}
/* <<< end tutorial code <<< */

