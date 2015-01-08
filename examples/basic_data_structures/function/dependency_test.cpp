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
 *    \file examples/integrator/dependency_test.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
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
    // -------------------------------------
    DifferentialState      x, y, z;
    IntermediateState      a      ;
    TIME                   t      ;
    Function               f      ;

    a = t*sin(y);

    f << x*x*a + t     ;
    f << x/(y*y) + x*x ;
    f << x*y/(z*z+1.0) ;

    if( f.isDependingOn( x ) == BT_TRUE )
             printf("f depends on x          \n");
    else     printf("f is not depending on x \n");

    if( f.isLinearIn( x ) == BT_TRUE )
             printf("f is linear in x     \n");
    else     printf("f is not linear in x \n");

    if( f.isPolynomialIn( x ) == BT_TRUE )
             printf("f is polynomial in x     \n");
    else     printf("f is not polynomial in x \n");

    if( f.isRationalIn( x ) == BT_TRUE )
             printf("f is rational in x       \n");
    else     printf("f is transcendental in x \n");

    if( f.isRationalIn( y ) == BT_TRUE )
             printf("f is rational in y       \n");
    else     printf("f is transcendental in y \n");

    if( f.isPolynomialIn( z ) == BT_TRUE )
             printf("f is polynomial in z     \n");
    else     printf("f is not polynomial in z \n");

    if( f.isRationalIn( z ) == BT_TRUE )
             printf("f is rational in z       \n");
    else     printf("f is transcendental in z \n");

    if( f.isLinearIn( t ) == BT_TRUE )
             printf("f is linear in t     \n");
    else     printf("f is not linear in t \n");


    return 0;
}
/* <<< end tutorial code <<< */
