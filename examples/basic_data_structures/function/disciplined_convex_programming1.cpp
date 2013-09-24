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
 *    \file examples/function/disciplined_convex_programming1.cpp
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

    int i;

    // DEFINE VALRIABLES:
    // ---------------------------
    Matrix                 A(3,3);
    Vector                 b(3);
    DifferentialState      x(3);
    DifferentialState      y;
    Function               f[7];

    // DEFINE THE VECTOR AND MATRIX ENTRIES:
    // -------------------------------------
    A.setZero() ;
    A(0,0) = 1.0;  A(1,1) = 2.0;  A(2,2) = 3.0;
    b(0)   = 1.0;  b(1)   = 1.0;  b(2)   = 1.0;


    // DEFINE TEST FUNCTIONS:
    // -----------------------
    f[0] << A*x + b;
    f[1] << y + euclidean_norm(A*x + b);
    f[2] << y*y;
    f[3] << square(y);
    f[4] << 5.0*log_sum_exp( x );
//    f[5] << entropy( y );
    f[6] << -sum_square( x );


    for( i = 0; i < 7; i++ ){
        if( f[i].isConvex() == BT_TRUE )
            printf("f[%d] is convex. \n", i );
        else{
            if( f[i].isConcave() == BT_TRUE )
                  printf("f[%d] is concave. \n", i );
            else
                  printf("f[%d] is neither convex nor concave. \n", i );
        }
    }
    return 0;
}
/* <<< end tutorial code <<< */
