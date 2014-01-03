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
 *    \file examples/function/disciplined_convex_programming1.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date 2008
 */

#include <time.h>
#include <acado_toolkit.hpp>

/* >>> start tutorial code >>> */
int main( ){

    USING_NAMESPACE_ACADO

    int i;

    // DEFINE VALRIABLES:
    // ---------------------------
    DifferentialState         x;
    IntermediateState         z;
    Function               f[7];

    // DEFINE TEST FUNCTIONS:
    // -----------------------
    z     = 0.5*x + 1.0;

    f[0] << sqrt(z);
    f[1] << z*sqrt(z);
    f[2] << pow(z,3.0/2.0);
    f[3] << pow(z,2.0/3.0);
    f[4] << pow(2.0,z);
    f[5] << exp(z+exp(z));
    f[6] << pow(0.5,entropy(x)-z);

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
