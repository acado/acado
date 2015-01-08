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
 *    \file examples/integrator/simple_function_test.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date 2008
 */

#include <time.h>

#include <acado/utils/acado_utils.hpp>
#include <acado/user_interaction/user_interaction.hpp>
#include <acado/symbolic_expression/symbolic_expression.hpp>
#include <acado/function/function.hpp>


int main( ){

    USING_NAMESPACE_ACADO


    // DEFINE A VALRIABLE:  (e.g. a differential state)
    // ------------------------------------------------
    DifferentialState      x,y;
    Parameter              p,q;

    // DEFINE THE RIGHT-HAND-SIDE-FUNCTION:
    // ------------------------------------
    Function f         ;

    f  <<  x + p + q   ;
    f  << -x           ;
    f  <<  x+y         ;
    f  <<  x-y         ;
    f  <<  x*y         ;
    f  <<  x/y         ;
    f  <<  pow(x,3)    ;

    f  <<  pow(x,y)    ;
    f  <<  exp(x)      ;

    f  <<  ln(x)       ;

    f  <<  sin(x)      ;
    f  <<  cos(x)      ;
    f  <<  tan(x)      ;

    f  <<  asin(x)     ;
    f  <<  acos(x)     ;
    f  <<  atan(x)     ;

    f  <<  sqrt(x)     ;


//     TEST THE FUNCTION f:
//     --------------------

    printf("dim = %d \n", f.getDim() );
    printf("nx  = %d \n", f.getNX() );
    printf("np  = %d \n", f.getNP() );


    f.substitute( VT_PARAMETER, 0, 1.0 );

    printf("dim = %d \n", f.getDim() );
    printf("nx  = %d \n", f.getNX() );
    printf("np  = %d \n", f.getNP() );



       int x_index, y_index, q_index;

       x_index = f.index(VT_DIFFERENTIAL_STATE,0);
       y_index = f.index(VT_DIFFERENTIAL_STATE,1);
       q_index = f.index(VT_PARAMETER         ,0);

       double *xx = new double[f.getNumberOfVariables()+1];
       double *result = new double[ f.getDim() ];

       xx[x_index] = 0.1;
       xx[y_index] = 0.2;
       xx[q_index] = 2.0;

       f.evaluate( 0, xx, result );

    // PRINT THE RESULTS:
    // ------------------

       int run1;

       for( run1 = 0; run1 < f.getDim(); run1++ )
           printf("f = %10e \n", result[run1] );


    // 1.0 <= x <= 2.0;

       delete[] xx;
       delete[] result;
}



