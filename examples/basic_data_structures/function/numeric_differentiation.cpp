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
 *    \file examples/integrator/numeric_differentiation.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date 2008
 */

#include <time.h>

#include <acado/utils/acado_utils.hpp>
#include <acado/user_interaction/user_interaction.hpp>
#include <acado/symbolic_expression/symbolic_expression.hpp>
#include <acado/function/function.hpp>


/* >>> start tutorial code >>> */
void my_function( double *x_, double *f, void *user_data ){

//  double          t =  x_[ 0];    // the time
    double          x =  x_[ 1];    // the first  differential state
    double          y =  x_[ 2];    // the second differential state

    f[0] = x*x + pow(y,3);
}



int main( ){

    USING_NAMESPACE_ACADO

    DifferentialState a,b;
    TIME t;

    CFunction myFunction( 1, my_function );

    IntermediateState x(3);

    x(0) = t;
    x(1) = a;
    x(2) = b;

    Function f;
    f << myFunction( x );


    // TEST THE FUNCTION f:
    // --------------------
       int x_index, y_index;

       x_index = f.index(VT_DIFFERENTIAL_STATE,0);
       y_index = f.index(VT_DIFFERENTIAL_STATE,1);

       double *xx    = new double[f.getNumberOfVariables()+1];
       double *seed  = new double[f.getNumberOfVariables()+1];
       double *ff    = new double[f.getDim()                ];
       double *df    = new double[f.getDim()                ];

       xx[x_index] = 1.0;
       xx[y_index] = 1.0;

       seed[x_index] = 0.5;
       seed[y_index] = 0.5;


    // FORWARD DIFFERENTIATION:
    // ------------------------
       f.evaluate  ( 0, xx  , ff );
       f.AD_forward( 0, seed, df );


    // PRINT THE RESULTS:
    // ------------------
       printf("     x = %10.16e \n",   xx[x_index] );
       printf("     y = %10.16e \n",   xx[y_index] );
       printf("seed_x = %10.16e \n", seed[x_index] );
       printf("seed_y = %10.16e \n", seed[y_index] );
       printf("     f = %10.16e \n",   ff[0      ] );
       printf("    df = %10.16e \n",   df[0      ] );

    delete[] xx;
    delete[] seed;
    delete[] ff;
    delete[] df;

    return 0;
}
/* <<< end tutorial code <<< */


