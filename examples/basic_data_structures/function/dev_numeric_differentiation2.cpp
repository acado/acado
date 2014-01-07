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
 *    \file examples/integrator/numeric_differentiation2.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date 2008
 */

#include <time.h>
#include <acado_toolkit.hpp>


/* >>> start tutorial code >>> */
void my_function( double *x_, double *f ){

//  double          t =  x_[ 0];    // the time
    double          x =  x_[ 1];    // the first  differential state
    double          y =  x_[ 2];    // the second differential state

    f[0] = x*x + pow(y,3);
}

int main( ){

    USING_NAMESPACE_ACADO

    Function               f   ;

    const int  dim = 1;  // the dimension of the right hand side
    const int  nt  = 1;  // the explicit time dependence
    const int  nx  = 2;  // the number of differential states
    const int  na  = 0;  // the number of algebraic states
    const int  nu  = 0;  // the number of controls
    const int  nv  = 0;  // the number of integer controls
    const int  np  = 0;  // the number of parameters
    const int  nq  = 0;  // the number of integer parameters
    const int  nw  = 0;  // the number of disturbances
    const int  ndx = 0;  // the number of differential state derivatives

    // Code cannot be even compiled!
//    f.setCFunction( dim, nt, nx, na, nu, nv, np, nq, nw, ndx, my_function );
//
//    // TEST THE FUNCTION f:
//    // --------------------
//       int x_index, y_index;
//
//       x_index = f.index(VT_DIFFERENTIAL_STATE,0);
//       y_index = f.index(VT_DIFFERENTIAL_STATE,1);
//
//       double *xx     = new double[f.getNumberOfVariables()+1];
//       double *lambda = new double[f.getNumberOfVariables()+1];
//       double *mu     = new double[f.getNumberOfVariables()+1];
//       double *mu_    = new double[f.getNumberOfVariables()+1];
//       double *ff     = new double[f.getDim()                ];
//       double *df1    = new double[f.getDim()                ];
//       double *df2    = new double[f.getDim()                ];
//       double *ddf    = new double[f.getDim()                ];
//
//       df1[0] = 0.0;
//       df2[0] = 0.0;
//       ddf[0] = 0.0;
//
//       xx[x_index] = 1.0;
//       xx[y_index] = 1.0;
//
//       lambda[x_index] = 0.5;
//       lambda[y_index] = 1.0;
//
//       mu[x_index] = 1.0;
//       mu[y_index] = 0.5;
//
//       mu_[x_index] = 0.0;
//       mu_[y_index] = 0.0;
//
//    // FORWARD DIFFERENTIATION:
//    // (FIRST AND SECOND ORDER DERIVATIVES)
//    // ------------------------------------
//       f.AD_forward(  0, xx, lambda, ff, df1 );
//       f.AD_forward2( 0, mu, mu_, df2, ddf );
//
//    // PRINT THE RESULTS:
//    // ------------------
//       printf("      x  = %10.16e \n",     xx[x_index] );
//       printf("      y  = %10.16e \n",     xx[y_index] );
//       printf("lambda_x = %10.16e \n", lambda[x_index] );
//       printf("lambda_y = %10.16e \n", lambda[y_index] );
//       printf("mu_x     = %10.16e \n",     mu[x_index] );
//       printf("mu_y     = %10.16e \n",     mu[y_index] );
//       printf("      f  = %10.16e \n",     ff[0      ] );
//       printf("     df1 = %10.16e \n",    df1[0      ] );
//       printf("     df2 = %10.16e \n",    df2[0      ] );
//       printf("    ddf  = %10.16e \n",    ddf[0      ] );
//
//    delete[] xx;
//    delete[] lambda;
//    delete[] mu;
//    delete[] mu_;
//    delete[] ff;
//    delete[] df1;
//    delete[] df2;
//    delete[] ddf;

    return 0;
}
/* <<< end tutorial code <<< */


