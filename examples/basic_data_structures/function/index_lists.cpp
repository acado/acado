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
 *    \file examples/integrator/intermediate_states.cpp
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

    // DEFINE VALRIABLES:
    // ---------------------------
       DifferentialState x, y;
       IntermediateState z   ;
       Function f;

       z = x*y;
       f << z + sin(z);

    // TEST THE FUNCTION f:
    // --------------------
       int x_index, y_index, z_index;

       x_index = f.index(VT_DIFFERENTIAL_STATE,0);
       y_index = f.index(VT_DIFFERENTIAL_STATE,1);
       z_index = f.index(VT_INTERMEDIATE_STATE,0);

       double *xx = new double[f.getNumberOfVariables()+1];
       double *result = new double[ f.getDim() ];

       xx[x_index] = 2.0;
       xx[y_index] = 2.0;

    // EVALUATE f AT THE POINT  (xx):
    // ------------------------------
       f.evaluate( 0, xx, result );

    // PRINT THE RESULTS:
    // ------------------
       printf("x = %10e \n",     xx[x_index] );
       printf("y = %10e \n",     xx[y_index] );
       printf("z = %10e \n",     xx[z_index] );
       printf("f = %10e \n", result[0      ] );

    delete[] xx;
    delete[] result;

    return 0;
}



