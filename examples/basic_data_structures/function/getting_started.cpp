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
 *    \file examples/integrator/getting_started.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date 2008
 */

#include <time.h>

#include <acado/utils/acado_utils.hpp>
#include <acado/symbolic_expression/symbolic_expression.hpp>
#include <acado/function/function.hpp>


/* >>> start tutorial code >>> */
int main( ){

    USING_NAMESPACE_ACADO

    // DEFINE VARIABLES:
    // ----------------------
    DifferentialState      x;
    TIME                   t;
    Function               f;

    f << pow(x,2) + t       ;
    f << pow(x,2)           ;


    // TEST THE FUNCTION f:
    // ---------------------------------------
    EvaluationPoint z(f);

    DVector xx(1);
    xx(0) = 2.0;
    double tt = 1.0;

    z.setT( tt );
    z.setX( xx );

    DVector result = f.evaluate( z );


    // PRINT THE RESULT:
    // -----------------
    printf("\nf( %.1e , %.1e ) = ", tt, xx(0) );
    result.print();

    printf("\n\n");
    printf("the dimension of f is      %d                      \n",  f.getDim() );
    printf("the function f depends on  %d  differential states \n",  f.getNX () );
    printf("the function f depends on  %d  controls            \n",  f.getNU () );
    printf("the function f depends on  %d  parameters          \n",  f.getNP () );

    return 0;
}
/* <<< end tutorial code <<< */


