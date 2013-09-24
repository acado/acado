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
 *    \file examples/integrator/automatic_backward_differentiation.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date 2008
 */

#include <time.h>

#include <acado/utils/acado_utils.hpp>
#include <acado/user_interaction/user_interaction.hpp>
#include <acado/symbolic_expression/symbolic_expression.hpp>
#include <acado/function/function.hpp>


USING_NAMESPACE_ACADO

/* >>> start tutorial code >>> */
int main( ){

    USING_NAMESPACE_ACADO

    // DEFINE VALRIABLES:
    // ---------------------------
    DifferentialState x,y;
   
 Function f;

    f << (x+1)*(y+1) + y*x*y; //pow(y,3);
    f << x           ;
    f << y           ;


    // EVALUATE THE FUNCTION f:
    // ------------------------
    EvaluationPoint z(f);

    Vector diffState(2);

    diffState(0) = 1.0;
    diffState(1) = 2.0;

    z.setX( diffState );

    Vector ff = f(z);

    ff.print();


    // COMPUTE THE BACKWARD DERIVATIVE:
    // --------------------------------

    Vector seed(f.getDim());

    seed(0) = 1.0;
    seed(1) = 0.0;
    seed(2) = 0.0;

    EvaluationPoint df(f);

    f.AD_backward( seed, df );

    (df.getX()).print("df");


    return 0;
}
/* <<< end tutorial code <<< */


