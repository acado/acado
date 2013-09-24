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
 *    \file examples/integrator/automatic_forward_differentiation.cpp
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
    // ---------------------------
       DifferentialState x, y;
       Function f;

       f << x*x + pow(y,2);

    // TEST THE FUNCTION f:
    // --------------------
       EvaluationPoint  z(f);
       EvaluationPoint dz(f);

       Vector xx(2);  Vector dx(2);

       xx(0) =  1.0;  dx(0) =  0.5;
       xx(1) =  1.0;  dx(1) =  0.1;

       z.setX( xx );  dz.setX( dx );


    // FORWARD DIFFERENTIATION:
    // ------------------------
       Vector ff = f.evaluate  ( z  );
       Vector df = f.AD_forward( dz );


    // PRINT THE RESULTS:
    // ------------------
       ff.print("result of evaluation      \n");
       df.print("result for the derivative \n");

    return 0;
}
/* <<< end tutorial code <<< */


