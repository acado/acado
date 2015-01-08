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
 *    \file examples/integrator/output_function.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date 2008
 */


#include <time.h>
#include <acado_toolkit.hpp>

/* >>> start tutorial code >>> */
int main( ){

    USING_NAMESPACE_ACADO

    // DEFINE VARIABLES:
    // ----------------------
    DifferentialState      x;
    TIME                   t;
    OutputFcn              f;

    f << x*x + t;


    // TEST THE FUNCTION f:
    // --------------------
    VariablesGrid X = readFromFile("evaluation_data.txt");
    VariablesGrid result = X;

    f.evaluate( &X, 0, 0, 0, 0, &result );

    // PRINT THE RESULT:
    // -----------------
    result.print();


    return 0;
}
/* <<< end tutorial code <<< */


