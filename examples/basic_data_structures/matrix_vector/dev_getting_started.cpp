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
 *    \file examples/matrix_vector/getting_started.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date 2008
 */


#include <acado/utils/acado_utils.hpp>
#include <acado/matrix_vector/matrix_vector.hpp>


/* >>> start tutorial code >>> */
int main( ){

    USING_NAMESPACE_ACADO

    // DEFINE SOME VECTORS:
    // --------------------
    DVector a(3), b(3), c;

    a(0) = 1.0; a(1) = 3.0; a(2) = 2.0;
    b(0) = 4.0; b(1) = 2.0; b(2) = 3.0;

    // COMPUTE THE SUM OF a AND b:
    // ---------------------------
    c = a+b;

    printf("\nThe result for a+b is:\n");
    c.print();
    printf("\n");

    printf("\nThe scalar product of a and b is:\n");
    printf("%.16e \n", a^b );
    printf("\n");

    // DEFINE SOME MATRICES:
    // ---------------------
    DMatrix A(2,2), B(2,2);

    A(0,0) = 1.0;  A(0,1) = 1.0;
    A(1,0) = 0.0;  A(1,1) = 2.0;

    B(0,0) = 1.0;  B(0,1) = 0.0;
    B(1,0) = 0.0;  B(1,1) = 3.0;

    printf("\nThe matrix A*B+A is:\n");
    (A*B + A).print();

    printf("\nThe dyadic product of a and b is:\n");
    (a%b).print();
    printf("\n");

    A.appendRows( A );
    A.print();

    return 0;
}
/* <<< end tutorial code <<< */


