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
 *    \file examples/matrix_vector/cholesky_tutorial.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date 2008
 *
 *    This tutorial example explains how to compute
 *    cholesky factorization with the ACADO
 *    Matrix class.
 */


#include <acado/utils/acado_utils.hpp>
#include <acado/matrix_vector/matrix_vector.hpp>


/* >>> start tutorial code >>> */
int main( ){

    USING_NAMESPACE_ACADO

    // DEFINE A MATRIX:
    // ----------------
    Matrix A(2,2);

    A(0,0) = 3.0;  A(0,1) = 0.5;
    A(1,0) = 1.0;  A(1,1) = 3.0;


    // COMPUTE THE CHOLESKY FACTORISATION OF A:
    // ----------------------------------------

    Matrix L;
    L = A.getCholeskyDecomposition();

    L.print("L");


    // COMPUTE THE INVERSE OF A:
    // -------------------------

    Matrix A_;  // invers of A
    A_ = A.getCholeskyInverse();

    A_.print("A_");


    // CHECK THE ACCURACY OF THE INVERSE:
    // ----------------------------------
    (A*A_).print("A*A_");

    return 0;
}
/* <<< end tutorial code <<< */


