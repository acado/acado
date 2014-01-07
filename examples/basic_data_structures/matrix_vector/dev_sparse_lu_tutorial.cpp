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
 *    \file examples/matrix_vector/sparse_lu_tutorial.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date 2010
 *
 *    This tutorial example explains how to compute
 *    sparse LU factorizations with the ACADO
 *    DMatrix class.
 */


#include <acado/utils/acado_utils.hpp>
#include <acado/matrix_vector/matrix_vector.hpp>


/* >>> start tutorial code >>> */
int main( ){

    USING_NAMESPACE_ACADO

    int i;

    // DEFINE A MATRIX:
    // ----------------
    DMatrix A = readFromFile("J.txt");

    const int m = A.getNumRows();

    DVector b(m);

    for( i = 0; i < m; i++ )
        b(i) = 1.0;

    DMatrix B = A;


    double t2 = -acadoGetTime();
    A.computeSparseLUdecomposition();
    t2 += acadoGetTime();
    DVector res2 = b - B*A.solveSparseLU(b);

    printf("LU:  TIME = %.16e   ,  error = %.16e \n", t2, res2.getNorm( VN_LINF ) );

    DVector res3 = b - (B^A.solveTransposeSparseLU(b));
    printf("LU:  (transpose)   ,  error = %.16e \n", res3.getNorm( VN_LINF ) );


//     double t1 = -acadoGetTime();
    A.computeQRdecomposition();
//     t1 += acadoGetTime();
//     DVector res1 = b - B*A.solveQR(b);
// 
//     printf("QR:  TIME = %.16e   ,  error = %.16e \n", t1, res1.getNorm( VN_LINF ) );

    return 0;
}
/* <<< end tutorial code <<< */


