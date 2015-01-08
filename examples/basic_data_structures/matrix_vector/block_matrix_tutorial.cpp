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


    // DEFINE SOME MATRICES:
    // ---------------------
    DMatrix A(2,2), B(2,3), C(2,2);

    A(0,0) = 1.0;    A(0,1) = 2.0;
    A(1,0) = 3.0;    A(1,1) = 4.0;

    B(0,0) = 1.0;    B(0,1) = 2.0;    B(0,2) = 3.0;
    B(1,0) = 4.0;    B(1,1) = 5.0;    B(1,2) = 6.0;

    C(0,0) = 1.0;    C(0,1) = 2.0;
    C(1,0) = 4.0;    C(1,1) = 5.0;


    // DEFINE SOME BLOCK MATRICES:
    // ---------------------------
    BlockMatrix M(2,2),N(2,3),P(2,3);

    // -------------------------------------
    // DEFINE A BLOCK MATRIX M OF THE FORM:
    //
    //         ( 1   A  )
    //   M :=  (        )
    //         ( 0   1  )
    //
    // WHERE 1 IS A 2x2 UNIT MATRIX:
    // -------------------------------------
    M.setIdentity(0,0,2);    M.setDense   (0,1,A);
       /* skip */            M.setIdentity(1,1,2);

    // -------------------------------------
    // DEFINE A BLOCK MATRIX N OF THE FORM:
    //
    //         ( 1   B   C )
    //   N :=  (           )
    //         ( 0   B   1 )
    //
    // -------------------------------------
    N.setIdentity(0,0,2);    N.setDense(0,1,B);    N.setDense   (0,2,C);
       /* skip */            N.setDense(1,1,B);    N.setIdentity(1,2,2);


    // PRINT THE MATRICES  M  AND  N :
    // -------------------------------
    printf("M = \n");  M.print();
    printf("N = \n");  N.print();

    // COMPUTE THE MATRIX PRODUCT  MN := M*N :
    // ---------------------------------------
    BlockMatrix MN;  MN = M*N;

    // PRINT THE RESULT FOR MN:
    // ------------------------
    printf("MN = \n");  MN.print();

    // COMPUTE THE MATRIX PRODUCT  MTN := M^T*N :
    // ------------------------------------------
    BlockMatrix MTN;  MTN = M^N;

    // PRINT THE RESULT FOR MN:
    // ------------------------
    printf("MTN = \n");  MTN.print();

    return 0;
}
/* <<< end tutorial code <<< */


