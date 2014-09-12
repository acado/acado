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
 *    \file examples/matrix_vector/svd_tutorial.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date 2008
 *
 *    This tutorial example explains how to compute
 *    singular value decompositions with the ACADO
 *    DMatrix class.
 */


#include <acado/utils/acado_utils.hpp>
#include <acado/matrix_vector/matrix_vector.hpp>

using namespace std;
using namespace Eigen;

USING_NAMESPACE_ACADO

/* >>> start tutorial code >>> */
int main( ){



    // DEFINE A MATRIX:
    // ----------------
    DMatrix A(3,2);

    A(0,0) = 1.0;  A(0,1) = 0.0;
    A(1,0) = 0.0;  A(1,1) = 3.0;
    A(2,0) = 0.0;  A(2,1) = 2.0;


//  ----------------------------------------------
//  Compute the singular value decomposition of A:
//
//               A = U D V^T
//
//  where U and V are orthogonal and D a diagonal
//  matrix.
//  ----------------------------------------------

    JacobiSVD< MatrixXd > svdA(A, ComputeThinU | ComputeThinV );

    DMatrix U = svdA.matrixU();
    DMatrix V = svdA.matrixV();
    DVector D = svdA.singularValues();

    cout << "U = " << endl << U << endl;
    cout << "D = " << endl << D << endl;
    cout << "V = " << endl << V << endl;


    // DEFINE ANOTHER MATRIX:
    // ----------------------
    DMatrix B(2,3);

    B(0,0) = 1.0;   B(0,1) = 0.0;  B(0,2) = 0.0;
    B(1,0) = 0.0;   B(1,1) = 3.0;  B(1,2) = 2.0;


//  ----------------------------------------------
//  Compute the singular value decomposition of B:
//
//               B = U D V^T
//
//  where U and V are orthogonal and D a diagonal
//  matrix.
//  ----------------------------------------------

    JacobiSVD< MatrixXd > svdB(B, ComputeThinU | ComputeThinV);

    U = svdB.matrixU();
    V = svdB.matrixV();
    D = svdB.singularValues();

    cout << "\n\nSVD of the matrix B: \n";

    cout << "U = " << endl << U << endl;
    cout << "D = " << endl << D << endl;
    cout << "V = " << endl << V << endl;

    return 0;
}
/* <<< end tutorial code <<< */


