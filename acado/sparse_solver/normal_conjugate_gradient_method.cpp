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
 *    \file src/sparse_solver/normal_conjugate_gradient_method.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date   2009
 */


#include <acado/sparse_solver/sparse_solver.hpp>


BEGIN_NAMESPACE_ACADO



//
// PUBLIC MEMBER FUNCTIONS:
//


NormalConjugateGradientMethod::NormalConjugateGradientMethod( )
                              :ConjugateGradientMethod( ){

    index     = 0;
    nIndex    = 0;
    iResult   = 0;
}


NormalConjugateGradientMethod::NormalConjugateGradientMethod( const NormalConjugateGradientMethod &arg )
                              :ConjugateGradientMethod(arg){

    int run1, run2;

    if( arg.nIndex != 0 ){
        nIndex = new int[dim];
        for( run1 = 0; run1 < dim; run1++ ){
             nIndex[run1] = arg.nIndex[run1];
        }
    }
    else  nIndex = 0;

    if( arg.index != 0 ){
        index = new int*[dim];
        for( run1 = 0; run1 < dim; run1++ ){
             index[run1] = new int[nIndex[run1]];
             for( run2 = 0; run2 < nIndex[run1]; run2++ )
                 index[run1][run2] = arg.index[run1][run2];
        }
    }
    else index = 0;

    if( arg.iResult != 0 ){
        iResult = new double[dim];
        for( run1 = 0; run1 < dim; run1++ ){
             iResult[run1] = arg.iResult[run1];
        }
    }
    else  iResult = 0;
}


NormalConjugateGradientMethod::~NormalConjugateGradientMethod( ){

    int run1;

    if( nIndex != 0 )  delete[] nIndex;

    if( iResult != 0 )  delete[] iResult;

    if( index != 0 ){
        for( run1 = 0; run1 < dim; run1++ )
            delete[] index[run1];
        delete[] index;
    }
}



SparseSolver* NormalConjugateGradientMethod::clone() const{

    return new NormalConjugateGradientMethod(*this);
}


returnValue NormalConjugateGradientMethod::setIndices( const int *rowIdx_, const int *colIdx_  ){

     return ACADOERROR( RET_NOT_IMPLEMENTED_YET );
}


returnValue NormalConjugateGradientMethod::setIndices( const int *indices ){


    // CONSISTENCY CHECK:
    // ------------------

    if( dim    <= 0 )  return ACADOERROR(RET_MEMBER_NOT_INITIALISED);
    if( nDense <= 0 )  return ACADOERROR(RET_MEMBER_NOT_INITIALISED);


    // ALLOCATE MEMORY:
    // ------------------

    if( nIndex != 0 )  delete[] nIndex;
    nIndex = new int[dim];

    if( iResult != 0 )  delete[] iResult;
    iResult = new double[dim];


    // LOCAL AUXILIARY VARIABLE:
    // -------------------------
    int run1,  run2;
    int counter0   ;
    int bound      ;
    int counter = 0;


    // DETERMINE THE INDEX LENGHTS:
    // ----------------------------


    for( run1 = 0; run1 < dim; run1++ ){

        counter0 = counter;
        bound    = dim*(run1+1);

        while( counter < nDense ){

            if( indices[counter] >= bound ) break;
            counter++;
        }

        nIndex[run1] = counter - counter0;
    }

    if( index != 0 ){
        for( run1 = 0; run1 < dim; run1++ )
            delete[] index[run1];
        delete[] index;
    }
    index     = new int*[dim];

    counter = 0;

    for( run1 = 0; run1 < dim; run1++ ){
        index[run1]     = new int[nIndex[run1]];
        for( run2 = 0; run2 < nIndex[run1]; run2++ ){
            index    [run1][run2] = indices[counter];
            counter++;
        }
    }

    return SUCCESSFUL_RETURN;
}








//
// PROTECTED MEMBER FUNCTIONS:
//



void NormalConjugateGradientMethod::multiply( double *xx , double *result ){


    int i,j,counter, offset;

    counter = 0;

    for( i = 0; i < dim; i++ ){
        iResult[i] = 0.0;
        result [i] = 0.0;
        offset = dim*i;
        for( j = 0; j < nIndex[i]; j++ ){
            iResult[i] += A[counter]*xx[index[i][j]-offset];
            counter++;
        }
    }

    counter = 0;

    for( i = 0; i < dim; i++ ){
        offset = dim*i;
        for( j = 0; j < nIndex[i]; j++ ){
            result[index[i][j]-offset] += A[counter]*iResult[i];
            counter++;
        }
    }
}




returnValue NormalConjugateGradientMethod::computePreconditioner( double* A_ ){

    int i,j,counter;

    for( i = 0; i < dim; i++ ){
        condScale[i] = 0.0;
    }

    int offset;
    counter = 0;

    for( i = 0; i < dim; i++ ){
        offset = dim*i;
        for( j = 0; j < nIndex[i]; j++ ){
            condScale[index[i][j]-offset] += A_[counter]*A_[counter];
            counter++;
        }
    }

    for( i = 0; i < dim; i++ ){
        condScale[i] = sqrt(condScale[i]);
    }

    counter = 0;

    for( i = 0; i < dim; i++ ){
        for( j = 0; j < nIndex[i]; j++ ){
            A[counter] = A_[counter]/condScale[i];
            counter++;
        }
    }

    return SUCCESSFUL_RETURN;
}



returnValue NormalConjugateGradientMethod::applyPreconditioner( double *b ){

    int i,j, counter;

    for( i = 0; i < dim; i++ )
        iResult[i] = b[i]/condScale[i];

    counter = 0;
    int offset;

    for( i = 0; i < dim; i++ )
        r[i] = 0.0;

    for( i = 0; i < dim; i++ ){
        offset = dim*i;
        for( j = 0; j < nIndex[i]; j++ ){
            r[index[i][j]-offset] += A[counter]*iResult[i];
            counter++;
        }
    }
    return SUCCESSFUL_RETURN;
}




returnValue NormalConjugateGradientMethod::applyInversePreconditioner( double *x_ ){

    int run1;

    for( run1 = 0; run1 < dim; run1++ )
        x_[run1] = x[run1];

    return SUCCESSFUL_RETURN;
}



CLOSE_NAMESPACE_ACADO


/*
 *   end of file
 */
