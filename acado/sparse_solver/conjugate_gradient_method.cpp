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
 *    \file src/sparse_solver/conjugate_gradient_method.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date   2009
 */


#include <acado/sparse_solver/sparse_solver.hpp>

#include <iostream>

BEGIN_NAMESPACE_ACADO



//
// PUBLIC MEMBER FUNCTIONS:
//



ConjugateGradientMethod::ConjugateGradientMethod( ){

    dim        = 0     ;
    nDense     = 0     ;
    A          = 0     ;
    x          = 0     ;
    norm2      = 0     ;
    p          = 0     ;
    r          = 0     ;
    TOL        = 1.e-10;
    printLevel = LOW;
    pCounter   = 0     ;
    condScale  = 0     ;
}


ConjugateGradientMethod::ConjugateGradientMethod( const ConjugateGradientMethod &arg ){

    int run1, run2;

    dim        = arg.dim;
    nDense     = arg.nDense;

    if( arg.A == 0 )  A = 0;
    else{
        A = new double[nDense];
        for( run1 = 0; run1 < nDense; run1++ )
            A[run1] = arg.A[run1];
    }

    if( arg.x == 0 )  x = 0;
    else{
        x = new double[dim];
        for( run1 = 0; run1 < dim; run1++ )
            x[run1] = arg.x[run1];
    }

    pCounter = arg.pCounter;


    if( arg.norm2 == 0 )  norm2 = 0;
    else{
        norm2 = new double[2*dim+1];
        for( run1 = 0; run1 < 2*dim+1; run1++ )
            norm2[run1] = arg.norm2[run1];
    }


    if( arg.p == 0 ) p = 0;
    else{
        p = new double*[2*dim+1];
        for( run1 = 0; run1 < 2*dim+1; run1++ ){
            p[run1] = new double[dim];
            for( run2 = 0; run2 < dim; run2++ ){
                p[run1][run2] = arg.p[run1][run2];
            }
        }
    }

    if( arg.r == 0 )  r = 0;
    else{
        r = new double[dim];
        for( run1 = 0; run1 < dim; run1++ )
            r[run1] = arg.r[run1];
    }

    TOL = arg.TOL;
    printLevel = arg.printLevel;

    if( arg.condScale == 0 ) condScale = 0;
    else{
        condScale = new double[dim];
        for( run1 = 0; run1 < dim; run1++ )
            condScale[run1] = arg.condScale[run1];
    }
}


ConjugateGradientMethod::~ConjugateGradientMethod( ){

    int run1;

    if( A != 0 ) delete[] A;
    if( x != 0 ) delete[] x;

    if( norm2 != 0 ) delete[] norm2;

    if( p != 0 ){
        for( run1 = 0; run1 < 2*dim+1; run1++ )
            delete[] p[run1];
        delete[] p;
    }

    if( r != 0 ) delete[] r;

    if( condScale != 0) delete[] condScale;
}




returnValue ConjugateGradientMethod::solve( double *b ){

    // CONSISTENCY CHECKS:
    // -------------------
    if( dim    <= 0 )  return ACADOERROR(RET_MEMBER_NOT_INITIALISED);
    if( nDense <= 0 )  return ACADOERROR(RET_MEMBER_NOT_INITIALISED);
    if( A      == 0 )  return ACADOERROR(RET_MEMBER_NOT_INITIALISED);


    int run1, run2;

    double *aux  = new double[dim];
    double auxR, auxR2, norm1;
    double alpha, beta;

    // INITIALISE  X:
    // --------------

    for( run1 = 0; run1 < dim; run1++ )
        x[run1] = 0.0;


    // RESET THE COUNTER IF TOO LARGE:
    // -------------------------------
    if( pCounter > dim ) pCounter = dim-1;


    // APPLY THE PRECONDITIONER ON b:
    // ------------------------------

    applyPreconditioner( b );


    // COMPUTE WARM START INITIALIZATION FOR X BASED ON PREVIOUS
    // DECOMPOSITIONS:
    // ----------------------------------------------------------

    for( run1 = 0; run1 < pCounter; run1++ ){
        alpha = scalarProduct( p[run1], r )/norm2[run1];
        for( run2 = 0; run2 < dim; run2++ ){
            x[run2] += alpha*p[run1][run2];
        }
    }

    // COMPUTE INITIAL RESIDUUM:
    // -------------------------

    multiply( x, aux );

    for( run1 = 0; run1 < dim; run1++ )
        r[run1] -= aux[run1];


    for( run1 = 0; run1 < dim; run1++ )
        p[pCounter][run1] = r[run1];


    while( pCounter <= 2*dim-1 ){

        norm1 = 0.0;

        for( run1 = 0; run1 < dim; run1++ ){
            if( r[run1] >= 0 && norm1 <  r[run1] ) norm1 =  r[run1];
            if( r[run1] <= 0 && norm1 < -r[run1] ) norm1 = -r[run1];
        }

        if( printLevel == HIGH )
            std::cout << "STEP NUMBER " << pCounter << ",  WEIGHTED NORM = "
                 << std::scientific << norm1 << std::endl;

        if( norm1 < TOL ) break;

        auxR = scalarProduct( r,r );
        multiply( p[pCounter], aux );
        norm2[pCounter] = scalarProduct( p[pCounter], aux );
        alpha = auxR/norm2[pCounter];

        for( run1 = 0; run1 < dim; run1++ ){
            x   [run1] += alpha*p[pCounter][run1];
            r   [run1] -= alpha*aux[run1];
        }

        auxR2 = scalarProduct( r,r );
        beta  = auxR2/auxR;

        for( run1 = 0; run1 < dim; run1++ )
            p[pCounter+1][run1] = r[run1] + beta*p[pCounter][run1];

        pCounter++;
    }

    delete[] aux ;

    if( pCounter >= 2*dim ){
        if( printLevel == MEDIUM || printLevel == HIGH )
            return ACADOWARNING( RET_LINEAR_SYSTEM_NUMERICALLY_SINGULAR );
        else return RET_LINEAR_SYSTEM_NUMERICALLY_SINGULAR;
    }

    return SUCCESSFUL_RETURN;
}


returnValue ConjugateGradientMethod::setDimension( const int &n ){

    int run1;

    dim = n;

    if( x != 0 ){
        delete[] x;
        x = 0;
    }
    x = new double[dim];

    if( norm2 != 0 ){
        delete[] norm2;
        norm2 = 0;
    }
    norm2 = new double[2*dim+1];

    if( p != 0 ){
        for( run1 = 0; run1 < 2*dim+1; run1++ )
            delete[] p[run1];
        delete[] p;
    }
    p = new double*[2*dim+1];
    for( run1 = 0; run1 < 2*dim+1; run1++ ){
        p[run1] = new double[dim];
    }


    if( r != 0 ){
        delete[] r;
        r = 0;
    }
    r = new double[dim];

    if( condScale != 0 ){
        delete[] condScale;
        condScale = 0;
    }
    condScale = new double[dim];

    pCounter = 0;

    return SUCCESSFUL_RETURN;
}



returnValue ConjugateGradientMethod::setNumberOfEntries( const int &nDense_ ){

    if( A != 0 ){
        delete[] A;
        A = 0;
    }

    nDense = nDense_;

    return SUCCESSFUL_RETURN;
}




returnValue ConjugateGradientMethod::setMatrix( double *A_ ){

    if( dim    <= 0 )  return ACADOERROR(RET_MEMBER_NOT_INITIALISED);
    if( nDense <= 0 )  return ACADOERROR(RET_MEMBER_NOT_INITIALISED);

    pCounter = 0;

    A = new double[nDense];
    return computePreconditioner( A_ );
}




returnValue ConjugateGradientMethod::getX( double *x_ ){

    return applyInversePreconditioner( x_ );
}


returnValue ConjugateGradientMethod::setTolerance( double TOL_ ){

    TOL = TOL_;
    return SUCCESSFUL_RETURN;
}


returnValue ConjugateGradientMethod::setPrintLevel( PrintLevel printLevel_ ){

    printLevel = printLevel_;
    return SUCCESSFUL_RETURN;
}


//
// PROTECTED MEMBER FUNCTIONS:
//



double ConjugateGradientMethod::scalarProduct( double *aa, double *bb ){

    int run1;
    double aux = 0.0;

    for( run1 = 0; run1 < dim; run1++ )
        aux += aa[run1]*bb[run1];

    return aux;
}



CLOSE_NAMESPACE_ACADO


/*
 *   end of file
 */
