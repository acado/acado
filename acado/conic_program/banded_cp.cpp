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
 *    \file src/conic_program/banded_cp.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *
 */

#include <acado/conic_program/banded_cp.hpp>


BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

BandedCP::BandedCP( ){

    nS = 0;

    B   = 0;
    lbB = 0;
    ubB = 0;

    ylbB = 0;
    yubB = 0;
}


BandedCP::BandedCP( const BandedCP& rhs ){

    copy(rhs);
}


BandedCP::~BandedCP( ){

    clean();
}


BandedCP& BandedCP::operator=( const BandedCP& rhs ){

    if ( this != &rhs ){

        clean()  ;
        copy(rhs);
    }
    return *this;
}



void BandedCP::copy( const BandedCP& rhs ){


    hessian                 = rhs.hessian                ;
    objectiveGradient       = rhs.objectiveGradient      ;

    lowerBoundResiduum      = rhs.lowerBoundResiduum     ;
    upperBoundResiduum      = rhs.upperBoundResiduum     ;

    dynGradient             = rhs.dynGradient            ;
    dynResiduum             = rhs.dynResiduum            ;

    constraintGradient      = rhs.constraintGradient     ;
    lowerConstraintResiduum = rhs.lowerConstraintResiduum;
    upperConstraintResiduum = rhs.upperConstraintResiduum;

    deltaX                  = rhs.deltaX                 ;
    lambdaBound             = rhs.lambdaBound            ;
    lambdaDynamic           = rhs.lambdaDynamic          ;
    lambdaConstraint        = rhs.lambdaConstraint       ;


//     int run1, run2;

//     nS = rhs.nS;
// 
// 
//     if( rhs.B != 0 ){
//         B = (BlockMatrix**)calloc(nS,sizeof(BlockMatrix*));
//         for( run1 = 0; run1 < nS; run1++ ){
//             B[run1] = new BlockMatrix[nV];
//             for( run2 = 0; run2 < nV; run2++ )
//                 B[run1][run2] = rhs.B[run1][run2];
//         }
//     }
//     else B = 0;
// 
// 
//     if( rhs.lbB != 0 ){
//         lbB = (BlockMatrix*)calloc(nS,sizeof(BlockMatrix));
//         for( run1 = 0; run1 < nS; run1++ )
//             lbB[run1] = rhs.lbB[run1];
//     }
//     else lbB = 0;
// 
//     if( rhs.ubB != 0 ){
//         ubB = (BlockMatrix*)calloc(nS,sizeof(BlockMatrix));
//         for( run1 = 0; run1 < nS; run1++ )
//             ubB[run1] = rhs.ubB[run1];
//     }
//     else ubB = 0;
// 
// 
//     if( nS > 0 ){
// 
//         ylbB = (BlockMatrix**)calloc(nS,sizeof(BlockMatrix*));
//         yubB = (BlockMatrix**)calloc(nS,sizeof(BlockMatrix*));
// 
//         for( run1 = 0; run1 < nS; run1++ ){
//             if( rhs.ylbB[run1] != 0 ) ylbB[run1] = new BlockMatrix(*rhs.ylbB[run1]);
//             else                      ylbB[run1] = 0                          ;
// 
//             if( rhs.yubB[run1] != 0 ) yubB[run1] = new BlockMatrix(*rhs.yubB[run1]);
//             else                      yubB[run1] = 0                          ;
//         }
//     }
//     else{
//         ylbB = 0;
//         yubB = 0;
//     }
}



void BandedCP::clean(){

//     int run1;
// 
//     for( run1 = 0; run1 < nS; run1++ ){
// 
//         if( B[run1]    != 0 ) delete[]    B[run1];
//         if( ylbB[run1] != 0 ) delete   ylbB[run1];
//         if( yubB[run1] != 0 ) delete   yubB[run1];
//     }
// 
//     if( B    != 0 ) free(B)   ;
//     if( ylbB != 0 ) free(ylbB);
//     if( yubB != 0 ) free(yubB);
// 
//     if( lbB != 0 ) free(lbB);
//     if( ubB != 0 ) free(ubB);
}




CLOSE_NAMESPACE_ACADO

// end of file.
