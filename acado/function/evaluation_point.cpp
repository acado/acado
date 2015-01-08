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
 *    \file src/function/evaluation_point.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date 2008
 */


#include <acado/function/function.hpp>

using namespace std;

BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//


EvaluationPoint::EvaluationPoint( ){

    z   = 0;
    idx = 0;
}


EvaluationPoint::EvaluationPoint( const EvaluationPoint& rhs ){ copy( rhs        ); }
EvaluationPoint::~EvaluationPoint(                           ){ deleteAll(       ); }


EvaluationPoint::EvaluationPoint( const Function &f ,
                                  uint nx_, uint na_,
                                  uint nu_, uint np_,
                                  uint nw_, uint nd_, uint N_){

    z   = 0;
    idx = 0;
    init(f,nx_,na_,nu_,np_,nw_,nd_,N_);
}

EvaluationPoint::EvaluationPoint( const Function   &f   ,
                                  const OCPiterate &iter  ){

    z   = 0;
    idx = 0;
    init( f, iter );
}


EvaluationPoint& EvaluationPoint::operator=( const EvaluationPoint& rhs ){

    if( this != &rhs ){
        deleteAll();
        copy(rhs);
    }
    return *this;
}


returnValue EvaluationPoint::init( const Function   &f   ,
                       const OCPiterate &iter  ){

    return init( f, iter.getNX(), iter.getNXA(), iter.getNP(), iter.getNU(), iter.getNW() );
}


returnValue EvaluationPoint::init( const Function &f ,
                                   uint nx_, uint na_, uint np_,
                                   uint nu_, uint nw_, uint nd_,
                                   uint N_                       ){

    uint run1;
    deleteAll();

    nx = acadoMax( nx_, f.getNX ()                 );
    na = acadoMax( na_, f.getNXA()                 );
    np = acadoMax( np_, f.getNP ()                 );
    nu = acadoMax( nu_, f.getNU ()                 );
    nw = acadoMax( nw_, f.getNW ()                 );
    nd = acadoMax( nd_, f.getNDX()                 );
    N  = acadoMax( N_ , f.getNumberOfVariables()+1 );

    if( N != 0 ) z = new double[N];
    else         z = 0            ;

	setZero( );

    idx = new int*[7 ];

    idx[0] = new int [1 ];
    idx[1] = new int [nx];
    idx[2] = new int [na];
    idx[3] = new int [np];
    idx[4] = new int [nu];
    idx[5] = new int [nw];
    idx[6] = new int [nd];

    idx[0][0] = f.index( VT_TIME, 0 );

    for( run1 = 0; run1 < nx; run1++ )
        idx[1][run1] = f.index( VT_DIFFERENTIAL_STATE, run1 );

    for( run1 = 0; run1 < na; run1++ )
        idx[2][run1] = f.index( VT_ALGEBRAIC_STATE, run1 );

    for( run1 = 0; run1 < np; run1++ )
        idx[3][run1] = f.index( VT_PARAMETER, run1 );

    for( run1 = 0; run1 < nu; run1++ )
        idx[4][run1] = f.index( VT_CONTROL, run1 );

    for( run1 = 0; run1 < nw; run1++ )
        idx[5][run1] = f.index( VT_DISTURBANCE, run1 );

    for( run1 = 0; run1 < nd; run1++ )
        idx[6][run1] = f.index( VT_DDIFFERENTIAL_STATE, run1 );

    return SUCCESSFUL_RETURN;
}


returnValue EvaluationPoint::print() const{

    uint run1;

    cout << "Time = " << scientific << z[idx[0][0]] << endl;

	for (run1 = 0; run1 < nx; run1++)
		cout << "x[" << run1 << "] = " << scientific << z[idx[1][run1]] << endl;

	for (run1 = 0; run1 < na; run1++)
		cout << "x[" << run1 << "] = " << scientific << z[idx[2][run1]] << endl;

	for (run1 = 0; run1 < np; run1++)
		cout << "x[" << run1 << "] = " << scientific << z[idx[3][run1]] << endl;

	for (run1 = 0; run1 < nu; run1++)
		cout << "x[" << run1 << "] = " << scientific << z[idx[4][run1]] << endl;

	for (run1 = 0; run1 < nw; run1++)
		cout << "x[" << run1 << "] = " << scientific << z[idx[5][run1]] << endl;

    return SUCCESSFUL_RETURN;
}



//
// PROTECTED MEMBER FUNCTIONS:
//


void EvaluationPoint::copyIdx( const uint &dim, const int *idx1, int **idx2 ){

    uint i;
    *idx2 = new int[dim];
    for( i = 0; i < N; i++ )
        *idx2[i] = idx1[i];
}


void EvaluationPoint::copy( const EvaluationPoint &rhs ){

    uint i;

    nx = rhs.nx;
    na = rhs.na;
    np = rhs.np;
    nu = rhs.nu;
    nw = rhs.nw;
    nd = rhs.nd;
    N  = rhs.N ;

    if( rhs.z != 0 ){
        z = new double[N];
        for( i = 0; i < N; i++ )
            z[i] = rhs.z[i];
    }
    else z = 0;

    if( rhs.idx != 0 ){

        idx = new int*[7];
        copyIdx(  1, rhs.idx[0], &idx[0] );
        copyIdx( nx, rhs.idx[1], &idx[1] );
        copyIdx( na, rhs.idx[2], &idx[2] );
        copyIdx( np, rhs.idx[3], &idx[3] );
        copyIdx( nu, rhs.idx[4], &idx[4] );
        copyIdx( nw, rhs.idx[5], &idx[5] );
        copyIdx( nd, rhs.idx[6], &idx[6] );
    }
    else idx = 0;
}


void EvaluationPoint::deleteAll(){

    if( z != 0 ) delete[] z;

    if( idx != 0 ){
        uint i;
        for( i = 0; i < 7; i++ )
            delete[] idx[i];
        delete[] idx;
    }
}


CLOSE_NAMESPACE_ACADO

// end of file.
