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
 *    \file src/objective/lsq_end_term.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *
 */


#include <acado/objective/lsq_end_term.hpp>


BEGIN_NAMESPACE_ACADO



//
// PUBLIC MEMBER FUNCTIONS:
//

//
// PUBLIC MEMBER FUNCTIONS:
//


LSQEndTerm::LSQEndTerm( )
           :ObjectiveElement(){

    S_h_res = 0;
}


LSQEndTerm::LSQEndTerm( const Grid& grid_, const DMatrix &S_,
                        const Function& m, const DVector &r_ )
           :ObjectiveElement( grid_ ){

    fcn  = m ;
    S    = S_;
    r    = r_;

    S_h_res = new double[fcn.getDim()];
}


LSQEndTerm::LSQEndTerm( const LSQEndTerm& rhs )
           :ObjectiveElement( rhs ){

    S = rhs.S;
    r = rhs.r;

    S_h_res = new double[fcn.getDim()];
}


LSQEndTerm::~LSQEndTerm( ){

    if( S_h_res != 0 ) delete[] S_h_res;
}


LSQEndTerm& LSQEndTerm::operator=( const LSQEndTerm& rhs ){

    if ( this != &rhs ){

        if( S_h_res != 0 ) delete[] S_h_res;

        ObjectiveElement::operator=(rhs);

        S = rhs.S;
        r = rhs.r;

        S_h_res = new double[fcn.getDim()];
    }
    return *this;
}


returnValue LSQEndTerm::evaluate( const OCPiterate &x ){

    uint run2, run3;
    const uint nh = fcn.getDim();

    ObjectiveElement::init( x );

    z.setZ( grid.getLastIndex(), x );

    DVector h_res = fcn.evaluate( z);

    // EVALUATE THE OBJECTIVE:
    // -----------------------

    obj = 0.0;

    for( run2 = 0; run2 < nh; run2++ )
        h_res(run2) -= r.operator()(run2);

    for( run2 = 0; run2 < nh; run2++ ){
        S_h_res[run2] = 0.0;
        for( run3 = 0; run3 < nh; run3++ )
            S_h_res[run2] += S.operator()(run2,run3)*h_res(run3);
    }

    for( run2 = 0; run2 < nh; run2++ )
        obj += 0.5*h_res(run2)*S_h_res[run2];


    return SUCCESSFUL_RETURN;
}


returnValue LSQEndTerm::evaluateSensitivities( BlockMatrix *hessian ){

    if( hessian == 0 ) return evaluateSensitivitiesGN(0);
    return ACADOERROR(RET_NOT_IMPLEMENTED_YET);
}


returnValue LSQEndTerm::evaluateSensitivitiesGN( BlockMatrix *GNhessian ){

    int run2, run3, run4;
    const int N = grid.getNumPoints();
    const int nh = fcn.getDim();

    if( bSeed != 0 ){

        if( xSeed  != 0 || pSeed  != 0 || uSeed  != 0 || wSeed  != 0 ||
            xSeed2 != 0 || pSeed2 != 0 || uSeed2 != 0 || wSeed2 != 0 )
            return ACADOERROR( RET_WRONG_DEFINITION_OF_SEEDS );

        double *bseed   = new double [nh];
        double **J      = new double*[nh];

        for( run2 = 0; run2 < nh; run2++ )
             J[run2] = new double[fcn.getNumberOfVariables() +1];

        if( bSeed->getNumRows( 0, 0 ) != 1 ) return ACADOWARNING( RET_WRONG_DEFINITION_OF_SEEDS );

        DMatrix bseed_;
        bSeed->getSubBlock( 0, 0, bseed_);

        dBackward.init( 1, 5*N );

        DMatrix Dx ( 1, nx );
        DMatrix Dxa( 1, na );
        DMatrix Dp ( 1, np );
        DMatrix Du ( 1, nu );
        DMatrix Dw ( 1, nw );

        Dx .setZero();
        Dxa.setZero();
        Dp .setZero();
        Du .setZero();
        Dw .setZero();

        for( run2 = 0; run2 < nh; run2++ ) bseed[run2] = 0;

        for( run2 = 0; run2 < nh; run2++ ){
             for(run3 = 0; (int) run3 < fcn.getNumberOfVariables() +1; run3++ )
                 J[run2][run3] = 0.0;

             bseed[run2] = 1.0;
             fcn.AD_backward( 0, bseed, J[run2] );
             bseed[run2] = 0.0;

             for( run3 = 0; run3 < nx; run3++ ){
                  Dx( 0, run3 ) += bseed_(0,0)*J[run2][y_index[run3]]*S_h_res[run2];
             }
             for( run3 = nx; run3 < nx+na; run3++ ){
                  Dxa( 0, run3-nx ) += bseed_(0,0)*J[run2][y_index[run3]]*S_h_res[run2];
             }
             for( run3 = nx+na; run3 < nx+na+np; run3++ ){
                  Dp( 0, run3-nx-na ) += bseed_(0,0)*J[run2][y_index[run3]]*S_h_res[run2];
             }
             for( run3 = nx+na+np; run3 < nx+na+np+nu; run3++ ){
                  Du( 0, run3-nx-na-np ) += bseed_(0,0)*J[run2][y_index[run3]]*S_h_res[run2];
             }
             for( run3 = nx+na+np+nu; run3 < nx+na+np+nu+nw; run3++ ){
                  Dw( 0, run3-nx-na-np-nu ) += bseed_(0,0)*J[run2][y_index[run3]]*S_h_res[run2];
             }
        }
        if( nx > 0 ) dBackward.setDense( 0,   N-1, Dx  );
        if( na > 0 ) dBackward.setDense( 0, 2*N-1, Dxa );
        if( np > 0 ) dBackward.setDense( 0, 3*N-1, Dp  );
        if( nu > 0 ) dBackward.setDense( 0, 4*N-1, Du  );
        if( nw > 0 ) dBackward.setDense( 0, 5*N-1, Dw  );


        // COMPUTE GAUSS-NEWTON HESSIAN APPROXIMATION IF REQUESTED:
        // --------------------------------------------------------

        if( GNhessian != 0 ){

            const int nnn = nx+na+np+nu+nw;
            DMatrix tmp( nh, nnn );

            for( run3 = 0; run3 < nnn; run3++ ){
                for( run2 = 0; run2 < nh; run2++ ){
                    tmp( run2, run3 ) = 0.0;
                    for( run4 = 0; run4 < nh; run4++ ){
                        tmp( run2, run3 ) += S.operator()(run2,run4)*J[run4][y_index[run3]];
                    }
                }
            }
            DMatrix tmp2;
            int i,j;
            int *Sidx = new int[6];
            int *Hidx = new int[5];

            Sidx[0] = 0;
            Sidx[1] = nx;
            Sidx[2] = nx+na;
            Sidx[3] = nx+na+np;
            Sidx[4] = nx+na+np+nu;
            Sidx[5] = nx+na+np+nu+nw;

            Hidx[0] =   N-1;
            Hidx[1] = 2*N-1;
            Hidx[2] = 3*N-1;
            Hidx[3] = 4*N-1;
            Hidx[4] = 5*N-1;

            for( i = 0; i < 5; i++ ){
                for( j = 0; j < 5; j++ ){

                    tmp2.init(Sidx[i+1]-Sidx[i],Sidx[j+1]-Sidx[j]);
                    tmp2.setZero();

                    for( run3 = Sidx[i]; run3 < Sidx[i+1]; run3++ )
                        for( run4 = Sidx[j]; run4 < Sidx[j+1]; run4++ )
                            for( run2 = 0; run2 < nh; run2++ )
                                tmp2(run3-Sidx[i],run4-Sidx[j]) += J[run2][y_index[run3]]*tmp(run2,run4);

                    if( tmp2.getDim() != 0 ) GNhessian->addDense(Hidx[i],Hidx[j],tmp2);
                }
            }
            delete[] Sidx;
            delete[] Hidx;
        }
        // --------------------------------------------------------

        for( run2 = 0; run2 < nh; run2++ )
            delete[] J[run2];
        delete[] J;
        delete[] bseed;
        return SUCCESSFUL_RETURN;
    }

    return ACADOERROR(RET_NOT_IMPLEMENTED_YET);
}




CLOSE_NAMESPACE_ACADO

// end of file.
