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
 *    \file src/objective/mayer_term.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *
 */

#include <acado/objective/mayer_term.hpp>



BEGIN_NAMESPACE_ACADO



//
// PUBLIC MEMBER FUNCTIONS:
//


MayerTerm::MayerTerm( ){ }


MayerTerm::MayerTerm( const Grid &grid_, const Expression& arg )
          :ObjectiveElement(grid_){

    fcn << arg;
}

MayerTerm::MayerTerm( const Grid &grid_, const Function& arg )
          :ObjectiveElement(grid_){

    fcn = arg;
}

MayerTerm::MayerTerm( const MayerTerm& rhs )
          :ObjectiveElement(rhs){ }

MayerTerm::~MayerTerm( ){ }


MayerTerm& MayerTerm::operator=( const MayerTerm& rhs ){

    if( this != &rhs ){

        ObjectiveElement::operator=(rhs);
    }
    return *this;
}



returnValue MayerTerm::evaluate( const OCPiterate &x ){

    ObjectiveElement::init( x );

    z.setZ( grid.getLastIndex(), x );

    DVector objective = fcn.evaluate( z );

    obj = objective(0);

    return SUCCESSFUL_RETURN;
}


returnValue MayerTerm::evaluateSensitivities( BlockMatrix *hessian ){

    int run1, run2;
    const int N = grid.getNumPoints();

    if( (bSeed != 0) & (hessian == 0) ){

        DMatrix bseed_;
        bSeed->getSubBlock( 0, 0, bseed_);

        fcn.AD_backward( bseed_.getRow(0), JJ );

        dBackward.init( 1, 5*N );

        DMatrix tmp1 = JJ.getX(); tmp1.transposeInPlace();
        if( nx > 0 ) dBackward.setDense( 0,   N-1, tmp1 );
        DMatrix tmp2 = JJ.getXA(); tmp2.transposeInPlace();
        if( na > 0 ) dBackward.setDense( 0, 2*N-1, tmp2 );
        DMatrix tmp3 = JJ.getP(); tmp3.transposeInPlace();
        if( np > 0 ) dBackward.setDense( 0, 3*N-1, tmp3 );
        DMatrix tmp4 = JJ.getU(); tmp4.transposeInPlace();
        if( nu > 0 ) dBackward.setDense( 0, 4*N-1, tmp4 );
        DMatrix tmp5 = JJ.getW(); tmp5.transposeInPlace();
        if( nw > 0 ) dBackward.setDense( 0, 5*N-1, tmp5 );

        return SUCCESSFUL_RETURN;
    }

    if( hessian != 0 ){

        double  bseed1 = 1.0;
        double  bseed2 = 0.0;
        double *J      = new double[fcn.getNumberOfVariables() +1];
        double *H      = new double[fcn.getNumberOfVariables() +1];
        double *fseed  = new double[fcn.getNumberOfVariables() +1];

        for( run1 = 0; run1 < fcn.getNumberOfVariables()+1; run1++ )
            fseed[run1] = 0.0;

        dBackward.init( 1, 5*N );

        DMatrix Dx ( 1, nx );
        DMatrix Dxa( 1, na );
        DMatrix Dp ( 1, np );
        DMatrix Du ( 1, nu );
        DMatrix Dw ( 1, nw );

        DMatrix Hx ( nx, nx );
        DMatrix Hxa( nx, na );
        DMatrix Hp ( nx, np );
        DMatrix Hu ( nx, nu );
        DMatrix Hw ( nx, nw );

        for( run2 = 0; run2 < nx; run2++ ){

            // FIRST ORDER DERIVATIVES:
            // ------------------------
            fseed[y_index[run2]] = 1.0;
            fcn.AD_forward( 0, fseed, J );
            Dx( 0, run2 ) = J[0];
            fseed[y_index[run2]] = 0.0;

            // SECOND ORDER DERIVATIVES:
            // -------------------------
            for( run1 = 0; run1 <= fcn.getNumberOfVariables(); run1++ ){
                J[run1] = 0.0;
                H[run1] = 0.0;
            }

            fcn.AD_backward2( 0, &bseed1, &bseed2, J, H );

            for( run1 = 0; run1 < nx; run1++ ){
                 Hx( run2, run1 ) = H[y_index[run1]];
            }
            for( run1 = nx; run1 < nx+na; run1++ ){
                 Hxa( run2, run1-nx ) = H[y_index[run1]];
            }
            for( run1 = nx+na; run1 < nx+na+np; run1++ ){
                 Hp( run2, run1-nx-na ) = H[y_index[run1]];
            }
            for( run1 = nx+na+np; run1 < nx+na+np+nu; run1++ ){
                 Hu( run2, run1-nx-na-np ) = H[y_index[run1]];
            }
            for( run1 = nx+na+np+nu; run1 < nx+na+np+nu+nw; run1++ ){
                 Hw( run2, run1-nx-na-np-nu ) = H[y_index[run1]];
            }
        }

        if( nx > 0 ){

            dBackward.setDense( 0, N-1, Dx );

            if( nx > 0 ) hessian->setDense( N-1,   N-1, Hx  );
            if( na > 0 ) hessian->setDense( N-1, 2*N-1, Hxa );
            if( np > 0 ) hessian->setDense( N-1, 3*N-1, Hp  );
            if( nu > 0 ) hessian->setDense( N-1, 4*N-1, Hu  );
            if( nw > 0 ) hessian->setDense( N-1, 5*N-1, Hw  );
        }

        Hx.init ( na, nx );
        Hxa.init( na, na );
        Hp.init ( na, np );
        Hu.init ( na, nu );
        Hw.init ( na, nw );

        for( run2 = nx; run2 < nx+na; run2++ ){

            // FIRST ORDER DERIVATIVES:
            // ------------------------
            fseed[y_index[run2]] = 1.0;
            fcn.AD_forward( 0, fseed, J );
            Dxa( 0, run2-nx ) = J[0];
            fseed[y_index[run2]] = 0.0;

            // SECOND ORDER DERIVATIVES:
            // -------------------------
            for( run1 = 0; run1 <= fcn.getNumberOfVariables(); run1++ ){
                J[run1] = 0.0;
                H[run1] = 0.0;
            }

            fcn.AD_backward2( 0, &bseed1, &bseed2, J, H );

            for( run1 = 0; run1 < nx; run1++ ){
                 Hx( run2-nx, run1 ) = H[y_index[run1]];
            }
            for( run1 = nx; run1 < nx+na; run1++ ){
                 Hxa( run2-nx, run1-nx ) = H[y_index[run1]];
            }
            for( run1 = nx+na; run1 < nx+na+np; run1++ ){
                 Hp( run2-nx, run1-nx-na ) = H[y_index[run1]];
            }
            for( run1 = nx+na+np; run1 < nx+na+np+nu; run1++ ){
                 Hu( run2-nx, run1-nx-na-np ) = H[y_index[run1]];
            }
            for( run1 = nx+na+np+nu; run1 < nx+na+np+nu+nw; run1++ ){
                 Hw( run2-nx, run1-nx-na-np-nu ) = H[y_index[run1]];
            }
        }

        if( na > 0 ){

            dBackward.setDense( 0, 2*N-1, Dxa );

            if( nx > 0 ) hessian->setDense( 2*N-1,   N-1, Hx  );
            if( na > 0 ) hessian->setDense( 2*N-1, 2*N-1, Hxa );
            if( np > 0 ) hessian->setDense( 2*N-1, 3*N-1, Hp  );
            if( nu > 0 ) hessian->setDense( 2*N-1, 4*N-1, Hu  );
            if( nw > 0 ) hessian->setDense( 2*N-1, 5*N-1, Hw  );
        }

        Hx.init ( np, nx );
        Hxa.init( np, na );
        Hp.init ( np, np );
        Hu.init ( np, nu );
        Hw.init ( np, nw );

        for( run2 = nx+na; run2 < nx+na+np; run2++ ){

            // FIRST ORDER DERIVATIVES:
            // ------------------------
            fseed[y_index[run2]] = 1.0;
            fcn.AD_forward( 0, fseed, J );
            Dp( 0, run2-nx-na ) = J[0];
            fseed[y_index[run2]] = 0.0;

            // SECOND ORDER DERIVATIVES:
            // -------------------------
            for( run1 = 0; run1 <= fcn.getNumberOfVariables(); run1++ ){
                J[run1] = 0.0;
                H[run1] = 0.0;
            }

            fcn.AD_backward2( 0, &bseed1, &bseed2, J, H );

            for( run1 = 0; run1 < nx; run1++ ){
                 Hx( run2-nx-na, run1 ) = H[y_index[run1]];
            }
            for( run1 = nx; run1 < nx+na; run1++ ){
                 Hxa( run2-nx-na, run1-nx ) = H[y_index[run1]];
            }
            for( run1 = nx+na; run1 < nx+na+np; run1++ ){
                 Hp( run2-nx-na, run1-nx-na ) = H[y_index[run1]];
            }
            for( run1 = nx+na+np; run1 < nx+na+np+nu; run1++ ){
                 Hu( run2-nx-na, run1-nx-na-np ) = H[y_index[run1]];
            }
            for( run1 = nx+na+np+nu; run1 < nx+na+np+nu+nw; run1++ ){
                 Hw( run2-nx-na, run1-nx-na-np-nu ) = H[y_index[run1]];
            }
        }

        if( np > 0 ){

            dBackward.setDense( 0, 3*N-1, Dp );

            if( nx > 0 ) hessian->setDense( 3*N-1,   N-1, Hx  );
            if( na > 0 ) hessian->setDense( 3*N-1, 2*N-1, Hxa );
            if( np > 0 ) hessian->setDense( 3*N-1, 3*N-1, Hp  );
            if( nu > 0 ) hessian->setDense( 3*N-1, 4*N-1, Hu  );
            if( nw > 0 ) hessian->setDense( 3*N-1, 5*N-1, Hw  );
        }


        Hx.init ( nu, nx );
        Hxa.init( nu, na );
        Hp.init ( nu, np );
        Hu.init ( nu, nu );
        Hw.init ( nu, nw );

        for( run2 = nx+na+np; run2 < nx+na+np+nu; run2++ ){

            // FIRST ORDER DERIVATIVES:
            // ------------------------
            fseed[y_index[run2]] = 1.0;
            fcn.AD_forward( 0, fseed, J );
            Du( 0, run2-nx-na-np ) = J[0];
            fseed[y_index[run2]] = 0.0;

            // SECOND ORDER DERIVATIVES:
            // -------------------------
            for( run1 = 0; run1 <= fcn.getNumberOfVariables(); run1++ ){
                J[run1] = 0.0;
                H[run1] = 0.0;
            }

            fcn.AD_backward2( 0, &bseed1, &bseed2, J, H );

            for( run1 = 0; run1 < nx; run1++ ){
                 Hx( run2-nx-na-np, run1 ) = H[y_index[run1]];
            }
            for( run1 = nx; run1 < nx+na; run1++ ){
                 Hxa( run2-nx-na-np, run1-nx ) = H[y_index[run1]];
            }
            for( run1 = nx+na; run1 < nx+na+np; run1++ ){
                 Hp( run2-nx-na-np, run1-nx-na ) = H[y_index[run1]];
            }
            for( run1 = nx+na+np; run1 < nx+na+np+nu; run1++ ){
                 Hu( run2-nx-na-np, run1-nx-na-np ) = H[y_index[run1]];
            }
            for( run1 = nx+na+np+nu; run1 < nx+na+np+nu+nw; run1++ ){
                 Hw( run2-nx-na-np, run1-nx-na-np-nu ) = H[y_index[run1]];
            }
        }

        if( nu > 0 ){

            dBackward.setDense( 0, 4*N-1, Du );

            if( nx > 0 ) hessian->setDense( 4*N-1,   N-1, Hx  );
            if( na > 0 ) hessian->setDense( 4*N-1, 2*N-1, Hxa );
            if( np > 0 ) hessian->setDense( 4*N-1, 3*N-1, Hp  );
            if( nu > 0 ) hessian->setDense( 4*N-1, 4*N-1, Hu  );
            if( nw > 0 ) hessian->setDense( 4*N-1, 5*N-1, Hw  );
        }


        Hx.init ( nw, nx );
        Hxa.init( nw, na );
        Hp.init ( nw, np );
        Hu.init ( nw, nu );
        Hw.init ( nw, nw );

        for( run2 = nx+na+np+nu; run2 < nx+na+np+nu+nw; run2++ ){

            // FIRST ORDER DERIVATIVES:
            // ------------------------
            fseed[y_index[run2]] = 1.0;
            fcn.AD_forward( 0, fseed, J );
            Dw( 0, run2-nx-na-np-nu ) = J[0];
            fseed[y_index[run2]] = 0.0;

            // SECOND ORDER DERIVATIVES:
            // -------------------------
            for( run1 = 0; run1 <= fcn.getNumberOfVariables(); run1++ ){
                J[run1] = 0.0;
                H[run1] = 0.0;
            }

            fcn.AD_backward2( 0, &bseed1, &bseed2, J, H );

            for( run1 = 0; run1 < nx; run1++ ){
                 Hx( run2-nx-na-np-nu, run1 ) = H[y_index[run1]];
            }
            for( run1 = nx; run1 < nx+na; run1++ ){
                 Hxa( run2-nx-na-np-nu, run1-nx ) = H[y_index[run1]];
            }
            for( run1 = nx+na; run1 < nx+na+np; run1++ ){
                 Hp( run2-nx-na-np-nu, run1-nx-na ) = H[y_index[run1]];
            }
            for( run1 = nx+na+np; run1 < nx+na+np+nu; run1++ ){
                 Hu( run2-nx-na-np-nu, run1-nx-na-np ) = H[y_index[run1]];
            }
            for( run1 = nx+na+np+nu; run1 < nx+na+np+nu+nw; run1++ ){
                 Hw( run2-nx-na-np-nu, run1-nx-na-np-nu ) = H[y_index[run1]];
            }
        }

        if( nw > 0 ){

            dBackward.setDense( 0, 5*N-1, Dw );

            if( nx > 0 ) hessian->setDense( 5*N-1,   N-1, Hx  );
            if( na > 0 ) hessian->setDense( 5*N-1, 2*N-1, Hxa );
            if( np > 0 ) hessian->setDense( 5*N-1, 3*N-1, Hp  );
            if( nu > 0 ) hessian->setDense( 5*N-1, 4*N-1, Hu  );
            if( nw > 0 ) hessian->setDense( 5*N-1, 5*N-1, Hw  );
        }

        delete[] J;
        delete[] H;
        delete[] fseed;

        return SUCCESSFUL_RETURN;
    }

    return ACADOERROR(RET_NOT_IMPLEMENTED_YET);
}





CLOSE_NAMESPACE_ACADO

// end of file.
