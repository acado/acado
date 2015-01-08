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
 *    \file src/constraint/path_constraint.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *
 */


#include <acado/constraint/path_constraint.hpp>


BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//


PathConstraint::PathConstraint( )
               :ConstraintElement(){

}

PathConstraint::PathConstraint( const Grid& grid_ )
               :ConstraintElement(grid_, 1, grid_.getNumPoints() ){

}

PathConstraint::PathConstraint( const PathConstraint& rhs )
               :ConstraintElement(rhs){

}

PathConstraint::~PathConstraint( ){

}

PathConstraint& PathConstraint::operator=( const PathConstraint& rhs ){

    if( this != &rhs ){

        ConstraintElement::operator=(rhs);
    }
    return *this;
}



returnValue PathConstraint::evaluate( const OCPiterate& iter ){

    int run1, run2;

    if( fcn == 0 ) return ACADOERROR(RET_MEMBER_NOT_INITIALISED);

    const int nc = fcn[0].getDim();
    const int T  = grid.getLastIndex();

    if( nc == 0 )  return ACADOERROR(RET_MEMBER_NOT_INITIALISED);

    residuumL.init(T+1,1);
    residuumU.init(T+1,1);

    for( run1 = 0; run1 <= T; run1++ ){

        DMatrix resL( nc, 1 );
        DMatrix resU( nc, 1 );

		z[0].setZ( run1, iter );
		DVector result = fcn[0].evaluate( z[0],run1 );

        for( run2 = 0; run2 < nc; run2++ ){
             resL( run2, 0 ) = lb[run1][run2] - result(run2);
             resU( run2, 0 ) = ub[run1][run2] - result(run2);
        }

        // STORE THE RESULTS:
        // ------------------
        residuumL.setDense( run1, 0, resL );
        residuumU.setDense( run1, 0, resU );
    }

    return SUCCESSFUL_RETURN;
}


returnValue PathConstraint::evaluateSensitivities(){


    int run1, run3;
    returnValue returnvalue;

    if( fcn == 0 ) return ACADOERROR(RET_MEMBER_NOT_INITIALISED);

    const int N  = grid.getNumPoints();

    // EVALUATION OF THE SENSITIVITIES:
    // --------------------------------

    if( bSeed != 0 )
	{

        if( xSeed  != 0 || pSeed  != 0 || uSeed  != 0 || wSeed  != 0 ||
            xSeed2 != 0 || pSeed2 != 0 || uSeed2 != 0 || wSeed2 != 0 )
            return ACADOERROR( RET_WRONG_DEFINITION_OF_SEEDS );

		int nBDirs;

        dBackward.init( N, 5*N );


        for( run3 = 0; run3 < N; run3++ )
		{
            DMatrix bseed_;
            bSeed->getSubBlock( 0, run3, bseed_);
			
            nBDirs = bSeed->getNumRows( 0, run3 );

            DMatrix Dx ( nBDirs, nx );
            DMatrix Dxa( nBDirs, na );
            DMatrix Dp ( nBDirs, np );
            DMatrix Du ( nBDirs, nu );
            DMatrix Dw ( nBDirs, nw );

			for( run1 = 0; run1 < nBDirs; run1++ )
			{
				ACADO_TRY( fcn[0].AD_backward( bseed_.getRow(run1),JJ[0],run3 ) );
		
				if( nx > 0 ) Dx .setRow( run1, JJ[0].getX () );
				if( na > 0 ) Dxa.setRow( run1, JJ[0].getXA() );
				if( np > 0 ) Dp .setRow( run1, JJ[0].getP () );
				if( nu > 0 ) Du .setRow( run1, JJ[0].getU () );
				if( nw > 0 ) Dw .setRow( run1, JJ[0].getW () );
				
				JJ[0].setZero( );

            }

			if( nx > 0 )
				dBackward.setDense( run3,     run3, Dx );

			if( na > 0 )
				dBackward.setDense( run3,   N+run3, Dxa );

			if( np > 0 )
				dBackward.setDense( run3, 2*N+run3, Dp );

			if( nu > 0 )
				dBackward.setDense( run3, 3*N+run3, Du );

			if( nw > 0 )
				dBackward.setDense( run3, 4*N+run3, Dw );
        }

		return SUCCESSFUL_RETURN;
	}
	
	// TODO: implement forward mode

    return ACADOERROR(RET_NOT_IMPLEMENTED_YET);
}



returnValue PathConstraint::evaluateSensitivities( int &count, const BlockMatrix &seed_, BlockMatrix &hessian ){

    int run3;
    const int N  = grid.getNumPoints();
    if( fcn == 0 ) return ACADOERROR(RET_MEMBER_NOT_INITIALISED);

    const int nc = fcn[0].getDim();

    dBackward.init( N, 5*N );

    for( run3 = 0; run3 < N; run3++ ){

        DMatrix seed;
        seed_.getSubBlock( count, 0, seed, nc, 1 );
        count++;

        // EVALUATION OF THE SENSITIVITIES:
        // --------------------------------

        int run1, run2;

        double *bseed1 = new double[nc];
        double *bseed2 = new double[nc];
        double *R      = new double[nc];
        double *J      = new double[fcn[0].getNumberOfVariables() +1];
        double *H      = new double[fcn[0].getNumberOfVariables() +1];
        double *fseed  = new double[fcn[0].getNumberOfVariables() +1];

        for( run1 = 0; run1 < nc; run1++ ){
            bseed1[run1] = seed(run1,0);
            bseed2[run1] = 0.0;
        }

        for( run1 = 0; run1 < fcn[0].getNumberOfVariables()+1; run1++ )
            fseed[run1] = 0.0;

        DMatrix Dx ( nc, nx );
        DMatrix Dxa( nc, na );
        DMatrix Dp ( nc, np );
        DMatrix Du ( nc, nu );
        DMatrix Dw ( nc, nw );

        DMatrix Hx ( nx, nx );
        DMatrix Hxa( nx, na );
        DMatrix Hp ( nx, np );
        DMatrix Hu ( nx, nu );
        DMatrix Hw ( nx, nw );

        for( run2 = 0; run2 < nx; run2++ ){

            // FIRST ORDER DERIVATIVES:
            // ------------------------
            fseed[y_index[0][run2]] = 1.0;
            fcn[0].AD_forward( run3, fseed, R );
            for( run1 = 0; run1 < nc; run1++ )
                Dx( run1, run2 ) = R[run1];
            fseed[y_index[0][run2]] = 0.0;

            // SECOND ORDER DERIVATIVES:
            // -------------------------
            for( run1 = 0; run1 <= fcn[0].getNumberOfVariables(); run1++ ){
                J[run1] = 0.0;
                H[run1] = 0.0;
            }

            fcn[0].AD_backward2( run3, bseed1, bseed2, J, H );

            for( run1 = 0          ; run1 < nx            ; run1++ ) Hx ( run2, run1             ) = -H[y_index[0][run1]];
            for( run1 = nx         ; run1 < nx+na         ; run1++ ) Hxa( run2, run1-nx          ) = -H[y_index[0][run1]];
            for( run1 = nx+na      ; run1 < nx+na+np      ; run1++ ) Hp ( run2, run1-nx-na       ) = -H[y_index[0][run1]];
            for( run1 = nx+na+np   ; run1 < nx+na+np+nu   ; run1++ ) Hu ( run2, run1-nx-na-np    ) = -H[y_index[0][run1]];
            for( run1 = nx+na+np+nu; run1 < nx+na+np+nu+nw; run1++ ) Hw ( run2, run1-nx-na-np-nu ) = -H[y_index[0][run1]];
        }

        if( nx > 0 ){

            dBackward.setDense( run3, run3, Dx );

            if( nx > 0 ) hessian.addDense( run3,       run3, Hx  );
            if( na > 0 ) hessian.addDense( run3,   N + run3, Hxa );
            if( np > 0 ) hessian.addDense( run3, 2*N + run3, Hp  );
            if( nu > 0 ) hessian.addDense( run3, 3*N + run3, Hu  );
            if( nw > 0 ) hessian.addDense( run3, 4*N + run3, Hw  );
        }

        Hx.init ( na, nx );
        Hxa.init( na, na );
        Hp.init ( na, np );
        Hu.init ( na, nu );
        Hw.init ( na, nw );

        for( run2 = nx; run2 < nx+na; run2++ ){

            // FIRST ORDER DERIVATIVES:
            // ------------------------
            fseed[y_index[0][run2]] = 1.0;
            fcn[0].AD_forward( run3, fseed, R );
            for( run1 = 0; run1 < nc; run1++ )
                Dxa( run1, run2-nx ) = R[run1];
            fseed[y_index[0][run2]] = 0.0;

            // SECOND ORDER DERIVATIVES:
            // -------------------------
            for( run1 = 0; run1 <= fcn[0].getNumberOfVariables(); run1++ ){
                J[run1] = 0.0;
                H[run1] = 0.0;
            }

            fcn[0].AD_backward2( run3, bseed1, bseed2, J, H );

            for( run1 = 0          ; run1 < nx            ; run1++ ) Hx ( run2-nx, run1             ) = -H[y_index[0][run1]];
            for( run1 = nx         ; run1 < nx+na         ; run1++ ) Hxa( run2-nx, run1-nx          ) = -H[y_index[0][run1]];
            for( run1 = nx+na      ; run1 < nx+na+np      ; run1++ ) Hp ( run2-nx, run1-nx-na       ) = -H[y_index[0][run1]];
            for( run1 = nx+na+np   ; run1 < nx+na+np+nu   ; run1++ ) Hu ( run2-nx, run1-nx-na-np    ) = -H[y_index[0][run1]];
            for( run1 = nx+na+np+nu; run1 < nx+na+np+nu+nw; run1++ ) Hw ( run2-nx, run1-nx-na-np-nu ) = -H[y_index[0][run1]];
        }

        if( na > 0 ){

            dBackward.setDense( run3, N+run3, Dxa );

            if( nx > 0 ) hessian.addDense( N+run3,       run3, Hx  );
            if( na > 0 ) hessian.addDense( N+run3,   N + run3, Hxa );
            if( np > 0 ) hessian.addDense( N+run3, 2*N + run3, Hp  );
            if( nu > 0 ) hessian.addDense( N+run3, 3*N + run3, Hu  );
            if( nw > 0 ) hessian.addDense( N+run3, 4*N + run3, Hw  );
        }

        Hx.init ( np, nx );
        Hxa.init( np, na );
        Hp.init ( np, np );
        Hu.init ( np, nu );
        Hw.init ( np, nw );

        for( run2 = nx+na; run2 < nx+na+np; run2++ ){

            // FIRST ORDER DERIVATIVES:
            // ------------------------
            fseed[y_index[0][run2]] = 1.0;
            fcn[0].AD_forward( run3, fseed, R );
            for( run1 = 0; run1 < nc; run1++ )
                Dp( run1, run2-nx-na ) = R[run1];
            fseed[y_index[0][run2]] = 0.0;

            // SECOND ORDER DERIVATIVES:
            // -------------------------
            for( run1 = 0; run1 <= fcn[0].getNumberOfVariables(); run1++ ){
                J[run1] = 0.0;
                H[run1] = 0.0;
            }

            fcn[0].AD_backward2( run3, bseed1, bseed2, J, H );

            for( run1 = 0          ; run1 < nx            ; run1++ ) Hx ( run2-nx-na, run1             ) = -H[y_index[0][run1]];
            for( run1 = nx         ; run1 < nx+na         ; run1++ ) Hxa( run2-nx-na, run1-nx          ) = -H[y_index[0][run1]];
            for( run1 = nx+na      ; run1 < nx+na+np      ; run1++ ) Hp ( run2-nx-na, run1-nx-na       ) = -H[y_index[0][run1]];
            for( run1 = nx+na+np   ; run1 < nx+na+np+nu   ; run1++ ) Hu ( run2-nx-na, run1-nx-na-np    ) = -H[y_index[0][run1]];
            for( run1 = nx+na+np+nu; run1 < nx+na+np+nu+nw; run1++ ) Hw ( run2-nx-na, run1-nx-na-np-nu ) = -H[y_index[0][run1]];
        }

        if( np > 0 ){

            dBackward.setDense( run3, 2*N+run3, Dp );

            if( nx > 0 ) hessian.addDense( 2*N+run3,       run3, Hx  );
            if( na > 0 ) hessian.addDense( 2*N+run3,   N + run3, Hxa );
            if( np > 0 ) hessian.addDense( 2*N+run3, 2*N + run3, Hp  );
            if( nu > 0 ) hessian.addDense( 2*N+run3, 3*N + run3, Hu  );
            if( nw > 0 ) hessian.addDense( 2*N+run3, 4*N + run3, Hw  );
        }


        Hx.init ( nu, nx );
        Hxa.init( nu, na );
        Hp.init ( nu, np );
        Hu.init ( nu, nu );
        Hw.init ( nu, nw );

        for( run2 = nx+na+np; run2 < nx+na+np+nu; run2++ ){

            // FIRST ORDER DERIVATIVES:
            // ------------------------
            fseed[y_index[0][run2]] = 1.0;
            fcn[0].AD_forward( run3, fseed, R );
            for( run1 = 0; run1 < nc; run1++ )
                Du( run1, run2-nx-na-np ) = R[run1];
            fseed[y_index[0][run2]] = 0.0;

            // SECOND ORDER DERIVATIVES:
            // -------------------------
            for( run1 = 0; run1 <= fcn[0].getNumberOfVariables(); run1++ ){
                J[run1] = 0.0;
                H[run1] = 0.0;
            }

            fcn[0].AD_backward2( run3, bseed1, bseed2, J, H );

            for( run1 = 0          ; run1 < nx            ; run1++ ) Hx ( run2-nx-na-np, run1             ) = -H[y_index[0][run1]];
            for( run1 = nx         ; run1 < nx+na         ; run1++ ) Hxa( run2-nx-na-np, run1-nx          ) = -H[y_index[0][run1]];
            for( run1 = nx+na      ; run1 < nx+na+np      ; run1++ ) Hp ( run2-nx-na-np, run1-nx-na       ) = -H[y_index[0][run1]];
            for( run1 = nx+na+np   ; run1 < nx+na+np+nu   ; run1++ ) Hu ( run2-nx-na-np, run1-nx-na-np    ) = -H[y_index[0][run1]];
            for( run1 = nx+na+np+nu; run1 < nx+na+np+nu+nw; run1++ ) Hw ( run2-nx-na-np, run1-nx-na-np-nu ) = -H[y_index[0][run1]];
        }

        if( nu > 0 ){

            dBackward.setDense( run3, 3*N+run3, Du );

            if( nx > 0 ) hessian.addDense( 3*N+run3,       run3, Hx  );
            if( na > 0 ) hessian.addDense( 3*N+run3,   N + run3, Hxa );
            if( np > 0 ) hessian.addDense( 3*N+run3, 2*N + run3, Hp  );
            if( nu > 0 ) hessian.addDense( 3*N+run3, 3*N + run3, Hu  );
            if( nw > 0 ) hessian.addDense( 3*N+run3, 4*N + run3, Hw  );
        }

        Hx.init ( nw, nx );
        Hxa.init( nw, na );
        Hp.init ( nw, np );
        Hu.init ( nw, nu );
        Hw.init ( nw, nw );

        for( run2 = nx+na+np+nu; run2 < nx+na+np+nu+nw; run2++ ){

            // FIRST ORDER DERIVATIVES:
            // ------------------------
            fseed[y_index[0][run2]] = 1.0;
            fcn[0].AD_forward( run3, fseed, R );
            for( run1 = 0; run1 < nc; run1++ )
                Dw( run1, run2-nx-na-np-nu ) = R[run1];
            fseed[y_index[0][run2]] = 0.0;

            // SECOND ORDER DERIVATIVES:
            // -------------------------
            for( run1 = 0; run1 <= fcn[0].getNumberOfVariables(); run1++ ){
                J[run1] = 0.0;
                H[run1] = 0.0;
            }

            fcn[0].AD_backward2( run3, bseed1, bseed2, J, H );

            for( run1 = 0          ; run1 < nx            ; run1++ ) Hx ( run2-nx-na-np-nu, run1             ) = -H[y_index[0][run1]];
            for( run1 = nx         ; run1 < nx+na         ; run1++ ) Hxa( run2-nx-na-np-nu, run1-nx          ) = -H[y_index[0][run1]];
            for( run1 = nx+na      ; run1 < nx+na+np      ; run1++ ) Hp ( run2-nx-na-np-nu, run1-nx-na       ) = -H[y_index[0][run1]];
            for( run1 = nx+na+np   ; run1 < nx+na+np+nu   ; run1++ ) Hu ( run2-nx-na-np-nu, run1-nx-na-np    ) = -H[y_index[0][run1]];
            for( run1 = nx+na+np+nu; run1 < nx+na+np+nu+nw; run1++ ) Hw ( run2-nx-na-np-nu, run1-nx-na-np-nu ) = -H[y_index[0][run1]];
        }

        if( nw > 0 ){

            dBackward.setDense( run3, 4*N+run3, Dw );

            if( nx > 0 ) hessian.addDense( 4*N+run3,       run3, Hx  );
            if( na > 0 ) hessian.addDense( 4*N+run3,   N + run3, Hxa );
            if( np > 0 ) hessian.addDense( 4*N+run3, 2*N + run3, Hp  );
            if( nu > 0 ) hessian.addDense( 4*N+run3, 3*N + run3, Hu  );
            if( nw > 0 ) hessian.addDense( 4*N+run3, 4*N + run3, Hw  );
        }

        delete[] bseed1;
        delete[] bseed2;
        delete[] R     ;
        delete[] J     ;
        delete[] H     ;
        delete[] fseed ;
    }


    return SUCCESSFUL_RETURN;
}

CLOSE_NAMESPACE_ACADO

// end of file.
