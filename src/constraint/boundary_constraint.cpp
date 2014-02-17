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
 *    \file src/constraint/boundary_constraint.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *
 */


#include <acado/constraint/boundary_constraint.hpp>



BEGIN_NAMESPACE_ACADO



//
// PUBLIC MEMBER FUNCTIONS:
//


BoundaryConstraint::BoundaryConstraint( )
                   :ConstraintElement(){

}


BoundaryConstraint::BoundaryConstraint( const Grid& grid_ )
                   :ConstraintElement(grid_, 2, 1 ){

}


BoundaryConstraint::BoundaryConstraint( const BoundaryConstraint& rhs )
                   :ConstraintElement(rhs){


}


BoundaryConstraint::~BoundaryConstraint( ){

}


BoundaryConstraint& BoundaryConstraint::operator=( const BoundaryConstraint& rhs ){

    if( this != &rhs ){

        ConstraintElement::operator=(rhs);

    }

    return *this;
}




returnValue BoundaryConstraint::evaluate( const OCPiterate& iter ){

    int run1;

    const int nc = getNC();
    const int T  = grid.getLastIndex();

    if( nc == 0 )  return ACADOERROR(RET_MEMBER_NOT_INITIALISED);

    DMatrix resL( nc, 1 );
    DMatrix resU( nc, 1 );


    // EVALUATION AT THE START POINT:
    // ------------------------------

	z[0].setZ( 0, iter );
	z[1].setZ( T, iter );
	
	DVector result = fcn[0].evaluate( z[0] );

    for( run1 = 0; run1 < nc; run1++ ){
        resL( run1, 0 ) = lb[0][run1] - result(run1);
        resU( run1, 0 ) = ub[0][run1] - result(run1);
    }


    // EVALUATION AT THE END POINT:
    // ------------------------------

	result = fcn[1].evaluate( z[1] );

    for( run1 = 0; run1 < nc; run1++ ){
        resL( run1, 0 ) -= result(run1);
        resU( run1, 0 ) -= result(run1);
	}


    // STORE THE RESULTS:
    // ------------------

    residuumL.init(1,1);
    residuumU.init(1,1);

    residuumL.setDense( 0, 0, resL );
    residuumU.setDense( 0, 0, resU );


    return SUCCESSFUL_RETURN;
}


returnValue BoundaryConstraint::evaluateSensitivities( ){

    int run1;

    const int N  = grid.getNumPoints();

    // EVALUATION OF THE SENSITIVITIES:
    // --------------------------------

    if( bSeed != 0 )
	{

        if( xSeed  != 0 || pSeed  != 0 || uSeed  != 0 || wSeed  != 0 ||
            xSeed2 != 0 || pSeed2 != 0 || uSeed2 != 0 || wSeed2 != 0 )
            return ACADOERROR( RET_WRONG_DEFINITION_OF_SEEDS );

        int nBDirs = bSeed->getNumRows( 0, 0 );

        DMatrix bseed_;
        bSeed->getSubBlock( 0, 0, bseed_);

        dBackward.init( 1, 5*N );

        DMatrix Dx ( nBDirs, nx );
        DMatrix Dxa( nBDirs, na );
        DMatrix Dp ( nBDirs, np );
        DMatrix Du ( nBDirs, nu );
        DMatrix Dw ( nBDirs, nw );

		// evaluate at start
        for( run1 = 0; run1 < nBDirs; run1++ )
		{
			ACADO_TRY( fcn[0].AD_backward( bseed_.getRow(run1), JJ[0], 0 ) );
	
			if( nx > 0 ) Dx .setRow( run1, JJ[0].getX () );
			if( na > 0 ) Dxa.setRow( run1, JJ[0].getXA() );
			if( np > 0 ) Dp .setRow( run1, JJ[0].getP () );
			if( nu > 0 ) Du .setRow( run1, JJ[0].getU () );
			if( nw > 0 ) Dw .setRow( run1, JJ[0].getW () );
			
			JJ[0].setZero( );
        }

		if( nx > 0 )
			dBackward.setDense( 0,   0 , Dx );

		if( na > 0 )
			dBackward.setDense( 0,   N, Dxa );

		if( np > 0 )
			dBackward.setDense( 0, 2*N, Dp );

		if( nu > 0 )
			dBackward.setDense( 0, 3*N, Du );

		if( nw > 0 )
			dBackward.setDense( 0, 4*N, Dw );

		// evaluate at end
        for( run1 = 0; run1 < nBDirs; run1++ )
		{
			ACADO_TRY( fcn[1].AD_backward( bseed_.getRow(run1), JJ[1], 0 ) );
	
			if( nx > 0 ) Dx .setRow( run1, JJ[1].getX () );
			if( na > 0 ) Dxa.setRow( run1, JJ[1].getXA() );
			if( np > 0 ) Dp .setRow( run1, JJ[1].getP () );
			if( nu > 0 ) Du .setRow( run1, JJ[1].getU () );
			if( nw > 0 ) Dw .setRow( run1, JJ[1].getW () );
			
			JJ[1].setZero( );
        }

		if( nx > 0 )
			dBackward.setDense( 0,   N-1, Dx );

		if( na > 0 )
			dBackward.setDense( 0, 2*N-1, Dxa );

		if( np > 0 )
			dBackward.setDense( 0, 3*N-1, Dp );

        if( nu > 0 )
			dBackward.setDense( 0, 4*N-1, Du );

		if( nw > 0 )
			dBackward.setDense( 0, 5*N-1, Dw );

        return SUCCESSFUL_RETURN;
    }

	// TODO: implement forward mode

    return ACADOERROR(RET_NOT_IMPLEMENTED_YET);
}


returnValue BoundaryConstraint::evaluateSensitivities( const DMatrix &seed, BlockMatrix &hessian ){

    // EVALUATION OF THE SENSITIVITIES:
    // --------------------------------

    int run1, run2;

    const int nc = getNC();
    const int N  = grid.getNumPoints();

    ASSERT( (int) seed.getNumRows() == nc );

    double *bseed1 = new double[nc];
    double *bseed2 = new double[nc];
    double *R      = new double[nc];

    double *J1      = new double[fcn[0].getNumberOfVariables() +1];
    double *H1      = new double[fcn[0].getNumberOfVariables() +1];
    double *fseed1  = new double[fcn[0].getNumberOfVariables() +1];

    double *J2      = new double[fcn[1].getNumberOfVariables() +1];
    double *H2      = new double[fcn[1].getNumberOfVariables() +1];
    double *fseed2  = new double[fcn[1].getNumberOfVariables() +1];

    for( run1 = 0; run1 < nc; run1++ ){
        bseed1[run1] = seed(run1,0);
        bseed2[run1] = 0.0;
    }

    for( run1 = 0; run1 < fcn[0].getNumberOfVariables()+1; run1++ )
        fseed1[run1] = 0.0;

    for( run1 = 0; run1 < fcn[1].getNumberOfVariables()+1; run1++ )
        fseed2[run1] = 0.0;

    dBackward.init( 1, 5*N );

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
        fseed1[y_index[0][run2]] = 1.0;
        fcn[0].AD_forward( 0, fseed1, R );
        for( run1 = 0; run1 < nc; run1++ )
            Dx( run1, run2 ) = R[run1];
        fseed1[y_index[0][run2]] = 0.0;

        // SECOND ORDER DERIVATIVES:
        // -------------------------
        for( run1 = 0; run1 <= fcn[0].getNumberOfVariables(); run1++ ){
            J1[run1] = 0.0;
            H1[run1] = 0.0;
        }

        fcn[0].AD_backward2( 0, bseed1, bseed2, J1, H1 );

        for( run1 = 0          ; run1 < nx            ; run1++ ) Hx ( run2, run1             ) = -H1[y_index[0][run1]];
        for( run1 = nx         ; run1 < nx+na         ; run1++ ) Hxa( run2, run1-nx          ) = -H1[y_index[0][run1]];
        for( run1 = nx+na      ; run1 < nx+na+np      ; run1++ ) Hp ( run2, run1-nx-na       ) = -H1[y_index[0][run1]];
        for( run1 = nx+na+np   ; run1 < nx+na+np+nu   ; run1++ ) Hu ( run2, run1-nx-na-np    ) = -H1[y_index[0][run1]];
        for( run1 = nx+na+np+nu; run1 < nx+na+np+nu+nw; run1++ ) Hw ( run2, run1-nx-na-np-nu ) = -H1[y_index[0][run1]];
    }

    if( nx > 0 ){

        dBackward.setDense( 0, 0, Dx );

        if( nx > 0 ) hessian.addDense( 0,   0, Hx  );
        if( na > 0 ) hessian.addDense( 0,   N, Hxa );
        if( np > 0 ) hessian.addDense( 0, 2*N, Hp  );
        if( nu > 0 ) hessian.addDense( 0, 3*N, Hu  );
        if( nw > 0 ) hessian.addDense( 0, 4*N, Hw  );
    }

    Hx.init ( nx, nx );
    Hxa.init( nx, na );
    Hp.init ( nx, np );
    Hu.init ( nx, nu );
    Hw.init ( nx, nw );


    for( run2 = 0; run2 < nx; run2++ ){

        // FIRST ORDER DERIVATIVES:
        // ------------------------
        fseed2[y_index[1][run2]] = 1.0;
        fcn[1].AD_forward( 0, fseed2, R );
        for( run1 = 0; run1 < nc; run1++ )
            Dx( run1, run2 ) = R[run1];
        fseed2[y_index[1][run2]] = 0.0;

        // SECOND ORDER DERIVATIVES:
        // -------------------------
        for( run1 = 0; run1 <= fcn[1].getNumberOfVariables(); run1++ ){
            J2[run1] = 0.0;
            H2[run1] = 0.0;
        }

        fcn[1].AD_backward2( 0, bseed1, bseed2, J2, H2 );

        for( run1 = 0          ; run1 < nx            ; run1++ ) Hx ( run2, run1             ) = -H2[y_index[1][run1]];
        for( run1 = nx         ; run1 < nx+na         ; run1++ ) Hxa( run2, run1-nx          ) = -H2[y_index[1][run1]];
        for( run1 = nx+na      ; run1 < nx+na+np      ; run1++ ) Hp ( run2, run1-nx-na       ) = -H2[y_index[1][run1]];
        for( run1 = nx+na+np   ; run1 < nx+na+np+nu   ; run1++ ) Hu ( run2, run1-nx-na-np    ) = -H2[y_index[1][run1]];
        for( run1 = nx+na+np+nu; run1 < nx+na+np+nu+nw; run1++ ) Hw ( run2, run1-nx-na-np-nu ) = -H2[y_index[1][run1]];
    }

    if( nx > 0 ){

        dBackward.setDense( 0, N-1, Dx );

        if( nx > 0 ) hessian.addDense( N-1,   N-1, Hx  );
        if( na > 0 ) hessian.addDense( N-1, 2*N-1, Hxa );
        if( np > 0 ) hessian.addDense( N-1, 3*N-1, Hp  );
        if( nu > 0 ) hessian.addDense( N-1, 4*N-1, Hu  );
        if( nw > 0 ) hessian.addDense( N-1, 5*N-1, Hw  );
    }

    Hx.init ( na, nx );
    Hxa.init( na, na );
    Hp.init ( na, np );
    Hu.init ( na, nu );
    Hw.init ( na, nw );


    for( run2 = nx; run2 < nx+na; run2++ ){

        // FIRST ORDER DERIVATIVES:
        // ------------------------
        fseed1[y_index[0][run2]] = 1.0;
        fcn[0].AD_forward( 0, fseed1, R );
        for( run1 = 0; run1 < nc; run1++ )
            Dxa( run1, run2-nx ) = R[run1];
        fseed1[y_index[0][run2]] = 0.0;

        // SECOND ORDER DERIVATIVES:
        // -------------------------
        for( run1 = 0; run1 <= fcn[0].getNumberOfVariables(); run1++ ){
            J1[run1] = 0.0;
            H1[run1] = 0.0;
        }

        fcn[0].AD_backward2( 0, bseed1, bseed2, J1, H1 );

        for( run1 = 0          ; run1 < nx            ; run1++ ) Hx ( run2-nx, run1             ) = -H1[y_index[0][run1]];
        for( run1 = nx         ; run1 < nx+na         ; run1++ ) Hxa( run2-nx, run1-nx          ) = -H1[y_index[0][run1]];
        for( run1 = nx+na      ; run1 < nx+na+np      ; run1++ ) Hp ( run2-nx, run1-nx-na       ) = -H1[y_index[0][run1]];
        for( run1 = nx+na+np   ; run1 < nx+na+np+nu   ; run1++ ) Hu ( run2-nx, run1-nx-na-np    ) = -H1[y_index[0][run1]];
        for( run1 = nx+na+np+nu; run1 < nx+na+np+nu+nw; run1++ ) Hw ( run2-nx, run1-nx-na-np-nu ) = -H1[y_index[0][run1]];
    }

    if( na > 0 ){

        dBackward.setDense( 0, N, Dxa );

        if( nx > 0 ) hessian.addDense( N,   0, Hx  );
        if( na > 0 ) hessian.addDense( N,   N, Hxa );
        if( np > 0 ) hessian.addDense( N, 2*N, Hp  );
        if( nu > 0 ) hessian.addDense( N, 3*N, Hu  );
        if( nw > 0 ) hessian.addDense( N, 4*N, Hw  );
    }

    Hx.init ( na, nx );
    Hxa.init( na, na );
    Hp.init ( na, np );
    Hu.init ( na, nu );
    Hw.init ( na, nw );


    for( run2 = nx; run2 < nx+na; run2++ ){

        // FIRST ORDER DERIVATIVES:
        // ------------------------
        fseed2[y_index[1][run2]] = 1.0;
        fcn[1].AD_forward( 0, fseed2, R );
        for( run1 = 0; run1 < nc; run1++ )
            Dxa( run1, run2-nx ) = R[run1];
        fseed2[y_index[1][run2]] = 0.0;

        // SECOND ORDER DERIVATIVES:
        // -------------------------
        for( run1 = 0; run1 <= fcn[1].getNumberOfVariables(); run1++ ){
            J2[run1] = 0.0;
            H2[run1] = 0.0;
        }

        fcn[1].AD_backward2( 0, bseed1, bseed2, J2, H2 );

        for( run1 = 0          ; run1 < nx            ; run1++ ) Hx ( run2-nx, run1             ) = -H2[y_index[1][run1]];
        for( run1 = nx         ; run1 < nx+na         ; run1++ ) Hxa( run2-nx, run1-nx          ) = -H2[y_index[1][run1]];
        for( run1 = nx+na      ; run1 < nx+na+np      ; run1++ ) Hp ( run2-nx, run1-nx-na       ) = -H2[y_index[1][run1]];
        for( run1 = nx+na+np   ; run1 < nx+na+np+nu   ; run1++ ) Hu ( run2-nx, run1-nx-na-np    ) = -H2[y_index[1][run1]];
        for( run1 = nx+na+np+nu; run1 < nx+na+np+nu+nw; run1++ ) Hw ( run2-nx, run1-nx-na-np-nu ) = -H2[y_index[1][run1]];
    }

    if( na > 0 ){

        dBackward.setDense( 0, 2*N-1, Dxa );

        if( nx > 0 ) hessian.addDense( 2*N-1,   N-1, Hx  );
        if( na > 0 ) hessian.addDense( 2*N-1, 2*N-1, Hxa );
        if( np > 0 ) hessian.addDense( 2*N-1, 3*N-1, Hp  );
        if( nu > 0 ) hessian.addDense( 2*N-1, 4*N-1, Hu  );
        if( nw > 0 ) hessian.addDense( 2*N-1, 5*N-1, Hw  );
    }

    Hx.init ( np, nx );
    Hxa.init( np, na );
    Hp.init ( np, np );
    Hu.init ( np, nu );
    Hw.init ( np, nw );


    for( run2 = nx+na; run2 < nx+na+np; run2++ ){

        // FIRST ORDER DERIVATIVES:
        // ------------------------
        fseed1[y_index[0][run2]] = 1.0;
        fcn[0].AD_forward( 0, fseed1, R );
        for( run1 = 0; run1 < nc; run1++ )
            Dp( run1, run2-nx-na ) = R[run1];
        fseed1[y_index[0][run2]] = 0.0;

        // SECOND ORDER DERIVATIVES:
        // -------------------------
        for( run1 = 0; run1 <= fcn[0].getNumberOfVariables(); run1++ ){
            J1[run1] = 0.0;
            H1[run1] = 0.0;
        }

        fcn[0].AD_backward2( 0, bseed1, bseed2, J1, H1 );

        for( run1 = 0          ; run1 < nx            ; run1++ ) Hx ( run2-nx-na, run1             ) = -H1[y_index[0][run1]];
        for( run1 = nx         ; run1 < nx+na         ; run1++ ) Hxa( run2-nx-na, run1-nx          ) = -H1[y_index[0][run1]];
        for( run1 = nx+na      ; run1 < nx+na+np      ; run1++ ) Hp ( run2-nx-na, run1-nx-na       ) = -H1[y_index[0][run1]];
        for( run1 = nx+na+np   ; run1 < nx+na+np+nu   ; run1++ ) Hu ( run2-nx-na, run1-nx-na-np    ) = -H1[y_index[0][run1]];
        for( run1 = nx+na+np+nu; run1 < nx+na+np+nu+nw; run1++ ) Hw ( run2-nx-na, run1-nx-na-np-nu ) = -H1[y_index[0][run1]];
    }

    if( np > 0 ){

        dBackward.setDense( 0, 2*N, Dp );

        if( nx > 0 ) hessian.addDense( 2*N,   0, Hx  );
        if( na > 0 ) hessian.addDense( 2*N,   N, Hxa );
        if( np > 0 ) hessian.addDense( 2*N, 2*N, Hp  );
        if( nu > 0 ) hessian.addDense( 2*N, 3*N, Hu  );
        if( nw > 0 ) hessian.addDense( 2*N, 4*N, Hw  );
    }


    Hx.init ( np, nx );
    Hxa.init( np, na );
    Hp.init ( np, np );
    Hu.init ( np, nu );
    Hw.init ( np, nw );


    for( run2 = nx+na; run2 < nx+na+np; run2++ ){

        // FIRST ORDER DERIVATIVES:
        // ------------------------
        fseed2[y_index[1][run2]] = 1.0;
        fcn[1].AD_forward( 0, fseed2, R );
        for( run1 = 0; run1 < nc; run1++ )
            Dp( run1, run2-nx-na ) = R[run1];
        fseed2[y_index[1][run2]] = 0.0;

        // SECOND ORDER DERIVATIVES:
        // -------------------------
        for( run1 = 0; run1 <= fcn[1].getNumberOfVariables(); run1++ ){
            J2[run1] = 0.0;
            H2[run1] = 0.0;
        }

        fcn[1].AD_backward2( 0, bseed1, bseed2, J2, H2 );

        for( run1 = 0          ; run1 < nx            ; run1++ ) Hx ( run2-nx-na, run1             ) = -H2[y_index[1][run1]];
        for( run1 = nx         ; run1 < nx+na         ; run1++ ) Hxa( run2-nx-na, run1-nx          ) = -H2[y_index[1][run1]];
        for( run1 = nx+na      ; run1 < nx+na+np      ; run1++ ) Hp ( run2-nx-na, run1-nx-na       ) = -H2[y_index[1][run1]];
        for( run1 = nx+na+np   ; run1 < nx+na+np+nu   ; run1++ ) Hu ( run2-nx-na, run1-nx-na-np    ) = -H2[y_index[1][run1]];
        for( run1 = nx+na+np+nu; run1 < nx+na+np+nu+nw; run1++ ) Hw ( run2-nx-na, run1-nx-na-np-nu ) = -H2[y_index[1][run1]];
    }

    if( np > 0 ){

        dBackward.setDense( 0, 3*N-1, Dp );

        if( nx > 0 ) hessian.addDense( 3*N-1,   N-1, Hx  );
        if( na > 0 ) hessian.addDense( 3*N-1, 2*N-1, Hxa );
        if( np > 0 ) hessian.addDense( 3*N-1, 3*N-1, Hp  );
        if( nu > 0 ) hessian.addDense( 3*N-1, 4*N-1, Hu  );
        if( nw > 0 ) hessian.addDense( 3*N-1, 5*N-1, Hw  );
    }


    Hx.init ( nu, nx );
    Hxa.init( nu, na );
    Hp.init ( nu, np );
    Hu.init ( nu, nu );
    Hw.init ( nu, nw );


    for( run2 = nx+na+np; run2 < nx+na+np+nu; run2++ ){

        // FIRST ORDER DERIVATIVES:
        // ------------------------
        fseed1[y_index[0][run2]] = 1.0;
        fcn[0].AD_forward( 0, fseed1, R );
        for( run1 = 0; run1 < nc; run1++ )
            Du( run1, run2-nx-na-np ) = R[run1];
        fseed1[y_index[0][run2]] = 0.0;

        // SECOND ORDER DERIVATIVES:
        // -------------------------
        for( run1 = 0; run1 <= fcn[0].getNumberOfVariables(); run1++ ){
            J1[run1] = 0.0;
            H1[run1] = 0.0;
        }

        fcn[0].AD_backward2( 0, bseed1, bseed2, J1, H1 );

        for( run1 = 0          ; run1 < nx            ; run1++ ) Hx ( run2-nx-na-np, run1             ) = -H1[y_index[0][run1]];
        for( run1 = nx         ; run1 < nx+na         ; run1++ ) Hxa( run2-nx-na-np, run1-nx          ) = -H1[y_index[0][run1]];
        for( run1 = nx+na      ; run1 < nx+na+np      ; run1++ ) Hp ( run2-nx-na-np, run1-nx-na       ) = -H1[y_index[0][run1]];
        for( run1 = nx+na+np   ; run1 < nx+na+np+nu   ; run1++ ) Hu ( run2-nx-na-np, run1-nx-na-np    ) = -H1[y_index[0][run1]];
        for( run1 = nx+na+np+nu; run1 < nx+na+np+nu+nw; run1++ ) Hw ( run2-nx-na-np, run1-nx-na-np-nu ) = -H1[y_index[0][run1]];
    }

    if( nu > 0 ){

        dBackward.setDense( 0, 3*N, Du );

        if( nx > 0 ) hessian.addDense( 3*N,   0, Hx  );
        if( na > 0 ) hessian.addDense( 3*N,   N, Hxa );
        if( np > 0 ) hessian.addDense( 3*N, 2*N, Hp  );
        if( nu > 0 ) hessian.addDense( 3*N, 3*N, Hu  );
        if( nw > 0 ) hessian.addDense( 3*N, 4*N, Hw  );
    }

    Hx.init ( nu, nx );
    Hxa.init( nu, na );
    Hp.init ( nu, np );
    Hu.init ( nu, nu );
    Hw.init ( nu, nw );


    for( run2 = nx+na+np; run2 < nx+na+np+nu; run2++ ){

        // FIRST ORDER DERIVATIVES:
        // ------------------------
        fseed2[y_index[1][run2]] = 1.0;
        fcn[1].AD_forward( 0, fseed2, R );
        for( run1 = 0; run1 < nc; run1++ )
            Du( run1, run2-nx-na-np ) = R[run1];
        fseed2[y_index[1][run2]] = 0.0;

        // SECOND ORDER DERIVATIVES:
        // -------------------------
        for( run1 = 0; run1 <= fcn[1].getNumberOfVariables(); run1++ ){
            J2[run1] = 0.0;
            H2[run1] = 0.0;
        }

        fcn[1].AD_backward2( 0, bseed1, bseed2, J2, H2 );

        for( run1 = 0          ; run1 < nx            ; run1++ ) Hx ( run2-nx-na-np, run1             ) = -H2[y_index[1][run1]];
        for( run1 = nx         ; run1 < nx+na         ; run1++ ) Hxa( run2-nx-na-np, run1-nx          ) = -H2[y_index[1][run1]];
        for( run1 = nx+na      ; run1 < nx+na+np      ; run1++ ) Hp ( run2-nx-na-np, run1-nx-na       ) = -H2[y_index[1][run1]];
        for( run1 = nx+na+np   ; run1 < nx+na+np+nu   ; run1++ ) Hu ( run2-nx-na-np, run1-nx-na-np    ) = -H2[y_index[1][run1]];
        for( run1 = nx+na+np+nu; run1 < nx+na+np+nu+nw; run1++ ) Hw ( run2-nx-na-np, run1-nx-na-np-nu ) = -H2[y_index[1][run1]];
    }

    if( nu > 0 ){

        dBackward.setDense( 0, 4*N-1, Du );

        if( nx > 0 ) hessian.addDense( 4*N-1,   N-1, Hx  );
        if( na > 0 ) hessian.addDense( 4*N-1, 2*N-1, Hxa );
        if( np > 0 ) hessian.addDense( 4*N-1, 3*N-1, Hp  );
        if( nu > 0 ) hessian.addDense( 4*N-1, 4*N-1, Hu  );
        if( nw > 0 ) hessian.addDense( 4*N-1, 5*N-1, Hw  );
    }

    Hx.init ( nw, nx );
    Hxa.init( nw, na );
    Hp.init ( nw, np );
    Hu.init ( nw, nu );
    Hw.init ( nw, nw );


    for( run2 = nx+na+np+nu; run2 < nx+na+np+nu+nw; run2++ ){

        // FIRST ORDER DERIVATIVES:
        // ------------------------
        fseed1[y_index[0][run2]] = 1.0;
        fcn[0].AD_forward( 0, fseed1, R );
        for( run1 = 0; run1 < nc; run1++ )
            Dw( run1, run2-nx-na-np-nu ) = R[run1];
        fseed1[y_index[0][run2]] = 0.0;

        // SECOND ORDER DERIVATIVES:
        // -------------------------
        for( run1 = 0; run1 <= fcn[0].getNumberOfVariables(); run1++ ){
            J1[run1] = 0.0;
            H1[run1] = 0.0;
        }

        fcn[0].AD_backward2( 0, bseed1, bseed2, J1, H1 );

        for( run1 = 0          ; run1 < nx            ; run1++ ) Hx ( run2-nx-na-np-nu, run1             ) = -H1[y_index[0][run1]];
        for( run1 = nx         ; run1 < nx+na         ; run1++ ) Hxa( run2-nx-na-np-nu, run1-nx          ) = -H1[y_index[0][run1]];
        for( run1 = nx+na      ; run1 < nx+na+np      ; run1++ ) Hp ( run2-nx-na-np-nu, run1-nx-na       ) = -H1[y_index[0][run1]];
        for( run1 = nx+na+np   ; run1 < nx+na+np+nu   ; run1++ ) Hu ( run2-nx-na-np-nu, run1-nx-na-np    ) = -H1[y_index[0][run1]];
        for( run1 = nx+na+np+nu; run1 < nx+na+np+nu+nw; run1++ ) Hw ( run2-nx-na-np-nu, run1-nx-na-np-nu ) = -H1[y_index[0][run1]];
    }

    if( nw > 0 ){

        dBackward.setDense( 0, 4*N, Dw );

        if( nx > 0 ) hessian.addDense( 4*N,   0, Hx  );
        if( na > 0 ) hessian.addDense( 4*N,   N, Hxa );
        if( np > 0 ) hessian.addDense( 4*N, 2*N, Hp  );
        if( nu > 0 ) hessian.addDense( 4*N, 3*N, Hu  );
        if( nw > 0 ) hessian.addDense( 4*N, 4*N, Hw  );
    }

    Hx.init ( nw, nx );
    Hxa.init( nw, na );
    Hp.init ( nw, np );
    Hu.init ( nw, nu );
    Hw.init ( nw, nw );


    for( run2 = nx+na+np+nu; run2 < nx+na+np+nu+nw; run2++ ){

        // FIRST ORDER DERIVATIVES:
        // ------------------------
        fseed2[y_index[1][run2]] = 1.0;
        fcn[1].AD_forward( 0, fseed2, R );
        for( run1 = 0; run1 < nc; run1++ )
            Dw( run1, run2-nx-na-np-nu ) = R[run1];
        fseed2[y_index[1][run2]] = 0.0;

        // SECOND ORDER DERIVATIVES:
        // -------------------------
        for( run1 = 0; run1 <= fcn[1].getNumberOfVariables(); run1++ ){
            J2[run1] = 0.0;
            H2[run1] = 0.0;
        }

        fcn[1].AD_backward2( 0, bseed1, bseed2, J2, H2 );

        for( run1 = 0          ; run1 < nx            ; run1++ ) Hx ( run2-nx-na-np-nu, run1             ) = -H2[y_index[1][run1]];
        for( run1 = nx         ; run1 < nx+na         ; run1++ ) Hxa( run2-nx-na-np-nu, run1-nx          ) = -H2[y_index[1][run1]];
        for( run1 = nx+na      ; run1 < nx+na+np      ; run1++ ) Hp ( run2-nx-na-np-nu, run1-nx-na       ) = -H2[y_index[1][run1]];
        for( run1 = nx+na+np   ; run1 < nx+na+np+nu   ; run1++ ) Hu ( run2-nx-na-np-nu, run1-nx-na-np    ) = -H2[y_index[1][run1]];
        for( run1 = nx+na+np+nu; run1 < nx+na+np+nu+nw; run1++ ) Hw ( run2-nx-na-np-nu, run1-nx-na-np-nu ) = -H2[y_index[1][run1]];
    }

    if( nw > 0 ){

        dBackward.setDense( 0, 5*N-1, Dw );

        if( nx > 0 ) hessian.addDense( 5*N-1,   N-1, Hx  );
        if( na > 0 ) hessian.addDense( 5*N-1, 2*N-1, Hxa );
        if( np > 0 ) hessian.addDense( 5*N-1, 3*N-1, Hp  );
        if( nu > 0 ) hessian.addDense( 5*N-1, 4*N-1, Hu  );
        if( nw > 0 ) hessian.addDense( 5*N-1, 5*N-1, Hw  );
    }

    delete[] bseed1;
    delete[] bseed2;
    delete[] R     ;
    delete[] J1    ;
    delete[] H1    ;
    delete[] fseed1;
    delete[] J2    ;
    delete[] H2    ;
    delete[] fseed2;

    return SUCCESSFUL_RETURN;
}



CLOSE_NAMESPACE_ACADO

// end of file.
