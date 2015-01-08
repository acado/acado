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
 *    \file src/constraint/point_constraint.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *
 */


#include <acado/constraint/point_constraint.hpp>



BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//


PointConstraint::PointConstraint( )
                :ConstraintElement(){

    point_index = 0;

    nb    = 0;
    var   = 0;
    index = 0;
    blb   = 0;
    bub   = 0;
}

PointConstraint::PointConstraint( const Grid& grid_, int point_index_ )
                :ConstraintElement(grid_, 1, 1 ){

    point_index = point_index_;

    nb    = 0;
    var   = 0;
    index = 0;
    blb   = 0;
    bub   = 0;
}

PointConstraint::PointConstraint( const PointConstraint& rhs )
                :ConstraintElement(rhs){

    int run1;
    point_index = rhs.point_index;

    nb = rhs.nb;
    if( nb > 0 ){
        var   = (VariableType*)calloc(nb,sizeof(VariableType));
        index = (int*)calloc(nb,sizeof(int));
        blb   = (double*)calloc(nb,sizeof(double));
        bub   = (double*)calloc(nb,sizeof(double));
        for( run1 = 0; run1 < nb; run1++ ){
            var  [run1] = rhs.var  [run1];
            index[run1] = rhs.index[run1];
            blb  [run1] = rhs.blb  [run1];
            bub  [run1] = rhs.bub  [run1];
			//printf( "CON:  run1: %d, index: %d!!!\n",run1,index[run1] );
        }
    }
    else{
        var   = 0;
        index = 0;
        blb   = 0;
        bub   = 0;
    }
}

PointConstraint::~PointConstraint( ){

    if(   var != 0 ) free(  var);
    if( index != 0 ) free(index);
    if(   blb != 0 ) free(  blb);
    if(   bub != 0 ) free(  bub);
}

PointConstraint& PointConstraint::operator=( const PointConstraint& rhs ){

    int run1;

    if( this != &rhs ){

        if(   var != 0 ) free(  var);
        if( index != 0 ) free(index);
        if(   blb != 0 ) free(  blb);
        if(   bub != 0 ) free(  bub);

        ConstraintElement::operator=(rhs);

        point_index = rhs.point_index;

        nb = rhs.nb;
        if( nb > 0 ){
            var   = (VariableType*)calloc(nb,sizeof(VariableType));
            index = (int*)calloc(nb,sizeof(int));
            blb   = (double*)calloc(nb,sizeof(double));
            bub   = (double*)calloc(nb,sizeof(double));
            for( run1 = 0; run1 < nb; run1++ ){
                var  [run1] = rhs.var  [run1];
                index[run1] = rhs.index[run1];
                blb  [run1] = rhs.blb  [run1];
                bub  [run1] = rhs.bub  [run1];
				//printf( "CPY:  run1: %d, index: %d!!!\n",run1,index[run1] );
            }
        }
        else{
            var   = 0;
            index = 0;
            blb   = 0;
            bub   = 0;
        }
    }
    return *this;
}



returnValue PointConstraint::add( const double lb_, const Expression& arg, const double ub_  ){

    if( fcn == 0 )
        return ACADOERROR(RET_MEMBER_NOT_INITIALISED);

    // CHECK FOR A SIMPLE BOUND:
    // -------------------------

    VariableType varType   = arg.getVariableType( );
    int          component = arg.getComponent   (0);

    if( arg.isVariable() == BT_TRUE ){
        if( varType != VT_INTERMEDIATE_STATE ){

             nb++;
             var   = (VariableType*)realloc(var  , nb*sizeof(VariableType));
             index = (int*         )realloc(index, nb*sizeof(int         ));
             blb   = (double*      )realloc(blb  , nb*sizeof(double      ));
             bub   = (double*      )realloc(bub  , nb*sizeof(double      ));

             var  [nb-1] = varType  ;
             index[nb-1] = component;
             blb  [nb-1] = lb_      ;
             bub  [nb-1] = ub_      ;
        }
    }

    // ADD THE ARGUMENT TO THE FUNCTION TO BE EVALUATED:
    // -------------------------------------------------

    fcn[0] << arg;

    lb[0] = (double*)realloc(lb[0],fcn[0].getDim()*sizeof(double));
    ub[0] = (double*)realloc(ub[0],fcn[0].getDim()*sizeof(double));

    ub[0][fcn[0].getDim()-1] = ub_;
    lb[0][fcn[0].getDim()-1] = lb_;

    return SUCCESSFUL_RETURN;
}


returnValue PointConstraint::evaluate( const OCPiterate& iter ){

    int run1;

    if( fcn == 0 ) return ACADOERROR(RET_MEMBER_NOT_INITIALISED);

    const int nc = fcn[0].getDim();

    if( nc == 0 )
		return ACADOERROR(RET_MEMBER_NOT_INITIALISED);

    DMatrix resL( nc, 1 );
    DMatrix resU( nc, 1 );

	z[0].setZ( point_index, iter );
	DVector result = fcn[0].evaluate( z[0] );

    for( run1 = 0; run1 < nc; run1++ ){
         resL( run1, 0 ) = lb[0][run1] - result(run1);
         resU( run1, 0 ) = ub[0][run1] - result(run1);
    }

    // STORE THE RESULTS:
    // ------------------

    residuumL.init(1,1);
    residuumU.init(1,1);

    residuumL.setDense( 0, 0, resL );
    residuumU.setDense( 0, 0, resU );

    return SUCCESSFUL_RETURN;
}


returnValue PointConstraint::evaluateSensitivities( ){


    // EVALUATION OF THE SENSITIVITIES:
    // --------------------------------

    int run1;
    returnValue returnvalue;

    if( fcn == 0 ) return ACADOERROR(RET_MEMBER_NOT_INITIALISED);

    const int N  = grid.getNumPoints();

    // EVALUATION OF THE SENSITIVITIES:
    // --------------------------------

    if( bSeed != 0 ){

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

        for( run1 = 0; run1 < nBDirs; run1++ )
		{
			ACADO_TRY( fcn[0].AD_backward( bseed_.getRow(run1), JJ[0] ) );
	
			if( nx > 0 ) Dx .setRow( run1, JJ[0].getX () );
			if( na > 0 ) Dxa.setRow( run1, JJ[0].getXA() );
			if( np > 0 ) Dp .setRow( run1, JJ[0].getP () );
			if( nu > 0 ) Du .setRow( run1, JJ[0].getU () );
			if( nw > 0 ) Dw .setRow( run1, JJ[0].getW () );
			
			JJ[0].setZero( );
		}

		if( nx > 0 )
			dBackward.setDense( 0, point_index , Dx );

		if( na > 0 )
			dBackward.setDense( 0, N+point_index, Dxa );

		if( np > 0 )
			dBackward.setDense( 0, 2*N+point_index, Dp );

		if( nu > 0 )
			dBackward.setDense( 0, 3*N+point_index, Du );

		if( nw > 0 )
			dBackward.setDense( 0, 4*N+point_index, Dw );

        return SUCCESSFUL_RETURN;
    }


    if( xSeed  != 0 || pSeed  != 0 || uSeed  != 0 || wSeed  != 0 ){

        if( bSeed    != 0         ) return ACADOERROR( RET_WRONG_DEFINITION_OF_SEEDS );
        if( condType != CT_SPARSE ) return ACADOERROR( RET_NOT_IMPLEMENTED_YET       );

        dForward.init( 1, 5*N );

        if( xSeed != 0 ){
            DMatrix tmp;
            xSeed->getSubBlock(0,0,tmp);
            returnvalue = computeForwardSensitivityBlock( 0, 0, &tmp );
            if( returnvalue != SUCCESSFUL_RETURN ) return ACADOERROR(returnvalue);
        }
        if( xaSeed != 0 ){
            DMatrix tmp;
            xaSeed->getSubBlock(0,0,tmp);
            returnvalue = computeForwardSensitivityBlock( nx, N+point_index, &tmp );
            if( returnvalue != SUCCESSFUL_RETURN ) return ACADOERROR(returnvalue);
        }
        if( pSeed != 0 ){
            DMatrix tmp;
            pSeed->getSubBlock(0,0,tmp);
            returnvalue = computeForwardSensitivityBlock( nx+na, 2*N+point_index, &tmp );
            if( returnvalue != SUCCESSFUL_RETURN ) return ACADOERROR(returnvalue);
        }
        if( uSeed != 0 ){
            DMatrix tmp;
            uSeed->getSubBlock(0,0,tmp);
            returnvalue = computeForwardSensitivityBlock( nx+na+np, 3*N+point_index, &tmp );
            if( returnvalue != SUCCESSFUL_RETURN ) return ACADOERROR(returnvalue);
        }
        if( wSeed != 0 ){
            DMatrix tmp;
            wSeed->getSubBlock(0,0,tmp);
            returnvalue = computeForwardSensitivityBlock( nx+na+np+nu, 4*N+point_index, &tmp );
            if( returnvalue != SUCCESSFUL_RETURN ) return ACADOERROR(returnvalue);
        }

        return SUCCESSFUL_RETURN;
    }




    return ACADOERROR(RET_NOT_IMPLEMENTED_YET);
}



returnValue PointConstraint::evaluateSensitivities( const DMatrix &seed, BlockMatrix &hessian ){

    // EVALUATION OF THE SENSITIVITIES:
    // --------------------------------

    int run1, run2;

    if( fcn == 0 ) return ACADOERROR(RET_MEMBER_NOT_INITIALISED);

    const int nc = fcn[0].getDim();
    const int N  = grid.getNumPoints();

    ASSERT( (int) seed.getNumRows() == nc );

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
        fseed[y_index[0][run2]] = 1.0;
        fcn[0].AD_forward( 0, fseed, R );
        for( run1 = 0; run1 < nc; run1++ )
            Dx( run1, run2 ) = R[run1];
        fseed[y_index[0][run2]] = 0.0;

        // SECOND ORDER DERIVATIVES:
        // -------------------------
        for( run1 = 0; run1 <= fcn[0].getNumberOfVariables(); run1++ ){
            J[run1] = 0.0;
            H[run1] = 0.0;
        }

        fcn[0].AD_backward2( 0, bseed1, bseed2, J, H );

        for( run1 = 0          ; run1 < nx            ; run1++ ) Hx ( run2, run1             ) = -H[y_index[0][run1]];
        for( run1 = nx         ; run1 < nx+na         ; run1++ ) Hxa( run2, run1-nx          ) = -H[y_index[0][run1]];
        for( run1 = nx+na      ; run1 < nx+na+np      ; run1++ ) Hp ( run2, run1-nx-na       ) = -H[y_index[0][run1]];
        for( run1 = nx+na+np   ; run1 < nx+na+np+nu   ; run1++ ) Hu ( run2, run1-nx-na-np    ) = -H[y_index[0][run1]];
        for( run1 = nx+na+np+nu; run1 < nx+na+np+nu+nw; run1++ ) Hw ( run2, run1-nx-na-np-nu ) = -H[y_index[0][run1]];
    }

    if( nx > 0 ){

        dBackward.setDense( 0, point_index, Dx );

        if( nx > 0 ) hessian.addDense( point_index,       point_index, Hx  );
        if( na > 0 ) hessian.addDense( point_index,   N + point_index, Hxa );
        if( np > 0 ) hessian.addDense( point_index, 2*N + point_index, Hp  );
        if( nu > 0 ) hessian.addDense( point_index, 3*N + point_index, Hu  );
        if( nw > 0 ) hessian.addDense( point_index, 4*N + point_index, Hw  );
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
        fcn[0].AD_forward( 0, fseed, R );
        for( run1 = 0; run1 < nc; run1++ )
            Dxa( run1, run2-nx ) = R[run1];
        fseed[y_index[0][run2]] = 0.0;

        // SECOND ORDER DERIVATIVES:
        // -------------------------
        for( run1 = 0; run1 <= fcn[0].getNumberOfVariables(); run1++ ){
            J[run1] = 0.0;
            H[run1] = 0.0;
        }

        fcn[0].AD_backward2( 0, bseed1, bseed2, J, H );

        for( run1 = 0          ; run1 < nx            ; run1++ ) Hx ( run2-nx, run1             ) = -H[y_index[0][run1]];
        for( run1 = nx         ; run1 < nx+na         ; run1++ ) Hxa( run2-nx, run1-nx          ) = -H[y_index[0][run1]];
        for( run1 = nx+na      ; run1 < nx+na+np      ; run1++ ) Hp ( run2-nx, run1-nx-na       ) = -H[y_index[0][run1]];
        for( run1 = nx+na+np   ; run1 < nx+na+np+nu   ; run1++ ) Hu ( run2-nx, run1-nx-na-np    ) = -H[y_index[0][run1]];
        for( run1 = nx+na+np+nu; run1 < nx+na+np+nu+nw; run1++ ) Hw ( run2-nx, run1-nx-na-np-nu ) = -H[y_index[0][run1]];
    }

    if( na > 0 ){

        dBackward.setDense( 0, N+point_index, Dxa );

        if( nx > 0 ) hessian.addDense( N+point_index,       point_index, Hx  );
        if( na > 0 ) hessian.addDense( N+point_index,   N + point_index, Hxa );
        if( np > 0 ) hessian.addDense( N+point_index, 2*N + point_index, Hp  );
        if( nu > 0 ) hessian.addDense( N+point_index, 3*N + point_index, Hu  );
        if( nw > 0 ) hessian.addDense( N+point_index, 4*N + point_index, Hw  );
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
        fcn[0].AD_forward( 0, fseed, R );
        for( run1 = 0; run1 < nc; run1++ )
            Dp( run1, run2-nx-na ) = R[run1];
        fseed[y_index[0][run2]] = 0.0;

        // SECOND ORDER DERIVATIVES:
        // -------------------------
        for( run1 = 0; run1 <= fcn[0].getNumberOfVariables(); run1++ ){
            J[run1] = 0.0;
            H[run1] = 0.0;
        }

        fcn[0].AD_backward2( 0, bseed1, bseed2, J, H );

        for( run1 = 0          ; run1 < nx            ; run1++ ) Hx ( run2-nx-na, run1             ) = -H[y_index[0][run1]];
        for( run1 = nx         ; run1 < nx+na         ; run1++ ) Hxa( run2-nx-na, run1-nx          ) = -H[y_index[0][run1]];
        for( run1 = nx+na      ; run1 < nx+na+np      ; run1++ ) Hp ( run2-nx-na, run1-nx-na       ) = -H[y_index[0][run1]];
        for( run1 = nx+na+np   ; run1 < nx+na+np+nu   ; run1++ ) Hu ( run2-nx-na, run1-nx-na-np    ) = -H[y_index[0][run1]];
        for( run1 = nx+na+np+nu; run1 < nx+na+np+nu+nw; run1++ ) Hw ( run2-nx-na, run1-nx-na-np-nu ) = -H[y_index[0][run1]];
    }

    if( np > 0 ){

        dBackward.setDense( 0, 2*N+point_index, Dp );

        if( nx > 0 ) hessian.addDense( 2*N+point_index,       point_index, Hx  );
        if( na > 0 ) hessian.addDense( 2*N+point_index,   N + point_index, Hxa );
        if( np > 0 ) hessian.addDense( 2*N+point_index, 2*N + point_index, Hp  );
        if( nu > 0 ) hessian.addDense( 2*N+point_index, 3*N + point_index, Hu  );
        if( nw > 0 ) hessian.addDense( 2*N+point_index, 4*N + point_index, Hw  );
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
        fcn[0].AD_forward( 0, fseed, R );
        for( run1 = 0; run1 < nc; run1++ )
            Du( run1, run2-nx-na-np ) = R[run1];
        fseed[y_index[0][run2]] = 0.0;

        // SECOND ORDER DERIVATIVES:
        // -------------------------
        for( run1 = 0; run1 <= fcn[0].getNumberOfVariables(); run1++ ){
            J[run1] = 0.0;
            H[run1] = 0.0;
        }

        fcn[0].AD_backward2( 0, bseed1, bseed2, J, H );

        for( run1 = 0          ; run1 < nx            ; run1++ ) Hx ( run2-nx-na-np, run1             ) = -H[y_index[0][run1]];
        for( run1 = nx         ; run1 < nx+na         ; run1++ ) Hxa( run2-nx-na-np, run1-nx          ) = -H[y_index[0][run1]];
        for( run1 = nx+na      ; run1 < nx+na+np      ; run1++ ) Hp ( run2-nx-na-np, run1-nx-na       ) = -H[y_index[0][run1]];
        for( run1 = nx+na+np   ; run1 < nx+na+np+nu   ; run1++ ) Hu ( run2-nx-na-np, run1-nx-na-np    ) = -H[y_index[0][run1]];
        for( run1 = nx+na+np+nu; run1 < nx+na+np+nu+nw; run1++ ) Hw ( run2-nx-na-np, run1-nx-na-np-nu ) = -H[y_index[0][run1]];
    }

    if( nu > 0 ){

        dBackward.setDense( 0, 3*N+point_index, Du );

        if( nx > 0 ) hessian.addDense( 3*N+point_index,       point_index, Hx  );
        if( na > 0 ) hessian.addDense( 3*N+point_index,   N + point_index, Hxa );
        if( np > 0 ) hessian.addDense( 3*N+point_index, 2*N + point_index, Hp  );
        if( nu > 0 ) hessian.addDense( 3*N+point_index, 3*N + point_index, Hu  );
        if( nw > 0 ) hessian.addDense( 3*N+point_index, 4*N + point_index, Hw  );
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
        fcn[0].AD_forward( 0, fseed, R );
        for( run1 = 0; run1 < nc; run1++ )
            Dw( run1, run2-nx-na-np-nu ) = R[run1];
        fseed[y_index[0][run2]] = 0.0;

        // SECOND ORDER DERIVATIVES:
        // -------------------------
        for( run1 = 0; run1 <= fcn[0].getNumberOfVariables(); run1++ ){
            J[run1] = 0.0;
            H[run1] = 0.0;
        }

        fcn[0].AD_backward2( 0, bseed1, bseed2, J, H );

        for( run1 = 0          ; run1 < nx            ; run1++ ) Hx ( run2-nx-na-np-nu, run1             ) = -H[y_index[0][run1]];
        for( run1 = nx         ; run1 < nx+na         ; run1++ ) Hxa( run2-nx-na-np-nu, run1-nx          ) = -H[y_index[0][run1]];
        for( run1 = nx+na      ; run1 < nx+na+np      ; run1++ ) Hp ( run2-nx-na-np-nu, run1-nx-na       ) = -H[y_index[0][run1]];
        for( run1 = nx+na+np   ; run1 < nx+na+np+nu   ; run1++ ) Hu ( run2-nx-na-np-nu, run1-nx-na-np    ) = -H[y_index[0][run1]];
        for( run1 = nx+na+np+nu; run1 < nx+na+np+nu+nw; run1++ ) Hw ( run2-nx-na-np-nu, run1-nx-na-np-nu ) = -H[y_index[0][run1]];
    }

    if( nw > 0 ){

        dBackward.setDense( 0, 4*N+point_index, Dw );

        if( nx > 0 ) hessian.addDense( 4*N+point_index,       point_index, Hx  );
        if( na > 0 ) hessian.addDense( 4*N+point_index,   N + point_index, Hxa );
        if( np > 0 ) hessian.addDense( 4*N+point_index, 2*N + point_index, Hp  );
        if( nu > 0 ) hessian.addDense( 4*N+point_index, 3*N + point_index, Hu  );
        if( nw > 0 ) hessian.addDense( 4*N+point_index, 4*N + point_index, Hw  );
    }

    delete[] bseed1;
    delete[] bseed2;
    delete[] R     ;
    delete[] J     ;
    delete[] H     ;
    delete[] fseed ;

    return SUCCESSFUL_RETURN;
}


returnValue PointConstraint::getBounds( const OCPiterate& iter ){


    uint run1, run2;

	//printf( "%d!!!\n",nb );
	
    for( run1 = 0; (int) run1 < nb; run1++ ){

        switch( var[run1] ){

            case VT_DIFFERENTIAL_STATE:
                if( iter.x != NULL ){
                    run2 = iter.x->getFloorIndex(grid.getTime(point_index));
					//printf( "run2: %d, index: %d!!!\n",run2,index[run1] );
                    iter.x->setLowerBound(run2,index[run1],blb[run1]);
                    iter.x->setUpperBound(run2,index[run1],bub[run1]);
                }
                else ACADOERROR( RET_INVALID_ARGUMENTS );
                break;

            case VT_ALGEBRAIC_STATE:
                if( iter.xa != NULL ){
                    run2 = iter.xa->getFloorIndex(grid.getTime(point_index));
                    iter.xa->setLowerBound(run2,index[run1],blb[run1]);
                    iter.xa->setUpperBound(run2,index[run1],bub[run1]);
                }
                else ACADOERROR( RET_INVALID_ARGUMENTS );
                break;

            case VT_PARAMETER:
                if( iter.p != NULL ){
                    run2 = iter.p->getFloorIndex(grid.getTime(point_index));
                    iter.p->setLowerBound(0,index[run1],blb[run1]);
                    iter.p->setUpperBound(0,index[run1],bub[run1]);
                }
                else ACADOERROR( RET_INVALID_ARGUMENTS );
                break;

            case VT_CONTROL:
                if( iter.u != NULL ){
                    run2 = iter.u->getFloorIndex(grid.getTime(point_index));
                    iter.u->setLowerBound(run2,index[run1],blb[run1]);
                    iter.u->setUpperBound(run2,index[run1],bub[run1]);
                }
                else ACADOERROR( RET_INVALID_ARGUMENTS );
                break;

            case VT_DISTURBANCE:
                if( iter.w != NULL ){
                    run2 = iter.w->getFloorIndex(grid.getTime(point_index));
                    iter.w->setLowerBound(run2,index[run1],blb[run1]);
                    iter.w->setUpperBound(run2,index[run1],bub[run1]);
                }
                else ACADOERROR( RET_INVALID_ARGUMENTS );
                break;

            default:
                ACADOERROR(RET_NOT_IMPLEMENTED_YET);
                break;
        }
    }

    return SUCCESSFUL_RETURN;
}



//
// PROTECTED MEMBER FUNCTIONS:
//

inline returnValue PointConstraint::computeForwardSensitivityBlock( int offset, int offset2, DMatrix *seed ){

    if( seed == 0 ) return SUCCESSFUL_RETURN;

    int run1, run2;
    returnValue returnvalue;

    const int nc = fcn[0].getDim();

    double* dresult1 = new double[nc                             ];
    double*   fseed1 = new double[fcn[0].getNumberOfVariables()+1];

    DMatrix tmp( nc, seed->getNumCols() );

    for( run1 = 0; run1 < (int) seed->getNumCols(); run1++ ){

        for( run2 = 0; run2 <= fcn[0].getNumberOfVariables(); run2++ )
            fseed1[run2] = 0.0;

        for( run2 = 0; run2 < (int) seed->getNumRows(); run2++ )
            fseed1[y_index[0][offset+run2]] = seed->operator()(run2,run1);

        returnvalue = fcn[0].AD_forward( 0, fseed1, dresult1 );
        if( returnvalue != SUCCESSFUL_RETURN ){
            delete[] dresult1;
            delete[] fseed1  ;
            return ACADOERROR(returnvalue);
        }
        for( run2 = 0; run2 < nc; run2++ )
            tmp( run2, run1 ) = dresult1[run2];
    }
    dForward.setDense( 0, offset2, tmp );

    delete[] dresult1;
    delete[] fseed1  ;

    return SUCCESSFUL_RETURN;
}



CLOSE_NAMESPACE_ACADO

// end of file.
