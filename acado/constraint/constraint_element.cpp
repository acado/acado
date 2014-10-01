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
 *    \file src/constraint/constraint_element.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *
 */

#include <acado/constraint/constraint_element.hpp>


BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//


ConstraintElement::ConstraintElement( ){

    fcn     = 0;
    lb      = 0;
    ub      = 0;
    y_index = 0;
    t_index = 0;

	z       = 0;
	JJ      = 0;

    nx      = 0;
    na      = 0;
    nu      = 0;
    np      = 0;
    nw      = 0;
    ny      = 0;
    nFcn    = 0;
    nB      = 0;

    xSeed         = 0     ;
    xaSeed        = 0     ;
    pSeed         = 0     ;
    uSeed         = 0     ;
    wSeed         = 0     ;
    bSeed         = 0     ;
    xSeed2        = 0     ;
    xaSeed2       = 0     ;
    pSeed2        = 0     ;
    uSeed2        = 0     ;
    wSeed2        = 0     ;
    bSeed2        = 0     ;

    condType      = CT_SPARSE;
}


ConstraintElement::ConstraintElement( const Grid& grid_, int nFcn_, int nB_ ){

    int run1;

    grid    = grid_;
    nFcn    = nFcn_;
    nB      = nB_  ;

    fcn     = new Function[nFcn];
    lb      = new double* [  nB];
    ub      = new double* [  nB];

    for( run1 = 0; run1 < nB; run1++ ){
        lb[run1] = 0;
        ub[run1] = 0;
    }

    y_index = new int*    [nFcn];
    t_index = new int     [nFcn];

    for( run1 = 0; run1 < nFcn; run1++ ){
        y_index[run1] = 0;
    }

	z       = 0;
	JJ      = 0;

    nx      = 0;
    na      = 0;
    nu      = 0;
    np      = 0;
    nw      = 0;
    ny      = 0;

    xSeed         = 0     ;
    xaSeed        = 0     ;
    pSeed         = 0     ;
    uSeed         = 0     ;
    wSeed         = 0     ;
    bSeed         = 0     ;
    xSeed2        = 0     ;
    xaSeed2       = 0     ;
    pSeed2        = 0     ;
    uSeed2        = 0     ;
    wSeed2        = 0     ;
    bSeed2        = 0     ;

    condType      = CT_SPARSE;
}


ConstraintElement::ConstraintElement( const ConstraintElement& rhs ){

    int run1, run2;

    grid    = rhs.grid;
    nFcn    = rhs.nFcn;
    nB      = rhs.nB  ;

    if( rhs.fcn     != 0 )  fcn     = new Function[nFcn];
    else                    fcn     = 0                 ;

    if( rhs.lb      != 0 )  lb      = new double* [nB  ];
    else                    lb      = 0                 ;

    if( rhs.ub      != 0 )  ub      = new double* [nB  ];
    else                    ub      = 0                 ;

    if( rhs.y_index != 0 )  y_index = new int*    [nFcn];
    else                    y_index = 0                 ;

    if( rhs.t_index != 0 )  t_index = new int     [nFcn];
    else                    t_index = 0                 ;

	if( rhs.z       != 0 )  z       = new EvaluationPoint( *(rhs.z) );
	else                    z       = 0                 ;

	if( rhs.JJ      != 0 )  JJ      = new EvaluationPoint( *(rhs.JJ) );
	else                    JJ      = 0                 ;
	
    nx      = rhs.nx;
    na      = rhs.na;
    nu      = rhs.nu;
    np      = rhs.np;
    nw      = rhs.nw;
    ny      = rhs.ny;

    for( run1 = 0; run1 < nFcn; run1++ ){

        fcn[run1] = rhs.fcn[run1];

        if( ny > 0 ){
            y_index[run1] = new int[ny];
            for( run2 = 0; run2 < ny; run2++ )
                y_index[run1][run2] = rhs.y_index[run1][run2];
        }
        else{
            y_index[run1] = 0;
        }

        t_index[run1] = rhs.t_index[run1];
    }

    if( fcn == 0 ){
        for( run1 = 0; run1 < nB; run1++ ){
            lb[run1] = 0;
            ub[run1] = 0;
        }
    }
    else{
        for( run1 = 0; run1 < nB; run1++ ){

            if( fcn[0].getDim() > 0 ){
                lb[run1] = (double*)calloc(fcn[0].getDim(),sizeof(double));
                ub[run1] = (double*)calloc(fcn[0].getDim(),sizeof(double));
            }
            else{
                lb[run1] = 0;
                ub[run1] = 0;
            }

            for( run2 = 0; run2 < fcn[0].getDim(); run2++ ){
                lb[run1][run2] = rhs.lb[run1][run2];
                ub[run1][run2] = rhs.ub[run1][run2];
            }
        }
    }

    if( rhs.xSeed != 0 )  xSeed  = new BlockMatrix(*rhs.xSeed );
    else                  xSeed  = 0                           ;
    if( rhs.xaSeed != 0 ) xaSeed = new BlockMatrix(*rhs.xaSeed);
    else                  xaSeed = 0                           ;
    if( rhs.pSeed != 0 )  pSeed  = new BlockMatrix(*rhs.pSeed) ;
    else                  pSeed  = 0                           ;
    if( rhs.uSeed != 0 )  uSeed  = new BlockMatrix(*rhs.uSeed) ;
    else                  uSeed  = 0                           ;
    if( rhs.wSeed != 0 )  wSeed  = new BlockMatrix(*rhs.wSeed) ;
    else                  wSeed  = 0                           ;

    if( rhs.bSeed != 0 )  bSeed  = new BlockMatrix(*rhs.bSeed );
    else                  bSeed  = 0                           ;

    if( rhs.xSeed2 != 0 )  xSeed2  = new BlockMatrix(*rhs.xSeed2) ;
    else                   xSeed2  = 0                            ;
    if( rhs.xaSeed2 != 0 ) xaSeed2 = new BlockMatrix(*rhs.xaSeed2);
    else                   xaSeed2 = 0                            ;
    if( rhs.pSeed2 != 0 )  pSeed2  = new BlockMatrix(*rhs.pSeed2 );
    else                   pSeed2  = 0                            ;
    if( rhs.uSeed2 != 0 )  uSeed2  = new BlockMatrix(*rhs.uSeed2 );
    else                   uSeed2  = 0                            ;
    if( rhs.wSeed2 != 0 )  wSeed2  = new BlockMatrix(*rhs.wSeed2 );
    else                   wSeed2  = 0                            ;

    if( rhs.bSeed != 0 )   bSeed2 = new BlockMatrix(*rhs.bSeed );
    else                   bSeed2 = 0                           ;


    residuumL = rhs.residuumL;
    residuumU = rhs.residuumU;
    dForward  = rhs.dForward ;
    dBackward = rhs.dBackward;

    condType  = rhs.condType;
}


ConstraintElement::~ConstraintElement( ){

    int run1;

    if( fcn != 0 )
        delete[] fcn;

    for( run1 = 0; run1 < nB; run1++ ){

        if( lb[run1] != 0 )
            free(lb[run1]);

        if( ub[run1] != 0 )
            free(ub[run1]);
    }

    if( lb != 0 )
        delete[] lb;

    if( ub != 0 )
        delete[] ub;

    for( run1 = 0; run1 < nFcn; run1++ ){

        if( y_index[run1] != 0 )
            delete[] y_index[run1];
    }

    if( y_index != 0 )
        delete[] y_index;

    if( t_index != 0 )
        delete[] t_index;

	if ( z != 0 )
		delete[] z;
	
	if ( JJ != 0 )
		delete[] JJ;

    if( xSeed   != 0 ) delete xSeed  ;
    if( xaSeed  != 0 ) delete xaSeed ;
    if( pSeed   != 0 ) delete pSeed  ;
    if( uSeed   != 0 ) delete uSeed  ;
    if( wSeed   != 0 ) delete wSeed  ;

    if( bSeed   != 0 ) delete bSeed  ;

    if( xSeed2  != 0 ) delete xSeed2 ;
    if( xaSeed2 != 0 ) delete xaSeed2;
    if( pSeed2  != 0 ) delete pSeed2 ;
    if( uSeed2  != 0 ) delete uSeed2 ;
    if( wSeed2  != 0 ) delete wSeed2 ;

    if( bSeed2  != 0 ) delete bSeed2 ;
}


ConstraintElement& ConstraintElement::operator=( const ConstraintElement& rhs ){

    int run1, run2;

    if( this != &rhs ){

        if( fcn != 0 )
            delete[] fcn;

        for( run1 = 0; run1 < nB; run1++ ){

            if( lb[run1] != 0 )
                free(lb[run1]);

            if( ub[run1] != 0 )
                free(ub[run1]);
        }

        if( lb != 0 )
            delete[] lb;

        if( ub != 0 )
            delete[] ub;

        for( run1 = 0; run1 < nFcn; run1++ ){

            if( y_index[run1] != 0 )
                delete[] y_index[run1];
        }

        if( y_index != 0 )
            delete[] y_index;

        if( t_index != 0 )
            delete[] t_index;

		if ( z != 0 )
			delete[] z;
		
		if ( JJ != 0 )
			delete[] JJ;

        if( xSeed   != 0 ) delete xSeed  ;
        if( xaSeed  != 0 ) delete xaSeed ;
        if( pSeed   != 0 ) delete pSeed  ;
        if( uSeed   != 0 ) delete uSeed  ;
        if( wSeed   != 0 ) delete wSeed  ;

        if( bSeed   != 0 ) delete bSeed  ;

        if( xSeed2  != 0 ) delete xSeed2 ;
        if( xaSeed2 != 0 ) delete xaSeed2;
        if( pSeed2  != 0 ) delete pSeed2 ;
        if( uSeed2  != 0 ) delete uSeed2 ;
        if( wSeed2  != 0 ) delete wSeed2 ;

        if( bSeed2  != 0 ) delete bSeed2 ;


        grid    = rhs.grid;
        nFcn    = rhs.nFcn;
        nB      = rhs.nB  ;

        if( rhs.fcn     != 0 )  fcn     = new Function[nFcn];
        else                    fcn     = 0                 ;

        if( rhs.lb      != 0 )  lb      = new double* [nB  ];
        else                    lb      = 0                 ;

        if( rhs.ub      != 0 )  ub      = new double* [nB  ];
        else                    ub      = 0                 ;

        if( rhs.y_index != 0 )  y_index = new int*    [nFcn];
        else                    y_index = 0                 ;

        if( rhs.t_index != 0 )  t_index = new int     [nFcn];
        else                    t_index = 0                 ;

		if( rhs.z       != 0 )  z       = new EvaluationPoint( *(rhs.z) );
		else                    z       = 0                 ;

		if( rhs.JJ      != 0 )  JJ      = new EvaluationPoint( *(rhs.JJ) );
		else                    JJ      = 0                 ;

        nx      = rhs.nx;
        na      = rhs.na;
        nu      = rhs.nu;
        np      = rhs.np;
        nw      = rhs.nw;
        ny      = rhs.ny;

        for( run1 = 0; run1 < nFcn; run1++ ){

            fcn[run1] = rhs.fcn[run1];

            if( ny > 0 ){
                y_index[run1] = new int[ny];
                for( run2 = 0; run2 < ny; run2++ )
                    y_index[run1][run2] = rhs.y_index[run1][run2];
            }
            else{
                y_index[run1] = 0;
            }

            t_index[run1] = rhs.t_index[run1];

        }

        if( fcn == 0 ){
            for( run1 = 0; run1 < nB; run1++ ){
                lb[run1] = 0;
                ub[run1] = 0;
            }
        }
        else{
            for( run1 = 0; run1 < nB; run1++ ){

                if( fcn[0].getDim() > 0 ){
                    lb[run1] = (double*)calloc(fcn[0].getDim(),sizeof(double));
                    ub[run1] = (double*)calloc(fcn[0].getDim(),sizeof(double));
                }
                else{
                    lb[run1] = 0;
                    ub[run1] = 0;
                }

                for( run2 = 0; run2 < fcn[0].getDim(); run2++ ){
                    lb[run1][run2] = rhs.lb[run1][run2];
                    ub[run1][run2] = rhs.ub[run1][run2];
                }
            }
        }

        if( rhs.xSeed != 0 )    xSeed   = new BlockMatrix(*rhs.xSeed  );
        else                    xSeed   = 0                            ;
        if( rhs.xaSeed != 0 )   xaSeed  = new BlockMatrix(*rhs.xaSeed );
        else                    xaSeed  = 0                            ;
        if( rhs.pSeed != 0 )    pSeed   = new BlockMatrix(*rhs.pSeed  );
        else                    pSeed   = 0                            ;
        if( rhs.uSeed != 0 )    uSeed   = new BlockMatrix(*rhs.uSeed  );
        else                    uSeed   = 0                            ;
        if( rhs.wSeed != 0 )    wSeed   = new BlockMatrix(*rhs.wSeed  );
        else                    wSeed   = 0                            ;

        if( rhs.bSeed != 0 )    bSeed   = new BlockMatrix(*rhs.bSeed  );
        else                    bSeed   = 0                            ;

        if( rhs.xSeed2 != 0 )   xSeed2  = new BlockMatrix(*rhs.xSeed2 );
        else                    xSeed2  = 0                            ;
        if( rhs.xaSeed2 != 0 )  xaSeed2 = new BlockMatrix(*rhs.xaSeed2);
        else                    xaSeed2 = 0                            ;
        if( rhs.pSeed2 != 0 )   pSeed2  = new BlockMatrix(*rhs.pSeed2 );
        else                    pSeed2  = 0                            ;
        if( rhs.uSeed2 != 0 )   uSeed2  = new BlockMatrix(*rhs.uSeed2 );
        else                    uSeed2  = 0                            ;
        if( rhs.wSeed2 != 0 )   wSeed2  = new BlockMatrix(*rhs.wSeed2 );
        else                    wSeed2  = 0                            ;

        if( rhs.bSeed != 0 )    bSeed2  = new BlockMatrix(*rhs.bSeed  );
        else                    bSeed2  = 0                            ;

        residuumL = rhs.residuumL;
        residuumU = rhs.residuumU;
        dForward  = rhs.dForward ;
        dBackward = rhs.dBackward;

        condType  = rhs.condType;
    }

    return *this;
}


returnValue ConstraintElement::init(  const OCPiterate& iter ){

    int run1, run2;
	
	initializeEvaluationPoints( iter );
	

    if( iter.x  != NULL ) nx = iter.x ->getNumValues();
    else             nx = 0                 ;

    if( iter.xa != NULL ) na = iter.xa->getNumValues();
    else             na = 0                 ;

    if( iter.p  != NULL ) np = iter.p ->getNumValues();
    else             np = 0                 ;

    if( iter.u  != NULL ) nu = iter.u ->getNumValues();
    else             nu = 0                 ;

    if( iter.w  != NULL ) nw = iter.w ->getNumValues();
    else             nw = 0                 ;

    ny = nx+na+nu+np+nw;

    for( run2 = 0; run2 < nFcn; run2++ ){

        if( y_index[run2] != 0 )  delete[] y_index[run2];
            y_index[run2]       = new      int[ny]      ;

        for( run1 = 0; run1 < nx; run1++ )
            y_index[run2][run1] = fcn[run2].index( VT_DIFFERENTIAL_STATE, run1 );

        for( run1 = 0; run1 < na; run1++ )
            y_index[run2][nx+run1] = fcn[run2].index( VT_ALGEBRAIC_STATE, run1 );

        for( run1 = 0; run1 < np; run1++ )
            y_index[run2][nx+na+run1] = fcn[run2].index( VT_PARAMETER, run1 );

        for( run1 = 0; run1 < nu; run1++ )
            y_index[run2][nx+na+np+run1] = fcn[run2].index( VT_CONTROL, run1 );

        for( run1 = 0; run1 < nw; run1++ )
            y_index[run2][nx+na+np+nu+run1] = fcn[run2].index( VT_DISTURBANCE, run1 );

        t_index[run2] = fcn[run2].index( VT_TIME, 0 );
    }

    return SUCCESSFUL_RETURN;
}


returnValue ConstraintElement::setForwardSeed( BlockMatrix *xSeed_,
                                               BlockMatrix *xaSeed_,
                                               BlockMatrix *pSeed_,
                                               BlockMatrix *uSeed_,
                                               BlockMatrix *wSeed_,
                                               int          order    ){

    if( order == 1 ){

        if( xSeed_ != 0 ){
            if( xSeed != 0 ) delete xSeed;
            xSeed = new BlockMatrix(*xSeed_);
        }
        else{
            if( xSeed != 0 ) delete xSeed;
            xSeed = 0;
        }
        if( xaSeed_ != 0 ){
            if( xaSeed != 0 ) delete xaSeed;
            xaSeed = new BlockMatrix(*xaSeed_);
        }
        else{
            if( xaSeed != 0 ) delete xaSeed;
            xaSeed = 0;
        }
        if( pSeed_ != 0 ){
            if( pSeed != 0 ) delete pSeed;
            pSeed = new BlockMatrix(*pSeed_);
        }
        else{
            if( pSeed != 0 ) delete pSeed;
            pSeed = 0;
        }
        if( uSeed_ != 0 ){
            if( uSeed != 0 ) delete uSeed;
            uSeed = new BlockMatrix(*uSeed_);
        }
        else{
            if( uSeed != 0 ) delete uSeed;
            uSeed = 0;
        }
        if( wSeed_ != 0 ){
            if( wSeed != 0 ) delete wSeed;
            wSeed = new BlockMatrix(*wSeed_);
        }
        else{
            if( wSeed != 0 ) delete wSeed;
            wSeed = 0;
        }

        return SUCCESSFUL_RETURN;
    }
    if( order == 2 ){

        if( xSeed_ != 0 ){
            if( xSeed2 != 0 ) delete xSeed2;
            xSeed2 = new BlockMatrix(*xSeed_);
        }
        else{
            if( xSeed2 != 0 ) delete xSeed2;
            xSeed2 = 0;
        }
        if( xaSeed_ != 0 ){
            if( xaSeed2 != 0 ) delete xaSeed2;
            xaSeed2 = new BlockMatrix(*xaSeed_);
        }
        else{
            if( xaSeed2 != 0 ) delete xaSeed2;
            xaSeed2 = 0;
        }
        if( pSeed_ != 0 ){
            if( pSeed2 != 0 ) delete pSeed2;
            pSeed2 = new BlockMatrix(*pSeed_);
        }
        else{
            if( pSeed2 != 0 ) delete pSeed2;
            pSeed2 = 0;
        }
        if( uSeed_ != 0 ){
            if( uSeed2 != 0 ) delete uSeed2;
            uSeed2 = new BlockMatrix(*uSeed_);
        }
        else{
            if( uSeed2 != 0 ) delete uSeed2;
            uSeed2 = 0;
        }
        if( wSeed_ != 0 ){
            if( wSeed2 != 0 ) delete wSeed2;
            wSeed2 = new BlockMatrix(*wSeed_);
        }
        else{
            if( wSeed2 != 0 ) delete wSeed2;
            wSeed2 = 0;
        }
        return SUCCESSFUL_RETURN;
    }

    return ACADOWARNING(RET_INPUT_OUT_OF_RANGE);
}


returnValue ConstraintElement::setUnitForwardSeed( ){

    BlockMatrix  xSeed_( 1, 1 );
    BlockMatrix xaSeed_( 1, 1 );
    BlockMatrix  pSeed_( 1, 1 );
    BlockMatrix  uSeed_( 1, 1 );
    BlockMatrix  wSeed_( 1, 1 );

    xSeed_.setIdentity ( 0, 0, nx );
    xaSeed_.setIdentity( 0, 0, na );
    pSeed_.setIdentity ( 0, 0, np );
    uSeed_.setIdentity ( 0, 0, nu );
    wSeed_.setIdentity ( 0, 0, nw );

    return setForwardSeed( &xSeed_, &xaSeed_, &pSeed_, &uSeed_, &wSeed_, 1 );
}


returnValue ConstraintElement::setBackwardSeed( BlockMatrix *seed, int order ){

    if( order == 1 ){

        if( seed != 0 ){
            if( bSeed != 0 ) delete bSeed;
            bSeed = new BlockMatrix(*seed);
        }
        else{
            if( bSeed != 0 ) delete bSeed;
            bSeed = 0;
        }

        return SUCCESSFUL_RETURN;
    }
    if( order == 2 ){

        if( seed != 0 ){
            if( bSeed2 != 0 ) delete bSeed2;
            bSeed2 = new BlockMatrix(*seed);
        }
        else{
            if( bSeed2 != 0 ) delete bSeed2;
            bSeed2 = 0;
        }
        return SUCCESSFUL_RETURN;
    }

    return ACADOWARNING(RET_INPUT_OUT_OF_RANGE);
}


returnValue ConstraintElement::getResiduum( BlockMatrix &lower_residuum,
                                            BlockMatrix &upper_residuum ){


    lower_residuum = residuumL;
    upper_residuum = residuumU;

    return SUCCESSFUL_RETURN;
}


returnValue ConstraintElement::getForwardSensitivities( BlockMatrix *D, int order ){

    ASSERT( D != 0 );

    if( order == 1 ){

        D[0] = dForward;
        return SUCCESSFUL_RETURN;
    }
    if( order == 2 ){

        return ACADOERROR(RET_NOT_IMPLEMENTED_YET);
    }
    return ACADOWARNING(RET_INPUT_OUT_OF_RANGE);
}


returnValue ConstraintElement::getBackwardSensitivities( BlockMatrix *D, int order ){

    ASSERT( D != 0 );

    if( order == 1 ){

        D[0] = dBackward;
        return SUCCESSFUL_RETURN;
    }
    if( order == 2 ){

        return ACADOERROR(RET_NOT_IMPLEMENTED_YET);
    }
    return ACADOWARNING(RET_INPUT_OUT_OF_RANGE);
}



//
// PROTECTED MEMBER FUNCTIONS:
//

returnValue ConstraintElement::initializeEvaluationPoints( const OCPiterate& iter )
{
	if ( z != 0 )
		delete[] z;
	
	if ( JJ != 0 )
		delete[] JJ;

	z = new EvaluationPoint[nFcn];
	JJ = new EvaluationPoint[nFcn];
	//HH = new EvaluationPoint[nFcn];

	for( int i=0; i<nFcn; ++i )
	{
		z[i].init(  fcn[i], iter );
		JJ[i].init(  fcn[i], iter );
		//HH[i].init(  fcn[i], iter );
	}

	return SUCCESSFUL_RETURN;
}

returnValue ConstraintElement::get(Function& function_, DMatrix& lb_, DMatrix& ub_)
{
	if ( fcn == NULL )
		return RET_INITIALIZE_FIRST;

	// This is not exactly bullet proof, but for now will serve the purpose
	function_ = fcn[ 0 ];

	int dimFcn = fcn[ 0 ].getDim();

	if ( dimFcn == 0 )
	{
		lb_.init(0, 0);
		ub_.init(0, 0);

		return SUCCESSFUL_RETURN;
	}

	lb_.init(nB, dimFcn);
	ub_.init(nB, dimFcn);

	int i, j;

	for (i = 0; i < nB; ++i)
		for (j = 0; j < dimFcn; ++j)
		{
			lb_(i, j) = lb[ i ][ j ];
			ub_(i, j) = ub[ i ][ j ];
		}

	return SUCCESSFUL_RETURN;
}


CLOSE_NAMESPACE_ACADO

// end of file.
