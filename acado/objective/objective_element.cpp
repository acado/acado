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
 *    \file src/objective/objective_element.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *
 */

#include <acado/objective/objective_element.hpp>



BEGIN_NAMESPACE_ACADO




ObjectiveElement::ObjectiveElement( ){

    y_index = 0;
    t_index = 0;
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

    obj = 0.0;
}


ObjectiveElement::ObjectiveElement( const Grid &grid_ ){

    grid    = grid_;
    y_index = 0    ;
    t_index = 0    ;
    nx      = 0    ;
    na      = 0    ;
    nu      = 0    ;
    np      = 0    ;
    nw      = 0    ;
    ny      = 0    ;

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

    obj = 0.0;
}


ObjectiveElement::ObjectiveElement( const ObjectiveElement& rhs ){

    int run1;

    grid = rhs.grid;
    fcn  = rhs.fcn ;

    if( rhs.y_index != 0 ){
        y_index = new int[rhs.ny];
        for( run1 = 0; run1 < rhs.ny; run1++ )
            y_index[run1] = rhs.y_index[run1];
    }
    else  y_index = 0;

    t_index = rhs.t_index;

    nx = rhs.nx;
    na = rhs.na;
    nu = rhs.nu;
    np = rhs.np;
    nw = rhs.nw;
    ny = rhs.ny;

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


    obj       = rhs.obj      ;
    dForward  = rhs.dForward ;
    dBackward = rhs.dBackward;
}


ObjectiveElement::~ObjectiveElement( ){

    if( y_index != 0 )
        delete[] y_index;

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


ObjectiveElement& ObjectiveElement::operator=( const ObjectiveElement& rhs ){

    int run1;

    if( this != &rhs ){

        if( y_index != 0 )
            delete[] y_index;

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

        grid = rhs.grid;
        fcn  = rhs.fcn ;

        if( rhs.y_index != 0 ){
            y_index = new int[ny];
            for( run1 = 0; run1 < ny; run1++ )
                y_index[run1] = rhs.y_index[run1];
        }
        else  y_index = 0;

        t_index = rhs.t_index;

        nx = rhs.nx;
        na = rhs.na;
        nu = rhs.nu;
        np = rhs.np;
        nw = rhs.nw;
        ny = rhs.ny;

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

        obj       = rhs.obj      ;
        dForward  = rhs.dForward ;
        dBackward = rhs.dBackward;
    }
    return *this;
}




returnValue ObjectiveElement::init( const OCPiterate &x ){

    int run1;

    z.init( fcn, x );
    JJ.init( fcn, x );
	//HH.init( fcn, x );

    if( x.x != NULL ) nx = x.x->getNumValues();
    else             nx = 0                 ;

    if( x.xa != NULL ) na = x.xa->getNumValues();
    else             na = 0                 ;

    if( x.p  != NULL ) np = x.p ->getNumValues();
    else             np = 0                 ;

    if( x.u  != NULL ) nu = x.u ->getNumValues();
    else             nu = 0                 ;

    if( x.w  != NULL ) nw = x.w ->getNumValues();
    else             nw = 0                 ;

    ny = nx+na+nu+np+nw;

    if( y_index != 0 )  delete[] y_index;
        y_index       = new      int[ny];

    for( run1 = 0; run1 < nx; run1++ )
        y_index[run1] = fcn.index( VT_DIFFERENTIAL_STATE, run1 );

    for( run1 = 0; run1 < na; run1++ )
        y_index[nx+run1] = fcn.index( VT_ALGEBRAIC_STATE, run1 );

    for( run1 = 0; run1 < np; run1++ )
        y_index[nx+na+run1] = fcn.index( VT_PARAMETER, run1 );

    for( run1 = 0; run1 < nu; run1++ )
        y_index[nx+na+np+run1] = fcn.index( VT_CONTROL, run1 );

    for( run1 = 0; run1 < nw; run1++ )
        y_index[nx+na+np+nu+run1] = fcn.index( VT_DISTURBANCE, run1 );

    t_index = fcn.index( VT_TIME, 0 );

    return SUCCESSFUL_RETURN;
}



returnValue ObjectiveElement::setForwardSeed( BlockMatrix *xSeed_,
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


returnValue ObjectiveElement::setBackwardSeed( BlockMatrix *seed, int order ){

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


returnValue ObjectiveElement::getObjectiveValue( double &objectiveValue ){

    objectiveValue = obj;
    return SUCCESSFUL_RETURN;
}


returnValue ObjectiveElement::getForwardSensitivities( BlockMatrix *D, int order ){

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


returnValue ObjectiveElement::getBackwardSensitivities( BlockMatrix *D, int order ){

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



CLOSE_NAMESPACE_ACADO

// end of file.
