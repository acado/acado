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
 *    \file src/symbolic_operator/c_function.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date 2010
 */



#include <acado/function/c_function.hpp>
#include <acado/function/c_operator.hpp>


BEGIN_NAMESPACE_ACADO



//
// PUBLIC MEMBER FUNCTIONS:
//

CFunction::CFunction( ){

    cFcn          = NULL;
    cFcnDForward  = NULL;
    cFcnDBackward = NULL;
    dim           = 0   ;

    user_data     = 0;

    initialize();
}


CFunction::CFunction( uint dim_, cFcnPtr cFcn_ ){

    cFcn          = cFcn_;
    cFcnDForward  = NULL ;
    cFcnDBackward = NULL ;
    dim           = dim_ ;

    user_data     = 0;

    initialize();
}


CFunction::CFunction( uint     dim_          , 
                      cFcnPtr  cFcn_         ,
                      cFcnDPtr cFcnDForward_ ,
                      cFcnDPtr cFcnDBackward_  ){

    cFcn          = cFcn_         ;
    cFcnDForward  = cFcnDForward_ ;
    cFcnDBackward = cFcnDBackward_;
    dim           = dim_          ;

    user_data     = 0;

    initialize();
}


CFunction:: CFunction( const CFunction& arg ){ copy     (arg); }
CFunction::~CFunction(                      ){ deleteAll(   ); }

CFunction& CFunction::operator=( const CFunction& arg ){

    if ( this != &arg ){
        deleteAll();
        copy(arg);
    }
    return *this;
}


returnValue CFunction::initialize(){

    nn            = 0;
    maxAlloc      = 1;

    xStore    = (double**)calloc(maxAlloc, sizeof(double*));
    seedStore = (double**)calloc(maxAlloc, sizeof(double*));

    xStore   [0]  = 0;
    seedStore[0]  = 0;

    return SUCCESSFUL_RETURN;
}


void CFunction::copy( const CFunction &arg ){

    uint run1, run2;

    if( arg.cFcn == 0          ) cFcn          = 0                ;
    else                         cFcn          = arg.cFcn         ;
    if( arg.cFcnDForward == 0  ) cFcnDForward  = 0                ;
    else                         cFcnDForward  = arg.cFcnDForward ;
    if( arg.cFcnDBackward == 0 ) cFcnDBackward = 0                ;
    else                         cFcnDBackward = arg.cFcnDBackward;

    dim = arg.dim;
    nn  = arg.nn ;

    user_data = arg.user_data;

    maxAlloc = arg.maxAlloc;

    xStore    = (double**)calloc(maxAlloc,sizeof(double*));
    seedStore = (double**)calloc(maxAlloc,sizeof(double*));

    for( run1 = 0; run1 < maxAlloc; run1++ ){
        if( nn != 0 ){
               xStore[run1] = new double[nn];
            seedStore[run1] = new double[nn];
            for( run2 = 0; run2 < nn; run2++ ){
                   xStore[run1][run2] = arg.xStore   [run1][run2];
                seedStore[run1][run2] = arg.seedStore[run1][run2];
            }
        }
        else{
               xStore[run1] = 0;
            seedStore[run1] = 0;
        }
    }
}


void CFunction::deleteAll(){

    uint run1;

    for( run1 = 0; run1 < maxAlloc; run1++ ){
        if( xStore    != 0 ) delete[] xStore   [run1];
        if( seedStore != 0 ) delete[] seedStore[run1];
    }
    if( xStore    != 0 ) free(xStore   );
    if( seedStore != 0 ) free(seedStore);
}


Expression CFunction::operator()( const Expression &arg ){

    uint run1,run2;

    CFunction thisFunction(*this);
    thisFunction.nn = arg.getDim();

    for( run1 = 0; run1 < maxAlloc; run1++ ){
        thisFunction.xStore   [run1] = new double[thisFunction.nn];
        thisFunction.seedStore[run1] = new double[thisFunction.nn];
        for( run2 = 0; run2 < thisFunction.nn; run2++ ){
            thisFunction.xStore   [run1][run2] = 0.0;
            thisFunction.seedStore[run1][run2] = 0.0;
        }
    }

    Expression tmp(dim);

    COperator dummy;
    dummy.increaseID();

    for( run1 = 0; run1 < dim; run1++ ){
        delete tmp.element[run1];
        tmp.element[run1] = new COperator( thisFunction, arg, run1 );
    }

    return tmp;
}


uint CFunction::getDim () const{

    return dim;
}


returnValue CFunction::evaluate( double *x, double *result ){

    if( cFcn != 0 ) cFcn( x, result, user_data );
    else evaluateCFunction( x, result );

    return SUCCESSFUL_RETURN;
}



void CFunction::evaluateCFunction( double *x, double *result ){

    // DUMMY IMPLEMENTATION, CAN BE IMPLEMENTED BY THE USER
    // IN DERIVED CLASSES.
}



returnValue CFunction::evaluate( double *x, double *result, PrintLevel printL ){

    uint run1;

    evaluate(x,result);

	if (printL == MEDIUM || printL == HIGH)
	{
		std::cout << "C-function evaluation: ";
		for (run1 = 0; run1 < dim; run1++)
			std::cout << "f[" << run1 << "] = " << std::scientific << result[ run1 ] << std::endl;
	}
    return SUCCESSFUL_RETURN;
}


returnValue CFunction::evaluate( int number, double *x, double *result ){

    uint run1;
    uint run2;

    evaluate(x,result);

    if( number >= (int) maxAlloc ){

        xStore= (double**)realloc(xStore, (number+1)*sizeof(double*));
        for( run1 = maxAlloc; (int) run1 < number+1; run1++ ){
           xStore[run1] = new double[nn];
        }
        seedStore = (double**)realloc(seedStore, (number+1)*sizeof(double*));
        for( run1 = maxAlloc; (int) run1 < number+1; run1++ ){
           seedStore[run1] = new double[nn];
        }
        maxAlloc = number+1;
    }

    for( run2 = 0; run2 < nn; run2++ ){
        xStore[number][run2] = x[run2];
    }

    return SUCCESSFUL_RETURN;
}


returnValue CFunction::AD_forward( double *x, double *seed, double *f, double *df ){

    if( cFcnDForward != NULL ){
        cFcnDForward( 0, x, seed, f, df, user_data );
        return SUCCESSFUL_RETURN;
    }
    else{

        uint run1;

        if( cFcn != 0 )
            cFcn( x, f, user_data );
        else evaluateCFunction( x, f );

        for( run1 = 0; run1 < nn; run1++ ){
            x[run1] = x[run1] + SQRT_EPS*seed[run1];
        }

        if( cFcn != 0 )
            cFcn( x, df, user_data );
        else evaluateCFunction( x, df );

        for( run1 = 0; run1 < dim; run1++ ){
            df[run1] = ( df[run1] - f[run1] )/SQRT_EPS;
        }
        for( run1 = 0; run1 < nn; run1++ ){
            x[run1] = x[run1] - SQRT_EPS*seed[run1];
        }
    }
    return SUCCESSFUL_RETURN;
}


returnValue CFunction::AD_forward( int number, double *x, double *seed, double *f, double *df ){

    uint run1;

    if( number >= (int) maxAlloc ){

        xStore= (double**)realloc(xStore, (number+1)*sizeof(double*));
        for( run1 = maxAlloc; (int) run1 < number+1; run1++ ){
           xStore[run1] = new double[nn];
        }
        seedStore = (double**)realloc(seedStore, (number+1)*sizeof(double*));
        for( run1 = maxAlloc; (int) run1 < number+1; run1++ ){
           seedStore[run1] = new double[nn];
        }
        maxAlloc = number+1;
    }

    if( cFcnDForward != NULL ){
        cFcnDForward( number, x, seed, f, df, user_data );

        for( run1 = 0; run1 < nn; run1++ ){

            xStore[number][run1]    = x   [run1];
            seedStore[number][run1] = seed[run1];
        }
        return SUCCESSFUL_RETURN;
    }

    if( cFcn != 0 )
        cFcn( x, f, user_data );
    else evaluateCFunction( x, f );

    for( run1 = 0; run1 < nn; run1++ ){
        xStore[number][run1] = x[run1];
        seedStore[number][run1] = seed[run1];
        x              [run1] = x[run1] + SQRT_EPS*seed[run1];
    }
    if( cFcn != 0 )
        cFcn( x, df, user_data );
    else evaluateCFunction( x, df );
    for( run1 = 0; run1 < dim; run1++ ){
        df[run1] = ( df[run1] - f[run1] )/SQRT_EPS;
    }
    for( run1 = 0; run1 < nn; run1++ ){
        x[run1] = xStore[number][run1];
    }

    return SUCCESSFUL_RETURN;
}


returnValue CFunction::AD_forward( int number, double *seed, double *df ){

    uint run1;

    ASSERT( number < (int) maxAlloc );

    double *f = new double[dim];

    if( cFcnDForward != NULL ){
        cFcnDForward( number, xStore[number], seed, f, df, user_data );

        for( run1 = 0; run1 < nn; run1++ )
            seedStore[number][run1] = seed[run1];

        delete[] f;
        return SUCCESSFUL_RETURN;
    }

    if( cFcn != 0 )
        cFcn( xStore[number], f, user_data );
    else evaluateCFunction( xStore[number], f );

    for( run1 = 0; run1 < nn; run1++ ){
        xStore[number][run1] = xStore[number][run1] + SQRT_EPS*seed[run1];
    }

    if( cFcn != 0 )
        cFcn( xStore[number], df, user_data );
    else evaluateCFunction( xStore[number], df );
    for( run1 = 0; run1 < dim; run1++ ){
        df[run1] = ( df[run1] - f[run1] )/SQRT_EPS;
    }
    for( run1 = 0; run1 < nn; run1++ ){
        xStore[number][run1] = xStore[number][run1] - SQRT_EPS*seed[run1];
    }

    for( run1 = 0; run1 < nn; run1++ ){
        seedStore[number][run1] = seed[run1];
    }

    delete[] f;

    return SUCCESSFUL_RETURN;
}



returnValue CFunction::AD_backward( double *seed, double  *df ){

    return AD_backward( 0, seed, df );
}


returnValue CFunction::AD_backward( int number, double *seed, double  *df ){

    uint run1;

    ASSERT( number < (int) maxAlloc );

    if( cFcnDBackward != NULL ){

        double *f = new double[dim];

        cFcnDBackward( number, xStore[number], seed, f, df, user_data );

        for( run1 = 0; run1 < nn; run1++ )
            seedStore[number][run1] = seed[run1];

        delete[] f;
        return SUCCESSFUL_RETURN;
    }
    return ACADOERROR(RET_INVALID_USE_OF_FUNCTION);
}


returnValue CFunction::AD_forward2( int number, double *seed, double *dseed, double *df, double *ddf ){

    uint run1;

    ASSERT( number < (int) maxAlloc );

    double *f1 = new double[dim];
    double *f2 = new double[dim];
    double *f3 = new double[dim];
    double *f4 = new double[dim];


    // FIRST ORDER DERIVATIVES:
    // ------------------------

    evaluate( xStore[number], f1 );

    for( run1 = 0; run1 < nn; run1++ ){
        xStore[number][run1] = xStore[number][run1] + SQRT_EPS*seed[run1];
    }
    evaluate( xStore[number], df );
    for( run1 = 0; run1 < dim; run1++ ){
        df[run1] = ( df[run1] - f1[run1] )/SQRT_EPS;
    }
    for( run1 = 0; run1 < nn; run1++ ){
        xStore[number][run1] = xStore[number][run1] - SQRT_EPS*seed[run1];
    }

    for( run1 = 0; run1 < nn; run1++ ){
        xStore[number][run1] = xStore[number][run1] + SQRT_EPS*dseed[run1];
    }
    evaluate( xStore[number], ddf );
    for( run1 = 0; run1 < dim; run1++ ){
        ddf[run1] = ( ddf[run1] - f1[run1] )/SQRT_EPS;
    }
    for( run1 = 0; run1 < nn; run1++ ){
        xStore[number][run1] = xStore[number][run1] - SQRT_EPS*dseed[run1];
    }


    // SECOND DERIVATIVES:
    // -------------------

    for( run1 = 0; run1 < nn; run1++ ){
        xStore[number][run1] = xStore[number][run1]
                                + 0.5*FOURTH_ROOT_EPS*(seed[run1]+seedStore[number][run1]);
    }
    evaluate( xStore[number], f1 );

    for( run1 = 0; run1 < nn; run1++ ){
        xStore[number][run1] = xStore[number][run1]
                                - 0.5*FOURTH_ROOT_EPS*(seed[run1]+seedStore[number][run1])
                                + 0.5*FOURTH_ROOT_EPS*(seed[run1]-seedStore[number][run1]);
    }
    evaluate( xStore[number], f2 );

    for( run1 = 0; run1 < nn; run1++ ){
        xStore[number][run1] = xStore[number][run1]
                                - FOURTH_ROOT_EPS*(seed[run1]-seedStore[number][run1]);
    }
    evaluate( xStore[number], f3 );

    for( run1 = 0; run1 < nn; run1++ ){
        xStore[number][run1] = xStore[number][run1]
                                + 0.5*FOURTH_ROOT_EPS*(seed[run1]-seedStore[number][run1])
                                - 0.5*FOURTH_ROOT_EPS*(seed[run1]+seedStore[number][run1]);
    }
    evaluate( xStore[number], f4 );

    for( run1 = 0; run1 < nn; run1++ ){
        xStore[number][run1] = xStore[number][run1]
                                + 0.5*FOURTH_ROOT_EPS*(seed[run1]+seedStore[number][run1]);
    }

    for( run1 = 0; run1 < dim; run1++ ){
        ddf[run1] = ddf[run1] + ( f1[run1] - f2[run1] - f3[run1] + f4[run1] )/SQRT_EPS;
    }

    delete[] f1;
    delete[] f2;
    delete[] f3;
    delete[] f4;

    return SUCCESSFUL_RETURN;
}


returnValue CFunction::AD_backward2( int number, double *seed1, double *seed2, double *df, double *ddf ){

    return ACADOERROR(RET_INVALID_USE_OF_FUNCTION);
}

returnValue CFunction::clearBuffer(){

    uint run1;

    if( maxAlloc > 1 ){

        for( run1 = 1; run1 < maxAlloc; run1++ ){
            delete[] xStore[run1];
            delete[] seedStore[run1];
        }

        maxAlloc = 1;

        xStore= (double**)realloc( xStore, maxAlloc*sizeof(double*) );
        seedStore = (double**)realloc( seedStore, maxAlloc*sizeof(double*) );
    }

    return SUCCESSFUL_RETURN;
}


returnValue CFunction::setUserData( void * user_data_){

  user_data = user_data_;

  return SUCCESSFUL_RETURN;
}



CLOSE_NAMESPACE_ACADO

// end of file.
