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
 *    \file src/integrator/integrator_runge_kutta.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *
 */

#include <acado/utils/acado_utils.hpp>
#include <acado/matrix_vector/matrix_vector.hpp>
#include <acado/symbolic_expression/symbolic_expression.hpp>
#include <acado/function/function_.hpp>
#include <acado/function/differential_equation.hpp>
#include <acado/integrator/integrator.hpp>
#include <acado/integrator/integrator_runge_kutta.hpp>

using namespace std;

BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

IntegratorRK::IntegratorRK( )
             :Integrator( ){

    initializeVariables();
}


IntegratorRK::IntegratorRK( int dim_ , double power_)
             :Integrator( ){

    int run1;

    initializeVariables();
    dim       = dim_  ;
    err_power = power_;

    // BUTCHER TABLEAU:
    // ----------------
    A  = new double*[dim];
    b4 = new double [dim];
    b5 = new double [dim];
    c  = new double [dim];

    for( run1 = 0; run1 < dim; run1++ ){
        A[run1] = new double[dim];
    }
}


IntegratorRK::IntegratorRK( const DifferentialEquation& rhs_, int dim_, double power_ )
             :Integrator( ){

    int run1;

    dim       = dim_  ;
    err_power = power_;

    // BUTCHER TABLEAU:
    // ----------------
    A  = new double*[dim];
    b4 = new double [dim];
    b5 = new double [dim];
    c  = new double [dim];

    for( run1 = 0; run1 < dim; run1++ ){
        A[run1] = new double[dim];
    }

    init( rhs_ );
}


IntegratorRK::IntegratorRK( const IntegratorRK& arg )
             :Integrator( arg ){

    constructAll( arg );
}


IntegratorRK::~IntegratorRK( ){

    deleteAll();
}


returnValue IntegratorRK::init( const DifferentialEquation &rhs_ ){

    rhs = new DifferentialEquation( rhs_ );
    m   = rhs->getDim ();
    ma  = 0;
    mn  = rhs->getN   ();
    mu  = rhs->getNU  ();
    mui = rhs->getNUI ();
    mp  = rhs->getNP  ();
    mpi = rhs->getNPI ();
    mw  = rhs->getNW  ();

    allocateMemory();

    return SUCCESSFUL_RETURN;
}


void IntegratorRK::initializeVariables(){

    dim = 0; A = 0; b4 = 0; b5 = 0; c = 0;
    eta4 = 0; eta5 = 0; eta4_ = 0; eta5_ = 0;
    k = 0; k2 = 0; l = 0; l2 = 0; x = 0;

    G = 0; etaG = 0;
    G2 = 0; G3 = 0; etaG2 = 0; etaG3 = 0;

    H = 0; etaH = 0; H2 = 0; H3 = 0;
    etaH2 = 0; etaH3 = 0;

    maxAlloc  = 0;
    err_power = 1.0;
}


void IntegratorRK::allocateMemory( ){

    int run1, run2;

    if( 0 != rhs->getNXA() ){
        ACADOERROR(RET_RK45_CAN_NOT_TREAT_DAE);
        ASSERT(1 == 0);
    }

    if( 0 != rhs->getNDX() ){
        ACADOERROR(RET_RK45_CAN_NOT_TREAT_DAE);
        ASSERT(1 == 0);
    }


    if( m < 1 ){
        ACADOERROR(RET_TRIVIAL_RHS);
        ASSERT(1 == 0);
    }

    // RK-ALGORITHM:
    // -------------
    eta4  = new double [m];
    eta5  = new double [m];
    eta4_ = new double [m];
    eta5_ = new double [m];

    for( run1 = 0; run1 < m; run1++ ){

        eta4 [run1] = 0.0;
        eta5 [run1] = 0.0;
        eta4_[run1] = 0.0;
        eta5_[run1] = 0.0;
    }

    k     = new double*[dim];
    k2    = new double*[dim];
    l     = new double*[dim];
    l2    = new double*[dim];
    x     = new double [rhs->getNumberOfVariables() + 1 + m];

    for( run1 = 0; run1 < rhs->getNumberOfVariables() + 1 + m; run1++ ){
        x[run1] = 0.0;
    }

    t     = 0.0;


    for( run1 = 0; run1 < dim; run1++ ){
        k    [run1] = new double[m];
        k2   [run1] = new double[m];

        for( run2 = 0; run2 < m; run2++ ){
            k [run1][run2] = 0.0;
            k2[run1][run2] = 0.0;
        }

        l    [run1] = new double[rhs->getNumberOfVariables() + 1 + m];
        l2   [run1] = new double[rhs->getNumberOfVariables() + 1 + m];

        for( run2 = 0; run2 < rhs->getNumberOfVariables() + 1 + m; run2++ ){
            l [run1][run2] = 0.0;
            l2[run1][run2] = 0.0;
        }
    }

    // INTERNAL INDEX LISTS:
    // ---------------------
    diff_index = new int[m];

    for( run1 = 0; run1 < m; run1++ ){
        diff_index[run1] = rhs->getStateEnumerationIndex( run1 );
        if( diff_index[run1] == rhs->getNumberOfVariables() ){
            diff_index[run1] = diff_index[run1] + 1 + run1;
        }
    }

    control_index       = new int[mu ];

    for( run1 = 0; run1 < mu; run1++ ){
        control_index[run1] = rhs->index( VT_CONTROL, run1 );
    }

    parameter_index     = new int[mp ];

    for( run1 = 0; run1 < mp; run1++ ){
        parameter_index[run1] = rhs->index( VT_PARAMETER, run1 );
    }

    int_control_index   = new int[mui];

    for( run1 = 0; run1 < mui; run1++ ){
        int_control_index[run1] = rhs->index( VT_INTEGER_CONTROL, run1 );
    }

    int_parameter_index = new int[mpi];

    for( run1 = 0; run1 < mpi; run1++ ){
        int_parameter_index[run1] = rhs->index( VT_INTEGER_PARAMETER, run1 );
    }

    disturbance_index   = new int[mw ];

    for( run1 = 0; run1 < mw; run1++ ){
        disturbance_index[run1] = rhs->index( VT_DISTURBANCE, run1 );
    }

    time_index = rhs->index( VT_TIME, 0 );

    diff_scale.init(m);
    for( run1 = 0; run1 < m; run1++ )
        diff_scale(run1) = rhs->scale( VT_DIFFERENTIAL_STATE, run1 );


    // SENSITIVITIES:
    // --------------
    G          = NULL;
    etaG       = NULL;

    G2         = NULL;
    G3         = NULL;
    etaG2      = NULL;
    etaG3      = NULL;

    H          = NULL;
    etaH       = NULL;

    H2         = NULL;
    H3         = NULL;
    etaH2      = NULL;
    etaH3      = NULL;


    // STORAGE:
    // --------
    maxAlloc = 1;
}



void IntegratorRK::deleteAll(){

    int run1;


    // BUTCHER-
    // TABLEAU:
    // ----------
    for( run1 = 0; run1 < dim; run1++ ){
        delete[] A[run1];
    }

    delete[] A;
    delete[] b4;
    delete[] b5;
    delete[] c ;


    // RK-ALGORITHM:
    // -------------
    if( eta4 != NULL ){
        delete[] eta4;
    }
    if( eta4_ != NULL ){
        delete[] eta4_;
    }
    if( eta5 != NULL ){
        delete[] eta5;
    }
    if( eta5_ != NULL ){
        delete[] eta5_;
    }

    for( run1 = 0; run1 < dim; run1++ ){
      if( k[run1]  != NULL )
          delete[] k[run1] ;
      if( k2[run1] != NULL )
          delete[] k2[run1];
      if( l[run1]  != NULL )
          delete[] l[run1] ;
      if( l2[run1] != NULL )
          delete[] l2[run1];
    }

    if( k != NULL )
        delete[] k ;

    if( k2!= NULL )
        delete[] k2;

    if( l != NULL )
        delete[] l ;

    if( l2!= NULL )
        delete[] l2;

    if( x != NULL )
        delete[] x;


    // SENSITIVITIES:
    // ----------------------------------------

    if( G  != NULL )
        delete[] G;

    if( etaG  != NULL )
        delete[] etaG;

    if( G2  != NULL )
        delete[] G2;

    if( G3  != NULL )
        delete[] G3;

    if( etaG2  != NULL )
        delete[] etaG2;

    if( etaG3  != NULL )
        delete[] etaG3;


    // ----------------------------------------

    if( H  != NULL )
        delete[] H;

    if( etaH  != NULL )
        delete[] etaH;

    if( H2  != NULL )
        delete[] H2;

    if( H3  != NULL )
        delete[] H3;

    if( etaH2  != NULL )
        delete[] etaH2;

    if( etaH3  != NULL )
        delete[] etaH3;
}


IntegratorRK& IntegratorRK::operator=( const IntegratorRK& arg ){

    if ( this != &arg ){
        deleteAll();
        Integrator::operator=( arg );
        constructAll( arg );
    }

    return *this;
}


void IntegratorRK::constructAll( const IntegratorRK& arg ){

    int run1, run2;

    rhs = new DifferentialEquation( *arg.rhs );

    m   = arg.m              ;
    ma  = arg.ma             ;
    mn  = arg.mn             ;
    mu  = arg.mu             ;
    mui = arg.mui            ;
    mp  = arg.mp             ;
    mpi = arg.mpi            ;
    mw  = arg.mw             ;


    if( m < 1 ){
        ACADOERROR(RET_TRIVIAL_RHS);
        ASSERT(1 == 0);
    }

    // BUTCHER TABLEAU:
    // ----------------
    dim = arg.dim;
    A  = new double*[dim];
    b4 = new double [dim];
    b5 = new double [dim];
    c  = new double [dim];

    for( run1 = 0; run1 < dim; run1++ ){

        b4[run1] = arg.b4[run1];
        b5[run1] = arg.b5[run1];
        c [run1] = arg.c [run1];

        A[run1] = new double[dim];
        for( run2 = 0; run2 < dim; run2++ )
            A[run1][run2] = arg.A[run1][run2];
    }

    // RK-ALGORITHM:
    // -------------
    eta4  = new double [m];
    eta5  = new double [m];
    eta4_ = new double [m];
    eta5_ = new double [m];

    for( run1 = 0; run1 < m; run1++ ){

        eta4 [run1] = arg.eta4 [run1];
        eta5 [run1] = arg.eta5 [run1];
        eta4_[run1] = arg.eta4_[run1];
        eta5_[run1] = arg.eta5_[run1];
    }

    k     = new double*[dim];
    k2    = new double*[dim];
    l     = new double*[dim];
    l2    = new double*[dim];
    x     = new double [rhs->getNumberOfVariables() + 1 + m];

    for( run1 = 0; run1 < rhs->getNumberOfVariables() + 1 + m; run1++ ){
        x[run1] = arg.x[run1];
    }

    t     = arg.t;


    for( run1 = 0; run1 < dim; run1++ ){
        k    [run1] = new double[m];
        k2   [run1] = new double[m];

        for( run2 = 0; run2 < m; run2++ ){
            k [run1][run2] = arg.k [run1][run2];
            k2[run1][run2] = arg.k2[run1][run2];
        }

        l    [run1] = new double[rhs->getNumberOfVariables() + 1 + m];
        l2   [run1] = new double[rhs->getNumberOfVariables() + 1 + m];

        for( run2 = 0; run2 < rhs->getNumberOfVariables() + 1 + m; run2++ ){
            l [run1][run2] = arg.l [run1][run2];
            l2[run1][run2] = arg.l2[run1][run2];
        }
    }


    // SETTINGS:
    // ---------
    h    = (double*)calloc(arg.maxAlloc,sizeof(double));
    for( run1 = 0; run1 < arg.maxAlloc; run1++ ){
       h[run1] = arg.h[run1];
    }
    hini = arg.hini;
    hmin = arg.hmin;
    hmax = arg.hmax;

    tune  = arg.tune;
    TOL   = arg.TOL;

    err_power = arg.err_power;


    // INTERNAL INDEX LISTS:
    // ---------------------
    diff_index = new int[m];

    for( run1 = 0; run1 < m; run1++ ){
        diff_index[run1] = arg.diff_index[run1];
    }

    ddiff_index = 0;
    alg_index   = 0;

    control_index       = new int[mu ];

    for( run1 = 0; run1 < mu; run1++ ){
        control_index[run1] = arg.control_index[run1];
    }

    parameter_index     = new int[mp ];

    for( run1 = 0; run1 < mp; run1++ ){
        parameter_index[run1] = arg.parameter_index[run1];
    }

    int_control_index   = new int[mui];

    for( run1 = 0; run1 < mui; run1++ ){
        int_control_index[run1] = arg.int_control_index[run1];
    }

    int_parameter_index = new int[mpi];

    for( run1 = 0; run1 < mpi; run1++ ){
        int_parameter_index[run1] = arg.int_parameter_index[run1];
    }

    disturbance_index   = new int[mw ];

    for( run1 = 0; run1 < mw; run1++ ){
        disturbance_index[run1] = arg.disturbance_index[run1];
    }

    time_index = arg.time_index;


    // OTHERS:
    // -------
    maxNumberOfSteps = arg.maxNumberOfSteps;
    count            = arg.count           ;
    count2           = arg.count2          ;
    count3           = arg.count3          ;

    diff_scale = arg.diff_scale;


    // PRINT-LEVEL:
    // ------------
    PrintLevel = arg.PrintLevel;


    // SENSITIVITIES:
    // ---------------
    nFDirs     = 0   ;
    nBDirs     = 0   ;

    nFDirs2    = 0   ;
    nBDirs2    = 0   ;

    G          = NULL;
    etaG       = NULL;

    G2         = NULL;
    G3         = NULL;
    etaG2      = NULL;
    etaG3      = NULL;

    H          = NULL;
    etaH       = NULL;

    H2         = NULL;
    H3         = NULL;
    etaH2      = NULL;
    etaH3      = NULL;


    // THE STATE OF AGGREGATION:
    // -------------------------
    soa        = arg.soa;


    // STORAGE:
    // --------
    maxAlloc = arg.maxAlloc;
}


returnValue IntegratorRK::freezeMesh(){


    if( soa != SOA_UNFROZEN ){
       if( PrintLevel != NONE ){
           return ACADOWARNING(RET_ALREADY_FROZEN);
       }
       return RET_ALREADY_FROZEN;
    }

    soa = SOA_FREEZING_MESH;
    return SUCCESSFUL_RETURN;
}



returnValue IntegratorRK::freezeAll(){

    if( soa != SOA_UNFROZEN ){
       if( PrintLevel != NONE ){
           return ACADOWARNING(RET_ALREADY_FROZEN);
       }
       return RET_ALREADY_FROZEN;
    }

    soa = SOA_FREEZING_ALL;
    return SUCCESSFUL_RETURN;
}


returnValue IntegratorRK::unfreeze(){

    maxAlloc = 1;
    h = (double*)realloc(h,maxAlloc*sizeof(double));
    soa = SOA_UNFROZEN;

    return SUCCESSFUL_RETURN;
}


returnValue IntegratorRK::evaluate( const DVector &x0  ,
                                    const DVector &xa  ,
                                    const DVector &p   ,
                                    const DVector &u   ,
                                    const DVector &w   ,
                                    const Grid   &t_    ){

    int         run1;
    returnValue returnvalue;

    if( rhs == NULL ){
        return ACADOERROR(RET_TRIVIAL_RHS);
    }

    if( xa.getDim() != 0 )
        ACADOWARNING(RET_RK45_CAN_NOT_TREAT_DAE);


    Integrator::initializeOptions();

    timeInterval  = t_;

    xStore.init(  m, timeInterval );
    iStore.init( mn, timeInterval );

    t             = timeInterval.getFirstTime();
    x[time_index] = timeInterval.getFirstTime();
// 	printf("initial integrator t = %e\n", t );
// 	printf("initial integrator x[time_index] = %e\n", x[time_index] );
// 	printf(" with time_index = %d\n", time_index );

    if( soa != SOA_MESH_FROZEN && soa != SOA_MESH_FROZEN_FREEZING_ALL && soa != SOA_EVERYTHING_FROZEN  ){
       h[0] = hini;

       if( timeInterval.getLastTime() - timeInterval.getFirstTime() - h[0] < EPS ){
           h[0] = timeInterval.getLastTime() - timeInterval.getFirstTime();

           if( h[0] < 10.0*EPS )
               return ACADOERROR(RET_TO_SMALL_OR_NEGATIVE_TIME_INTERVAL);
       }
    }

    if( x0.isEmpty() == BT_TRUE ) return ACADOERROR(RET_MISSING_INPUTS);


    if( (int) x0.getDim() < m )
        return ACADOERROR(RET_INPUT_HAS_WRONG_DIMENSION);

    for( run1 = 0; run1 < m; run1++ ){
        eta4[run1]     = x0(run1);
        eta5[run1]     = x0(run1);
        xStore(0,run1) = x0(run1);
    }

    if( nFDirs != 0 ){
        for( run1 = 0; run1 < m; run1++ ){
            etaG[run1] = fseed(diff_index[run1]);
        }
    }

    if( mp > 0 ){
        if( (int) p.getDim() < mp )
            return ACADOERROR(RET_INPUT_HAS_WRONG_DIMENSION);

        for( run1 = 0; run1 < mp; run1++ ){
            x[parameter_index[run1]] = p(run1);
        }
    }

    if( mu > 0 ){
        if( (int) u.getDim() < mu )
            return ACADOERROR(RET_INPUT_HAS_WRONG_DIMENSION);

        for( run1 = 0; run1 < mu; run1++ ){
            x[control_index[run1]] = u(run1);
        }
    }


    if( mw > 0 ){
        if( (int) w.getDim() < mw )
            return ACADOERROR(RET_INPUT_HAS_WRONG_DIMENSION);

        for( run1 = 0; run1 < mw; run1++ ){
            x[disturbance_index[run1]] = w(run1);
        }
    }


    totalTime.start();
    nFcnEvaluations = 0;


     // Initialize the scaling based on the initial states:
     // ---------------------------------------------------

        double atol;
        get( ABSOLUTE_TOLERANCE, atol );

        for( run1 = 0; run1 < m; run1++ )
            diff_scale(run1) = fabs(eta4[run1]) + atol/TOL;


     // PRINTING:
     // ---------
        if( PrintLevel == HIGH || PrintLevel == MEDIUM ){
            acadoPrintCopyrightNotice( "IntegratorRK -- A Runge Kutta integrator." );
        }
        if( PrintLevel == HIGH ){
            cout << "RK: t = " << t << "\t";
            for( run1 = 0; run1 < m; run1++ )
            	cout << "x[" << run1 << "] = " << scientific << eta4[ run1 ];
            cout << endl;
        }


    returnvalue = RET_FINAL_STEP_NOT_PERFORMED_YET;

    count3 = 0;
    count  = 1;

    while( returnvalue == RET_FINAL_STEP_NOT_PERFORMED_YET && count <= maxNumberOfSteps ){

        returnvalue = step(count);
        count++;
    }

    count2 = count-1;

    for( run1 = 0; run1 < mn; run1++ )
        iStore( 0, run1 ) = iStore( 1, run1 );

    totalTime.stop();

    if( count > maxNumberOfSteps ){
        if( PrintLevel != NONE )
            return ACADOERROR(RET_MAX_NUMBER_OF_STEPS_EXCEEDED);
        return RET_MAX_NUMBER_OF_STEPS_EXCEEDED;
    }


// 	   cout << "numIntSteps = %d\n",count-1);


    // SET THE LOGGING INFORMATION:
    // ----------------------------------------------------------------------------------------

       setLast( LOG_TIME_INTEGRATOR                              , totalTime.getTime()           );
       setLast( LOG_NUMBER_OF_INTEGRATOR_STEPS                   , count-1                       );
       setLast( LOG_NUMBER_OF_INTEGRATOR_REJECTED_STEPS          , getNumberOfRejectedSteps()    );
       setLast( LOG_NUMBER_OF_INTEGRATOR_FUNCTION_EVALUATIONS    , nFcnEvaluations               );
       setLast( LOG_NUMBER_OF_BDF_INTEGRATOR_JACOBIAN_EVALUATIONS, 0                             );
       setLast( LOG_TIME_INTEGRATOR_FUNCTION_EVALUATIONS         , functionEvaluation.getTime()  );
       setLast( LOG_TIME_BDF_INTEGRATOR_JACOBIAN_EVALUATION      , 0.0                           );
       setLast( LOG_TIME_BDF_INTEGRATOR_JACOBIAN_DECOMPOSITION   , 0.0                           );

    // ----------------------------------------------------------------------------------------


     // PRINTING:
     // ---------
        if( PrintLevel == MEDIUM ){

            if( soa == SOA_EVERYTHING_FROZEN ){
            	cout << "\n Results at  t =  " << t << "\t";
                for( run1 = 0; run1 < m; run1++ )
                	cout << "x[" << run1 << "] = " << scientific << eta4[ run1 ];
                cout << endl;
            }
            printIntermediateResults();
        }
	
	int printIntegratorProfile = 0;
	get( PRINT_INTEGRATOR_PROFILE,printIntegratorProfile );
	
	if ( (BooleanType)printIntegratorProfile == BT_TRUE )
	{
		printRunTimeProfile( );
	}
	else
	{
		if( PrintLevel == MEDIUM  || PrintLevel == HIGH )
			cout << "RK: number of steps:  " << count - 1 << endl;
	}

    return returnvalue;
}



returnValue IntegratorRK::setProtectedForwardSeed( const DVector &xSeed,
                                                   const DVector &pSeed,
                                                   const DVector &uSeed,
                                                   const DVector &wSeed,
                                                   const int    &order  ){

    if( order == 2 ){
        return setForwardSeed2( xSeed, pSeed, uSeed, wSeed);
    }
    if( order < 1 || order > 2 ){
        return ACADOERROR(RET_INPUT_OUT_OF_RANGE);
    }

    if( nBDirs > 0 ){
        return ACADOERROR(RET_INPUT_OUT_OF_RANGE);
    }

    int run2;

    if( G  != NULL ){
        delete[] G;
        G = NULL;
    }

    if( etaG  != NULL ){
        delete[] etaG;
        etaG = NULL;
    }

    nFDirs = 1;

    fseed.init(rhs->getNumberOfVariables()+1+m);
    fseed.setZero();

    G = new double[rhs->getNumberOfVariables() + 1 + m];

    for( run2 = 0; run2 < (rhs->getNumberOfVariables()+1+m); run2++ ){
        G[run2] = 0.0;
    }

    etaG = new double[m];

    if( xSeed.getDim() != 0 ){
        for( run2 = 0; run2 < m; run2++ ){
            fseed(diff_index[run2]) = xSeed(run2);
        }
    }

    if( pSeed.getDim() != 0 ){
        for( run2 = 0; run2 < mp; run2++ ){
             fseed(parameter_index[run2]) = pSeed(run2);
             G    [parameter_index[run2]] = pSeed(run2);
        }
    }

    if( uSeed.getDim() != 0 ){
        for( run2 = 0; run2 < mu; run2++ ){
            fseed(control_index[run2]) = uSeed(run2);
            G    [control_index[run2]] = uSeed(run2);
        }
    }

    if( wSeed.getDim() != 0 ){

        for( run2 = 0; run2 < mw; run2++ ){
            fseed(disturbance_index[run2]) = wSeed(run2);
                G[disturbance_index[run2]] = wSeed(run2);
        }
    }

    return SUCCESSFUL_RETURN;
}


returnValue IntegratorRK::setForwardSeed2( const DVector &xSeed ,
                                           const DVector &pSeed ,
                                           const DVector &uSeed ,
                                           const DVector &wSeed   ){

    int run2;

    if( G2  != NULL ){
        delete[] G2;
        G2 = NULL;
    }

    if( G3  != NULL ){
        delete[] G3;
        G3 = NULL;
    }

    if( etaG2  != NULL ){
        delete[] etaG2;
        etaG2 = NULL;
    }

    if( etaG3  != NULL ){
        delete[] etaG3;
        etaG3 = NULL;
    }

    nFDirs2 = 1;

    fseed2.init(rhs->getNumberOfVariables() + 1 + m);
    fseed2.setZero();

    G2 = new double[rhs->getNumberOfVariables() + 1 + m];
    G3 = new double[rhs->getNumberOfVariables() + 1 + m];

    for( run2 = 0; run2 < (rhs->getNumberOfVariables()+1+m); run2++ ){
         G2[run2] = 0.0;
         G3[run2] = 0.0;
    }
    etaG2 = new double[m];
    etaG3 = new double[m];

    if( xSeed.getDim() != 0 ){
        for( run2 = 0; run2 < m; run2++ ){
            fseed2(diff_index[run2]) = xSeed(run2);
        }
    }

    if( pSeed.getDim() != 0 ){
        for( run2 = 0; run2 < mp; run2++ ){
            fseed2(parameter_index[run2]) = pSeed(run2);
            G2    [parameter_index[run2]] = pSeed(run2);
        }
    }

    if( uSeed.getDim() != 0 ){
        for( run2 = 0; run2 < mu; run2++ ){
             fseed2(control_index[run2]) = uSeed(run2);
             G2    [control_index[run2]] = uSeed(run2);
        }
    }

    if( wSeed.getDim() != 0 ){
        for( run2 = 0; run2 < mw; run2++ ){
            fseed2(disturbance_index[run2]) = wSeed(run2);
            G2    [disturbance_index[run2]] = wSeed(run2);
        }
    }
    return SUCCESSFUL_RETURN;
}


returnValue IntegratorRK::setProtectedBackwardSeed( const DVector &seed, const int &order ){

    if( order == 2 ){
        return setBackwardSeed2(seed);
    }
    if( order < 1 || order > 2 ){
        return ACADOERROR(RET_INPUT_OUT_OF_RANGE);
    }

    if( nFDirs > 0 ){
        return ACADOERROR(RET_INPUT_OUT_OF_RANGE);
    }

    int run2;

    if( H  != NULL ){
        delete[] H;
        H = NULL;
    }

    if( etaH  != NULL ){
        delete[] etaH;
        etaH = NULL;
    }

    nBDirs = 1;

    bseed.init( m );
    bseed.setZero();

    H = new double[m];

    etaH = new double[rhs->getNumberOfVariables()+1+m];
    for( run2 = 0; run2 < rhs->getNumberOfVariables()+1+m; run2++ ){
        etaH[run2] = 0.0;
    }

    if( seed.getDim() != 0 ){
        for( run2 = 0; run2 < m; run2++ ){
            bseed(run2) = seed(run2);
        }
    }

    return SUCCESSFUL_RETURN;
}



returnValue IntegratorRK::setBackwardSeed2( const DVector &seed ){

    int run2;

    if( H2  != NULL ){
        delete[] H2;
        H2 = NULL;
    }

    if( H3  != NULL ){
        delete[] H3;
        H3 = NULL;
    }

    if( etaH2  != NULL ){
        delete[] etaH2;
        etaH2 = NULL;
    }

    if( etaH3  != NULL ){
        delete[] etaH3;
        etaH3 = NULL;
    }


    nBDirs2 = 1;

    bseed2.init(m);
    bseed2.setZero();

    H2 = new double[m];
    H3 = new double[m];

    etaH2 = new double[rhs->getNumberOfVariables()+1+m];
    etaH3 = new double[rhs->getNumberOfVariables()+1+m];

    if( seed.getDim() != 0 ){
         for( run2 = 0; run2 < m; run2++ ){
             bseed2(run2) = seed(run2);
         }
    }

    return SUCCESSFUL_RETURN;
}


returnValue IntegratorRK::evaluateSensitivities(){

    int         run1, run2 ;
    returnValue returnvalue;

    if( rhs == NULL ){
        return ACADOERROR(RET_TRIVIAL_RHS);
    }

    if( soa != SOA_EVERYTHING_FROZEN ){
        return ACADOERROR(RET_NOT_FROZEN);
    }


    if( nFDirs != 0 ){
        t = timeInterval.getFirstTime();
        dxStore.init( m, timeInterval );
        for( run1 = 0; run1 < m; run1++ ){
            etaG[run1] = fseed(diff_index[run1]);
        }
    }

    if( nBDirs != 0 ){
        for( run2 = 0; run2 < (rhs->getNumberOfVariables()+1+m); run2++){
            etaH[run2] = 0.0;
        }
    }

    if( nBDirs != 0 ){
        for( run1 = 0; run1 < m; run1++ ){
            etaH[diff_index[run1]] = bseed(run1);
        }
    }

    if( nFDirs2 != 0 ){
        t = timeInterval.getFirstTime();
        ddxStore.init( m, timeInterval );
        for( run1 = 0; run1 < m; run1++ ){
            etaG2[run1] = fseed2(diff_index[run1]);
            etaG3[run1] = 0.0;
        }
    }

    if( nBDirs2 != 0 ){
        for( run2 = 0; run2 < (rhs->getNumberOfVariables()+1+m); run2++){
            etaH2[run2] = 0.0;
            etaH3[run2] = 0.0;
        }
    }

    if( nBDirs2 != 0 ){
        for( run1 = 0; run1 < m; run1++ ){
            etaH2[diff_index[run1]] = bseed2(run1);
        }
    }

    if( PrintLevel == HIGH ){
        printIntermediateResults();
    }

    returnvalue = RET_FINAL_STEP_NOT_PERFORMED_YET;


    if( nBDirs > 0 || nBDirs2 > 0 ){

        int oldCount = count;

        count--;
        while( returnvalue == RET_FINAL_STEP_NOT_PERFORMED_YET && count >= 1 ){

            returnvalue = step( count );
            count--;
        }

        if( count == 0 && (returnvalue == RET_FINAL_STEP_NOT_PERFORMED_YET ||
                           returnvalue == SUCCESSFUL_RETURN   )            ){


            if( PrintLevel == MEDIUM ){
                printIntermediateResults();
            }
            count = oldCount;

            return SUCCESSFUL_RETURN;
        }
        count = oldCount;
    }
    else{

        count = 1;
        while( returnvalue == RET_FINAL_STEP_NOT_PERFORMED_YET &&
               count <= maxNumberOfSteps ){

            returnvalue = step(count);
            count++;
        }

        if( nBDirs2 == 0 && nFDirs != 0 )
            for( run1 = 0; run1 < m; run1++ )
                dxStore( 0, run1 ) = dxStore( 1, run1 );

        if( nFDirs2 != 0 )
            for( run1 = 0; run1 < m; run1++ )
                ddxStore( 0, run1 ) = ddxStore( 1, run1 );

        if( count > maxNumberOfSteps ){
            if( PrintLevel != NONE )
                return ACADOERROR(RET_MAX_NUMBER_OF_STEPS_EXCEEDED);
            return RET_MAX_NUMBER_OF_STEPS_EXCEEDED;
        }

        if( PrintLevel == MEDIUM ){
            printIntermediateResults();
        }
    }
    return returnvalue;
}


returnValue IntegratorRK::step(int number_){

    int run1;
    double E = EPS;

    if( soa == SOA_EVERYTHING_FROZEN || soa == SOA_MESH_FROZEN || soa == SOA_MESH_FROZEN_FREEZING_ALL ){
        h[0] = h[number_];
    }

    if( soa == SOA_FREEZING_MESH ||
        soa == SOA_MESH_FROZEN   ||
        soa == SOA_MESH_FROZEN_FREEZING_ALL ||
        soa == SOA_UNFROZEN      ){

        if( nFDirs > 0 ){
            E = determineEta45(0);
        }
        else{
            E = determineEta45();
        }
    }
    if( soa == SOA_FREEZING_ALL ){
        E = determineEta45(dim*number_);
    }


    if( soa != SOA_EVERYTHING_FROZEN && soa != SOA_MESH_FROZEN && soa != SOA_MESH_FROZEN_FREEZING_ALL ){

        int number_of_rejected_steps = 0;

        if( E < 0.0 ){
            return ACADOERROR(RET_UNSUCCESSFUL_RETURN_FROM_INTEGRATOR_RK45);
        }

        // REJECT THE STEP IF GIVEN TOLERANCE IS NOT ACHIEVED:
        // ---------------------------------------------------
        while( E >= TOL*h[0] ){

            if( PrintLevel == HIGH ){
                cout << "STEP REJECTED: error estimate           = " << scientific << E << endl
                	 << "               required local tolerance = " << TOL*h[0] << endl;
            }

            number_of_rejected_steps++;

            for( run1 = 0; run1 < m; run1++ ){
                eta4[run1] = eta4_[run1];
            }
            if( h[0] <= hmin + EPS ){
                return ACADOERROR(RET_UNSUCCESSFUL_RETURN_FROM_INTEGRATOR_RK45);
            }
            h[0] = 0.5*h[0];
            if( h[0] < hmin ){
                h[0] = hmin;
            }

            if( soa == SOA_FREEZING_MESH ||
                soa == SOA_UNFROZEN      ){

                if( nFDirs > 0 ){
                    E = determineEta45(0);
                }
                else{
                    E = determineEta45();
                }
            }
            if( soa == SOA_FREEZING_ALL ){
                E = determineEta45(dim*number_);
            }

            if( E < 0.0 ){
                return ACADOERROR(RET_UNSUCCESSFUL_RETURN_FROM_INTEGRATOR_RK45);
            }
        }

        count3 += number_of_rejected_steps;
    }

    // PROCEED IF THE STEP IS ACCEPTED:
    // --------------------------------

       double *etaG_  = new double[m];
       double *etaG3_ = new double[m];


     // compute forward derivatives if requested:
     // ------------------------------------------

     if( nFDirs > 0 && nBDirs2 == 0 && nFDirs2 == 0 ){

         for( run1 = 0; run1 < m; run1++ )
             etaG_[run1]  = etaG[run1];

         if( nBDirs != 0 ){
             return ACADOERROR(RET_WRONG_DEFINITION_OF_SEEDS);
         }

         if( soa == SOA_FREEZING_ALL || soa == SOA_EVERYTHING_FROZEN ){
             determineEtaGForward(dim*number_);
         }
         else{
             determineEtaGForward(0);
         }
     }
     if( nBDirs > 0 ){

         if( soa != SOA_EVERYTHING_FROZEN ){
             return ACADOERROR(RET_NOT_FROZEN);
         }
         if( nFDirs != 0 || nBDirs2 != 0 || nFDirs2 != 0 ){
             return ACADOERROR(RET_WRONG_DEFINITION_OF_SEEDS);
         }
         determineEtaHBackward(dim*number_);
     }
     if( nFDirs2 > 0 ){

         for( run1 = 0; run1 < m; run1++ )
             etaG3_[run1]  = etaG3[run1];

         if( soa != SOA_EVERYTHING_FROZEN ){
             return ACADOERROR(RET_NOT_FROZEN);
         }
         if( nBDirs != 0 || nBDirs2 != 0 || nFDirs != 1 ){
             return ACADOERROR(RET_WRONG_DEFINITION_OF_SEEDS);
         }
         determineEtaGForward2(dim*number_);
     }
     if( nBDirs2 > 0 ){

         if( soa != SOA_EVERYTHING_FROZEN ){
             return ACADOERROR(RET_NOT_FROZEN);
         }
         if( nBDirs != 0 || nFDirs2 != 0 || nFDirs != 1 ){
             return ACADOERROR(RET_WRONG_DEFINITION_OF_SEEDS);
         }

         determineEtaHBackward2(dim*number_);
     }


     // increase the time:
     // ----------------------------------------------

     if( nBDirs > 0 || nBDirs2 > 0 ){

         t = t - h[0];
     }
     else{

         t = t + h[0];
     }
// 	printf("t = %e\n",t );

     // PRINTING:
     // ---------
     if( PrintLevel == HIGH ){
    	 cout << "RK: t = " << scientific << t << "  h = " << h[ 0 ] << "  ";
         printIntermediateResults();
     }


     // STORAGE:
     // --------

     if( soa == SOA_FREEZING_MESH || soa == SOA_FREEZING_ALL || soa == SOA_MESH_FROZEN_FREEZING_ALL ){

         if( number_ >= maxAlloc){

             maxAlloc = 2*maxAlloc;
             h = (double*)realloc(h,maxAlloc*sizeof(double));
         }
         h[number_] = h[0];
     }

     int i1 = timeInterval.getFloorIndex( t-h[0] );
     int i2 = timeInterval.getFloorIndex( t      );
     int jj;

     for( jj = i1+1; jj <= i2; jj++ ){

         if( nFDirs == 0 && nBDirs  == 0 && nFDirs2 == 0 && nBDirs == 0 ) interpolate( jj, eta4_ , k[0], eta4 ,   xStore );
         if( nFDirs  > 0 && nBDirs2 == 0 && nFDirs2 == 0                ) interpolate( jj, etaG_ , k[0], etaG ,  dxStore );
         if( nFDirs2 > 0                                                ) interpolate( jj, etaG3_, k[0], etaG3, ddxStore );

         for( run1 = 0; run1 < mn; run1++ )
             iStore( jj, run1 ) = x[rhs->index( VT_INTERMEDIATE_STATE, run1 )];
     }

     delete[] etaG_ ;
     delete[] etaG3_;


     if( nBDirs == 0 || nBDirs2 == 0 ){

     // Stop the algorithm if  t >= te:
     // ----------------------------------------------
        if( t >= timeInterval.getLastTime() - EPS ){
            x[time_index] = timeInterval.getLastTime();
            for( run1 = 0; run1 < m; run1++ ){
				if ( acadoIsNaN( eta4[run1] ) == BT_TRUE )
					return ACADOERROR( RET_UNSUCCESSFUL_RETURN_FROM_INTEGRATOR_RK45 );
                x[diff_index[run1]] = eta4[run1];
            }

            if( soa == SOA_FREEZING_MESH ){
                soa = SOA_MESH_FROZEN;
            }
            if( soa == SOA_FREEZING_ALL || soa == SOA_MESH_FROZEN_FREEZING_ALL ){
                soa = SOA_EVERYTHING_FROZEN;
            }
            
            return SUCCESSFUL_RETURN;
        }
     }


     if( soa != SOA_EVERYTHING_FROZEN && soa != SOA_MESH_FROZEN && soa != SOA_MESH_FROZEN_FREEZING_ALL ){


     // recompute the scaling based on the actual states:
     // -------------------------------------------------

        double atol;
        get( ABSOLUTE_TOLERANCE, atol );

        for( run1 = 0; run1 < m; run1++ )
            diff_scale(run1) = fabs(eta4[run1]) + atol/TOL;



     // apply a numeric stabilization of the step size control:
     // -------------------------------------------------------
        double Emin = 1e-3*sqrt(TOL)*pow(hini, ((1.0/err_power)+1.0)/2.0 );

        if( E < Emin     ) E = Emin    ;
        if( E < 10.0*EPS ) E = 10.0*EPS;


     // determine the new step size:
     // ----------------------------------------------
        h[0] = h[0]*pow( tune*(TOL*h[0]/E), err_power );

        if( h[0] > hmax ){
          h[0] = hmax;
        }
        if( h[0] < hmin ){
          h[0] = hmin;
        }

        if( t + h[0] >= timeInterval.getLastTime() ){
          h[0] = timeInterval.getLastTime()-t;
        }
        
//         printf( "t = %e,  stepsize = %e\n", t,h[0] );
    }

    return RET_FINAL_STEP_NOT_PERFORMED_YET;
}



returnValue IntegratorRK::stop(){

    return ACADOERROR(RET_NOT_IMPLEMENTED_YET);
}


returnValue IntegratorRK::getProtectedX( DVector *xEnd ) const{

    int run1;

    if( (int) xEnd[0].getDim() != m )
        return RET_INPUT_HAS_WRONG_DIMENSION;

    for( run1 = 0; run1 < m; run1++ )
        xEnd[0](run1) = eta4[run1];

    return SUCCESSFUL_RETURN;
}


returnValue IntegratorRK::getProtectedForwardSensitivities( DMatrix *Dx, int order ) const{

    int run1;

    if( Dx == NULL ){
        return SUCCESSFUL_RETURN;
    }

    if( order == 1 && nFDirs2 == 0 ){
        for( run1 = 0; run1 < m; run1++ ){
            Dx[0](run1,0) = etaG[run1];
        }
    }

    if( order == 2 ){
        for( run1 = 0; run1 < m; run1++ ){
            Dx[0](run1,0) = etaG3[run1];
        }
    }

    if( order == 1 && nFDirs2 > 0 ){
        for( run1 = 0; run1 < m; run1++ ){
            Dx[0](run1,0) = etaG2[run1];
        }
    }

    if( order < 1 || order > 2 ){
        return ACADOERROR(RET_INPUT_OUT_OF_RANGE);
    }

    return SUCCESSFUL_RETURN;
}


returnValue IntegratorRK::getProtectedBackwardSensitivities( DVector &Dx_x0,
                                                             DVector &Dx_p ,
                                                             DVector &Dx_u ,
                                                             DVector &Dx_w ,
                                                             int order      ) const{

    int run2;

    if( order == 1 && nBDirs2 == 0 ){

        if( Dx_x0.getDim() != 0 ){
            for( run2 = 0; run2 < m; run2++ )
                Dx_x0(run2) = etaH[diff_index[run2]];
        }
        if( Dx_p.getDim() != 0 ){
            for( run2 = 0; run2 < mp; run2++ ){
                Dx_p(run2) = etaH[parameter_index[run2]];
            }
        }
        if( Dx_u.getDim() != 0 ){
            for( run2 = 0; run2 < mu; run2++ ){
                Dx_u(run2) = etaH[control_index[run2]];
            }
        }
        if( Dx_w.getDim() != 0 ){
            for( run2 = 0; run2 < mw; run2++ ){
                Dx_w(run2) = etaH[disturbance_index[run2]];
            }
        }
    }

    if( order == 1 && nBDirs2 > 0 ){

        if( Dx_x0.getDim() != 0 ){
            for( run2 = 0; run2 < m; run2++ ){
                Dx_x0(run2) = etaH2[diff_index[run2]];
            }
        }
        if( Dx_u.getDim() != 0 ){
            for( run2 = 0; run2 < mu; run2++ ){
                Dx_u(run2) = etaH2[control_index[run2]];
            }
        }
        if( Dx_p.getDim() != 0 ){
            for( run2 = 0; run2 < mp; run2++ ){
                Dx_p(run2) = etaH2[parameter_index[run2]];
            }
        }
        if( Dx_w.getDim() != 0 ){
            for( run2 = 0; run2 < mw; run2++ ){
                 Dx_w(run2) = etaH2[disturbance_index[run2]];
            }
        }
    }


    if( order == 2 ){

        if( Dx_x0.getDim() != 0 ){
            for( run2 = 0; run2 < m; run2++ ){
                Dx_x0(run2) = etaH3[diff_index[run2]   ];
            }
        }
        if( Dx_u.getDim() != 0 ){
            for( run2 = 0; run2 < mu; run2++ ){
               Dx_u(run2) = etaH3[control_index[run2]];
            }
        }
        if( Dx_p.getDim() != 0 ){
            for( run2 = 0; run2 < mp; run2++ ){
                 Dx_p(run2) = etaH3[parameter_index[run2]];
            }
        }
        if( Dx_w.getDim() != 0 ){
            for( run2 = 0; run2 < mw; run2++ ){
                 Dx_w(run2) = etaH3[disturbance_index[run2]];
            }
        }
    }

    if( order < 1 || order > 2 ){

        return ACADOERROR(RET_INPUT_OUT_OF_RANGE);
    }

    return SUCCESSFUL_RETURN;
}


int IntegratorRK::getNumberOfSteps() const{

    return count2;
}

int IntegratorRK::getNumberOfRejectedSteps() const{

    return count3;
}


double IntegratorRK::getStepSize() const{

    return h[0];
}


returnValue IntegratorRK::setDxInitialization( double *dx0 ){

    return SUCCESSFUL_RETURN;
}

//
// PROTECTED MEMBER FUNCTIONS:
//


double IntegratorRK::determineEta45(){

    int run1, run2, run3;
    double E;

    // determine k:
    // -----------------------------------------------
       for( run1 = 0; run1 < dim; run1++ ){
           x[time_index] = t + c[run1]*h[0];
           for( run2 = 0; run2 < m; run2++ ){
               x[diff_index[run2]] = eta4[run2];
               for( run3 = 0; run3 < run1; run3++ ){
                   x[diff_index[run2]] = x[diff_index[run2]] +
                                         A[run1][run3]*h[0]*k[run3][run2];
               }
           }
           functionEvaluation.start();

           if( rhs[0].evaluate( 0, x, k[run1] ) != SUCCESSFUL_RETURN ){
               ACADOERROR(RET_UNSUCCESSFUL_RETURN_FROM_INTEGRATOR_RK45);
               return -1.0;
           }

           functionEvaluation.stop();
           nFcnEvaluations++;
       }

    // save previous eta4:
    // ----------------------------------------------

       for( run1 = 0; run1 < m; run1++ ){
           eta4_[run1]  = eta4[run1];
           eta5 [run1]  = eta4[run1]; // TODO: check if eta4 is correct here!?
		   //printf( "%e\n",eta4[run1] );
       }

    // determine eta4 and eta5:
    // ----------------------------------------------

       for( run1 = 0; run1 < dim; run1++ ){
           for( run2 = 0; run2 < m; run2++ ){
               eta4[run2] = eta4[run2] + b4[run1]*h[0]*k[run1][run2];
               eta5[run2] = eta5[run2] + b5[run1]*h[0]*k[run1][run2];
           }
       }

    // determine local error estimate E:
    // ----------------------------------------------

       E = EPS;
       for( run2 = 0; run2 < m; run2++ ){
           if( (eta4[run2]-eta5[run2])/diff_scale(run2) >= E  ){
               E = (eta4[run2]-eta5[run2])/diff_scale(run2);
           }
           if( (eta4[run2]-eta5[run2])/diff_scale(run2) <= -E ){
               E = (-eta4[run2]+eta5[run2])/diff_scale(run2);
           }
       }

    return E;
}



double IntegratorRK::determineEta45( int number_ ){

    int run1, run2, run3;
    double E;

    // determine k:
    // -----------------------------------------------
       for( run1 = 0; run1 < dim; run1++ ){
           x[time_index] = t + c[run1]*h[0];
           for( run2 = 0; run2 < m; run2++ ){
               x[diff_index[run2]] = eta4[run2];
               for( run3 = 0; run3 < run1; run3++ ){
                   x[diff_index[run2]] = x[diff_index[run2]] +
                                         A[run1][run3]*h[0]*k[run3][run2];
               }
           }
           functionEvaluation.start();

           if( rhs[0].evaluate( number_+run1, x, k[run1] ) != SUCCESSFUL_RETURN ){
               ACADOERROR(RET_UNSUCCESSFUL_RETURN_FROM_INTEGRATOR_RK45);
               return -1.0;
           }

           functionEvaluation.stop();
           nFcnEvaluations++;
       }

    // save previous eta4:
    // ----------------------------------------------

       for( run1 = 0; run1 < m; run1++ ){
           eta4_[run1]  = eta4[run1];
           eta5 [run1]  = eta4[run1];
       }

    // determine eta4 and eta5:
    // ----------------------------------------------
       for( run1 = 0; run1 < dim; run1++ ){
           for( run2 = 0; run2 < m; run2++ ){
               eta4[run2] = eta4[run2] + b4[run1]*h[0]*k[run1][run2];
               eta5[run2] = eta5[run2] + b5[run1]*h[0]*k[run1][run2];
           }
       }

    // determine local error estimate E:
    // ----------------------------------------------

       E = EPS;
       for( run2 = 0; run2 < m; run2++ ){
           if( (eta4[run2]-eta5[run2])/diff_scale(run2) >= E  )
               E = (eta4[run2]-eta5[run2])/diff_scale(run2);
           if( (eta4[run2]-eta5[run2])/diff_scale(run2) <= -E )
               E = (-eta4[run2]+eta5[run2])/diff_scale(run2);
       }

    return E;
}


void IntegratorRK::determineEtaGForward( int number_ ){

    int run1, run2, run3;

    // determine k:
    // -----------------------------------------------
       for( run1 = 0; run1 < dim; run1++ ){
           for( run2 = 0; run2 < m; run2++ ){
               G[diff_index[run2]] = etaG[run2];
               for( run3 = 0; run3 < run1; run3++ ){
                   G[diff_index[run2]] = G[diff_index[run2]] +
                                               A[run1][run3]*h[0]*k[run3][run2];
               }
           }
           if( rhs[0].AD_forward( number_+run1, G, k[run1] ) != SUCCESSFUL_RETURN ){
               ACADOERROR(RET_UNSUCCESSFUL_RETURN_FROM_INTEGRATOR_RK45);
               return;
           }
       }

    // determine etaG:
    // ----------------------------------------------
       for( run1 = 0; run1 < dim; run1++ ){
           for( run2 = 0; run2 < m; run2++ ){
               etaG[run2] = etaG[run2] + b4[run1]*h[0]*k[run1][run2];
           }
       }
}



void IntegratorRK::determineEtaGForward2( int number_ ){

    int run1, run2, run3;

    // determine k:
    // -----------------------------------------------
       for( run1 = 0; run1 < dim; run1++ ){
           for( run2 = 0; run2 < m; run2++ ){
               G2[diff_index[run2]] = etaG2[run2];
               G3[diff_index[run2]] = etaG3[run2];
               for( run3 = 0; run3 < run1; run3++ ){
                   G2[diff_index[run2]] = G2[diff_index[run2]] +
                                                A[run1][run3]*h[0]*k[run3][run2];
                   G3[diff_index[run2]] = G3[diff_index[run2]] +
                                                A[run1][run3]*h[0]*k2[run3][run2];
               }
           }

           if( rhs[0].AD_forward2( number_+run1, G2, G3, k[run1], k2[run1] )
               != SUCCESSFUL_RETURN ){
               ACADOERROR(RET_UNSUCCESSFUL_RETURN_FROM_INTEGRATOR_RK45);
               return;
           }
       }

    // determine etaG2:
    // ----------------------------------------------

       for( run1 = 0; run1 < dim; run1++ ){
           for( run2 = 0; run2 < m; run2++ ){
               etaG2[run2] = etaG2[run2] + b4[run1]*h[0]*k[run1][run2];
               etaG3[run2] = etaG3[run2] + b4[run1]*h[0]*k2[run1][run2];
           }
       }
}



void IntegratorRK::determineEtaHBackward( int number_ ){

    int run1, run2, run3;
    const int ndir = rhs->getNumberOfVariables() + 1 + m;

        for( run1 = 0; run1 < dim; run1++ ){
            for( run2 = 0; run2 < ndir; run2++ ){
                l[run1][run2] = 0.0;
            }
        }
        for( run1 = dim-1; run1 >= 0; run1--){
             for( run2 = 0; run2 < m; run2++ ){
                 H[run2] = b4[run1]*h[0]*etaH[diff_index[run2]];
                 for( run3 = run1+1; run3 < dim; run3++ ){
                      H[run2] = H[run2] + A[run3][run1]*h[0]*l[run3][diff_index[run2]];
                 }
             }

             if( rhs[0].AD_backward( number_+run1, H, l[run1] )!= SUCCESSFUL_RETURN ){
                 ACADOERROR(RET_UNSUCCESSFUL_RETURN_FROM_INTEGRATOR_RK45);
                 return;
             }
        }

    // determine etaH:
    // ----------------------------------------------
       for( run1 = 0; run1 < dim; run1++ ){
           for( run2 = 0; run2 < ndir; run2++ ){
               etaH[run2] = etaH[run2] + l[run1][run2];
           }
       }
}


void IntegratorRK::determineEtaHBackward2( int number_ ){

    int run1, run2, run3;
    const int ndir = rhs->getNumberOfVariables() + 1 + m;

    for( run1 = 0; run1 < dim; run1++ ){
        for( run2 = 0; run2 < ndir; run2++ ){
            l [run1][run2] = 0.0;
            l2[run1][run2] = 0.0;
        }
    }

    for( run1 = dim-1; run1 >= 0; run1--){
         for( run2 = 0; run2 < m; run2++ ){
             H2[run2] = b4[run1]*h[0]*etaH2[diff_index[run2]];
             H3[run2] = b4[run1]*h[0]*etaH3[diff_index[run2]];
             for( run3 = run1+1; run3 < dim; run3++ ){
                  H2[run2] = H2[run2] + A[run3][run1]*h[0]*l[run3][diff_index[run2]];
                  H3[run2] = H3[run2] + A[run3][run1]*h[0]*l2[run3][diff_index[run2]];
             }
         }
         if( rhs[0].AD_backward2( number_+run1, H2, H3, l[run1], l2[run1] )
             != SUCCESSFUL_RETURN ){
             ACADOERROR(RET_UNSUCCESSFUL_RETURN_FROM_INTEGRATOR_RK45);
              return;
         }
    }

    // determine etaH:
    // ----------------------------------------------

    for( run1 = 0; run1 < dim; run1++ ){
        for( run2 = 0; run2 < ndir; run2++ ){
            etaH2[run2] = etaH2[run2] + l [run1][run2];
            etaH3[run2] = etaH3[run2] + l2[run1][run2];
        }
    }
}


void IntegratorRK::printIntermediateResults(){

    int run1, run2;

        if( soa != SOA_EVERYTHING_FROZEN ){
            for( run1 = 0; run1 < m; run1++ ){
            	cout << "x[" << run1 << "] = " << eta4[run1] << "  ";
            }
            cout << endl;
        }
        else{

            cout << endl;
        }

        // Forward Sensitivities:
        // ----------------------

        if( nFDirs > 0 && nBDirs2 == 0 && nFDirs2 == 0 ){
            cout << "RK: Forward Sensitivities:\n";
            for( run1 = 0; run1 < m; run1++ ){
                cout << scientific << etaG[run1] << "  ";
            }
            cout << endl;
        }


        // 2nd Order Forward Sensitivities:
        // ---------------------------------

        if( nFDirs2 > 0 ){

            cout << "RK: First Order Forward Sensitivities:\n";
            for( run1 = 0; run1 < m; run1++ ){
            	cout << scientific << etaG2[run1] << "  ";
            }
            cout << endl;

            cout << "RK: Second Order Forward Sensitivities:\n";
            for( run1 = 0; run1 < m; run1++ ){
            	cout << scientific << etaG3[run1] << "  ";
            }
            cout << endl;
        }

        // Backward Sensitivities:
        // -----------------------

        if( nBDirs > 0 ){

            cout << "RK: Backward Sensitivities:\n";

            cout << "w.r.t. the states:\n" << scientific;
            for( run2 = 0; run2 < m; run2++ ){
            	cout << etaH[diff_index[run2]] << "  ";
            }
            cout << endl;

            if( mu > 0 ){
                cout << "w.r.t. the controls:\n" << scientific;
                for( run2 = 0; run2 < mu; run2++ ){
                	cout << etaH[control_index[run2]] << "  ";
                }
                cout << endl;
            }
            if( mp > 0 ){
                cout << "w.r.t. the parameters:\n" << scientific;;
                for( run2 = 0; run2 < mp; run2++ ){
                	cout << etaH[parameter_index[run2]] << "  ";
                }
                cout << endl;
            }
            if( mw > 0 ){
                cout << "w.r.t. the disturbances:\n" << scientific;;
                for( run2 = 0; run2 < mw; run2++ ){
                	cout << etaH[disturbance_index[run2]] << "  ";
                }
                cout << endl;
            }
        }


        // 2nd order Backward Sensitivities:
        // ---------------------------------

        if( nBDirs2 > 0 ){

            cout << "RK: First order Backward Sensitivities:\n";

            cout << "w.r.t. the states:\n" << scientific;;
            for( run2 = 0; run2 < m; run2++ ){
            	cout << etaH2[diff_index[run2]] << "  ";
            }
            cout << endl;

            if( mu > 0 ){
                cout << "w.r.t. the controls:\n" << scientific;;
                for( run2 = 0; run2 < mu; run2++ ){
                	cout << etaH2[control_index[run2]] << "  ";
                }
                cout << endl;
            }
            if( mp > 0 ){
                cout << "w.r.t. the parameters:\n" << scientific;;
                for( run2 = 0; run2 < mp; run2++ ){
                	cout << etaH2[parameter_index[run2]] << "  ";
                }
                cout << endl;
            }
            if( mw > 0 ){
                cout << "w.r.t. the disturbances:\n" << scientific;;
                for( run2 = 0; run2 < mw; run2++ ){
                	cout << etaH2[disturbance_index[run2]] << "  ";
                }
                cout << endl;
            }

            cout << "RK: Second order Backward Sensitivities:\n";

            cout << "w.r.t. the states:\n" << scientific;
            for( run2 = 0; run2 < m; run2++ ){
            	cout << etaH3[diff_index[run2]] << "  ";
            }
            cout << endl;

            if( mu > 0 ){
                cout << "w.r.t. the controls:\n" << scientific;
                for( run2 = 0; run2 < mu; run2++ ){
                	cout << etaH3[control_index[run2]] << "  ";
                }
                cout << "\n";
            }

            if( mp > 0 ){
                cout << "w.r.t. the parameters:\n" << scientific;
                for( run2 = 0; run2 < mp; run2++ ){
                	cout << etaH3[parameter_index[run2]] << "  ";
                }
                cout << "\n";
            }

            if( mw > 0 ){
                cout << "w.r.t. the disturbances:\n" << scientific;
                for( run2 = 0; run2 < mw; run2++ ){
                	cout << etaH3[disturbance_index[run2]] << "  ";
                }
                cout << "\n";
            }
     }
}

void IntegratorRK::interpolate( int jj, double *e1, double *d1, double *e2, VariablesGrid &poly ){

    int run1;

    for( run1 = 0; run1 < m; run1++ ){

        double cc = e1[run1];
        double bb = d1[run1];
        double aa = (e2[run1] - bb*h[0] - cc)/(h[0]*h[0]);

        double tt = timeInterval.getTime(jj) - t + h[0];

        poly( jj, run1 ) = aa*tt*tt + bb*tt + cc;
    }
}


int IntegratorRK::getDim() const{

    return m;
}


CLOSE_NAMESPACE_ACADO


// end of file.
