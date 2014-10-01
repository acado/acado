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
 *    \file src/integrator/integrator.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date 2008 - 2013
 */

#include <acado/utils/acado_utils.hpp>
#include <acado/symbolic_expression/symbolic_expression.hpp>
#include <acado/function/function_.hpp>
#include <acado/function/differential_equation.hpp>
#include <acado/integrator/integrator.hpp>
#include <acado/integrator/integrator_runge_kutta.hpp>
#include <acado/integrator/integrator_runge_kutta12.hpp>
#include <acado/integrator/integrator_runge_kutta23.hpp>
#include <acado/integrator/integrator_runge_kutta45.hpp>
#include <acado/integrator/integrator_runge_kutta78.hpp>
#include <acado/integrator/integrator_bdf.hpp>

using namespace std;

BEGIN_NAMESPACE_ACADO



//
// PUBLIC MEMBER FUNCTIONS:
//

Integrator::Integrator( )
           :AlgorithmicBase( ){

    // RHS:
    // --------
       rhs = 0;
       m   = 0;
       ma  = 0;
       mdx = 0;
       md  = 0;
       mn  = 0;
       mu  = 0;
       mui = 0;
       mp  = 0;
       mpi = 0;
       mw  = 0;

    transition = 0;


    // SETTINGS:
    // ---------
    h     = (double*)calloc(1,sizeof(double));
    h[0]  = 0.001    ;
    hmin  = 0.000001 ;
    hmax  = 1.0e10   ;

    tune  = 0.5      ;
    TOL   = 0.000001 ;


    // INTERNAL INDEX LISTS:
    // ---------------------
    diff_index          = 0;
    ddiff_index         = 0;
    alg_index           = 0;
    control_index       = 0;
    parameter_index     = 0;
    int_control_index   = 0;
    int_parameter_index = 0;
    disturbance_index   = 0;
    time_index          = 0;


    // OTHERS:
    // -------
    count            = 0   ;
    count2           = 0   ;
    count3           = 0   ;
    maxNumberOfSteps = 1000;


    // PRINT-LEVEL:
    // ------------
    PrintLevel = LOW;


    // SEED DIMENSIONS:
    // ----------------
    nFDirs         = 0;
    nBDirs         = 0;
    nFDirs2        = 0;
    nBDirs2        = 0;


    // THE STATE OF AGGREGATION:
    // -------------------------
    soa        = SOA_UNFROZEN;

    setupOptions( );
    setupLogging( );
}



Integrator::Integrator( const Integrator &arg )
           :AlgorithmicBase( arg ){

    if( arg.transition == 0 )  transition = 0;
    else                       transition = new Transition( *arg.transition );
}


Integrator::~Integrator( ){


    // RHS:
    // ---------
    if( rhs != NULL )
        delete rhs;

    if( transition != 0 )
        delete transition;


    // SETTINGS:
    // ---------
    free(h);


    // INTERNAL INDEX LISTS:
    // ---------------------

    if( diff_index != 0 )
        delete[] diff_index;

    if( ddiff_index != 0 )
        delete[] ddiff_index;

    if( alg_index != 0 )
        delete[] alg_index;

    if( control_index != 0 )
        delete[] control_index;

    if( parameter_index != 0 )
        delete[] parameter_index;

    if( int_control_index != 0 )
        delete[] int_control_index;

    if( int_parameter_index != 0 )
        delete[] int_parameter_index;

    if( disturbance_index != 0 )
        delete[] disturbance_index;
}


returnValue Integrator::init(	const DifferentialEquation &rhs_,
								const Transition           &trs 
								)
{
    returnValue returnvalue;

    returnvalue = init( rhs_ );
    transition = new Transition(trs);

    return returnvalue;
}


returnValue Integrator::setTransition(	const Transition &trs
										)
{
    if( transition != 0 ) delete transition;
    transition = new Transition(trs);
    return SUCCESSFUL_RETURN;
}


returnValue Integrator::integrate(	double t0_  ,
									double tend_,
									double *x0  ,
									double *xa  ,
									double *p   ,
									double *u   ,
									double *w    ){

    Grid t_( t0_, tend_, 2 );
    return integrate( t_, x0, xa, p, u, w );
}


returnValue Integrator::integrate(	const Grid  &t_,
									double      *x0,
									double      *xa,
									double      *p ,
									double      *u ,
									double      *w  ){

    if( rhs == 0 ) return ACADOERROR( RET_TRIVIAL_RHS );
    DVector components = rhs->getDifferentialStateComponents();

    DVector tmpX ( components.getDim(), x0 );
    DVector tmpXA( rhs->getNXA()      , xa );
    DVector tmpP ( rhs->getNP ()      , p  );
    DVector tmpU ( rhs->getNU ()      , u  );
    DVector tmpW ( rhs->getNW ()      , w  );

    return integrate( t_, tmpX, tmpXA, tmpP, tmpU, tmpW );
}



returnValue Integrator::integrate(	double       t0_  ,
									double       tend_,
									const DVector &x0  ,
									const DVector &xa  ,
									const DVector &p   ,
									const DVector &u   ,
									const DVector &w    ){

    Grid t_( t0_, tend_, 2 );
    return integrate( t_, x0, xa, p, u, w );
}



returnValue Integrator::integrate(	const Grid   &t_  ,
									const DVector &x0  ,
									const DVector &xa  ,
									const DVector &p   ,
									const DVector &u   ,
									const DVector &w    ){

    int run1;
    returnValue returnvalue;
    if( rhs == 0 ) return ACADOERROR( RET_TRIVIAL_RHS );

    DVector tmpX;

    DVector components = rhs->getDifferentialStateComponents();

    const int N = components.getDim();

    if( x0.getDim() != 0 ){
        tmpX.init( components.getDim() );
        for( run1 = 0; run1 < (int) components.getDim(); run1++ )
            tmpX(run1) = x0((int) components(run1));
    }


// 	tmpX.print( "integrator x0" );
// 	u.print( "integrator u0" );
// 	p.print( "integrator p0" );
    returnvalue = evaluate( tmpX, xa, p, u, w, t_ );

    if( returnvalue != SUCCESSFUL_RETURN )
        return returnvalue;

    xE.init(rhs->getDim());
    xE.setZero();

    DVector tmp(rhs->getDim());
    getProtectedX(&tmp);

    for( run1 = 0; run1 < N; run1++ )
        xE((int) components(run1)) = tmp(run1);

    for( run1 = N; run1 < N + ma; run1++ )
        xE(run1) = tmp(run1);

    if( transition != 0 )
        returnvalue = evaluateTransition( t_.getLastTime(), xE, xa, p, u, w );

    return returnvalue;
}


// ======================================================================================

returnValue Integrator::setForwardSeed(	const int    &order,
										const DVector &xSeed,
										const DVector &pSeed,
										const DVector &uSeed,
										const DVector &wSeed  ){


    int run1;
    if( rhs == 0 ) return ACADOERROR( RET_TRIVIAL_RHS );

    DVector tmpX;
    DVector components = rhs->getDifferentialStateComponents();

    dP = pSeed;
    dU = uSeed;
    dW = wSeed;

    if( xSeed.getDim() != 0 ){

        tmpX.init( components.getDim() );
        for( run1 = 0; run1 < (int) components.getDim(); run1++ )
             tmpX(run1) = xSeed((int) components(run1));
    }

    return setProtectedForwardSeed( tmpX, pSeed, uSeed, wSeed, order );
}


// ======================================================================================

returnValue Integrator::setBackwardSeed(	const int    &order,
											const DVector &seed   ){

    dXb = seed;

    uint run1;
    DVector tmp( seed.getDim() );
    DVector components = rhs->getDifferentialStateComponents();

    if( seed.getDim() != 0 ){
        tmp.init( components.getDim() );
        for( run1 = 0; run1 < components.getDim(); run1++ )
             tmp(run1) = seed((int) components(run1));
    }
    return setProtectedBackwardSeed( tmp, order );
}



returnValue Integrator::integrateSensitivities( ){

    uint run1;
    returnValue returnvalue;


    if( ( nBDirs > 0 || nBDirs2 > 0 ) && transition != 0 ){

        int order;
        if( nBDirs2 > 0 ) order = 2;
        else              order = 1;

        returnvalue = diffTransitionBackward( dXb, dPb, dUb, dWb, order );

        setBackwardSeed( order, dXb );

        if( returnvalue != SUCCESSFUL_RETURN ) return ACADOERROR(returnvalue);
    }

    returnvalue = evaluateSensitivities();

    if( returnvalue != SUCCESSFUL_RETURN ) return ACADOERROR(returnvalue);


    if( nBDirs > 0 || nBDirs2 > 0 ) return SUCCESSFUL_RETURN;

    int order = 1;
    if( nFDirs2 > 0 ) order = 2;

    DMatrix tmp( rhs->getDim(), 1 );
    returnvalue = getProtectedForwardSensitivities(&tmp,order);

    DVector components = rhs->getDifferentialStateComponents();

    dX.init(rhs->getDim()-ma);
    dX.setZero();

    for( run1 = 0; run1 < components.getDim(); run1++ )
        dX((int) components(run1)) = tmp(run1,0);

    if( returnvalue != SUCCESSFUL_RETURN ) return ACADOERROR(returnvalue);

    if( transition != 0 )
        returnvalue = diffTransitionForward( dX, dP, dU, dW, order );

    return returnvalue;
}


returnValue Integrator::getForwardSensitivities(	DVector &Dx,
													int order ) const{

    Dx = dX;
    return SUCCESSFUL_RETURN;
}


returnValue Integrator::getForwardSensitivities(	VariablesGrid &Dx,
													int order ) const{

   if( order == 1 ) Dx =  dxStore;
   if( order == 2 ) Dx = ddxStore;

   return SUCCESSFUL_RETURN;
}


returnValue Integrator::getBackwardSensitivities(	DVector &DX,
													DVector &DP ,
													DVector &DU ,
													DVector &DW ,
													int    order   ) const{

    int run2;
    returnValue returnvalue;

    DVector tmpX ( rhs->getDim() );

    DX.setZero();
    DP.setZero();
    DU.setZero();
    DW.setZero();

    returnvalue = getProtectedBackwardSensitivities( tmpX, DP, DU, DW, order );
    DVector components = rhs->getDifferentialStateComponents();

    for( run2 = 0; run2 < (int) components.getDim(); run2++ )
        DX((int) components(run2)) = tmpX(run2);

    for( run2 = 0; run2 < (int) dPb.getDim(); run2++ )
        DP(run2) += dPb(run2);

    for( run2 = 0; run2 < (int) dUb.getDim(); run2++ )
        DU(run2) += dUb(run2);

    for( run2 = 0; run2 < (int) dWb.getDim(); run2++ )
        DW(run2) += dWb(run2);

    return returnvalue;
}


BooleanType Integrator::canHandleImplicitSwitches( ) const{

    return BT_FALSE;
}


BooleanType Integrator::isDifferentialEquationDefined( ) const{

    if ( rhs != 0 ) return BT_TRUE ;
    else            return BT_FALSE;
}

BooleanType Integrator::isDifferentialEquationAffine( ) const
{
    if ( rhs == 0 ) return BT_FALSE ;
    else            return rhs->isAffine();
}


double Integrator::getDifferentialEquationSampleTime( ) const
{
	if ( rhs == 0 )
		return -INFTY;
	else
		return rhs->getStepLength( );
}



returnValue Integrator::deleteAllSeeds(){

    nBDirs  = 0;
    nFDirs  = 0;
    nBDirs2 = 0;
    nFDirs2 = 0;

    return SUCCESSFUL_RETURN;
}




//
// PROTECTED MEMBER FUNCTIONS:
//

returnValue Integrator::evaluateTransition( const double time, DVector &xd, const DVector &xa,
                                            const DVector &p, const DVector &u, const DVector &w ){

    ASSERT( transition != 0 );
    EvaluationPoint z( *transition, xd.getDim(), xa.getDim(), p.getDim(), u.getDim(), w.getDim() );
    z.setT ( time );
    z.setX ( xd   );
    z.setXA( xa   );
    z.setP ( p    );
    z.setU ( u    );
    z.setW ( w    );
    xd = transition->evaluate( z );
    return SUCCESSFUL_RETURN;
}


returnValue Integrator::diffTransitionForward(       DVector &DX,
                                               const DVector &DP,
                                               const DVector &DU,
                                               const DVector &DW,
                                               const int    &order ){

    ASSERT( transition != 0 );
    EvaluationPoint z( *transition, DX.getDim(), 0, DP.getDim(), DU.getDim(), DW.getDim() );
    z.setX ( DX );
    z.setP ( DP );
    z.setU ( DU );
    z.setW ( DW );

    if( order == 1 ) DX = transition->AD_forward( z );

    if( order != 1 ) return ACADOERROR( RET_NOT_IMPLEMENTED_YET );

    return SUCCESSFUL_RETURN;
}


returnValue Integrator::diffTransitionBackward( DVector &DX,
                                                DVector &DP,
                                                DVector &DU,
                                                DVector &DW,
                                                int    &order ){

    ASSERT( transition != 0 );
    if( order != 1 ) return ACADOERROR( RET_NOT_IMPLEMENTED_YET );

    EvaluationPoint z( *transition, DX.getDim(), 0, DP.getDim(), DU.getDim(), DW.getDim() );

    transition->AD_backward( DX, z );

    DX = z.getX();
    DP = z.getP();
    DU = z.getU();
    DW = z.getW();

    return SUCCESSFUL_RETURN;
}




returnValue Integrator::setupOptions( )
{
	addOption( MAX_NUM_INTEGRATOR_STEPS    , defaultMaxNumSteps             );
	addOption( INTEGRATOR_TOLERANCE        , defaultIntegratorTolerance     );
	addOption( ABSOLUTE_TOLERANCE          , defaultAbsoluteTolerance       );
	addOption( INITIAL_INTEGRATOR_STEPSIZE , defaultInitialStepsize         );
	addOption( MIN_INTEGRATOR_STEPSIZE     , defaultMinStepsize             );
	addOption( MAX_INTEGRATOR_STEPSIZE     , defaultMaxStepsize             );
	addOption( STEPSIZE_TUNING             , defaultStepsizeTuning          );
	addOption( CORRECTOR_TOLERANCE         , defaultCorrectorTolerance      );
	addOption( INTEGRATOR_PRINTLEVEL       , defaultIntegratorPrintlevel    );
	addOption( LINEAR_ALGEBRA_SOLVER       , defaultLinearAlgebraSolver     );
	addOption( ALGEBRAIC_RELAXATION        , defaultAlgebraicRelaxation     );
	addOption( RELAXATION_PARAMETER        , defaultRelaxationParameter     );
	addOption( PRINT_INTEGRATOR_PROFILE    , defaultprintIntegratorProfile  );
	
	return SUCCESSFUL_RETURN;
}



void Integrator::initializeOptions(){

    get( MAX_NUM_INTEGRATOR_STEPS         , maxNumberOfSteps  );
    get( INTEGRATOR_TOLERANCE  , TOL               );
    get( INITIAL_INTEGRATOR_STEPSIZE      , hini              );
    get( MIN_INTEGRATOR_STEPSIZE          , hmin              );
    get( MAX_INTEGRATOR_STEPSIZE          , hmax              );
    get( STEPSIZE_TUNING       , tune              );
    get( INTEGRATOR_PRINTLEVEL , PrintLevel        );
    get( LINEAR_ALGEBRA_SOLVER , las               );
}

returnValue Integrator::setupLogging( ){

    LogRecord tmp(LOG_AT_EACH_ITERATION, PS_DEFAULT);

    tmp.addItem( LOG_TIME_INTEGRATOR,                              "INTEGRATION TIME                 [sec]:  ");
    tmp.addItem( LOG_NUMBER_OF_INTEGRATOR_STEPS,                   "NUMBER OF STEPS                       :  ");
    tmp.addItem( LOG_NUMBER_OF_INTEGRATOR_REJECTED_STEPS,          "NUMBER OF REJECTED STEPS              :  ");
    tmp.addItem( LOG_NUMBER_OF_INTEGRATOR_FUNCTION_EVALUATIONS,    "NUMBER OF RHS EVALUATIONS             :  ");
    tmp.addItem( LOG_NUMBER_OF_BDF_INTEGRATOR_JACOBIAN_EVALUATIONS,"NUMBER OF JACOBIAN EVALUATIONS        :  ");
    tmp.addItem( LOG_TIME_INTEGRATOR_FUNCTION_EVALUATIONS,         "TIME FOR RHS EVALUATIONS         [sec]:  ");
    tmp.addItem( LOG_TIME_BDF_INTEGRATOR_JACOBIAN_EVALUATION,      "TIME FOR JACOBIAN EVALUATIONS    [sec]:  ");
    tmp.addItem( LOG_TIME_BDF_INTEGRATOR_JACOBIAN_DECOMPOSITION,   "TIME FOR JACOBIAN DECOMPOSITIONS [sec]:  ");

    outputLoggingIdx = addLogRecord( tmp );

    return SUCCESSFUL_RETURN;
}


int Integrator::getDimX() const{

    return getDim();
}


returnValue Integrator::printRunTimeProfile() const{

	return printLogRecord(cout, outputLoggingIdx, PRINT_LAST_ITER);
}


CLOSE_NAMESPACE_ACADO

// end of file.
