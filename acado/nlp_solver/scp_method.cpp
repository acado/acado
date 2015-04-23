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
 *    \file src/nlp_solver/scp_method.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *
 */


#include <acado/nlp_solver/scp_method.hpp>
#include <iomanip>
#include <iostream>


// #define SIM_DEBUG

using namespace std;

BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

SCPmethod::SCPmethod( ) : NLPsolver( )
{
	setupLogging( );

	eval = 0;
	scpStep = 0;
	derivativeApproximation = 0;

	bandedCPsolver = 0;
	
	status = BS_NOT_INITIALIZED;
	isCP = BT_FALSE;
	
	hasPerformedStep = BT_FALSE;
	isInRealTimeMode = BT_FALSE;
	needToReevaluate = BT_FALSE;
}


SCPmethod::SCPmethod(	UserInteraction* _userInteraction,
						const Objective             *objective_          ,
						const DynamicDiscretization *dynamic_discretization_,
						const Constraint            *constraint_,
						BooleanType _isCP
						) : NLPsolver( _userInteraction )
{
	eval = new SCPevaluation( _userInteraction,objective_,dynamic_discretization_,constraint_,_isCP );
	scpStep = 0;
	derivativeApproximation = 0;

	bandedCPsolver = 0;

	status = BS_NOT_INITIALIZED;
	isCP = _isCP;
	
	hasPerformedStep = BT_FALSE;
	isInRealTimeMode = BT_FALSE;
	needToReevaluate = BT_FALSE;

	setupLogging( );
}


SCPmethod::SCPmethod( const SCPmethod& rhs ) : NLPsolver( rhs )
{

	timeLoggingIdx = rhs.timeLoggingIdx;
	clock = rhs.clock;
	clockTotalTime = rhs.clockTotalTime;

    iter    = rhs.iter;
	oldIter = rhs.oldIter;

    if( rhs.eval != 0 ) eval = (rhs.eval)->clone( );
    else                eval = 0;

    if( rhs.scpStep != 0 ) scpStep = (rhs.scpStep)->clone( );
    else                   scpStep = 0;

    if( rhs.derivativeApproximation != 0 ) derivativeApproximation = (rhs.derivativeApproximation)->clone( );
    else                                   derivativeApproximation = 0;

    if( rhs.bandedCPsolver != 0 ) bandedCPsolver = (rhs.bandedCPsolver)->clone( );
    else                          bandedCPsolver = 0;

	status = rhs.status;
	isCP   = rhs.isCP;
	
	hasPerformedStep = rhs.hasPerformedStep;
	isInRealTimeMode = rhs.isInRealTimeMode;
	needToReevaluate = rhs.needToReevaluate;
}


SCPmethod::~SCPmethod( )
{
    if( eval != 0 )
    	delete eval;

	if( scpStep != 0 )
		delete scpStep;

    if( derivativeApproximation != 0 )
		delete derivativeApproximation;

	if( bandedCPsolver != 0 )
		delete bandedCPsolver;
}


SCPmethod& SCPmethod::operator=( const SCPmethod& rhs ){

    if ( this != &rhs )
    {
		if( eval != 0 )
			delete eval;

		if( scpStep != 0 )
			delete scpStep;

		if( derivativeApproximation != 0 )
			delete derivativeApproximation;

		if( bandedCPsolver != 0 )
			delete bandedCPsolver;


        NLPsolver::operator=( rhs );


		timeLoggingIdx = rhs.timeLoggingIdx;
		clock = rhs.clock;
		clockTotalTime = rhs.clockTotalTime;

		iter    = rhs.iter;
		oldIter = rhs.oldIter;

		if( rhs.eval != 0 ) eval = (rhs.eval)->clone( );
		else                eval = 0;

		if( rhs.scpStep != 0 ) scpStep = (rhs.scpStep)->clone( );
		else                   scpStep = 0;

		if( rhs.derivativeApproximation != 0 ) derivativeApproximation = (rhs.derivativeApproximation)->clone( );
		else                                   derivativeApproximation = 0;

        if( rhs.bandedCPsolver != 0 ) bandedCPsolver = (rhs.bandedCPsolver)->clone( );
        else                          bandedCPsolver = 0;

		status = rhs.status;
		isCP   = rhs.isCP;
		
		hasPerformedStep = rhs.hasPerformedStep;
		isInRealTimeMode = rhs.isInRealTimeMode;
		needToReevaluate = rhs.needToReevaluate;
	}

    return *this;
}


NLPsolver* SCPmethod::clone( ) const
{
	return new SCPmethod( *this );
}



returnValue SCPmethod::init(	VariablesGrid* x_init ,
								VariablesGrid* xa_init,
								VariablesGrid* p_init ,
								VariablesGrid* u_init ,
								VariablesGrid* w_init   )
{
    int printC;
    get( PRINT_COPYRIGHT, printC );

    // PRINT THE HEADER:
    // -----------------
    if( printC == BT_TRUE )
        acadoPrintCopyrightNotice( "SCPmethod -- A Sequential Quadratic Programming Algorithm." );

	iter.init( x_init, xa_init, p_init, u_init, w_init );

// 	iter.print(); // already here different!!

	if ( setup( ) != SUCCESSFUL_RETURN )
		return ACADOERROR( RET_NLP_INIT_FAILED );


	// COMPUTATION OF DERIVATIVES:
	// ---------------------------
	int printLevel;
	get( PRINTLEVEL,printLevel );
	
	if ( (PrintLevel)printLevel >= HIGH ) 
		cout << "--> Computing initial linearization of NLP system ...\n";

// 	iter.print();
	
    ACADO_TRY( eval->evaluateSensitivities( iter,bandedCP ) ).changeType( RET_NLP_INIT_FAILED );

// 	iter.print();
	
	if ( (PrintLevel)printLevel >= HIGH ) 
		cout << "<-- Computing initial linearization of NLP system done.\n";

	int useRealtimeIterations;
	get( USE_REALTIME_ITERATIONS,useRealtimeIterations );

	if ( (BooleanType)useRealtimeIterations == BT_TRUE )
	{
		if ( bandedCPsolver->prepareSolve( bandedCP ) != SUCCESSFUL_RETURN )
			return ACADOERROR( RET_NLP_STEP_FAILED );
	}


	// freeze condensing in case OCP is QP -- isCP is a bit misleading...
	if ( ( isCP == BT_TRUE ) && ( eval->hasLSQobjective( ) == BT_TRUE ) )
	{
 		// bandedCPsolver->freezeCondensing( );
 		// eval->freezeSensitivities( );
	}

	status = BS_READY;

    return SUCCESSFUL_RETURN;
}


returnValue SCPmethod::solve(	const DVector &x0_,
								const DVector &p_
								)
{
	if ( ( status != BS_READY ) && ( status != BS_RUNNING ) )
		return ACADOERROR( RET_INITIALIZE_FIRST );

	returnValue returnvalue = SUCCESSFUL_RETURN;
	numberOfSteps = 0;

	int maxNumberOfSteps;
	get( MAX_NUM_ITERATIONS, maxNumberOfSteps );

	while( numberOfSteps < maxNumberOfSteps )
	{
		returnvalue = step( x0_,p_ );
		// also increases numberOfSteps by one

		if( returnvalue == CONVERGENCE_ACHIEVED )
			break;

		if( returnvalue != CONVERGENCE_NOT_YET_ACHIEVED )
			return ACADOERROR( RET_NLP_SOLUTION_FAILED );
	}

	replot( PLOT_AT_END );

    if( numberOfSteps == maxNumberOfSteps )
        return RET_MAX_NUMBER_OF_STEPS_EXCEEDED;

    return returnvalue;
}



returnValue SCPmethod::step(	const DVector& x0_,
								const DVector& p_
								)
{
	if ( numberOfSteps == 0 )
		replot( PLOT_AT_START );

	if ( feedbackStep( x0_,p_ ) != SUCCESSFUL_RETURN )
		return RET_NLP_STEP_FAILED;

	if ( performCurrentStep( ) == CONVERGENCE_ACHIEVED )
		return CONVERGENCE_ACHIEVED;
	
	returnValue returnValue = prepareNextStep( );

	if ( ( returnValue != CONVERGENCE_ACHIEVED ) && ( returnValue != CONVERGENCE_NOT_YET_ACHIEVED ) )
		return RET_NLP_STEP_FAILED;
	
	return returnValue;
}


returnValue SCPmethod::feedbackStep(	const DVector& x0_,
										const DVector& p_
										)
{
  
  #ifdef SIM_DEBUG
  printf("START OF THE FEEDBACK STEP \n");
  
  x0_.print("x0");
  #endif
  
  
	returnValue returnvalue;

	if ( ( status != BS_READY ) && ( status != BS_RUNNING ) )
		return ACADOERROR( RET_INITIALIZE_FIRST );

	clockTotalTime.reset( );
	clockTotalTime.start( );
	
	status = BS_RUNNING;
	hasPerformedStep = BT_FALSE;

	if ( checkForRealTimeMode( x0_,p_ ) != SUCCESSFUL_RETURN )
		return ACADOERROR( RET_NLP_STEP_FAILED );

    int hessianMode;
    get( HESSIAN_APPROXIMATION,hessianMode );

	if ( ( isInRealTimeMode == BT_FALSE ) && ( (HessianApproximationMode)hessianMode == EXACT_HESSIAN ) )
	{
		returnvalue = initializeHessianProjection();
		if( returnvalue != SUCCESSFUL_RETURN )
			return ACADOERROR( returnvalue );
	}

	//bandedCP.objectiveGradient.print();
	//x0_.print("x0");
	
    if ( isInRealTimeMode == BT_TRUE )
	{
		if ( setupRealTimeParameters( x0_,p_ ) != SUCCESSFUL_RETURN )
			return ACADOERROR( RET_NLP_STEP_FAILED );
	}

	int printLevel;
	get( PRINTLEVEL,printLevel );

	if ( (PrintLevel)printLevel >= HIGH ) 
		cout << "--> Solving banded QP ...\n";

// 	iter.print();

	if ( bandedCPsolver->solve( bandedCP ) != SUCCESSFUL_RETURN )
		return ACADOERROR( RET_NLP_STEP_FAILED );

// 	bandedCP.deltaX.print();
	
	if ( (PrintLevel)printLevel >= HIGH ) 
		cout << "<-- Solving banded QP done.\n";

	++numberOfSteps;

	return SUCCESSFUL_RETURN;
}


returnValue SCPmethod::performCurrentStep( )
{
	returnValue returnvalue;

	if ( isInRealTimeMode == BT_TRUE )
	{
		if ( bandedCPsolver->finalizeSolve( bandedCP ) != SUCCESSFUL_RETURN )
			return ACADOERROR( RET_NLP_STEP_FAILED );
		
// 		bandedCP.deltaX.print();
    }

//     cout <<"bandedCP.dynResiduum = \n");
//     bandedCP.dynResiduum.print();
//     cout <<"bandedCP.lambdaDynamic = \n");
//     bandedCP.lambdaDynamic.print();

	oldIter = iter;

    // Perform a globalized step:
    // --------------------------
	int printLevel;
	get( PRINTLEVEL,printLevel );

	if ( (PrintLevel)printLevel >= HIGH ) 
		cout << "--> Perform globalized SQP step ...\n";
	
	clock.reset( );
	clock.start( );

	#ifdef SIM_DEBUG
/*	printf("performing the current step...: old iterate \n");
	(iter.x->getVector(0)).print("iter.x(0)");
	(iter.u->getVector(0)).print("iter.u(0)");
	(iter.x->getVector(1)).print("iter.x(1)");
	(iter.u->getVector(1)).print("iter.u(1)");*/
	#endif

	returnvalue = scpStep->performStep( iter,bandedCP,eval );
	if( returnvalue != SUCCESSFUL_RETURN )
		ACADOERROR( RET_NLP_STEP_FAILED );

	hasPerformedStep = BT_TRUE;

	clock.stop( );
	setLast( LOG_TIME_GLOBALIZATION,clock.getTime() );

	if ( (PrintLevel)printLevel >= HIGH ) 
		cout << "<-- Perform globalized SQP step done.\n";

	printIteration( );

	// Check convergence criterion if no real-time iterations are performed
	int terminateAtConvergence = 0;
	get( TERMINATE_AT_CONVERGENCE,terminateAtConvergence );

	if ( (BooleanType)terminateAtConvergence == BT_TRUE )
	{
		if ( checkForConvergence( ) == CONVERGENCE_ACHIEVED )
		{
			stopClockAndPrintRuntimeProfile( );
			return CONVERGENCE_ACHIEVED;
		}
	}

	if ( numberOfSteps >= 0 )
		set( KKT_TOLERANCE_SAFEGUARD,0.0 );
	
	return CONVERGENCE_NOT_YET_ACHIEVED;
}


returnValue SCPmethod::prepareNextStep( )
{
    returnValue returnvalue;
    RealClock clockLG;

	BlockMatrix oldLagrangeGradient;
	BlockMatrix newLagrangeGradient;

// 	cout <<"bandedCP.dynResiduum (possibly shifted) = \n");
//     bandedCP.dynResiduum.print();
// 	cout <<"bandedCP.lambdaDynamic (possibly shifted) = \n");
//     bandedCP.lambdaDynamic.print();

    // Coumpute the "old" Lagrange Gradient with the latest multipliers:
    // -----------------------------------------------------------------
	clockLG.reset( );
	clockLG.start( );

	returnvalue = eval->evaluateLagrangeGradient( getNumPoints(),oldIter,bandedCP, oldLagrangeGradient );
	if( returnvalue != SUCCESSFUL_RETURN )
		ACADOERROR( RET_NLP_STEP_FAILED );

	clockLG.stop( );
	
	
    // Linearize the NLP system at the new point:
    // ------------------------------------------
	int printLevel;
	get( PRINTLEVEL,printLevel );


	#ifdef SIM_DEBUG
/*	printf("preparation step \n");
	(iter.x->getVector(0)).print("iter.x(0)");
	(iter.u->getVector(0)).print("iter.u(0)");
	(iter.x->getVector(1)).print("iter.x(1)");
	(iter.u->getVector(1)).print("iter.u(1)");*/
	#endif
	
	if ( (PrintLevel)printLevel >= HIGH ) 
		cout << "--> Computing new linearization of NLP system ...\n";

	clock.reset( );
	clock.start( );

	if ( needToReevaluate == BT_TRUE )
	{
		// needs to re-evaluate due to eval->evaluate call within merit function evaluation!
		eval->clearDynamicDiscretization( );
		if ( eval->evaluate( iter,bandedCP ) != SUCCESSFUL_RETURN )
			return ACADOERROR( RET_NLP_STEP_FAILED );
		needToReevaluate = BT_FALSE;
	}

	returnvalue = eval->evaluateSensitivities( iter,bandedCP );
	if( returnvalue != SUCCESSFUL_RETURN )
		ACADOERROR( RET_NLP_STEP_FAILED );

	clock.stop( );
	setLast( LOG_TIME_SENSITIVITIES,clock.getTime() );

	if ( (PrintLevel)printLevel >= HIGH ) 
		cout << "<-- Computing new linearization of NLP system done.\n";
	//bandedCP.objectiveGradient.print();
	

    // Coumpute the "new" Lagrange Gradient with the latest multipliers:
    // -----------------------------------------------------------------
    clockLG.start( );

	returnvalue = eval->evaluateLagrangeGradient( getNumPoints(),iter,bandedCP, newLagrangeGradient );
	if( returnvalue != SUCCESSFUL_RETURN )
		ACADOERROR( RET_NLP_STEP_FAILED );

	clockLG.stop( );
	setLast( LOG_TIME_LAGRANGE_GRADIENT,clockLG.getTime() );

	
    // Compute the next Hessian:
    // -------------------------
	if ( (PrintLevel)printLevel >= HIGH ) 
		cout << "--> Computing or approximating Hessian matrix ...\n";
	
	clock.reset( );
	clock.start( );

	returnvalue = computeHessianMatrix( oldLagrangeGradient,newLagrangeGradient );
	if( returnvalue != SUCCESSFUL_RETURN )
		ACADOERROR( RET_NLP_STEP_FAILED );

	clock.stop( );
	setLast( LOG_TIME_HESSIAN_COMPUTATION,clock.getTime() );

	if ( (PrintLevel)printLevel >= HIGH ) 
		cout << "<-- Computing or approximating Hessian matrix done.\n";

	// CONDENSE THE KKT-SYSTEM:
    // ------------------------
    if ( isInRealTimeMode == BT_TRUE )
	{
		if ( bandedCPsolver->prepareSolve( bandedCP ) != SUCCESSFUL_RETURN )
			return ACADOERROR( RET_NLP_STEP_FAILED );
	}

	stopClockAndPrintRuntimeProfile( );

	return CONVERGENCE_NOT_YET_ACHIEVED;
}



returnValue SCPmethod::setReference( const VariablesGrid &ref )
{
	needToReevaluate = BT_TRUE;
// 	printf("new reference!\n");
    return eval->setReference( ref );
}


// returnValue SCPmethod::enableNeedToReevaluate( )
// {
// 	needToReevaluate = BT_TRUE;
// 	return SUCCESSFUL_RETURN;
// }


returnValue SCPmethod::shiftVariables(	double timeShift,
										DVector  lastX,
										DVector  lastXA,
										DVector  lastP,
										DVector  lastU,
										DVector  lastW
										)
{
	#ifdef SIM_DEBUG
	cout << "SCPmethod::shiftVariables\n" );
	#endif
	
	if ( acadoIsNegative( timeShift ) == BT_TRUE )
		return ACADOERROR( RET_INVALID_ARGUMENTS );

// 	DMatrix tmp;
// 	for( uint i=1; i<iter.getNumPoints()-1; ++i )
// 	{
// 		bandedCP.lambdaDynamic.getSubBlock( i,0, tmp );
// 		bandedCP.lambdaDynamic.setDense( i-1,0, tmp );
// 	}

// 	printf("shifted!\n");
	needToReevaluate = BT_TRUE;
	return iter.shift( timeShift, lastX, lastXA, lastP, lastU, lastW );
}



returnValue SCPmethod::getVarianceCovariance( DMatrix &var )
{
	if( eval->hasLSQobjective( ) == BT_FALSE )
		return ACADOERROR( RET_NOT_YET_IMPLEMENTED );

	if( bandedCPsolver == 0 )
		return ACADOERROR( RET_MEMBER_NOT_INITIALISED );

	return bandedCPsolver->getVarianceCovariance( var );
}


returnValue SCPmethod::printRuntimeProfile() const
{
	return printLogRecord(cout, timeLoggingIdx, PRINT_LAST_ITER);
}


//
// PROTECTED MEMBER FUNCTIONS:
//


returnValue SCPmethod::setupLogging( )
{
	LogRecord tmp(LOG_AT_EACH_ITERATION, PS_PLAIN);

	tmp.addItem( LOG_TIME_SQP_ITERATION,         "TIME FOR THE WHOLE SQP ITERATION     [sec]" );
	tmp.addItem( LOG_TIME_CONDENSING,            "TIME FOR CONDENSING                  [sec]" );
	tmp.addItem( LOG_TIME_QP,                    "TIME FOR SOLVING THE QP              [sec]" );
// 	tmp.addItem( LOG_TIME_RELAXED_QP,            "TIME FOR SOLVING RELAXED QP's        [sec]" );
// 	tmp.addItem( LOG_TIME_EXPAND,                "TIME FOR EXPANSION                   [sec]" );
// 	tmp.addItem( LOG_TIME_EVALUATION,            "TIME FOR FUNCTION EVALUATIONS        [sec]" );
	tmp.addItem( LOG_TIME_GLOBALIZATION,         "TIME FOR GLOBALIZATION               [sec]" );
	tmp.addItem( LOG_TIME_SENSITIVITIES,         "TIME FOR SENSITIVITY GENERATION      [sec]" );
// 	tmp.addItem( LOG_TIME_LAGRANGE_GRADIENT,     "TIME FOR COMPUTING LAGRANGE GRADIENT [sec]" );
// 	tmp.addItem( LOG_TIME_HESSIAN_COMPUTATION,   "TIME FOR HESSIAN EVALUATION          [sec]" );

	timeLoggingIdx = addLogRecord( tmp );

	LogRecord iterationOutput(LOG_AT_EACH_ITERATION, PS_PLAIN);

	iterationOutput.addItem( LOG_NUM_SQP_ITERATIONS,"SQP iteration");
	iterationOutput.addItem( LOG_KKT_TOLERANCE,"KKT tolerance");
	iterationOutput.addItem( LOG_LINESEARCH_STEPLENGTH,"line search parameter");
	iterationOutput.addItem( LOG_OBJECTIVE_VALUE,"objective value");
	iterationOutput.addItem( LOG_MERIT_FUNCTION_VALUE,"merit function value");
	iterationOutput.addItem( LOG_IS_QP_RELAXED,"QP relaxation");
	iterationOutput.addItem( LOG_NUM_QP_ITERATIONS,"No. QP iterations");
	
	outputLoggingIdx = addLogRecord( iterationOutput );

	return SUCCESSFUL_RETURN;
}


returnValue SCPmethod::setup( )
{
	// CONSISTENCY CHECKS:
	// -------------------
	if ( isCP == BT_TRUE )
	{
		int hessianApproximation;
		get( HESSIAN_APPROXIMATION,hessianApproximation );

		// Gauss-Newton is exact for linear-quadratic systems
		if ( (HessianApproximationMode)hessianApproximation == EXACT_HESSIAN )
			set( HESSIAN_APPROXIMATION,GAUSS_NEWTON );
	}


    // PREPARE THE SQP ALGORITHM:
    // --------------------------

    if ( eval == 0 )
    	return ACADOERROR( RET_UNKNOWN_BUG );

	if ( eval->init( iter ) != SUCCESSFUL_RETURN )
		return ACADOERROR( RET_NLP_INIT_FAILED );


    // PREPARE THE DATA FOR THE SQP ALGORITHM:
    // ---------------------------------------

	if ( bandedCPsolver != 0 )
		delete bandedCPsolver;

	int sparseQPsolution;
	get( SPARSE_QP_SOLUTION,sparseQPsolution );
	
	int printLevel;
	get( PRINTLEVEL,printLevel );

	if ( (PrintLevel)printLevel >= HIGH ) 
		cout << "--> Initializing banded QP solver ...\n";

	if ( (SparseQPsolutionMethods)sparseQPsolution == CONDENSING )
	{
    	bandedCP.lambdaConstraint.init( eval->getNumConstraintBlocks(), 1 );
    	bandedCP.lambdaDynamic.init( getNumPoints()-1, 1 );

		bandedCPsolver = new CondensingBasedCPsolver( userInteraction,eval->getNumConstraints(),eval->getConstraintBlockDims() );
		bandedCPsolver->init( iter );
	}
	else
	{
		return ACADOERROR( RET_NOT_YET_IMPLEMENTED );
	}

	if ( (PrintLevel)printLevel >= HIGH ) 
		cout << "<-- Initializing banded QP solver done.\n";

    // INITIALIZE GLOBALIZATION STRATEGY (SCPstep):
    // --------------------------------------------

	if ( scpStep != 0 )
		delete scpStep;

    int globalizationStrategy;
    get( GLOBALIZATION_STRATEGY,globalizationStrategy );

	switch( (GlobalizationStrategy)globalizationStrategy )
	{
		case GS_FULLSTEP:
			scpStep = new SCPstepFullstep( userInteraction );
			break;

		case GS_LINESEARCH:
			scpStep = new SCPstepLinesearch( userInteraction );
			break;

		default:
			return ACADOERROR( RET_UNKNOWN_BUG );
	}


	// EVALUATION OF THE NLP FUNCTIONS:
	// --------------------------------
	if ( (PrintLevel)printLevel >= HIGH ) 
		cout << "--> Initial integration of dynamic system ...\n";
	
	ACADO_TRY( eval->evaluate(iter,bandedCP) ).changeType( RET_NLP_INIT_FAILED );

	if ( (PrintLevel)printLevel >= HIGH ) 
		cout << "<-- Initial integration of dynamic system done.\n";

    // INITIALIZE HESSIAN MATRIX:
    // --------------------------
    int hessianMode;
    get( HESSIAN_APPROXIMATION,hessianMode );

    if( ( (HessianApproximationMode)hessianMode == GAUSS_NEWTON ) || ( (HessianApproximationMode)hessianMode == GAUSS_NEWTON_WITH_BLOCK_BFGS ) )
	{
        if( eval->hasLSQobjective( ) == BT_FALSE ){
//             ACADOWARNING( RET_GAUSS_NEWTON_APPROXIMATION_NOT_SUPPORTED );
//             set( HESSIAN_APPROXIMATION, BLOCK_BFGS_UPDATE );
//             get( HESSIAN_APPROXIMATION, hessianMode );
        }
    }

	if ( derivativeApproximation != 0 )
		delete derivativeApproximation;

	switch( (HessianApproximationMode)hessianMode )
	{
		case EXACT_HESSIAN:
			derivativeApproximation = new ExactHessian( userInteraction );
			break;

		case CONSTANT_HESSIAN:
			derivativeApproximation = new ConstantHessian( userInteraction );
			break;

		case FULL_BFGS_UPDATE:
			derivativeApproximation = new BFGSupdate( userInteraction );
			break;

		case BLOCK_BFGS_UPDATE:
			derivativeApproximation = new BFGSupdate( userInteraction,getNumPoints() );
			break;

		case GAUSS_NEWTON:
			derivativeApproximation = new GaussNewtonApproximation( userInteraction );
			break;

		case GAUSS_NEWTON_WITH_BLOCK_BFGS:
			derivativeApproximation = new GaussNewtonApproximationWithBFGS( userInteraction,getNumPoints() );
			break;

		default:
			return ACADOERROR( RET_UNKNOWN_BUG );
	}

	bandedCP.hessian.init( 5*getNumPoints(), 5*getNumPoints() );

	if ( (PrintLevel)printLevel >= HIGH ) 
		cout << "--> Initializing Hessian computations ...\n";
	
	ACADO_TRY( derivativeApproximation->initHessian( bandedCP.hessian,getNumPoints(),iter ) );

	if ( (PrintLevel)printLevel >= HIGH ) 
		cout << "<-- Initializing Hessian computations done.\n";

	// SWITCH BETWEEN SINGLE- AND MULTIPLE SHOOTING:
	// ---------------------------------------------
	int discretizationMode;
	get( DISCRETIZATION_TYPE, discretizationMode );

	if( (StateDiscretizationType)discretizationMode != SINGLE_SHOOTING ){
		if( iter.x  != 0 ) iter.x ->disableAutoInit();
		if( iter.xa != 0 ) iter.xa->disableAutoInit();
	}

	return SUCCESSFUL_RETURN;
}


returnValue SCPmethod::printIterate( ) const
{
	return iter.print( );
}


returnValue SCPmethod::printIteration( )
{
	double KKTmultiplierRegularisation;
	get( KKT_TOLERANCE_SAFEGUARD,KKTmultiplierRegularisation );
	
	setLast( LOG_NUM_SQP_ITERATIONS, numberOfSteps );
	setLast( LOG_KKT_TOLERANCE, eval->getKKTtolerance( iter,bandedCP,KKTmultiplierRegularisation ) );
	setLast( LOG_OBJECTIVE_VALUE, eval->getObjectiveValue() );

	int printLevel;
	get( PRINTLEVEL,printLevel );

	if ( (PrintLevel)printLevel >= MEDIUM ) 
	{
		if (numberOfSteps == 1 || (numberOfSteps % 10) == 0)
			cout	<< "sqp it | "
					<< "qp its | "
					<< "      kkt tol | "
					<< "      obj val | "
					<< "    merit val | "
					<< "     ls param | "
					<< endl;

		DMatrix foo;

		IoFormatter iof( cout );

		getLast(LOG_NUM_SQP_ITERATIONS, foo);
		cout << setw( 6 ) << right << (int) foo(0, 0) << " | ";
		getLast(LOG_NUM_QP_ITERATIONS, foo);
		cout << setw( 6 ) << right << (int) foo(0, 0) << " | ";
		getLast(LOG_KKT_TOLERANCE, foo);
		cout << setw( 13 ) << setprecision( 6 ) << right << scientific << foo(0, 0) << " | ";
		getLast(LOG_OBJECTIVE_VALUE, foo);
		cout << setw( 13 ) << setprecision( 6 ) << right << scientific << foo(0, 0) << " | ";
		getLast(LOG_MERIT_FUNCTION_VALUE, foo);
		cout << setw( 13 ) << setprecision( 6 ) << right << scientific << foo(0, 0) << " | ";
		getLast(LOG_LINESEARCH_STEPLENGTH, foo);
		cout << setw( 13 ) << setprecision( 6 ) << right << scientific << foo(0, 0) << " | ";
		cout << endl;

		// Restore cout flags
		iof.reset();
	}

	replot( PLOT_AT_EACH_ITERATION );

	return SUCCESSFUL_RETURN;
}


returnValue SCPmethod::checkForConvergence( )
{
	double tol;
	get( KKT_TOLERANCE,tol );

	// NEEDS TO BE CHECKED CARFULLY !!!
	double KKTmultiplierRegularisation;
	get(KKT_TOLERANCE_SAFEGUARD, KKTmultiplierRegularisation);

	if( eval->getKKTtolerance( iter,bandedCP,KKTmultiplierRegularisation ) <= tol )
	{
		int discretizationMode;
		get( DISCRETIZATION_TYPE, discretizationMode );

		if( (StateDiscretizationType)discretizationMode == SINGLE_SHOOTING )
		{
			eval->clearDynamicDiscretization( );
			if ( eval->evaluate( iter,bandedCP ) != SUCCESSFUL_RETURN )
				return ACADOERROR( RET_NLP_STEP_FAILED );
		}

		int printLevel;
		get( PRINTLEVEL,printLevel );

		if ( (PrintLevel)printLevel >= MEDIUM )
		{
			cout	<< endl
					<< "Covergence achieved. Demanded KKT tolerance is "
					<< scientific << tol
					<< "." << endl << endl;
		}
		return CONVERGENCE_ACHIEVED;
	}
	
	return CONVERGENCE_NOT_YET_ACHIEVED;
}


returnValue SCPmethod::computeHessianMatrix(	const BlockMatrix& oldLagrangeGradient,
												const BlockMatrix& newLagrangeGradient
												)
{
	returnValue returnvalue;

	if ( numberOfSteps == 1 )
	{
		returnvalue = derivativeApproximation->initScaling( bandedCP.hessian, bandedCP.deltaX, newLagrangeGradient-oldLagrangeGradient );
		if( returnvalue != SUCCESSFUL_RETURN )
			ACADOERROR( returnvalue );
	}

	returnvalue = derivativeApproximation->apply( bandedCP.hessian, bandedCP.deltaX, newLagrangeGradient-oldLagrangeGradient );
	if( returnvalue != SUCCESSFUL_RETURN )
		ACADOERROR( returnvalue );

	return SUCCESSFUL_RETURN;
}



returnValue SCPmethod::initializeHessianProjection( )
{
    // COMPUTE A HEURISTIC DAMPING FACTOR:
    // -----------------------------------
    double damping = 1.0;
    int run1 = 0;
    while( run1 < numberOfSteps ){
        if( run1 == 5 ) break;
        damping *= 0.01;
        ++run1;
    }

    return bandedCPsolver->set( HESSIAN_PROJECTION_FACTOR, derivativeApproximation->getHessianScaling()*damping );
}



returnValue SCPmethod::checkForRealTimeMode(	const DVector &x0_,
												const DVector &p_
												)
{
	int useRealtimeIterations;
	get( USE_REALTIME_ITERATIONS,useRealtimeIterations );
	isInRealTimeMode = (BooleanType)useRealtimeIterations;

	if ( ( isInRealTimeMode == BT_FALSE ) && 
		 ( ( x0_.isEmpty( ) == BT_FALSE ) || ( p_.isEmpty( ) == BT_FALSE ) ) )
		return ACADOERROR( RET_NEED_TO_ACTIVATE_RTI );

	return SUCCESSFUL_RETURN;
}


returnValue SCPmethod::setupRealTimeParameters(	const DVector &x0_,
												const DVector &p_
												)
{
	DVector deltaX;
	DVector deltaP;

	if( x0_.isEmpty( ) == BT_FALSE )
		deltaX = x0_ - iter.x->getVector(0);

	if( p_ .isEmpty( ) == BT_FALSE )
		deltaP = p_ - iter.p->getVector(0);

	return bandedCPsolver->setRealTimeParameters( deltaX, deltaP );
}


returnValue SCPmethod::stopClockAndPrintRuntimeProfile( )
{
	clockTotalTime.stop( );
	setLast( LOG_TIME_SQP_ITERATION,clockTotalTime.getTime() );
	
	int printProfile;
	get( PRINT_SCP_METHOD_PROFILE, printProfile );

	if( (BooleanType) printProfile == BT_TRUE )
		printRuntimeProfile();
	
	return SUCCESSFUL_RETURN;
}



returnValue SCPmethod::getDifferentialStates( VariablesGrid &xd_ ) const{

    if( iter.x == 0 )
		return ACADOERROR( RET_MEMBER_NOT_INITIALISED );

    xd_ = iter.x[0];
    return SUCCESSFUL_RETURN;
}


returnValue SCPmethod::getAlgebraicStates( VariablesGrid &xa_ ) const{

    if( iter.xa == 0 )
		return ACADOERROR( RET_MEMBER_NOT_INITIALISED );
    xa_ = iter.xa[0];
    return SUCCESSFUL_RETURN;
}


returnValue SCPmethod::getParameters( VariablesGrid &p_  ) const{

    if( iter.p == 0 )
		return ACADOERROR( RET_MEMBER_NOT_INITIALISED );
//     p_.init( iter.p[0].getVector( 0 ),iter.p[0].getTimePoints( ) );
	p_ = iter.p[0];
    return SUCCESSFUL_RETURN;
}


returnValue SCPmethod::getParameters( DVector& p_  ) const{

	if( iter.p == 0 )
		return ACADOERROR( RET_MEMBER_NOT_INITIALISED );

	p_ = iter.p->getVector( 0 );
    return SUCCESSFUL_RETURN;
}


returnValue SCPmethod::getControls( VariablesGrid &u_  ) const{

    if( iter.u == 0 )
		return ACADOERROR( RET_MEMBER_NOT_INITIALISED );

    u_ = iter.u[0];
    return SUCCESSFUL_RETURN;
}


returnValue SCPmethod::getFirstControl( DVector& u0_  ) const
{
	#ifdef SIM_DEBUG
	cout << "SCPmethod::getFirstControl\n";
	#endif
	
    if( iter.u == 0 )
		return ACADOERROR( RET_MEMBER_NOT_INITIALISED );

    u0_ = iter.u->getVector( 0 );
	
	if ( hasPerformedStep == BT_FALSE )
	{
		DVector deltaU0( getNU() );
		bandedCPsolver->getFirstControl( deltaU0 );
		
		u0_ += deltaU0;
	}

    return SUCCESSFUL_RETURN;
}


returnValue SCPmethod::getDisturbances( VariablesGrid &w_  ) const{

    if( iter.w == 0 )
		return ACADOERROR( RET_MEMBER_NOT_INITIALISED );

    w_ = iter.w[0];
    return SUCCESSFUL_RETURN;
}



double SCPmethod::getObjectiveValue( ) const
{
    ASSERT( eval != 0 );
    return eval->getObjectiveValue();
}



returnValue SCPmethod::getSensitivitiesX(	BlockMatrix& _sens
											) const
{
	return getAnySensitivities( _sens,0 );
}


returnValue SCPmethod::getSensitivitiesXA(	BlockMatrix& _sens
											) const
{
	return getAnySensitivities( _sens,1 );
}

returnValue SCPmethod::getSensitivitiesP(	BlockMatrix& _sens
											) const
{
	return getAnySensitivities( _sens,2 );
}


returnValue SCPmethod::getSensitivitiesU(	BlockMatrix& _sens
											) const
{
	return getAnySensitivities( _sens,3 );
}


returnValue SCPmethod::getSensitivitiesW(	BlockMatrix& _sens
											) const
{
	return getAnySensitivities( _sens,4 );
}


returnValue SCPmethod::getAnySensitivities(	BlockMatrix& _sens,
											uint idx
											) const
{
	if ( idx > 4 )
		return ACADOERROR( RET_INVALID_ARGUMENTS );

	uint N = bandedCP.dynGradient.getNumRows();
	DMatrix tmp;
	
	_sens.init( N,1 );
	
	for( uint i=0; i<N; ++i )
	{
		bandedCP.dynGradient.getSubBlock( i,idx,tmp );
		_sens.setDense( i,0,tmp );
	}
	
	return SUCCESSFUL_RETURN;
}


CLOSE_NAMESPACE_ACADO

// end of file.
