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
 *    \file src/optimization_algorithm/optimization_algorithm.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date 2009
 */


#include <acado/optimization_algorithm/optimization_algorithm.hpp>


BEGIN_NAMESPACE_ACADO



//
// PUBLIC MEMBER FUNCTIONS:
//


OptimizationAlgorithm::OptimizationAlgorithm( ) : OptimizationAlgorithmBase( ), UserInteraction( )
{
	setupOptions( );
	setupLogging( );
	
	setStatus( BS_UNDEFINED );
}


OptimizationAlgorithm::OptimizationAlgorithm( const OCP& ocp_ ) : OptimizationAlgorithmBase( ocp_ ), UserInteraction( )
{
	setupOptions( );
	setupLogging( );
}


OptimizationAlgorithm::OptimizationAlgorithm( const OptimizationAlgorithm& arg )
                      : OptimizationAlgorithmBase( arg ), UserInteraction( arg )
{
}


OptimizationAlgorithm::~OptimizationAlgorithm( )
{
}



OptimizationAlgorithm& OptimizationAlgorithm::operator=( const OptimizationAlgorithm& arg )
{
	if( this != &arg )
	{
		OptimizationAlgorithmBase::operator=( arg );
		UserInteraction::operator=( arg );
	}

	return *this;
}


returnValue OptimizationAlgorithm::init( )
{
	if ( ( getStatus( ) == BS_READY ) && ( haveOptionsChanged( ) == BT_FALSE ) )
		return SUCCESSFUL_RETURN;

	returnValue returnvalue = OptimizationAlgorithmBase::init( this );

	setStatus( BS_READY );
	declareOptionsUnchanged( );

	return returnvalue;
}


returnValue OptimizationAlgorithm::solve( ){

    returnValue returnvalue = SUCCESSFUL_RETURN;

	if ( ( getStatus( ) != BS_READY ) || ( haveOptionsChanged( ) == BT_TRUE ) )
    	returnvalue = init( );

    if( returnvalue != SUCCESSFUL_RETURN ) return returnvalue;
    
    returnvalue = nlpSolver->solve( );

    if( returnvalue != SUCCESSFUL_RETURN &&
        returnvalue != CONVERGENCE_ACHIEVED )
	{
		if ( returnvalue == RET_MAX_NUMBER_OF_STEPS_EXCEEDED)
    {
      int PrintLevel; 
      get( PRINTLEVEL, PrintLevel ); 
      if (PrintLevel != NONE)
      {
  			return ACADOERROR( RET_MAX_NUMBER_OF_STEPS_EXCEEDED );
      }
    }
		else
    {
			return ACADOERROR( RET_OPTALG_SOLVE_FAILED );
    }
	}
    return SUCCESSFUL_RETURN;
}




//
// PROTECTED MEMBER FUNCTIONS:
//

returnValue OptimizationAlgorithm::setupOptions( )
{
	// add NLP solver options
	addOption( MAX_NUM_ITERATIONS          , defaultMaxNumIterations        );
	addOption( KKT_TOLERANCE               , defaultKKTtolerance            );
	addOption( KKT_TOLERANCE_SAFEGUARD     , defaultKKTtoleranceSafeguard   );
	addOption( LEVENBERG_MARQUARDT         , defaultLevenbergMarguardt      );
	addOption( HESSIAN_PROJECTION_FACTOR   , defaultHessianProjectionFactor );
	addOption( PRINTLEVEL                  , defaultPrintlevel              );
	addOption( PRINT_COPYRIGHT             , defaultPrintCopyright          );
	addOption( HESSIAN_APPROXIMATION       , defaultHessianApproximation    );
	addOption( DYNAMIC_HESSIAN_APPROXIMATION, defaultDynamicHessianApproximation );
	addOption( DYNAMIC_SENSITIVITY         , defaultDynamicSensitivity      );
	addOption( OBJECTIVE_SENSITIVITY       , defaultObjectiveSensitivity    );
	addOption( CONSTRAINT_SENSITIVITY      , defaultConstraintSensitivity   );
	addOption( DISCRETIZATION_TYPE         , defaultDiscretizationType      );
	addOption( LINESEARCH_TOLERANCE        , defaultLinesearchTolerance     );
	addOption( MIN_LINESEARCH_PARAMETER    , defaultMinLinesearchParameter  );
	addOption( MAX_NUM_QP_ITERATIONS       , defaultMaxNumQPiterations      );
	addOption( HOTSTART_QP                 , defaultHotstartQP              );
	addOption( INFEASIBLE_QP_RELAXATION    , defaultInfeasibleQPrelaxation  );
	addOption( INFEASIBLE_QP_HANDLING      , defaultInfeasibleQPhandling    );
	addOption( USE_REALTIME_ITERATIONS     , defaultUseRealtimeIterations   );
	addOption( TERMINATE_AT_CONVERGENCE    , defaultTerminateAtConvergence  );
	addOption( SPARSE_QP_SOLUTION          , defaultSparseQPsolution        );
	addOption( GLOBALIZATION_STRATEGY      , defaultGlobalizationStrategy   );
	addOption( PRINT_SCP_METHOD_PROFILE    , defaultprintSCPmethodProfile   );

	// add integration options
	addOption( FREEZE_INTEGRATOR           , defaultFreezeIntegrator        );
	addOption( INTEGRATOR_TYPE             , defaultIntegratorType          );
	addOption( FEASIBILITY_CHECK           , defaultFeasibilityCheck        );
	addOption( PLOT_RESOLUTION             , defaultPlotResoltion           );
	
	// add integrator options
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


returnValue OptimizationAlgorithm::setupLogging( )
{
// 	LogRecord tmp( LOG_AT_EACH_ITERATION,stdout,PS_DEFAULT,BT_FALSE );
// 
// 	tmp.addItem( LOG_NUM_NLP_ITERATIONS );
// 
// 	addLogRecord( tmp );
  
	return SUCCESSFUL_RETURN;
}


returnValue OptimizationAlgorithm::allocateNlpSolver( Objective *F, DynamicDiscretization *G, Constraint *H )
{
	if( nlpSolver != 0 )
		delete nlpSolver;

	nlpSolver = new SCPmethod( this, F,G,H, isLinearQuadratic( F,G,H ) );

	return SUCCESSFUL_RETURN;
}


returnValue OptimizationAlgorithm::initializeNlpSolver( const OCPiterate& _userInit )
{
	ACADO_TRY( nlpSolver->init( _userInit.x, _userInit.xa, _userInit.p, _userInit.u, _userInit.w ) );
	return SUCCESSFUL_RETURN;
}


returnValue OptimizationAlgorithm::initializeObjective(	Objective* F
														)
{
	return SUCCESSFUL_RETURN;
}




CLOSE_NAMESPACE_ACADO


// end of file.
