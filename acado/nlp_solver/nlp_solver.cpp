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
 *    \file src/nlp_solver/nlp_solver.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date   2009
 */


#include <acado/nlp_solver/nlp_solver.hpp>



BEGIN_NAMESPACE_ACADO



//
// PUBLIC MEMBER FUNCTIONS:
//

NLPsolver::NLPsolver( UserInteraction* _userInteraction ) : AlgorithmicBase( _userInteraction )
{
	// setup options and loggings for stand-alone instances
	if ( _userInteraction == 0 )
	{
		setupOptions( );
		setupLogging( );
	}
	
    numberOfSteps = 0;
}


NLPsolver::NLPsolver( const NLPsolver& rhs ) : AlgorithmicBase( rhs )
{
    numberOfSteps = rhs.numberOfSteps;
}


NLPsolver::~NLPsolver( ){

}


NLPsolver& NLPsolver::operator=( const NLPsolver& rhs )
{
    if ( this != &rhs )
    {
		AlgorithmicBase::operator=( rhs );

		numberOfSteps = rhs.numberOfSteps;
    }

    return *this;
}

returnValue NLPsolver::solve(	const DVector &x0_, const DVector &p_
								){
    return ACADOERROR(RET_SOLVER_NOT_SUTIABLE_FOR_REAL_TIME_MODE);
}



returnValue NLPsolver::feedbackStep( const DVector &x0_, const DVector &p_ )
{
    return ACADOERROR(RET_SOLVER_NOT_SUTIABLE_FOR_REAL_TIME_MODE);
}


returnValue NLPsolver::performCurrentStep( )
{
    return ACADOERROR(RET_SOLVER_NOT_SUTIABLE_FOR_REAL_TIME_MODE);
}


returnValue NLPsolver::prepareNextStep( )
{
    return ACADOERROR(RET_SOLVER_NOT_SUTIABLE_FOR_REAL_TIME_MODE);
}


returnValue NLPsolver::step( const DVector &x0_, const DVector &p_ )
{
    return ACADOERROR(RET_SOLVER_NOT_SUTIABLE_FOR_REAL_TIME_MODE);
}



returnValue NLPsolver::shiftVariables(	double timeShift,
							    DVector  lastX,
							    DVector  lastXA,
								DVector  lastP,
								DVector  lastU,
								DVector  lastW	)
{
    return ACADOERROR(RET_SOLVER_NOT_SUTIABLE_FOR_REAL_TIME_MODE);
}


returnValue NLPsolver::setReference( const VariablesGrid &ref ){

    return ACADOERROR(RET_SOLVER_NOT_SUTIABLE_FOR_REAL_TIME_MODE);
}


returnValue NLPsolver::getDifferentialStates( VariablesGrid &xd_ ) const{

    return ACADOERROR(RET_NOT_IMPLEMENTED_YET);
}


returnValue NLPsolver::getAlgebraicStates( VariablesGrid &xa_ ) const{

    return ACADOERROR(RET_NOT_IMPLEMENTED_YET);
}


returnValue NLPsolver::getParameters( VariablesGrid &p_  ) const{

    return ACADOERROR(RET_NOT_IMPLEMENTED_YET);
}

returnValue NLPsolver::getParameters( DVector &p_  ) const{

    return ACADOERROR(RET_NOT_IMPLEMENTED_YET);
}


returnValue NLPsolver::getControls( VariablesGrid &u_  ) const{

    return ACADOERROR(RET_NOT_IMPLEMENTED_YET);
}


returnValue NLPsolver::getFirstControl( DVector& u_  ) const{

    return ACADOERROR(RET_NOT_IMPLEMENTED_YET);
}


returnValue NLPsolver::getDisturbances( VariablesGrid &w_  ) const{

    return ACADOERROR(RET_NOT_IMPLEMENTED_YET);
}


double NLPsolver::getObjectiveValue( ) const{

    return ACADOERROR(RET_NOT_IMPLEMENTED_YET);
}


returnValue NLPsolver::getSensitivitiesX(	BlockMatrix& _sens
											) const
{
	return ACADOERROR(RET_NOT_IMPLEMENTED_YET);
}


returnValue NLPsolver::getSensitivitiesXA(	BlockMatrix& _sens
											) const
{
	return ACADOERROR(RET_NOT_IMPLEMENTED_YET);
}

returnValue NLPsolver::getSensitivitiesP(	BlockMatrix& _sens
											) const
{
	return ACADOERROR(RET_NOT_IMPLEMENTED_YET);
}


returnValue NLPsolver::getSensitivitiesU(	BlockMatrix& _sens
											) const
{
	return ACADOERROR(RET_NOT_IMPLEMENTED_YET);
}


returnValue NLPsolver::getSensitivitiesW(	BlockMatrix& _sens
											) const
{
	return ACADOERROR(RET_NOT_IMPLEMENTED_YET);
}



//
// PROTECTED MEMBER FUNCTIONS:
//

returnValue NLPsolver::setupOptions( )
{
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

	return SUCCESSFUL_RETURN;
}


returnValue NLPsolver::setupLogging( )
{
// 	LogRecord tmp( LOG_AT_EACH_ITERATION,stdout,PS_DEFAULT,BT_FALSE );

// 	addLogRecord( tmp );

	return SUCCESSFUL_RETURN;
}


CLOSE_NAMESPACE_ACADO

// end of file.
