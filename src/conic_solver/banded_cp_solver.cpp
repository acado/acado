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
 *    \file src/conic_solver/banded_cp_solver.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *
 */

#include <acado/conic_solver/banded_cp_solver.hpp>


BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

BandedCPsolver::BandedCPsolver( ) : AlgorithmicBase( )
{
	setupOptions( );
	setupLogging( );
}


BandedCPsolver::BandedCPsolver( UserInteraction* _userInteraction ) : AlgorithmicBase( _userInteraction )
{
	// setup options and loggings for stand-alone instances
	if ( _userInteraction == 0 )
	{
		setupOptions( );
		setupLogging( );
	}
}


BandedCPsolver::BandedCPsolver( const BandedCPsolver& rhs ) : AlgorithmicBase( rhs ){

}


BandedCPsolver::~BandedCPsolver( ){

}


BandedCPsolver& BandedCPsolver::operator=( const BandedCPsolver& rhs ){

    if ( this != &rhs ){

        AlgorithmicBase::operator=(rhs);

    }
    return *this;
}



returnValue BandedCPsolver::prepareSolve(	BandedCP& cp
											)
{
	return SUCCESSFUL_RETURN;
}


returnValue BandedCPsolver::finalizeSolve(	BandedCP& cp
											)
{
	return SUCCESSFUL_RETURN;
}



returnValue BandedCPsolver::getVarianceCovariance( DMatrix &var )
{
	return ACADOERROR( RET_NOT_IMPLEMENTED_YET );
}



returnValue BandedCPsolver::setRealTimeParameters(	const DVector& DeltaX,
													const DVector& DeltaP
													)
{
	return ACADOERROR( RET_NOT_IMPLEMENTED_IN_BASE_CLASS );
}



returnValue BandedCPsolver::freezeCondensing( )
{
	return ACADOERROR( RET_NOT_IMPLEMENTED_IN_BASE_CLASS );
}


returnValue BandedCPsolver::unfreezeCondensing( )
{
	return ACADOERROR( RET_NOT_IMPLEMENTED_IN_BASE_CLASS );
}



//
// PROTECTED MEMBER FUNCTIONS:
//

returnValue BandedCPsolver::setupOptions( )
{
	addOption( LEVENBERG_MARQUARDT         , defaultLevenbergMarguardt       );
	addOption( HESSIAN_PROJECTION_FACTOR   , defaultHessianProjectionFactor  );
	addOption( INFEASIBLE_QP_RELAXATION    , defaultInfeasibleQPrelaxation   );
	addOption( INFEASIBLE_QP_HANDLING      , defaultInfeasibleQPhandling     );
	addOption( MAX_NUM_QP_ITERATIONS       , defaultMaxNumQPiterations       );

	return SUCCESSFUL_RETURN;
}


returnValue BandedCPsolver::setupLogging( )
{
    //LogRecord tmp( LOG_AT_END );

    //tmp.addItem( LOG_DIFFERENTIAL_STATES      );

    //outputLoggingIdx = addLogRecord( tmp );

	return SUCCESSFUL_RETURN;
}


CLOSE_NAMESPACE_ACADO

// end of file.
