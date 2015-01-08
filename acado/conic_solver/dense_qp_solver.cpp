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
 *    \file src/conic_solver/dense_qp_solver.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date   2010
 */


#include <acado/conic_solver/dense_qp_solver.hpp>


BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

DenseQPsolver::DenseQPsolver( ) : DenseCPsolver( )
{
	setupLogging( );
	
    qpStatus = QPS_NOT_INITIALIZED;
    numberOfSteps = 0;
}


DenseQPsolver::DenseQPsolver( UserInteraction* _userInteraction ) : DenseCPsolver( _userInteraction )
{
	setupLogging( );
	
    qpStatus = QPS_NOT_INITIALIZED;
    numberOfSteps = 0;
}


DenseQPsolver::DenseQPsolver( const DenseQPsolver& rhs ) : DenseCPsolver( rhs )
{
    qpStatus = rhs.qpStatus;
    numberOfSteps = rhs.numberOfSteps;
}


DenseQPsolver::~DenseQPsolver( ){

}


DenseQPsolver& DenseQPsolver::operator=( const DenseQPsolver& rhs ){

    if ( this != &rhs ){

        DenseCPsolver::operator=( rhs );

        qpStatus = rhs.qpStatus;
    }
    return *this;
}


returnValue DenseQPsolver::init( uint nV, uint nC )
{
	//printf( "nV: %d,   nC: %d!!\n",nV,nC );
	return setupQPobject( nV,nC );
}


returnValue DenseQPsolver::init( const DenseCP *cp )
{
    ASSERT( cp != 0 );

    if( cp->isQP() == BT_FALSE )
        return ACADOERROR( RET_QP_SOLVER_CAN_ONLY_SOLVE_QP );

    return init( cp->getNV(),cp->getNC() );
}


returnValue DenseQPsolver::solve( DenseCP *cp )
{
    ASSERT( cp != 0 );

    if( cp->isQP() == BT_FALSE )
        return ACADOERROR( RET_QP_SOLVER_CAN_ONLY_SOLVE_QP );

	if ( makeBoundsConsistent( cp ) != SUCCESSFUL_RETURN )
		return ACADOERROR( RET_QP_HAS_INCONSISTENT_BOUNDS );

	int maxNumQPiterations;
	get( MAX_NUM_QP_ITERATIONS, maxNumQPiterations );

    returnValue returnvalue;
    returnvalue = solve( &cp->H, &cp->A, &cp->g, &cp->lb, &cp->ub, &cp->lbA, &cp->ubA, maxNumQPiterations );

	if ( ( returnvalue != SUCCESSFUL_RETURN ) && ( returnvalue != RET_QP_SOLUTION_REACHED_LIMIT ) )
		return returnvalue;


    // GET THE PRIMAL AND DUAL SOLUTION FROM THE QP SOLVER AND
    // STORE THEM IN THE RIGHT FORMAT:
    // -------------------------------------------------------
    DVector xOpt, yOpt;

    getPrimalSolution( xOpt );
    getDualSolution  ( yOpt );

// 	xOpt.print("xOpt");
// 	yOpt.print("yOpt");

    cp->setQPsolution( xOpt,yOpt );
	
    return returnvalue;
}


uint DenseQPsolver::getNumberOfIterations( ) const
{
    return numberOfSteps;
}




//
// PROTECTED MEMBER FUNCTIONS:
//


returnValue DenseQPsolver::setupLogging( )
{
	LogRecord tmp(LOG_AT_EACH_ITERATION, PS_DEFAULT);

	tmp.addItem( LOG_NUM_QP_ITERATIONS );
	tmp.addItem( LOG_IS_QP_RELAXED );

	addLogRecord( tmp );
  
	return SUCCESSFUL_RETURN;
}



returnValue DenseQPsolver::makeBoundsConsistent(	DenseCP *cp
													) const
{
	uint i;
	double mean;

	if ( cp->lb.getDim( ) != cp->ub.getDim( ) )
		return ACADOERROR( RET_INVALID_ARGUMENTS );

	for( i=0; i<cp->lb.getDim( ); ++i )
	{
		if ( cp->lb(i) > cp->ub(i) )
		{
			if ( cp->lb(i) > cp->ub(i)+SQRT_EPS )
				return RET_QP_HAS_INCONSISTENT_BOUNDS;
			else
			{
				mean = ( cp->lb(i) + cp->ub(i) ) / 2.0;
				cp->lb(i) = mean;
				cp->ub(i) = mean;
			}
		}
	}

	if ( cp->lbA.getDim( ) != cp->ubA.getDim( ) )
		return ACADOERROR( RET_INVALID_ARGUMENTS );

	for( i=0; i<cp->lbA.getDim( ); ++i )
	{
		if ( cp->lbA(i) > cp->ubA(i) )
		{
			if ( cp->lbA(i) > cp->ubA(i)+SQRT_EPS )
				return RET_QP_HAS_INCONSISTENT_BOUNDS;
			else
			{
				mean = ( cp->lbA(i) + cp->ubA(i) ) / 2.0;
				cp->lbA(i) = mean;
				cp->ubA(i) = mean;
			}
		}
	}
	
	return SUCCESSFUL_RETURN;
}



CLOSE_NAMESPACE_ACADO

// end of file.
