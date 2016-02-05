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
 *    \file external_packages/src/acado_qpoases/qp_solver_qpoases.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 19.08.2008
 */


#include <acado/bindings/acado_qpoases/qp_solver_qpoases.hpp>
#include <qpOASES-3.2.0/include/qpOASES.hpp>

BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

QPsolver_qpOASES::QPsolver_qpOASES( ) : DenseQPsolver( )
{
	qp = 0;
}


QPsolver_qpOASES::QPsolver_qpOASES( UserInteraction* _userInteraction ) : DenseQPsolver( _userInteraction )
{
	qp = 0;
}


QPsolver_qpOASES::QPsolver_qpOASES( const QPsolver_qpOASES& rhs ) : DenseQPsolver( rhs )
{
	if ( rhs.qp != 0 )
		qp = new qpOASES::SQProblem( *(rhs.qp) );
	else
		qp = 0;
}


QPsolver_qpOASES::~QPsolver_qpOASES( )
{
	if ( qp != 0 )
		delete qp;
}


QPsolver_qpOASES& QPsolver_qpOASES::operator=( const QPsolver_qpOASES& rhs )
{
    if ( this != &rhs )
    {
		DenseQPsolver::operator=( rhs );

		if ( qp != 0 )
			delete qp;


		if ( rhs.qp != 0 )
			qp = new qpOASES::SQProblem( *(rhs.qp) );
		else
			qp = 0;

    }

    return *this;
}


DenseCPsolver* QPsolver_qpOASES::clone( ) const
{
	return new QPsolver_qpOASES(*this);
}


DenseQPsolver* QPsolver_qpOASES::cloneDenseQPsolver( ) const
{
	return new QPsolver_qpOASES(*this);
}


returnValue QPsolver_qpOASES::solve( DenseCP *cp_  )
{
	return DenseQPsolver::solve( cp_ );
}


returnValue QPsolver_qpOASES::solve(	double* H,
										double* A,
										double* g,
										double* lb,
										double* ub,
										double* lbA,
										double* ubA,
										uint maxIter
										)
{
	if ( qp == 0 )
		return ACADOERROR( RET_INITIALIZE_FIRST );

	/* call to qpOASES, using hotstart if possible and desired */
	numberOfSteps = maxIter;
	qpOASES::returnValue returnvalue;
	qpStatus = QPS_SOLVING;

	//printf( "nV: %d,  nC: %d \n",qp->getNV(),qp->getNC() );

	if ( (bool)qp->isInitialised( ) == false )
	{
		returnvalue = qp->init( H,g,A,lb,ub,lbA,ubA,numberOfSteps,0 );
	}
	else
	{
		int performHotstart = 0;
		get( HOTSTART_QP,performHotstart );

		if ( (bool)performHotstart == true )
		{
			 returnvalue = qp->hotstart( H,g,A,lb,ub,lbA,ubA,numberOfSteps,0 );
		}
		else
		{
			/* if no hotstart is desired, reset QP and use cold start */
			qp->reset( );
			returnvalue = qp->init( H,g,A,lb,ub,lbA,ubA,numberOfSteps,0 );
		}
	}
	setLast( LOG_NUM_QP_ITERATIONS, numberOfSteps );

	/* update QP status and determine return value */
	return updateQPstatus( returnvalue );
}


returnValue QPsolver_qpOASES::solve( DMatrix *H,
                                     DMatrix *A,
                                     DVector *g,
                                     DVector *lb,
                                     DVector *ub,
                                     DVector *lbA,
                                     DVector *ubA,
                                     uint maxIter       )
{
	return solve(	H->data(),
					A->data(),
					g->data(),
					lb->data(),
					ub->data(),
					lbA->data(),
					ubA->data(),
					maxIter
					);
}



returnValue QPsolver_qpOASES::step(	double* H,
									double* A,
									double* g,
									double* lb,
									double* ub,
									double* lbA,
									double* ubA
									)
{
	/* perform a single QP iteration */
	return solve( H,A,g,lb,ub,lbA,ubA,1 );
}


returnValue QPsolver_qpOASES::step(	DMatrix *H,
									DMatrix *A,
									DVector *g,
									DVector *lb,
									DVector *ub,
									DVector *lbA,
									DVector *ubA
									)
{
	/* perform a single QP iteration */
	return solve( H,A,g,lb,ub,lbA,ubA,1 );
}


returnValue QPsolver_qpOASES::getPrimalSolution( DVector& xOpt ) const
{
	if ( qp == 0 )
		return ACADOERROR( RET_INITIALIZE_FIRST );

	uint dim = qp->getNV( );
	double* xOptTmp = new double[dim];

	if ( qp->getPrimalSolution( xOptTmp ) == qpOASES::SUCCESSFUL_RETURN )
	{
		xOpt = DVector(dim, xOptTmp);
		delete[] xOptTmp;
		return SUCCESSFUL_RETURN;
	}
	else
	{
		delete[] xOptTmp;
		return ACADOERROR( RET_QP_NOT_SOLVED );
	}
}


returnValue QPsolver_qpOASES::getDualSolution( DVector& yOpt ) const
{
	if ( qp == 0 )
		return ACADOERROR( RET_INITIALIZE_FIRST );

	uint dim = qp->getNV( ) + qp->getNC( );
	double* yOptTmp = new double[dim];

	if ( qp->getDualSolution( yOptTmp ) == qpOASES::SUCCESSFUL_RETURN )
	{
		yOpt = DVector(dim, yOptTmp);
		delete[] yOptTmp;
		return SUCCESSFUL_RETURN;
	}
	else
	{
		delete[] yOptTmp;
		return ACADOERROR( RET_QP_NOT_SOLVED );
	}
}


double QPsolver_qpOASES::getObjVal( ) const
{
	if ( isUnbounded( ) == true )
		return -INFTY;

	if ( ( isSolved( ) == false ) || ( qp == 0 ) )
		return INFTY;

	return qp->getObjVal( );
}


returnValue QPsolver_qpOASES::getVarianceCovariance( DMatrix &var ){

    return ACADOERROR( RET_NOT_IMPLEMENTED_YET );
}


uint QPsolver_qpOASES::getNumberOfVariables( ) const
{
	if ( qp != 0 )
		return qp->getNV( );
	else
		return 0;
}

uint QPsolver_qpOASES::getNumberOfConstraints( ) const
{
	if ( qp != 0 )
		return qp->getNC( );
	else
		return 0;
}


returnValue QPsolver_qpOASES::getVarianceCovariance( DMatrix &H, DMatrix &var ){

    if ( qp == 0 )
        return ACADOERROR( RET_INITIALIZE_FIRST );

    if ( (bool)isSolved( ) == false ) return ACADOERROR( RET_QP_NOT_SOLVED );

    qpOASES::returnValue      returnvalue;
    qpOASES::SolutionAnalysis analyser   ;

    uint             NV, NC     ;
    uint             run1, run2 ;

    NV = qp->getNV();
    NC = qp->getNC();

    double *Var            = new double[(2*NV+NC)*(2*NV+NC)];
    double *PrimalDualVar  = new double[(2*NV+NC)*(2*NV+NC)];

    for( run1 = 0; run1 < (2*NV+NC)*(2*NV+NC); run1++ )
        Var[run1] = 0.0;

    for( run1 = 0; run1 < NV; run1++ )
        for( run2 = 0; run2 < NV; run2++ )
            Var[run1*(2*NV+NC)+run2] = H(run1,run2);

    returnvalue = analyser.getVarianceCovariance( qp, Var, PrimalDualVar );

    if( returnvalue != qpOASES::SUCCESSFUL_RETURN ){
        delete[] Var          ;
        delete[] PrimalDualVar;
        return ACADOERROR(RET_QP_NOT_SOLVED);
    }

    var.init( NV, NV );

    for( run1 = 0; run1 < NV; run1++ )
        for( run2 = 0; run2 < NV; run2++ )
            var( run1, run2 ) = PrimalDualVar[run1*(2*NV+NC)+run2];

    delete[] Var          ;
    delete[] PrimalDualVar;
    return SUCCESSFUL_RETURN;
}



//
// PROTECTED MEMBER FUNCTIONS:
//


returnValue QPsolver_qpOASES::setupQPobject( uint nV, uint nC )
{
	if ( qp != 0 )
		delete qp;

	/* create new qpOASES QP object... */
	qp = new qpOASES::SQProblem( nV,nC );
	
	qpOASES::Options options;
	options.setToFast();
	
	qp->setOptions( options );
	
	/* ... and define its printLevel */
	int printLevel = 0;
	//get( PRINTLEVEL,printLevel );

	switch( (PrintLevel) printLevel )
	{
		case HIGH:
			qp->setPrintLevel( qpOASES::PL_MEDIUM );
			break;

		case DEBUG:
			qp->setPrintLevel( qpOASES::PL_HIGH );
			break;

		// PL_NONE, PL_LOW, PL_MEDIUM
		default:
			qp->setPrintLevel( qpOASES::PL_NONE );
	}

	qpStatus = QPS_INITIALIZED;

	return SUCCESSFUL_RETURN;
}


returnValue QPsolver_qpOASES::updateQPstatus( int ret )
{
	switch ( (qpOASES::returnValue)ret )
	{
		case qpOASES::SUCCESSFUL_RETURN:
			qpStatus = QPS_SOLVED;
			return SUCCESSFUL_RETURN;

		case qpOASES::RET_MAX_NWSR_REACHED:
			qpStatus = QPS_NOTSOLVED;
			return RET_QP_SOLUTION_REACHED_LIMIT;

		default:
			qpStatus = QPS_NOTSOLVED;

			/* check for infeasibility */
			if ( (bool)qp->isInfeasible( ) == true )
			{
				qpStatus = QPS_INFEASIBLE;
				return RET_QP_INFEASIBLE;
			}

			/* check for unboundedness */
			if ( (bool)qp->isUnbounded( ) == true )
			{
				qpStatus = QPS_UNBOUNDED;
				return RET_QP_UNBOUNDED;
			}

			return RET_QP_SOLUTION_FAILED;
	}
}


CLOSE_NAMESPACE_ACADO

// end of file.
