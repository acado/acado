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
 *    \file src/conic_solver/condensing_based_cp_solver.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *
 */

#include <acado/conic_solver/condensing_based_cp_solver.hpp>
#include <acado/bindings/acado_qpoases/qp_solver_qpoases.hpp>

using namespace Eigen;
using namespace std;

BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

CondensingBasedCPsolver::CondensingBasedCPsolver( ) : BandedCPsolver( )
{
	nConstraints = 0;
    blockDims = 0;

	condensingStatus = COS_NOT_INITIALIZED;

    cpSolver = 0;
    cpSolverRelaxed = 0;
}


CondensingBasedCPsolver::CondensingBasedCPsolver(	UserInteraction* _userInteraction,
													uint nConstraints_,
        											const DVector& blockDims_
        											) : BandedCPsolver( _userInteraction )
{
	nConstraints = nConstraints_;
    blockDims = blockDims_;

	condensingStatus = COS_NOT_INITIALIZED;

    cpSolver = new QPsolver_qpOASES( _userInteraction );
    cpSolverRelaxed = new QPsolver_qpOASES( _userInteraction );
}


CondensingBasedCPsolver::CondensingBasedCPsolver( const CondensingBasedCPsolver& rhs )
                        :BandedCPsolver( rhs )
{
	nConstraints = rhs.nConstraints;
    blockDims = rhs.blockDims;
	
	condensingStatus = rhs.condensingStatus;

    if( rhs.cpSolver != 0 ) cpSolver = rhs.cpSolver->clone();
    else                    cpSolver = 0                    ;

    if( rhs.cpSolverRelaxed != 0 ) cpSolverRelaxed = rhs.cpSolverRelaxed->cloneDenseQPsolver();
    else                           cpSolverRelaxed = 0;

    denseCP = rhs.denseCP;
	
	deltaX = rhs.deltaX;
	deltaP = rhs.deltaP;
}


CondensingBasedCPsolver::~CondensingBasedCPsolver( ){

    if( cpSolver != 0 ) delete cpSolver ;
    if( cpSolverRelaxed != 0 ) delete cpSolverRelaxed;
}


CondensingBasedCPsolver& CondensingBasedCPsolver::operator=( const CondensingBasedCPsolver& rhs ){

    if ( this != &rhs ){

        if( cpSolver != 0 ) delete cpSolver ;
        if( cpSolverRelaxed != 0 ) delete cpSolverRelaxed;

        BandedCPsolver::operator=( rhs );

		nConstraints = rhs.nConstraints;
		blockDims = rhs.blockDims;

		condensingStatus = rhs.condensingStatus;

        if( rhs.cpSolver != 0 ) cpSolver = rhs.cpSolver->clone();
        else                    cpSolver = 0                    ;

        if( rhs.cpSolverRelaxed != 0 ) cpSolverRelaxed = rhs.cpSolverRelaxed->cloneDenseQPsolver();
        else                           cpSolverRelaxed = 0;

        denseCP = rhs.denseCP;

		deltaX = rhs.deltaX;
		deltaP = rhs.deltaP;
    }
    return *this;
}


BandedCPsolver* CondensingBasedCPsolver::clone() const
{
     return new CondensingBasedCPsolver(*this);
}



returnValue CondensingBasedCPsolver::init(	const OCPiterate &iter_
											)
{
    iter = iter_;

    // INITIALIZE THE CONDENSING MATRICES:
    // -----------------------------------
    if ( initializeCondensingOperator( ) != SUCCESSFUL_RETURN )
		return ACADOERROR( RET_BANDED_CP_INIT_FAILED );


    // INITIALIZE THE CP SOLVER:
    // -------------------------
	int infeasibleQPhandling;
	get( INFEASIBLE_QP_HANDLING,infeasibleQPhandling );
	
	if ( initializeCPsolver( (InfeasibleQPhandling)infeasibleQPhandling ) != SUCCESSFUL_RETURN )
		return ACADOERROR( RET_BANDED_CP_INIT_FAILED );


    return SUCCESSFUL_RETURN;
}



returnValue CondensingBasedCPsolver::prepareSolve(	BandedCP& cp
													)
{
	RealClock clock;

    // CONDENSE THE KKT-SYSTEM:
    // ------------------------

	int printLevel;
	get( PRINTLEVEL,printLevel );

	if ( (PrintLevel)printLevel >= HIGH ) 
		cout << "--> Condesing banded QP ...\n";

	clock.reset( );
	clock.start( );

    returnValue returnvalue = condense( cp );
    if( returnvalue != SUCCESSFUL_RETURN ) return ACADOERROR(returnvalue);

	clock.stop( );
	setLast( LOG_TIME_CONDENSING,clock.getTime() );

	if ( (PrintLevel)printLevel >= HIGH ) 
		cout << "<-- Condesing banded QP done.\n";

	return SUCCESSFUL_RETURN;
}


returnValue CondensingBasedCPsolver::solve(	BandedCP& cp
											)
{
	if ( areRealTimeParametersDefined( ) == BT_FALSE )
		prepareSolve( cp );


    // ADD THE FEEDBACK DATA TO THE DENSE QP (IF SPECIFIED):
    // -----------------------------------------------------

	if ( deltaX.isEmpty( ) == BT_FALSE )
	{
		for( uint run1 = 0; run1 < getNX(); run1++ )
		{
			denseCP.lb(run1) = deltaX(run1);
			denseCP.ub(run1) = deltaX(run1);
		}
	}

	if ( deltaP.isEmpty( ) == BT_FALSE )
	{
		for( uint run1 = 0; run1 < getNP(); run1++ )
		{
			denseCP.lb(getNX()+getNumPoints()*getNXA()+run1) = deltaP(run1);
			denseCP.ub(getNX()+getNumPoints()*getNXA()+run1) = deltaP(run1);
		}
	}
	
// 	cp.deltaX.print();

    // Solve QP subproblem
    // ------------------------------------
	int printLevel;
	get( PRINTLEVEL,printLevel );

	if ( (PrintLevel)printLevel >= HIGH ) 
		cout << "--> Solving condesed QP ...\n";
	
	
	
    returnValue returnvalue = solveCPsubproblem( );
    if( returnvalue != SUCCESSFUL_RETURN ) return ACADOERROR( RET_BANDED_CP_SOLUTION_FAILED );

	if ( (PrintLevel)printLevel >= HIGH ) 
		cout << "<-- Solving condesed QP done.\n";

    // Expand the KKT-System if neccessary:
    // ------------------------------------

	if ( areRealTimeParametersDefined( ) == BT_FALSE )
		return finalizeSolve( cp );
	else
		return SUCCESSFUL_RETURN;
}


returnValue CondensingBasedCPsolver::finalizeSolve(	BandedCP& cp
													)
{
	RealClock clock;

	int printLevel;
	get( PRINTLEVEL,printLevel );

	if ( (PrintLevel)printLevel >= HIGH ) 
		cout << "--> Expanding condensed QP solution ...\n";

    // Expand the KKT-System if neccessary:
    // ------------------------------------
	clock.reset( );
	clock.start( );

    returnValue returnvalue = expand( cp );
    if( returnvalue != SUCCESSFUL_RETURN ) return ACADOERROR(returnvalue);

	clock.stop( );
	setLast( LOG_TIME_EXPAND,clock.getTime() );
	
	if ( (PrintLevel)printLevel >= HIGH ) 
		cout << "<-- Expanding condensed QP solution done.\n";

    return returnvalue;
}



returnValue CondensingBasedCPsolver::getParameters( DVector &p_  ) const
{
	if ( p_.getDim( ) != getNP( ) )
		return ACADOERROR( RET_INCOMPATIBLE_DIMENSIONS );

	uint startIdx = getNX() + getNumPoints()*getNXA();
	
	for( uint i=0; i<getNP(); ++i )
		p_( i ) = (*denseCP.x)( startIdx+i );
	
	return SUCCESSFUL_RETURN;
}


returnValue CondensingBasedCPsolver::getFirstControl( DVector &u0_ ) const
{
	if ( u0_.getDim( ) != getNU( ) )
		return ACADOERROR( RET_INCOMPATIBLE_DIMENSIONS );

	uint startIdx = getNX() + getNumPoints()*getNXA() + getNP();
	
	for( uint i=0; i<getNU(); ++i )
		u0_( i ) = (*denseCP.x)( startIdx+i );
	
	return SUCCESSFUL_RETURN;
}



returnValue CondensingBasedCPsolver::getVarianceCovariance( DMatrix &var )
{
	if ( cpSolver == 0 )
		return ACADOERROR( RET_MEMBER_NOT_INITIALISED );
		
	return cpSolver->getVarianceCovariance( denseCP.H,var );
}



returnValue CondensingBasedCPsolver::setRealTimeParameters(	const DVector& DeltaX,
															const DVector& DeltaP
															)
{
	deltaX = DeltaX;
	deltaP = DeltaP;
	
	return SUCCESSFUL_RETURN;
}



returnValue CondensingBasedCPsolver::freezeCondensing( )
{
	if ( ( condensingStatus != COS_CONDENSED ) && ( condensingStatus != COS_FROZEN ) )
		return ACADOERROR( RET_NEED_TO_CONDENSE_FIRST );

	condensingStatus = COS_FROZEN;

	return SUCCESSFUL_RETURN;
}


returnValue CondensingBasedCPsolver::unfreezeCondensing( )
{
	if ( condensingStatus == COS_FROZEN )
		condensingStatus = COS_CONDENSED;

	return SUCCESSFUL_RETURN;
}



//
// PROTECTED MEMBER FUNCTIONS:
//

returnValue CondensingBasedCPsolver::projectHessian( DMatrix &H_, double dampingFactor ){

    if( dampingFactor < 0.0 ) return SUCCESSFUL_RETURN;

    // COMPUTE THE EIGENVALUES OF THE HESSIAN:
    // ---------------------------------------

    SelfAdjointEigenSolver< MatrixXd > es( H_ );
    MatrixXd V = es.eigenvectors();
    VectorXd D = es.eigenvalues();

    // OVER-PROJECT THE EIGENVALUES BASED ON THE DAMPING TECHNIQUE:
    // ------------------------------------------------------------

	for (unsigned el = 0; el < D.size(); el++)
		if (D( el ) <= 0.1 * dampingFactor)
		{
			if (fabs(D( el )) >= dampingFactor)
				D( el ) = fabs(D( el ));
			else
				D( el ) = dampingFactor;
		}

    // RECONSTRUCT THE PROJECTED HESSIAN MATRIX:
    // -----------------------------------------

    H_ = V * D.asDiagonal() * V.inverse();

    return SUCCESSFUL_RETURN;
}



returnValue CondensingBasedCPsolver::solveCPsubproblem( )
{
	if( denseCP.isQP() == BT_FALSE )
		return ACADOERROR( RET_QP_SOLVER_CAN_ONLY_SOLVE_QP );

    returnValue returnvalue;

	// ensure that Hessian matrix is symmetric
	if ( denseCP.H.isSymmetric( ) == BT_FALSE )
	{
		ACADOINFO( RET_NONSYMMETRIC_HESSIAN_MATRIX );
		denseCP.H.symmetrize();
	}


    // PROJECT HESSIAN TO POSITIVE DEFINITE CONE IF NECESSARY:
    // -------------------------------------------------------
	int hessianMode;
	get( HESSIAN_APPROXIMATION,hessianMode );

	if ( (HessianApproximationMode)hessianMode == EXACT_HESSIAN )
	{
		double hessianProjectionFactor;
		get( HESSIAN_PROJECTION_FACTOR, hessianProjectionFactor );
		projectHessian( denseCP.H, hessianProjectionFactor );
	}

    // APPLY LEVENBERG-MARQUARD REGULARISATION IF DESIRED:
    // -------------------------------------------------------
    double levenbergMarquard;
    get(LEVENBERG_MARQUARDT, levenbergMarquard );

    if( levenbergMarquard > EPS )
    	denseCP.H += eye<double>( denseCP.H.rows() ) * levenbergMarquard;

    if ( LVL_WARNING < Logger::instance().getLogLevel() )
    {
      // Check condition number of the condensed Hessian.
      double denseHConditionNumber = denseCP.H.getConditionNumber();
      if (denseHConditionNumber > 1.0e16)
      {
        LOG( LVL_WARNING )
            << "Condition number of the condensed Hessian is quite high: log_10(kappa( H )) = "
            << log10( denseHConditionNumber ) << endl;
        ACADOWARNING( RET_ILLFORMED_HESSIAN_MATRIX );
      }
      // Check for max and min entry in the condensed Hessian:
      if (denseCP.H.getMin() < -1.0e16 || denseCP.H.getMax() > 1.0e16)
      {
        LOG( LVL_WARNING ) << "Ill formed condensed Hessian: min(.) < -1e16 or max(.) > 1e16" << endl;
        ACADOWARNING( RET_ILLFORMED_HESSIAN_MATRIX );
      }
    }

    // SOLVE QP ALLOWING THE GIVEN NUMBER OF ITERATIONS:
    // -------------------------------------------------------
    int maxQPiter;
    get( MAX_NUM_QP_ITERATIONS, maxQPiter );


	RealClock clock;
	clock.start( );

// 	denseCP.print( "",PS_MATLAB );
	returnvalue = solveQP( maxQPiter );

	clock.stop( );
	setLast( LOG_TIME_QP,clock.getTime() );

	switch( returnvalue )
	{
		case SUCCESSFUL_RETURN:
			setLast( LOG_TIME_RELAXED_QP,0.0 );
			setLast( LOG_IS_QP_RELAXED, BT_FALSE );
			break;

		case RET_QP_SOLUTION_REACHED_LIMIT:
			setLast( LOG_TIME_RELAXED_QP,0.0 );
			setLast( LOG_IS_QP_RELAXED, BT_FALSE );
			ACADOWARNING( RET_QP_SOLUTION_REACHED_LIMIT );
			break;

		case RET_QP_UNBOUNDED:
			setLast( LOG_TIME_RELAXED_QP,0.0 );
			setLast( LOG_IS_QP_RELAXED, BT_FALSE );
			return ACADOERROR( RET_QP_UNBOUNDED );
			break;

		default: //case: RET_QP_INFEASIBLE:
			int infeasibleQPhandling;
			get( INFEASIBLE_QP_HANDLING,infeasibleQPhandling );

			switch( (InfeasibleQPhandling) infeasibleQPhandling )
			{

				case IQH_STOP:
					return ACADOERROR( RET_QP_INFEASIBLE );

				case IQH_IGNORE:
					setLast( LOG_TIME_RELAXED_QP,0.0 );
					setLast( LOG_IS_QP_RELAXED, BT_FALSE );
					break;

				default:
					clock.reset( );
					clock.start( );

					ACADOWARNING( RET_RELAXING_QP );
					
                    if ( solveQP(	maxQPiter - cpSolver->getNumberOfIterations( ),
									(InfeasibleQPhandling) infeasibleQPhandling
									) != SUCCESSFUL_RETURN )
						return ACADOERROR( RET_QP_SOLUTION_FAILED );

					clock.stop( );
					setLast( LOG_TIME_RELAXED_QP,clock.getTime() );
					setLast( LOG_IS_QP_RELAXED, BT_TRUE );
			}
			break;
	}

    return SUCCESSFUL_RETURN;
}



returnValue CondensingBasedCPsolver::condense(	BandedCP& cp
												)
{
	if ( ( condensingStatus != COS_INITIALIZED ) && ( condensingStatus != COS_FROZEN ) )
		return ACADOERROR( RET_UNABLE_TO_CONDENSE );

	uint run1, run2, run3;
	
    uint nF = getNF();
    uint nA = getNA();
    uint N = getNumPoints();


	if( getNX() + getNXA() > 0 )
	{
		if ( computeCondensingOperator( cp ) != SUCCESSFUL_RETURN )
			return ACADOERROR( RET_UNABLE_TO_CONDENSE );

		int rowOffset  = 0;
		uint rowOffset1 = 0;

		if ( condensingStatus != COS_FROZEN )
		{
			// generate H
			hT       = cp.hessian*T;
			HDense   = T^hT;

			if( getNX() != 0 ) generateHessianBlockLine( getNX(), rowOffset, rowOffset1 );
			rowOffset++;

			for( run3 = 0; run3 < N; run3++ ){
				if( getNXA() != 0 ) generateHessianBlockLine( getNXA(), rowOffset, rowOffset1 );
				rowOffset++;
			}

			if( getNP() != 0 ) generateHessianBlockLine( getNP(), rowOffset, rowOffset1 );
			rowOffset++;

			for( run3 = 0; run3 < N-1; run3++ ){
				if( getNU() != 0 ) generateHessianBlockLine( getNU(), rowOffset, rowOffset1 );
				rowOffset++;
			}

			for( run3 = 0; run3 < N-1; run3++ ){
				if( getNW() != 0 ) generateHessianBlockLine( getNW(), rowOffset, rowOffset1 );
				rowOffset++;
			}


			// generate A
			ADense   = cp.constraintGradient*T;

			denseCP.A.setZero();

			rowOffset1 = 0;
			for( run3 = 0; run3 < ADense.getNumRows(); run3++ ){
				ASSERT( getNC() != 0 );
				generateConstraintBlockLine( (int) blockDims(run3), run3, rowOffset1 );
				rowOffset++;
			}

			for( run3 = 1; run3 < N; run3++ ){
				generateStateBoundBlockLine( getNX(), run3, rowOffset1 );
			}
		}


		// generate g
		gDense   = (cp.objectiveGradient*T) + (d^hT);
		
        generateObjectiveGradient( );


		// generate lb, ub
        BlockMatrix dCut(4*N+1,1);
        DMatrix tmp;
        for( run1 = 0; run1 < N; run1++ ){
            d.getSubBlock(run1,0,tmp);
            if( tmp.getDim() != 0 )
                dCut.setDense(run1,0,tmp);
        }

        lbDense  = cp.lowerBoundResiduum - dCut;
        ubDense  = cp.upperBoundResiduum - dCut;

        generateBoundVectors( );


		// generate lbA, ubA
		lbADense = cp.lowerConstraintResiduum - cp.constraintGradient*d;
		ubADense = cp.upperConstraintResiduum - cp.constraintGradient*d;

        rowOffset1 = 0;
        for( run3 = 0; run3 < ADense.getNumRows(); run3++ ){
            ASSERT( getNC() != 0 );
            generateConstraintVectors( (int) blockDims(run3), run3, rowOffset1 );
            rowOffset++;
        }

        for( run3 = 1; run3 < N; run3++ ){
            generateStateBoundVectors( getNX(), run3, rowOffset1 );
        }


//         acadoPrintf("denseCP.H = \n");
//         denseCP.H.print();
//
//         acadoPrintf("denseCP.A = \n");
//         denseCP.A.print();
//
//         acadoPrintf("denseCP.g = \n");
//         denseCP.g.print();
//
//         acadoPrintf("denseCP.lbA = \n");
//         denseCP.lbA.print();
//
//         acadoPrintf("denseCP.ubA = \n");
//         denseCP.ubA.print();
//
//         acadoPrintf("denseCP.lb = \n");
//         denseCP.lb.print();
//
//         acadoPrintf("denseCP.ub = \n");
//         denseCP.ub.print();
    }
    else{

        ASSERT( N  == 1 );
        ASSERT( getNU() == 0 );
        ASSERT( getNW() == 0 );

        denseCP.H.init( nF, nF );
        denseCP.g.init( nF );

        denseCP.A.init( nA, nF );
        denseCP.lbA.init( nA );
        denseCP.ubA.init( nA );

        denseCP.lb.init( nF );
        denseCP.ub.init( nF );

        DMatrix tmp;

        cp.hessian           .getSubBlock( 2, 2, denseCP.H  , getNP(), getNP() );
        cp.objectiveGradient .getSubBlock( 0, 2, tmp, 1 , getNP() );
        for( run1 = 0; run1 < getNP(); run1++ )
            denseCP.g(run1) = tmp(0,run1);

        uint offset = 0;
        for( run2 = 0; run2 < cp.constraintGradient.getNumRows(); run2++ ){

            cp.constraintGradient.getSubBlock( run2, 2, tmp );

            for( run1 = 0; run1 < tmp.getNumRows(); run1++ )
                for( run3 = 0; run3 < tmp.getNumCols(); run3++ )
                    denseCP.A(offset + run1,run3) = tmp(run1,run3);

            cp.lowerConstraintResiduum.getSubBlock( run2, 0, tmp );
            for( run1 = 0; run1 < tmp.getNumRows(); run1++ )
                denseCP.lbA(offset + run1) = tmp(run1,0);

            cp.upperConstraintResiduum.getSubBlock( run2, 0, tmp );
            for( run1 = 0; run1 < tmp.getNumRows(); run1++ )
                denseCP.ubA(offset + run1) = tmp(run1,0);

            offset += tmp.getNumRows();
        }

        cp.lowerBoundResiduum.getSubBlock( 2, 0, tmp, getNP(), 1 );
        for( run1 = 0; run1 < getNP(); run1++ )
            denseCP.lb(run1) = tmp(run1,0);
        cp.upperBoundResiduum.getSubBlock( 2, 0, tmp, getNP(), 1 );
        for( run1 = 0; run1 < getNP(); run1++ )
            denseCP.ub(run1) = tmp(run1,0);
    }

	if ( condensingStatus != COS_FROZEN )
		condensingStatus = COS_CONDENSED;

    return SUCCESSFUL_RETURN;
}



returnValue CondensingBasedCPsolver::generateHessianBlockLine( uint nn, uint rowOffset, uint& rowOffset1 ){

    uint run1, run2, run3;

    uint  colOffset  = 0;
    uint  colOffset1 = 0;

    uint N = getNumPoints();

    DMatrix tmp;

    if( getNX() != 0 ){
        HDense.getSubBlock( rowOffset, 0, tmp, nn, getNX() );

        for( run1 = 0; run1 < nn; run1++ )
            for( run2 = 0; run2 < getNX(); run2++ )
                denseCP.H(rowOffset1+run1,run2) = tmp(run1,run2);
        colOffset1 += getNX();
    }
    colOffset++;

    for( run3 = 0; run3 < N; run3++ ){
         if( getNXA() != 0 ){
            HDense.getSubBlock( rowOffset, colOffset, tmp, nn, getNXA() );
            for( run1 = 0; run1 < nn; run1++ )
                for( run2 = 0; run2 < getNXA(); run2++ )
                    denseCP.H(rowOffset1+run1,colOffset1+run2) = tmp(run1,run2);
            colOffset1 += getNXA();
        }
        colOffset  ++;
    }

    if( getNP() != 0 ){
        HDense.getSubBlock( rowOffset, colOffset, tmp, nn, getNP() );
        for( run1 = 0; run1 < nn; run1++ )
            for( run2 = 0; run2 < getNP(); run2++ )
                denseCP.H(rowOffset1+run1,colOffset1+run2) = tmp(run1,run2);
         colOffset1 += getNP();
     }
     colOffset  ++;

     for( run3 = 0; run3 < N-1; run3++ ){
          if( getNU() != 0 ){
             HDense.getSubBlock( rowOffset, colOffset, tmp, nn, getNU() );
             for( run1 = 0; run1 < nn; run1++ )
                 for( run2 = 0; run2 < getNU(); run2++ )
                     denseCP.H(rowOffset1+run1,colOffset1+run2) = tmp(run1,run2);
             colOffset1 += getNU();
         }
         colOffset  ++;
     }

     for( run3 = 0; run3 < N-1; run3++ ){
         if( getNW() != 0 ){
              HDense.getSubBlock( rowOffset, colOffset, tmp, nn, getNW() );
              for( run1 = 0; run1 < nn; run1++ )
                  for( run2 = 0; run2 < getNW(); run2++ )
                      denseCP.H(rowOffset1+run1,colOffset1+run2) = tmp(run1,run2);
              colOffset1 += getNW();
         }
         colOffset  ++;
     }
     rowOffset1 += nn;

     return SUCCESSFUL_RETURN;
}


returnValue CondensingBasedCPsolver::generateConstraintBlockLine( uint nn, uint rowOffset, uint& rowOffset1 ){


    uint run1, run2, run3;

    uint  colOffset  = 0;
    uint  colOffset1 = 0;

    uint N = getNumPoints();

    DMatrix tmp;

    if( getNX() != 0 ){
        ADense.getSubBlock( rowOffset, 0, tmp, nn, getNX() );
        for( run1 = 0; run1 < nn; run1++ )
            for( run2 = 0; run2 < getNX(); run2++ )
                denseCP.A(rowOffset1+run1,run2) = tmp(run1,run2);
        colOffset1 += getNX();
    }
    colOffset++;

    for( run3 = 0; run3 < N; run3++ ){
         if( getNXA() != 0 ){
            ADense.getSubBlock( rowOffset, colOffset, tmp, nn, getNXA() );
            for( run1 = 0; run1 < nn; run1++ )
                for( run2 = 0; run2 < getNXA(); run2++ )
                    denseCP.A(rowOffset1+run1,colOffset1+run2) = tmp(run1,run2);
            colOffset1 += getNXA();
        }
        colOffset  ++;
    }

    if( getNP() != 0 ){
        ADense.getSubBlock( rowOffset, colOffset, tmp, nn, getNP() );
        for( run1 = 0; run1 < nn; run1++ )
            for( run2 = 0; run2 < getNP(); run2++ )
                denseCP.A(rowOffset1+run1,colOffset1+run2) = tmp(run1,run2);
         colOffset1 += getNP();
     }
     colOffset  ++;

     for( run3 = 0; run3 < N-1; run3++ ){
          if( getNU() != 0 ){
             ADense.getSubBlock( rowOffset, colOffset, tmp, nn, getNU() );
             for( run1 = 0; run1 < nn; run1++ )
                 for( run2 = 0; run2 < getNU(); run2++ )
                     denseCP.A(rowOffset1+run1,colOffset1+run2) = tmp(run1,run2);
             colOffset1 += getNU();
         }
         colOffset  ++;
     }

     for( run3 = 0; run3 < N-1; run3++ ){
         if( getNW() != 0 ){
              ADense.getSubBlock( rowOffset, colOffset, tmp, nn, getNW() );
              for( run1 = 0; run1 < nn; run1++ )
                  for( run2 = 0; run2 < getNW(); run2++ )
                      denseCP.A(rowOffset1+run1,colOffset1+run2) = tmp(run1,run2);
              colOffset1 += getNW();
         }
         colOffset  ++;
     }
     rowOffset1 += nn;

     return SUCCESSFUL_RETURN;
}


returnValue CondensingBasedCPsolver::generateStateBoundBlockLine( uint nn, uint rowOffset, uint& rowOffset1 ){


    uint run1, run2, run3;

    uint  colOffset  = 0;
    uint  colOffset1 = 0;

    uint N = getNumPoints();

    DMatrix tmp;

    if( getNX() != 0 ){
        T.getSubBlock( rowOffset, 0, tmp, nn, getNX() );
        for( run1 = 0; run1 < nn; run1++ )
            for( run2 = 0; run2 < getNX(); run2++ )
                denseCP.A(rowOffset1+run1,run2) = tmp(run1,run2);
        colOffset1 += getNX();
    }
    colOffset++;

    for( run3 = 0; run3 < N; run3++ ){
         if( getNXA() != 0 ){
            T.getSubBlock( rowOffset, colOffset, tmp, nn, getNXA() );
            for( run1 = 0; run1 < nn; run1++ )
                for( run2 = 0; run2 < getNXA(); run2++ )
                    denseCP.A(rowOffset1+run1,colOffset1+run2) = tmp(run1,run2);
            colOffset1 += getNXA();
        }
        colOffset  ++;
    }

    if( getNP() != 0 ){
        T.getSubBlock( rowOffset, colOffset, tmp, nn, getNP() );
        for( run1 = 0; run1 < nn; run1++ )
            for( run2 = 0; run2 < getNP(); run2++ )
                denseCP.A(rowOffset1+run1,colOffset1+run2) = tmp(run1,run2);
         colOffset1 += getNP();
     }
     colOffset  ++;

     for( run3 = 0; run3 < N-1; run3++ ){
          if( getNU() != 0 ){
             T.getSubBlock( rowOffset, colOffset, tmp, nn, getNU() );
             for( run1 = 0; run1 < nn; run1++ )
                 for( run2 = 0; run2 < getNU(); run2++ )
                     denseCP.A(rowOffset1+run1,colOffset1+run2) = tmp(run1,run2);
             colOffset1 += getNU();
         }
         colOffset  ++;
     }

     for( run3 = 0; run3 < N-1; run3++ ){
         if( getNW() != 0 ){
              T.getSubBlock( rowOffset, colOffset, tmp, nn, getNW() );
              for( run1 = 0; run1 < nn; run1++ )
                  for( run2 = 0; run2 < getNW(); run2++ )
                      denseCP.A(rowOffset1+run1,colOffset1+run2) = tmp(run1,run2);
              colOffset1 += getNW();
         }
         colOffset  ++;
     }
     rowOffset1 += nn;

     return SUCCESSFUL_RETURN;
}


returnValue CondensingBasedCPsolver::generateConstraintVectors( uint nn, uint rowOffset, uint& rowOffset1 ){

    uint run1;

    DMatrix tmp;

    lbADense.getSubBlock( rowOffset, 0, tmp, nn, 1 );
    for( run1 = 0; run1 < nn; run1++ )
        denseCP.lbA(rowOffset1+run1) = tmp(run1,0);

    ubADense.getSubBlock( rowOffset, 0, tmp, nn, 1 );
    for( run1 = 0; run1 < nn; run1++ )
        denseCP.ubA(rowOffset1+run1) = tmp(run1,0);

    rowOffset1 += nn;

    return SUCCESSFUL_RETURN;
}


returnValue CondensingBasedCPsolver::generateStateBoundVectors( uint nn, uint rowOffset, uint& rowOffset1 ){

    uint run1;

    DMatrix tmp;

    lbDense.getSubBlock( rowOffset, 0, tmp, nn, 1 );
    for( run1 = 0; run1 < nn; run1++ )
        denseCP.lbA(rowOffset1+run1) = tmp(run1,0);

    ubDense.getSubBlock( rowOffset, 0, tmp, nn, 1 );
    for( run1 = 0; run1 < nn; run1++ )
        denseCP.ubA(rowOffset1+run1) = tmp(run1,0);

    rowOffset1 += nn;

    return SUCCESSFUL_RETURN;
}


returnValue CondensingBasedCPsolver::generateBoundVectors( ){

    uint run1, run3;

    uint N = getNumPoints();

    uint  rowOffset  = N;
    uint  rowOffset1 = 0;

    DMatrix tmp;

    if( getNX() != 0 ){
        lbDense.getSubBlock( 0, 0, tmp, getNX(), 1 );
        for( run1 = 0; run1 < getNX(); run1++ )
            denseCP.lb(rowOffset1+run1) = tmp(run1,0);
        ubDense.getSubBlock( 0, 0, tmp, getNX(), 1 );
        for( run1 = 0; run1 < getNX(); run1++ )
            denseCP.ub(rowOffset1+run1) = tmp(run1,0);
        rowOffset1 += getNX();
    }

    for( run3 = 0; run3 < N; run3++ ){
         if( getNXA() != 0 ){
            lbDense.getSubBlock( rowOffset, 0, tmp, getNXA(), 1 );
            for( run1 = 0; run1 < getNXA(); run1++ )
                denseCP.lb(rowOffset1+run1) = tmp(run1,0);
            ubDense.getSubBlock( rowOffset, 0, tmp, getNXA(), 1 );
            for( run1 = 0; run1 < getNXA(); run1++ )
                denseCP.ub(rowOffset1+run1) = tmp(run1,0);
            rowOffset1 += getNXA();
        }
        rowOffset++;
    }

    if( getNP() != 0 ){
        lbDense.getSubBlock( rowOffset, 0, tmp, getNP(), 1 );
        for( run1 = 0; run1 < getNP(); run1++ )
            denseCP.lb(rowOffset1+run1) = tmp(run1,0);
        ubDense.getSubBlock( rowOffset, 0, tmp, getNP(), 1 );
        for( run1 = 0; run1 < getNP(); run1++ )
            denseCP.ub(rowOffset1+run1) = tmp(run1,0);
         rowOffset1 += getNP();
     }
     rowOffset++;

     for( run3 = 0; run3 < N-1; run3++ ){
          if( getNU() != 0 ){
             lbDense.getSubBlock( rowOffset, 0, tmp, getNU(), 1 );
             for( run1 = 0; run1 < getNU(); run1++ )
                 denseCP.lb(rowOffset1+run1) = tmp(run1,0);
             ubDense.getSubBlock( rowOffset, 0, tmp, getNU(), 1 );
             for( run1 = 0; run1 < getNU(); run1++ )
                 denseCP.ub(rowOffset1+run1) = tmp(run1,0);
             rowOffset1 += getNU();
         }
         rowOffset++;
     }
     rowOffset++;

     for( run3 = 0; run3 < N-1; run3++ ){
         if( getNW() != 0 ){
              lbDense.getSubBlock( rowOffset, 0, tmp, getNW(), 1 );
              for( run1 = 0; run1 < getNW(); run1++ )
                  denseCP.lb(rowOffset1+run1) = tmp(run1,0);
              ubDense.getSubBlock( rowOffset, 0, tmp, getNW(), 1 );
              for( run1 = 0; run1 < getNW(); run1++ )
                  denseCP.ub(rowOffset1+run1) = tmp(run1,0);
              rowOffset1 += getNW();
         }
         rowOffset  ++;
     }

     return SUCCESSFUL_RETURN;
}


returnValue CondensingBasedCPsolver::generateObjectiveGradient( ){

    uint run1, run3;

    uint  rowOffset  = 0;
    uint  rowOffset1 = 0;

    uint N = getNumPoints();

    DMatrix tmp;

    if( getNX() != 0 ){
        gDense.getSubBlock( 0, 0, tmp, 1, getNX() );
        for( run1 = 0; run1 < getNX(); run1++ )
            denseCP.g(rowOffset1+run1) = tmp(0,run1);
        rowOffset1 += getNX();
    }
    rowOffset++;

    for( run3 = 0; run3 < N; run3++ ){
         if( getNXA() != 0 ){
            gDense.getSubBlock( 0, rowOffset, tmp, 1, getNXA() );
            for( run1 = 0; run1 < getNXA(); run1++ )
                denseCP.g(rowOffset1+run1) = tmp(0,run1);
            rowOffset1 += getNXA();
        }
        rowOffset++;
    }

    if( getNP() != 0 ){
        gDense.getSubBlock( 0, rowOffset, tmp, 1, getNP() );
        for( run1 = 0; run1 < getNP(); run1++ )
            denseCP.g(rowOffset1+run1) = tmp(0,run1);
         rowOffset1 += getNP();
     }
     rowOffset++;

     for( run3 = 0; run3 < N-1; run3++ ){
          if( getNU() != 0 ){
             gDense.getSubBlock( 0, rowOffset, tmp, 1, getNU() );
             for( run1 = 0; run1 < getNU(); run1++ )
                 denseCP.g(rowOffset1+run1) = tmp(0,run1);
             rowOffset1 += getNU();
         }
         rowOffset++;
     }

     for( run3 = 0; run3 < N-1; run3++ ){
         if( getNW() != 0 ){
              gDense.getSubBlock( 0, rowOffset, tmp, 1, getNW() );
              for( run1 = 0; run1 < getNW(); run1++ )
                  denseCP.g(rowOffset1+run1) = tmp(0,run1);
              rowOffset1 += getNW();
         }
         rowOffset  ++;
     }

     return SUCCESSFUL_RETURN;
}



returnValue CondensingBasedCPsolver::expand(	BandedCP& cp
												)
{
	// do not expand if condensing is frozen
// 	if ( condensingStatus == COS_FROZEN )
// 		return SUCCESSFUL_RETURN;

	if ( ( condensingStatus != COS_CONDENSED ) && ( condensingStatus != COS_FROZEN ) )
		return ACADOERROR( RET_UNABLE_TO_EXPAND );


    uint run1,run2;

    int rowCount  = 0;
    int rowCount1 = 0;

    uint nF = getNF();
    uint N = getNumPoints();


	DVector denseDualSolution( denseCP.getMergedDualSolution( ) );

    if( getNX() != 0 ){

        BlockMatrix primalDense;
        primalDense.init( 3*N, 1 );

        DMatrix tmp;

        rowCount  = 0;
        rowCount1 = 0;

        tmp.init( getNX(), 1 );
        for( run1 = 0; run1 < getNX(); run1++ )
            tmp(run1,0) = (*denseCP.x)(run1);
        primalDense.setDense( 0, 0, tmp );
        rowCount1 += getNX();
        rowCount++;

        for( run2 = 0; run2 < N; run2++ ){
            if( getNXA() != 0 ){
                tmp.init( getNXA(), 1 );
                for( run1 = 0; run1 < getNXA(); run1++ )
                    tmp(run1,0) = (*denseCP.x)(rowCount1+run1);
                primalDense.setDense( rowCount, 0, tmp );
                rowCount1 += getNXA();
            }
            rowCount++;
        }

        if( getNP() != 0 ){
            tmp.init( getNP(), 1 );
            for( run1 = 0; run1 < getNP(); run1++ )
                tmp(run1,0) = (*denseCP.x)(rowCount1+run1);
            primalDense.setDense( rowCount, 0, tmp );
            rowCount1 += getNP();
        }
        rowCount++;

        for( run2 = 0; run2 < N-1; run2++ ){
            if( getNU() != 0 ){
                tmp.init( getNU(), 1 );
                for( run1 = 0; run1 < getNU(); run1++ )
                    tmp(run1,0) = (*denseCP.x)(rowCount1+run1);
                primalDense.setDense( rowCount, 0, tmp );
                rowCount1 += getNU();
            }
            rowCount++;
        }

        for( run2 = 0; run2 < N-1; run2++ ){
            if( getNW() != 0 ){
                tmp.init( getNW(), 1 );
                for( run1 = 0; run1 < getNW(); run1++ )
                    tmp(run1,0) = (*denseCP.x)(rowCount1+run1);
                primalDense.setDense( rowCount, 0, tmp );
                rowCount1 += getNW();
            }
            rowCount++;
        }

        cp.deltaX = T*primalDense + d;

        BlockMatrix aux;

        aux = (cp.deltaX^cp.hessian) + cp.objectiveGradient;

        DVector aux2(N*getNX());
        aux2.setZero();

        int run = 0;
        uint run3, run4;

        for( run1 = 0; run1 < (uint)cp.constraintGradient.getNumRows(); run1++ ){
            for( run2 = 0; run2 < N; run2++ ){
                cp.constraintGradient.getSubBlock( run1, run2, tmp );
                for( run3 = 0; run3 < (uint)tmp.getNumRows(); run3++ ){
                    for( run4 = 0; run4 < getNX(); run4++ ){
                        aux2(run2*getNX()+run4) += denseDualSolution(nF+run+run3)*tmp(run3,run4);
                    }
                }
                run += tmp.getNumRows();
            }
        }

        DVector aux3(N*getNX());

        for( run2 = 0; run2 < getNX(); run2++ )
            aux3(run2) = denseDualSolution(run2);

        run = nF;
        run += getNC();

        for( run1 = 0; run1 < N-1; run1++ )
            for( run2 = 0; run2 < getNX(); run2++ )
                aux3((run1+1)*getNX()+run2) = denseDualSolution(run+run1*getNX()+run2);

        DVector *aux4 = new DVector[N-1];

        // aux  = x^T denseCP.H + denseCP.g      (BlockMatrix)
        // aux2 = lambda^T denseCP.A     (DVector     )
        // aux3 = lambda_bound   (DVector     )

        for( run1 = 0; run1 < N-1; run1++ ){
            aux4[run1].init(getNX());
            aux.getSubBlock( 0, run1+1, tmp );

            for( run2 = 0; run2 < getNX(); run2++ ){
                aux4[run1](run2) = tmp(0,run2) - aux2( (run1+1)*getNX()+run2 ) - aux3( (run1+1)*getNX()+run2 );
            }
        }

        // aux4[...] = x^T denseCP.H + denseCP.g - lambda^T denseCP.A - lambda_bound

        DMatrix Gx;
        DVector *lambdaDyn;
        lambdaDyn = new DVector[N-1];
        lambdaDyn[N-2] = aux4[N-2];

        for( run1 = N-2; run1 >= 1; run1-- ){
            cp.dynGradient.getSubBlock( run1, 0,  Gx );
            lambdaDyn[run1-1] = (Gx.transpose() * lambdaDyn[run1]) + aux4[run1-1];
        }

        cp.lambdaDynamic.init( N-1, 1 );

        for( run1 = 0; run1 < N-1; run1++ ){
            tmp.init( lambdaDyn[run1].getDim(), 1 );
            for( run2 = 0; run2 < lambdaDyn[run1].getDim(); run2++ )
                tmp(run2,0) = lambdaDyn[run1].operator()(run2);
            cp.lambdaDynamic.setDense( run1, 0, tmp );
        }


        int dynMode;
        get( DYNAMIC_SENSITIVITY, dynMode );

        if( dynMode == FORWARD_SENSITIVITY_LIFTED ){
            for( run1 = 0; run1 < N-1; run1++ ){
                 tmp.init( lambdaDyn[run1].getDim(), 1 );
                 tmp.setAll(1.0/((double) lambdaDyn[run1].getDim()));
                 cp.lambdaDynamic.setDense( run1, 0, tmp );
            }
        }

        delete[] lambdaDyn;
        delete[] aux4;
    }
    else{
        DMatrix tmp ( getNP(),1 );
        cp.deltaX.init( 5, 1 );

        for( run1 = 0; run1 < getNP(); run1++ )
            tmp( run1, 0 ) = (*denseCP.x)(run1);

        cp.deltaX.setDense( 2, 0, tmp );
    }


    DMatrix tmp;

    cp.lambdaConstraint.init( blockDims.getDim(), 1 );

    int run = 0;
    for( run1 = 0; run1 < blockDims.getDim(); run1++ ){
        tmp.init( (int) blockDims(run1), 1 );

        for( run2 = 0; run2 < blockDims(run1); run2++ ){
            tmp(run2,0) = denseDualSolution(nF+run+run2);
        }
        cp.lambdaConstraint.setDense( run1, 0, tmp );
        run += (int) blockDims(run1);
    }

    cp.lambdaBound.init(4*N+1,1);
    rowCount = 0;

    if( getNX() != 0 ){
        tmp.init(getNX(),1);
        for( run1 = 0; run1 < getNX(); run1++ )
            tmp(run1,0) = denseDualSolution(run1);
        cp.lambdaBound.setDense( 0, 0, tmp );
        rowCount++;
    }
    rowCount1 = 0;

    for( run2 = 1; run2 < N; run2++ ){
        if( getNX() != 0 ){
            tmp.init( getNX(), 1 );
            for( run1 = 0; run1 < getNX(); run1++ )
                tmp(run1,0) = denseDualSolution(nF+getNC()+rowCount1+run1);
            cp.lambdaBound.setDense( rowCount, 0, tmp );
            rowCount1 += getNX();
        }
        rowCount++;
    }
    rowCount1 = getNX();

    for( run2 = 0; run2 < N; run2++ ){
        if( getNXA() != 0 ){
            tmp.init( getNXA(), 1 );
            for( run1 = 0; run1 < getNXA(); run1++ )
                tmp(run1,0) = denseDualSolution(rowCount1+run1);
            cp.lambdaBound.setDense( rowCount, 0, tmp );
            rowCount1 += getNXA();
        }
        rowCount++;
    }

    if( getNP() != 0 ){
        tmp.init( getNP(), 1 );
        for( run1 = 0; run1 < getNP(); run1++ )
            tmp(run1,0) = denseDualSolution(rowCount1+run1);
        cp.lambdaBound.setDense( rowCount, 0, tmp );
        rowCount1 += getNP();
    }
    rowCount++;

    for( run2 = 0; run2 < N-1; run2++ ){
        if( getNU() != 0 ){
            tmp.init( getNU(), 1 );
            for( run1 = 0; run1 < getNU(); run1++ )
                tmp(run1,0) = denseDualSolution(rowCount1+run1);
            cp.lambdaBound.setDense( rowCount, 0, tmp );
            rowCount1 += getNU();
        }
        rowCount++;
    }
    rowCount++;

    for( run2 = 0; run2 < N-1; run2++ ){
        if( getNW() != 0 ){
            tmp.init( getNW(), 1 );
            for( run1 = 0; run1 < getNW(); run1++ )
                tmp(run1,0) = denseDualSolution(rowCount1+run1);
            cp.lambdaBound.setDense( rowCount, 0, tmp );
            rowCount1 += getNW();
        }
        rowCount++;
    }

	if ( condensingStatus != COS_FROZEN )
		condensingStatus = COS_INITIALIZED;

    return SUCCESSFUL_RETURN;
}



returnValue CondensingBasedCPsolver::initializeCondensingOperator( )
{
    uint N = getNumPoints();

	T.init( 5*N, 3*N );
    d.init( 5*N, 1   );

    T.setZero();
    d.setZero();

    if( getNX() != 0 ) T.setIdentity( 0, 0, getNX() );
    for( uint run1 = 0; run1 < N; run1++ ){
        if( getNXA() != 0 )        T.setIdentity(   N+run1,   1+run1, getNXA() );
        if( getNP()  != 0 )        T.setIdentity( 2*N+run1, N+1     , getNP() );
        if( getNU()  != 0 ){
            if( run1 != N-1 ) T.setIdentity( 3*N+run1, N+2+run1, getNU() );
            else              T.setIdentity( 3*N+run1, N+1+run1, getNU() );
        }
        if( getNW()  != 0 ){
            if( run1 != N-1 ) T.setIdentity( 4*N+run1, 2*N+1+run1, getNW() );
            else              T.setIdentity( 4*N+run1, 2*N  +run1, getNW() );
        }
    }

	condensingStatus = COS_INITIALIZED;

    return SUCCESSFUL_RETURN;
}


returnValue CondensingBasedCPsolver::computeCondensingOperator(	BandedCP& cp
																)
{
	uint run1, run2;
	uint N = getNumPoints();

	DMatrix  Gx;
	DMatrix   G;
	DMatrix tmp;

	for( run1 = 0; run1 < N-1; run1++ )
	{
			// DIFFERENTIAL STATES:
			// --------------------
			cp.dynGradient.getSubBlock( run1, 0,  Gx );   // Get the sensitivity G_x^i with respect to x
			
		if ( condensingStatus != COS_FROZEN )
		{
			T             .getSubBlock( run1, 0, tmp );   // get the corresponding  C_i .

			T.setDense( run1+1, 0, Gx*tmp );		   // compute C_{i+1} := G_x^i * C_i

			// ALGEBRAIC STATES:
			// --------------------

			cp.dynGradient.getSubBlock( run1, 1, G );

			for( run2 = 0; run2 <= run1; run2++ ){

				T.getSubBlock( run1, run2+1, tmp );

				if( G.getDim() != 0 ){

					if( run1 == run2 ) T.setDense( run1+1, run2+1, G      );
					else               T.setDense( run1+1, run2+1, Gx*tmp );
				}
			}

			// PARAMETERS:
			// --------------------

			cp.dynGradient.getSubBlock( run1, 2  ,  G  ); // Get the sensitivity G_p^i with respect to p
			T             .getSubBlock( run1, N+1, tmp ); // get the corresponding  D_p^i.

			if( tmp.getDim() != 0 ){
				if( G.getDim() != 0 )
					T.setDense( run1+1, N+1, Gx*tmp + G );   // compute  D_p^{i+1} := G_x^i D_p^i + G_p^i
			}
			else{
				if( G.getDim() != 0 )
					T.setDense( run1+1, N+1,          G );   // compute  D_p^{i+1} := G_x^i D_p^i + G_p^i
			}

			// CONTROLS:
			// --------------------

			cp.dynGradient.getSubBlock( run1, 3, G );
			for( run2 = 0; run2 <= run1; run2++ ){

				T.getSubBlock( run1, run2+2+N, tmp );

				if( G.getDim() != 0 ){

					if( run1 == run2 ) T.setDense( run1+1, run2+2+N, G      );
					else               T.setDense( run1+1, run2+2+N, Gx*tmp );
				}
			}

			// DISTURBANCES:
			// --------------------

			cp.dynGradient.getSubBlock( run1, 4, G );

			for( run2 = 0; run2 <= run1; run2++ ){

				T.getSubBlock( run1, run2+1+2*N, tmp );

				if( G.getDim() != 0 ){

					if( run1 == run2 ) T.setDense( run1+1, run2+1+2*N, G      );
					else               T.setDense( run1+1, run2+1+2*N, Gx*tmp );
				}
			}
		}

		// RESIDUUM:
		// --------------------

		cp.dynResiduum.getSubBlock( run1, 0,  G  );   // Get the residuum  b^i
		d             .getSubBlock( run1, 0, tmp );   // get the corresponding  d^i.

		if( tmp.getDim() != 0 ){
			if( G.getDim() != 0 )
				d.setDense( run1+1, 0, Gx*tmp + G );   // compute  d^{i+1} := G_x^i d^i + b^i
		}
		else{
			if( G.getDim() != 0 )
				d.setDense( run1+1, 0, G );   // compute  d^{i+1} := b^i
		}
	}

	return SUCCESSFUL_RETURN;
}



returnValue CondensingBasedCPsolver::setupRelaxedQPdata(	InfeasibleQPhandling infeasibleQPhandling,
															DenseCP& _denseCPrelaxed
															) const
{
	switch( infeasibleQPhandling )
	{
		case IQH_RELAX_L1:
			return setupRelaxedQPdataL2( _denseCPrelaxed );

		case IQH_RELAX_L2:
			return setupRelaxedQPdataL2( _denseCPrelaxed );

		default:
			return ACADOERROR( RET_INVALID_ARGUMENTS );
	}
}


returnValue CondensingBasedCPsolver::setupRelaxedQPdataL1(	DenseCP& _denseCPrelaxed
															) const
{
	return ACADOERROR( RET_NOT_YET_IMPLEMENTED );
	
	uint nV = denseCP.getNV();
	uint nC = denseCP.getNC();


	/* allocate memory for relaxed QP data */
	_denseCPrelaxed.H.init( nV+nC,nV+nC );
	_denseCPrelaxed.A.init( nC,nV+nC );
	_denseCPrelaxed.g.init( nV+nC );
	_denseCPrelaxed.lb.init( nV+nC );
	_denseCPrelaxed.ub.init( nV+nC );
	_denseCPrelaxed.lbA = denseCP.lbA;
	_denseCPrelaxed.ubA = denseCP.ubA;

// 	acadoPrintf( "frob:  %e\n", H.getNorm( MN_FROBENIUS ) );
// 	acadoPrintf( "trace: %e\n", H.getTrace() );
// 	acadoPrintf( "min diag: %e\n", (H.getDiag()).getMin() );
// 	acadoPrintf( "max diag: %e\n", (H.getDiag()).getMax() );
// 	acadoPrintf( "max(abs(g)): %e\n", (g.getAbsolute()).getMax() );

	_denseCPrelaxed.H.setZero( );
	for( uint i=0; i<nV; ++i )
		for( uint j=0; j<nV; ++j )
			_denseCPrelaxed.H(i,j) = denseCP.H(i,j);

	_denseCPrelaxed.A.setZero( );
	for( uint i=0; i<nC; ++i )
		for( uint j=0; j<nV; ++j )
			_denseCPrelaxed.A(i,j) = denseCP.A(i,j);
	for( uint i=0; i<nC; ++i )
		_denseCPrelaxed.A(i,nV+i) = 1.0;

	_denseCPrelaxed.g.setZero( );
	for( uint i=0; i<nV; ++i )
		_denseCPrelaxed.g(i) = denseCP.g(i);
	double l1entry = 1.0e5 * ((denseCP.g).getAbsolute()).getMax( );
	for( uint i=nV; i<nV+nC; ++i )
		_denseCPrelaxed.g(i) = l1entry;

	_denseCPrelaxed.lb.setZero( );
	for( uint i=0; i<nV; ++i )
		_denseCPrelaxed.lb(i) = denseCP.lb(i);
	for( uint i=nV; i<nV+nC; ++i )
		_denseCPrelaxed.lb(i) = 0.0;

	_denseCPrelaxed.ub.setZero( );
	for( uint i=0; i<nV; ++i )
		_denseCPrelaxed.ub(i) = denseCP.ub(i);
	for( uint i=nV; i<nV+nC; ++i )
		_denseCPrelaxed.ub(i) =  INFTY;

	return SUCCESSFUL_RETURN;
}


returnValue CondensingBasedCPsolver::setupRelaxedQPdataL2(	DenseCP& _denseCPrelaxed
															) const
{
	uint nV = denseCP.getNV();
	uint nC = denseCP.getNC();


	/* allocate memory for relaxed QP data */
	_denseCPrelaxed.H.init( nV+nC,nV+nC );
	_denseCPrelaxed.A.init( nC,nV+nC );
	_denseCPrelaxed.g.init( nV+nC );
	_denseCPrelaxed.lb.init( nV+nC );
	_denseCPrelaxed.ub.init( nV+nC );
	_denseCPrelaxed.lbA = denseCP.lbA;
	_denseCPrelaxed.ubA = denseCP.ubA;

// 	acadoPrintf( "frob:  %e\n", H.getNorm( MN_FROBENIUS ) );
// 	acadoPrintf( "trace: %e\n", H.getTrace() );
// 	acadoPrintf( "min diag: %e\n", (H.getDiag()).getMin() );
// 	acadoPrintf( "max diag: %e\n", (H.getDiag()).getMax() );
// 	acadoPrintf( "max(abs(g)): %e\n", (g.getAbsolute()).getMax() );

	_denseCPrelaxed.H.setZero( );
	for( uint i=0; i<nV; ++i )
		for( uint j=0; j<nV; ++j )
			_denseCPrelaxed.H(i,j) = denseCP.H(i,j);
	double diagonalEntry = 1.0e5 * acadoMax( (denseCP.H).getTrace( ),((denseCP.g).getAbsolute()).getMax( ) );
	for( uint i=nV; i<nV+nC; ++i )
		_denseCPrelaxed.H(i,i) = diagonalEntry;

	_denseCPrelaxed.A.setZero( );
	for( uint i=0; i<nC; ++i )
		for( uint j=0; j<nV; ++j )
			_denseCPrelaxed.A(i,j) = denseCP.A(i,j);
	for( uint i=0; i<nC; ++i )
		_denseCPrelaxed.A(i,nV+i) = 1.0;

	_denseCPrelaxed.g.setZero( );
	for( uint i=0; i<nV; ++i )
		_denseCPrelaxed.g(i) = denseCP.g(i);

	_denseCPrelaxed.lb.setZero( );
	for( uint i=0; i<nV; ++i )
		_denseCPrelaxed.lb(i) = denseCP.lb(i);
	for( uint i=nV; i<nV+nC; ++i )
		_denseCPrelaxed.lb(i) = -INFTY;

	_denseCPrelaxed.ub.setZero( );
	for( uint i=0; i<nV; ++i )
		_denseCPrelaxed.ub(i) = denseCP.ub(i);
	for( uint i=nV; i<nV+nC; ++i )
		_denseCPrelaxed.ub(i) = INFTY;

	return SUCCESSFUL_RETURN;
}



returnValue CondensingBasedCPsolver::initializeCPsolver(	InfeasibleQPhandling infeasibleQPhandling
															)
{
    denseCP.init( getNF(),getNA() );
	
	denseCP.g.init( getNF() );
	denseCP.lb.init( getNF() );
	denseCP.ub.init( getNF() );
	denseCP.lbA.init( getNA() );
	denseCP.ubA.init( getNA() );

	cpSolver->init( &denseCP );

	// setup relaxed QP
// 	uint nV_relaxed, nC_relaxed;
// 	getRelaxedQPdimensions( infeasibleQPhandling,nV_relaxed,nC_relaxed );
// 	cpSolverRelaxed->init( nV_relaxed,nC_relaxed );

	return SUCCESSFUL_RETURN;
}



returnValue CondensingBasedCPsolver::solveQP(	uint maxIter,
												InfeasibleQPhandling infeasibleQPhandling
												)
{
// 	denseCP.lb.print( "lb" );
// // 	denseCP.ub.print( "ub" );
// 	(denseCP.ub - denseCP.lb).print( "ub-lb" );
	
	if ( infeasibleQPhandling == IQH_UNDEFINED )
		return cpSolver->solve( &denseCP );
	

	/* relax QP... */
	DenseCP denseCPrelaxed;

	if ( setupRelaxedQPdata( infeasibleQPhandling,denseCPrelaxed ) != SUCCESSFUL_RETURN )
		return ACADOERROR( RET_COULD_NOT_RELAX_QP );

	/* ... and solve relaxed QP */
	if ( cpSolverRelaxed == 0 )
		cpSolverRelaxed = new QPsolver_qpOASES( userInteraction );

	if ( ( cpSolverRelaxed->getNumberOfVariables( ) != denseCPrelaxed.getNV() ) ||
	 	 ( cpSolverRelaxed->getNumberOfConstraints( ) != denseCPrelaxed.getNC() ) )
	{
		cpSolverRelaxed->init( denseCPrelaxed.getNV(),denseCPrelaxed.getNC() );
	}

    returnValue returnvalue;
    returnvalue = cpSolverRelaxed->solve( &denseCPrelaxed );

	if ( ( returnvalue != SUCCESSFUL_RETURN ) && ( returnvalue != RET_QP_SOLUTION_REACHED_LIMIT ) )
		return returnvalue;

	const uint nV = denseCP.getNV();
	const uint nC = denseCP.getNC();

	DVector deltaDenseTmp;
	DVector lambdaDenseTmp;

	DVector deltaDense (nV);
	DVector lambdaDense(nV+nC);

	cpSolverRelaxed->getPrimalSolution( deltaDenseTmp  );
	cpSolverRelaxed->getDualSolution  ( lambdaDenseTmp );

	for( uint i=0; i < nV; ++i )
		deltaDense(i) = deltaDenseTmp(i);

// 	double maxRelaxed = 0.0;
// 
// 	for( uint i=nV; i < nV+nC; ++i )
// 		if ( fabs(deltaDenseTmp(i)) > maxRelaxed )
// 			maxRelaxed = fabs(deltaDenseTmp(i));
// 	printf ("maxRelaxed: %e\n",maxRelaxed);
	
	for( uint i=0; i < nV; ++i )
		lambdaDense(i) = lambdaDenseTmp(i);
	for( uint i=0; i < nC; ++i )
		lambdaDense(nV+i) = lambdaDenseTmp(denseCPrelaxed.getNV()+i);

	// deltaDense.print( "xOpt" );
	denseCP.setQPsolution( deltaDense,lambdaDense );

    return returnvalue;
}




CLOSE_NAMESPACE_ACADO

// end of file.
