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
 *    \file src/nlp_solver/scp_evaluation.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *
 */


#include <acado/nlp_solver/scp_evaluation.hpp>



BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

SCPevaluation::SCPevaluation( ) : AlgorithmicBase( )
{
	setupOptions( );
	setupLogging( );

	objective             = 0;
	dynamicDiscretization = 0;
	constraint            = 0;

	objectiveValue = 0.0;
	
	isCP = BT_FALSE;
	areSensitivitiesFrozen = BT_FALSE;
}


SCPevaluation::SCPevaluation(	UserInteraction* _userInteraction,
								const Objective* const objective_,
								const DynamicDiscretization* const dynamic_discretization_,
								const Constraint* const constraint_,
								BooleanType _isCP
								) : AlgorithmicBase( _userInteraction )
{
	// setup options and loggings for stand-alone instances
	if ( _userInteraction == 0 )
	{
		setupOptions( );
		setupLogging( );
	}

	if( objective_              != 0 ) objective = new Objective( *objective_ );
    else                               objective = 0;

    if( dynamic_discretization_ != 0 ) dynamicDiscretization = dynamic_discretization_->clone();
    else                               dynamicDiscretization = 0;

    if( constraint_             != 0 ) constraint = new Constraint( *constraint_ );
    else                               constraint = 0;

    objectiveValue = 0.0;
	
	isCP = _isCP;
	areSensitivitiesFrozen = BT_FALSE;
}


SCPevaluation::SCPevaluation( const SCPevaluation& rhs ) : AlgorithmicBase( rhs )
{
    if( rhs.objective != 0             ) objective = new Objective( *rhs.objective );
    else                                 objective = 0;

    if( rhs.dynamicDiscretization != 0 ) dynamicDiscretization = rhs.dynamicDiscretization->clone();
    else                                 dynamicDiscretization = 0;

    if( rhs.constraint != 0            ) constraint = new Constraint( *rhs.constraint );
    else                                 constraint = 0;

    objectiveValue = rhs.objectiveValue;
	
	isCP = rhs.isCP;
	areSensitivitiesFrozen = rhs.areSensitivitiesFrozen;
}


SCPevaluation::~SCPevaluation( )
{
    if( objective             != 0 ) delete objective;
    if( dynamicDiscretization != 0 ) delete dynamicDiscretization;
    if( constraint            != 0 ) delete constraint;
}


SCPevaluation& SCPevaluation::operator=( const SCPevaluation& rhs )
{
	if ( this != &rhs )
	{
    	if( objective             != 0 ) delete objective;
    	if( dynamicDiscretization != 0 ) delete dynamicDiscretization;
    	if( constraint            != 0 ) delete constraint;

		AlgorithmicBase::operator=( rhs );

        if( rhs.objective != 0             ) objective = new Objective( *rhs.objective );
        else                                 objective = 0;

        if( rhs.dynamicDiscretization != 0 ) dynamicDiscretization = rhs.dynamicDiscretization->clone( );
        else                                 dynamicDiscretization = 0;

        if( rhs.constraint != 0            ) constraint = new Constraint( *rhs.constraint );
        else                                 constraint = 0;

        objectiveValue = rhs.objectiveValue;

		isCP = rhs.isCP;
		areSensitivitiesFrozen = rhs.areSensitivitiesFrozen;
	}

    return *this;
}


SCPevaluation* SCPevaluation::clone( ) const
{
	return new SCPevaluation( *this );
}



returnValue SCPevaluation::init(	const OCPiterate& iter
									){

	return SUCCESSFUL_RETURN;
}



returnValue SCPevaluation::evaluate( OCPiterate& iter, BandedCP& cp ){

    // EVALUATE THE OBJECTIVE AND CONSTRAINTS:
    // ---------------------------------------
    if( dynamicDiscretization != 0 )
       ACADO_TRY( dynamicDiscretization->evaluate( iter ) ).changeType( RET_UNABLE_TO_INTEGRATE_SYSTEM );

    if( constraint != 0 )
        ACADO_TRY( constraint->evaluate( iter ) ).changeType( RET_UNABLE_TO_EVALUATE_CONSTRAINTS );

    ACADO_TRY( objective->evaluate(iter) ).changeType( RET_UNABLE_TO_EVALUATE_OBJECTIVE );


    objective->getObjectiveValue( objectiveValue );

    if( dynamicDiscretization != 0 )
        dynamicDiscretization->getResiduum( cp.dynResiduum );

    if( constraint != 0 )
    {
        constraint->getBoundResiduum     ( cp.lowerBoundResiduum     , cp.upperBoundResiduum      );
        constraint->getConstraintResiduum( cp.lowerConstraintResiduum, cp.upperConstraintResiduum );
    }

    if( dynamicDiscretization == 0 ){
        cp.lowerBoundResiduum.setZero(0,0);
        cp.lowerBoundResiduum.setZero(1,0);
        cp.lowerBoundResiduum.setZero(3,0);
        cp.lowerBoundResiduum.setZero(4,0);
        cp.upperBoundResiduum.setZero(0,0);
        cp.upperBoundResiduum.setZero(1,0);
        cp.upperBoundResiduum.setZero(3,0);
        cp.upperBoundResiduum.setZero(4,0);
    }

    return SUCCESSFUL_RETURN;
}



returnValue SCPevaluation::evaluateSensitivities(	const OCPiterate& iter,
        											BandedCP& cp
        											)
{
	if ( areSensitivitiesFrozen == BT_TRUE )
		return SUCCESSFUL_RETURN;


    // DETERMINE THE HESSIAN APPROXIMATION MODE:
    // -----------------------------------------
    int hessMode;
    get( HESSIAN_APPROXIMATION, hessMode );

    int dynHessMode;
    get( DYNAMIC_HESSIAN_APPROXIMATION, dynHessMode );
	if ( (HessianApproximationMode)dynHessMode == DEFAULT_HESSIAN_APPROXIMATION )
		dynHessMode = hessMode;

    int dynMode;
    get( DYNAMIC_SENSITIVITY, dynMode );

    int objMode;
	get( OBJECTIVE_SENSITIVITY, objMode );

    int conMode;
    get( CONSTRAINT_SENSITIVITY, conMode );


    // COMPUTE THE 1st ORDER DERIVATIVES:
    // ----------------------------------

    objective->setUnitBackwardSeed( );
    if( ( (HessianApproximationMode)hessMode == GAUSS_NEWTON ) || ( (HessianApproximationMode)hessMode == GAUSS_NEWTON_WITH_BLOCK_BFGS ) )
           objective->evaluateSensitivitiesGN( cp.hessian );
    else{
        if( (HessianApproximationMode)hessMode == EXACT_HESSIAN ){
            cp.hessian.setZero();
            objective->evaluateSensitivities( cp.hessian );
        }
        else objective->evaluateSensitivities();
    }


    objective->getBackwardSensitivities( cp.objectiveGradient, 1 );

	
// 	printf("cp.hessian = \n");
//     cp.hessian.print();

    if( dynamicDiscretization != 0 ){

        if( (HessianApproximationMode)dynHessMode == EXACT_HESSIAN ){

            ACADO_TRY( dynamicDiscretization->setUnitForwardSeed()                                  );
            ACADO_TRY( dynamicDiscretization->evaluateSensitivities( cp.lambdaDynamic, cp.hessian ) );
            ACADO_TRY( dynamicDiscretization->getForwardSensitivities( cp.dynGradient )             );
        }
        else{
            if( dynMode == BACKWARD_SENSITIVITY ){

                ACADO_TRY( dynamicDiscretization->setUnitBackwardSeed()                      );
                ACADO_TRY( dynamicDiscretization->evaluateSensitivities()                    );
                ACADO_TRY( dynamicDiscretization->getBackwardSensitivities( cp.dynGradient ) );
            }
            if( dynMode == FORWARD_SENSITIVITY ){

                ACADO_TRY( dynamicDiscretization->setUnitForwardSeed()                      );
                ACADO_TRY( dynamicDiscretization->evaluateSensitivities()                   );
                ACADO_TRY( dynamicDiscretization->getForwardSensitivities( cp.dynGradient ) );
            }
            if( dynMode == FORWARD_SENSITIVITY_LIFTED ){

                ACADO_TRY( dynamicDiscretization->setUnitForwardSeed()                      );
                ACADO_TRY( dynamicDiscretization->evaluateSensitivitiesLifted()             );
                ACADO_TRY( dynamicDiscretization->getForwardSensitivities( cp.dynGradient ) );
            }
        }
    }

// 	printf("cp.dynGradient = \n");
// (cp.dynGradient).print();

    if( constraint != 0 ){

        if( (HessianApproximationMode)hessMode == EXACT_HESSIAN ){

            constraint->setUnitBackwardSeed();
            constraint->evaluateSensitivities( cp.lambdaConstraint, cp.hessian );
            constraint->getBackwardSensitivities( cp.constraintGradient, 1 );
        }
        else{
            if( conMode == BACKWARD_SENSITIVITY ){
                constraint->setUnitBackwardSeed();
                constraint->evaluateSensitivities( );
                constraint->getBackwardSensitivities( cp.constraintGradient, 1 );
            }
            if( conMode == FORWARD_SENSITIVITY ){
                constraint->setUnitForwardSeed();
                constraint->evaluateSensitivities( );
                constraint->getForwardSensitivities( cp.constraintGradient, 1 );
            }
        }
    }

    return SUCCESSFUL_RETURN;
}



returnValue SCPevaluation::evaluateLagrangeGradient(	uint N,
														const OCPiterate& iter,
        												const BandedCP& cp,
        												BlockMatrix &nablaL
        												)
{
    uint run1;
    DMatrix tmp1, tmp2;
    BlockMatrix aux( 5*N, 1 );

    for( run1 = 0; run1 < N-1; run1++ ){

        if( run1 < N-1 ) cp.lambdaDynamic.getSubBlock( run1, 0, tmp1, iter.getNX(), 1 );

        if( iter.getNX() != 0 ){
            cp.dynGradient.getSubBlock( run1, 0, tmp2, iter.getNX(), iter.getNX() );
            aux.addDense( run1  , 0, tmp2.transpose() * tmp1             );
            aux.setDense( run1+1, 0, -tmp1          );
        }
        if( iter.getNXA() != 0 ){
            cp.dynGradient.getSubBlock( run1, 1, tmp2, iter.getNX(), iter.getNXA() );
            aux.setDense( N+run1, 0, tmp2.transpose() * tmp1 );
        }
        if( iter.getNP() != 0 ){
            cp.dynGradient.getSubBlock( run1, 2, tmp2, iter.getNX(), iter.getNP() );
            aux.setDense( 2*N+run1, 0, tmp2.transpose() * tmp1 );
        }
        if( iter.getNU() != 0 ){
            cp.dynGradient.getSubBlock( run1, 3, tmp2, iter.getNX(), iter.getNU() );
            aux.setDense( 3*N+run1, 0, tmp2.transpose() * tmp1 );
        }
        if( iter.getNW() != 0 ){
            cp.dynGradient.getSubBlock( run1, 4, tmp2, iter.getNX(), iter.getNW() );
            aux.setDense( 4*N+run1, 0, tmp2.transpose() * tmp1 );
        }
    }

//     printf("aux = \n");
//     aux.print();
//
//    cp.objectiveGradient.print();
//     (cp.objectiveGradient.transpose()).print();
//     lambdaBound.print();
//     (cp.constraintGradient^cp.lambdaConstraint).print();

    if( dynamicDiscretization != 0 ){

        BlockMatrix lambdaBoundExpand(5*N,1);
        for( run1 = 0; run1 < N; run1++ ){

            cp.lambdaBound.getSubBlock(run1,0,tmp1);
            if( tmp1.getDim() != 0 )
                lambdaBoundExpand.setDense(run1,0,tmp1);

            cp.lambdaBound.getSubBlock(N+run1,0,tmp1);
            if( tmp1.getDim() != 0 )
                lambdaBoundExpand.setDense(N+run1,0,tmp1);

            cp.lambdaBound.getSubBlock(2*N,0,tmp1);
            if( tmp1.getDim() != 0 )
                lambdaBoundExpand.setDense(2*N+run1,0,tmp1);

            cp.lambdaBound.getSubBlock(2*N+1+run1,0,tmp1);
            if( tmp1.getDim() != 0 )
                lambdaBoundExpand.setDense(3*N+run1,0,tmp1);

            cp.lambdaBound.getSubBlock(3*N+1+run1,0,tmp1);
            if( tmp1.getDim() != 0 )
                lambdaBoundExpand.setDense(4*N+run1,0,tmp1);
        }

        nablaL  = cp.objectiveGradient.transpose() - lambdaBoundExpand - (cp.constraintGradient^cp.lambdaConstraint) + aux;
    }
    else{
        BlockMatrix lambdaBoundExpand(5,1);
        cp.lambdaBound.getSubBlock(1,0,tmp1);
        if( tmp1.getDim() != 0 )
             lambdaBoundExpand.setDense(2,0,tmp1);
        nablaL  = cp.objectiveGradient.transpose() - lambdaBoundExpand - (cp.constraintGradient^cp.lambdaConstraint);
    }

    return SUCCESSFUL_RETURN;
}



double SCPevaluation::getKKTtolerance(	const OCPiterate& iter,
        								const BandedCP& cp,
        								double KKTmultiplierRegularisation
        								)
{

//     printf("get KKT tolerance. \n \n");

    double KKTtol = 0.0;
	double eps = 0.0;

    DMatrix tmp;

//     printf("obj Gradient \n");
//     cp.objectiveGradient.print();
// 
//     printf("cp.deltaX \n");
//     cp.deltaX.print();

	int hessianApproximation;
	get( HESSIAN_APPROXIMATION,hessianApproximation );

	if ( ( isCP == BT_FALSE ) || 
		 ( ( (HessianApproximationMode)hessianApproximation != GAUSS_NEWTON ) && ( (HessianApproximationMode)hessianApproximation != GAUSS_NEWTON_WITH_BLOCK_BFGS ) )
		 )
	{
		eps = KKTmultiplierRegularisation;

		(cp.objectiveGradient*cp.deltaX).getSubBlock( 0, 0, tmp, 1, 1 );
// 		(cp.objectiveGradient*cp.deltaX).print();
		KKTtol = fabs(tmp(0,0));
	}

    // --------

//     printf("lambda Dynamic \n");
//     cp.lambdaDynamic.print();
// 
//     printf("dynamic residuum \n");
//     cp.dynResiduum.print();
//       printf("interm. = %.16e \n", KKTtol );


    if( dynamicDiscretization != 0 )
    {
        ( (cp.lambdaDynamic.getAbsolute()).addRegularisation(eps)^cp.dynResiduum.getAbsolute()).getSubBlock( 0, 0, tmp, 1, 1 );
        KKTtol += tmp(0,0);
    }

    // --------

//     printf("cp.lowerBoundResiduum \n");
//     cp.lowerBoundResiduum.print();
// 
//     printf("cp.upperBoundResiduum \n");
//     cp.upperBoundResiduum.print();

//     printf("cp.lambdaBound \n");
//     cp.lambdaBound.print();
// 
//        printf("interm. = %.16e \n", KKTtol );

    ((cp.lambdaBound.getAbsolute()).addRegularisation(eps)^cp.upperBoundResiduum.getNegative()).getSubBlock( 0, 0, tmp, 1, 1 );
    KKTtol -= tmp(0,0);

    ((cp.lambdaBound.getAbsolute()).addRegularisation(eps)^cp.lowerBoundResiduum.getPositive()).getSubBlock( 0, 0, tmp, 1, 1 );
    KKTtol += tmp(0,0);

//
// //    --------
//
//     printf("cp.lowerConstraintResiduum \n");
//     cp.lowerConstraintResiduum.print();
// 
//     printf("cp.upperConstraintResiduum \n");
//     cp.upperConstraintResiduum.print();
// 
//     printf("cp.lambdaConstraint = \n");
//     cp.lambdaConstraint.print();
//       printf("interm. = %.16e \n", KKTtol );

    if( ( constraint != 0 ) || ( dynamicDiscretization != 0 ) )
    {
        ((cp.lambdaConstraint.getAbsolute()).addRegularisation(eps)^cp.upperConstraintResiduum.getNegative()).getSubBlock( 0, 0, tmp, 1, 1 );
        KKTtol -= tmp(0,0);

        ((cp.lambdaConstraint.getAbsolute()).addRegularisation(eps)^cp.lowerConstraintResiduum.getPositive()).getSubBlock( 0, 0, tmp, 1, 1 );
        KKTtol += tmp(0,0);
    }

//         printf("interm. = %.16e \n", KKTtol );

//ASSERT( 1 == 0 );

    return KKTtol;
}




double SCPevaluation::getObjectiveValue( ) const
{
    return objectiveValue;
}



returnValue SCPevaluation::freezeSensitivities( )
{
	areSensitivitiesFrozen = BT_TRUE;
	return SUCCESSFUL_RETURN;
}


returnValue SCPevaluation::unfreezeSensitivities( )
{
	areSensitivitiesFrozen = BT_FALSE;
	return SUCCESSFUL_RETURN;
}



returnValue SCPevaluation::setReference( const VariablesGrid &ref )
{
    if( objective == 0 )
    	return ACADOERROR(RET_MEMBER_NOT_INITIALISED);

// 	ref.print("ref");
	
    if( objective->setReference( ref ) != SUCCESSFUL_RETURN )
    	return ACADOERROR( RET_UNKNOWN_BUG );

    return SUCCESSFUL_RETURN;
}



returnValue SCPevaluation::clearDynamicDiscretization( )
{
	if( dynamicDiscretization != 0 )
	{
		dynamicDiscretization->unfreeze( );
		dynamicDiscretization->deleteAllSeeds( );
	}

	return SUCCESSFUL_RETURN;
}




//
// PROTECTED MEMBER FUNCTIONS:
//

returnValue SCPevaluation::setupOptions( )
{
	return SUCCESSFUL_RETURN;
}


returnValue SCPevaluation::setupLogging( )
{
	return SUCCESSFUL_RETURN;
}



CLOSE_NAMESPACE_ACADO

// end of file.
