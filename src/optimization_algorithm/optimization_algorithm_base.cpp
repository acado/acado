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
 *    \file src/optimization_algorithm/optimization_algorithm_base.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date 2009
 */


#include <acado/optimization_algorithm/optimization_algorithm_base.hpp>
#include <acado/ocp/ocp.hpp>

using namespace std;

BEGIN_NAMESPACE_ACADO



//
// PUBLIC MEMBER FUNCTIONS:
//


OptimizationAlgorithmBase::OptimizationAlgorithmBase( )
{
    ocp       = 0;
    nlpSolver = 0;

	userInit.allocateAll( );
}


OptimizationAlgorithmBase::OptimizationAlgorithmBase( const OCP& ocp_ )
{
    ocp = new OCP(ocp_);
    nlpSolver = 0;

	userInit.allocateAll( );
}


OptimizationAlgorithmBase::OptimizationAlgorithmBase( const OptimizationAlgorithmBase& arg )
{
    if( arg.ocp != 0 )        ocp       = new OCP(*arg.ocp)      ;
    else                      ocp       = 0                      ;

    if( arg.nlpSolver != 0 )  nlpSolver = arg.nlpSolver->clone() ;
    else                      nlpSolver = 0                      ;

	iter     = arg.iter;
	userInit = arg.userInit;
}


OptimizationAlgorithmBase::~OptimizationAlgorithmBase( )
{
	clear( );
}



OptimizationAlgorithmBase& OptimizationAlgorithmBase::operator=( const OptimizationAlgorithmBase& arg ){

    if( this != &arg )
	{
		clear( );

        if( arg.ocp != 0 )        ocp       = new OCP(*arg.ocp)      ;
        else                      ocp       = 0                      ;

        if( arg.nlpSolver != 0 )  nlpSolver = arg.nlpSolver->clone() ;
        else                      nlpSolver = 0                      ;

		iter    = arg.iter;
		userInit = arg.userInit;
    }
    return *this;
}



returnValue OptimizationAlgorithmBase::initializeDifferentialStates( const char* fileName , BooleanType autoinit)
{
	VariablesGrid tmp;
	tmp.read( fileName );
	
	if ( tmp.isEmpty() == BT_TRUE )
		return RET_FILE_CAN_NOT_BE_OPENED;
	
    return initializeDifferentialStates(tmp,autoinit);
}


returnValue OptimizationAlgorithmBase::initializeAlgebraicStates( const char* fileName , BooleanType autoinit)
{
	VariablesGrid tmp;
	tmp.read( fileName );
	
	if ( tmp.isEmpty() == BT_TRUE )
		return RET_FILE_CAN_NOT_BE_OPENED;
	
    return initializeAlgebraicStates(tmp,autoinit);
}


returnValue OptimizationAlgorithmBase::initializeParameters( const char* fileName)
{
	VariablesGrid tmp;
	tmp.read( fileName );
	
	if ( tmp.isEmpty() == BT_TRUE )
		return RET_FILE_CAN_NOT_BE_OPENED;
	
    return initializeParameters(tmp);
}


returnValue OptimizationAlgorithmBase::initializeControls( const char* fileName)
{
	VariablesGrid tmp;
	tmp.read( fileName );
	
	if ( tmp.isEmpty() == BT_TRUE )
		return RET_FILE_CAN_NOT_BE_OPENED;

    return initializeControls(tmp);
}


returnValue OptimizationAlgorithmBase::initializeDisturbances( const char* fileName)
{
	VariablesGrid tmp;
	tmp.read( fileName );
	
	if ( tmp.isEmpty() == BT_TRUE )
		return RET_FILE_CAN_NOT_BE_OPENED;
	
    return initializeDisturbances(tmp);
}



returnValue OptimizationAlgorithmBase::initializeDifferentialStates( const VariablesGrid &xd_init_  , BooleanType autoinit){

	if ( userInit.x != 0 ) delete userInit.x;
	userInit.x = new VariablesGrid( xd_init_ );
        if (autoinit == BT_TRUE)
		userInit.x->enableAutoInit();
        if (autoinit == BT_FALSE)
		userInit.x->disableAutoInit();

    return SUCCESSFUL_RETURN;
}


returnValue OptimizationAlgorithmBase::simulateStatesForInitialization( ){
    if( userInit.x  != 0 ) userInit.x ->enableAutoInit();
    if( userInit.xa != 0 ) userInit.xa->enableAutoInit();
    return SUCCESSFUL_RETURN;
}


returnValue OptimizationAlgorithmBase::initializeAlgebraicStates( const VariablesGrid &xa_init_  , BooleanType autoinit){

	if ( userInit.xa != 0 ) delete userInit.xa;
    userInit.xa = new VariablesGrid( xa_init_ );
        if (autoinit == BT_TRUE) 
		userInit.xa->enableAutoInit();
        if (autoinit == BT_FALSE){ 
		  
		userInit.xa->disableAutoInit();
		}
    return SUCCESSFUL_RETURN;
}


returnValue OptimizationAlgorithmBase::initializeParameters ( const VariablesGrid &p_init_ ){

	if ( userInit.p != 0 ) delete userInit.p;
    userInit.p = new VariablesGrid( p_init_ );
    return SUCCESSFUL_RETURN;
}


returnValue OptimizationAlgorithmBase::initializeControls( const VariablesGrid &u_init_){

	if ( userInit.u != 0 ) delete userInit.u;
    userInit.u = new VariablesGrid( u_init_ );
    return SUCCESSFUL_RETURN;
}


returnValue OptimizationAlgorithmBase::initializeDisturbances( const VariablesGrid &w_init_){

	if ( userInit.w != 0 ) delete userInit.w;
    userInit.w = new VariablesGrid( w_init_ );
    return SUCCESSFUL_RETURN;
}


returnValue OptimizationAlgorithmBase::getDifferentialStates( VariablesGrid &xd_ ) const{

    if( nlpSolver == 0 ) return ACADOWARNING( RET_MEMBER_NOT_INITIALISED );
    return nlpSolver->getDifferentialStates( xd_ );
}


returnValue OptimizationAlgorithmBase::getAlgebraicStates( VariablesGrid &xa_ ) const{

    if( nlpSolver == 0 ) return ACADOWARNING( RET_MEMBER_NOT_INITIALISED );
    return nlpSolver->getAlgebraicStates( xa_ );
}


returnValue OptimizationAlgorithmBase::getParameters( VariablesGrid &p_  ) const
{
	if( nlpSolver == 0 ) return ACADOWARNING( RET_MEMBER_NOT_INITIALISED );
	return nlpSolver->getParameters( p_ );
}


returnValue OptimizationAlgorithmBase::getParameters( DVector &p_  ) const
{
	if( nlpSolver == 0 ) return ACADOWARNING( RET_MEMBER_NOT_INITIALISED );

	VariablesGrid tmp;

	returnValue returnvalue = nlpSolver->getParameters( tmp );
	if ( returnvalue != SUCCESSFUL_RETURN )
		return returnvalue;

	p_ = tmp.getVector( 0 );

	return SUCCESSFUL_RETURN;
}


returnValue OptimizationAlgorithmBase::getControls( VariablesGrid &u_  ) const{

    if( nlpSolver == 0 ) return ACADOWARNING( RET_MEMBER_NOT_INITIALISED );
    return nlpSolver->getControls( u_ );
}


returnValue OptimizationAlgorithmBase::getDisturbances( VariablesGrid &w_  ) const{

    if( nlpSolver == 0 ) return ACADOWARNING( RET_MEMBER_NOT_INITIALISED );
    return nlpSolver->getDisturbances( w_ );
}


returnValue OptimizationAlgorithmBase::getDifferentialStates( const char* fileName ) const{

    returnValue returnvalue;
    if( nlpSolver == 0 ) return ACADOWARNING( RET_MEMBER_NOT_INITIALISED );

    VariablesGrid xx;
    returnvalue = nlpSolver->getDifferentialStates( xx );
    if( returnvalue != SUCCESSFUL_RETURN ) return returnvalue;

    xx.print( fileName );

    return SUCCESSFUL_RETURN;
}


returnValue OptimizationAlgorithmBase::getAlgebraicStates( const char* fileName ) const{

    returnValue returnvalue;
    if( nlpSolver == 0 ) return ACADOWARNING( RET_MEMBER_NOT_INITIALISED );

    VariablesGrid xx;
    returnvalue = nlpSolver->getAlgebraicStates( xx );
    if( returnvalue != SUCCESSFUL_RETURN ) return returnvalue;

    xx.print( fileName );

    return SUCCESSFUL_RETURN;
}


returnValue OptimizationAlgorithmBase::getParameters( const char* fileName ) const{

    returnValue returnvalue;
    if( nlpSolver == 0 ) return ACADOWARNING( RET_MEMBER_NOT_INITIALISED );

    VariablesGrid xx;
    returnvalue = nlpSolver->getParameters( xx );
    if( returnvalue != SUCCESSFUL_RETURN ) return returnvalue;

    xx.print( fileName );

    return SUCCESSFUL_RETURN;
}


returnValue OptimizationAlgorithmBase::getControls( const char* fileName ) const{

    returnValue returnvalue;
    if( nlpSolver == 0 ) return ACADOWARNING( RET_MEMBER_NOT_INITIALISED );

    VariablesGrid xx;
    returnvalue = nlpSolver->getControls( xx );
    if( returnvalue != SUCCESSFUL_RETURN ) return returnvalue;

    xx.print( fileName );

    return SUCCESSFUL_RETURN;
}


returnValue OptimizationAlgorithmBase::getDisturbances( const char* fileName ) const{

    returnValue returnvalue;
    if( nlpSolver == 0 ) return ACADOWARNING( RET_MEMBER_NOT_INITIALISED );

    VariablesGrid xx;
    returnvalue = nlpSolver->getDisturbances( xx );
    if( returnvalue != SUCCESSFUL_RETURN ) return returnvalue;

    xx.print( fileName );

    return SUCCESSFUL_RETURN;
}


double OptimizationAlgorithmBase::getObjectiveValue( const char* fileName ) const
{
	ofstream stream( fileName );
	stream << scientific << getObjectiveValue();
	stream.close();

    return SUCCESSFUL_RETURN;
}


double OptimizationAlgorithmBase::getObjectiveValue() const{

    if( nlpSolver == 0 ) return ACADOWARNING( RET_MEMBER_NOT_INITIALISED );
    return nlpSolver->getObjectiveValue();
}



returnValue OptimizationAlgorithmBase::getSensitivitiesX(	BlockMatrix& _sens
															) const
{
	return nlpSolver->getSensitivitiesX( _sens );
}


returnValue OptimizationAlgorithmBase::getSensitivitiesXA(	BlockMatrix& _sens
															) const
{
	return nlpSolver->getSensitivitiesXA( _sens );
}

returnValue OptimizationAlgorithmBase::getSensitivitiesP(	BlockMatrix& _sens
															) const
{
	return nlpSolver->getSensitivitiesP( _sens );
}


returnValue OptimizationAlgorithmBase::getSensitivitiesU(	BlockMatrix& _sens
															) const
{
	return nlpSolver->getSensitivitiesU( _sens );
}


returnValue OptimizationAlgorithmBase::getSensitivitiesW(	BlockMatrix& _sens
															) const
{
	return nlpSolver->getSensitivitiesW( _sens );
}



uint OptimizationAlgorithmBase::getNX( ) const
{
	return iter.getNX( );
}


uint OptimizationAlgorithmBase::getNXA( ) const
{
	return iter.getNXA( );
}


uint OptimizationAlgorithmBase::getNP( ) const
{
	return iter.getNP( );
}


uint OptimizationAlgorithmBase::getNU( ) const
{
	return iter.getNU( );
}


uint OptimizationAlgorithmBase::getNW( ) const
{
	return iter.getNW( );
}


double OptimizationAlgorithmBase::getStartTime( ) const
{
	if ( ocp != 0 )
		return ocp->getStartTime( );
	else
		return -INFTY;
}


double OptimizationAlgorithmBase::getEndTime( ) const
{
	if ( ocp != 0 )
		return ocp->getEndTime( );
	else
		return -INFTY;
}



//
// PROTECTED MEMBER FUNCTIONS:
//

returnValue OptimizationAlgorithmBase::clear( )
{
	if ( ocp != 0 )
	{
		delete ocp;
		ocp = 0;
	}

    if ( nlpSolver != 0 )
	{
		delete nlpSolver;
		nlpSolver = 0;
	}

    iter.clear( );

	userInit.clear( );
	userInit.allocateAll( );

	return SUCCESSFUL_RETURN;
}


returnValue OptimizationAlgorithmBase::init(	UserInteraction* _userIteraction
												)
{
    // EXTRACT INFORMATION PACKED IN THE DATA WRAPPER OCP:
    // ---------------------------------------------------
    Objective             *objective           ;
    DifferentialEquation **differentialEquation;
    Constraint            *constraint          ;

	Grid unionGrid;
	
	if ( extractOCPdata(	&objective,&differentialEquation,&constraint,
							unionGrid 
							) != SUCCESSFUL_RETURN )
		return ACADOERROR(RET_OPTALG_INIT_FAILED);

    // REFORMULATE THE OBJECTIVE IF NECESSARY:
    // ---------------------------------------

	if ( setupObjective(	objective,differentialEquation,constraint,
							unionGrid 
							) != SUCCESSFUL_RETURN )
		return ACADOERROR(RET_OPTALG_INIT_FAILED);

    // REFORMULATE THE CONSTRAINT IF NECESSARY:
    // ----------------------------------------

	if ( setupDifferentialEquation(	objective,differentialEquation,constraint,
									unionGrid 
									) != SUCCESSFUL_RETURN )
		return ACADOERROR(RET_OPTALG_INIT_FAILED);

    // DISCRETIZE THE DIFFERENTIAL EQUATION IF NECESSARY:
    // --------------------------------------------------

    DynamicDiscretization* dynamicDiscretization = 0;

	if ( setupDynamicDiscretization(	_userIteraction,
										objective,differentialEquation,constraint,
										unionGrid,
										&dynamicDiscretization
										) != SUCCESSFUL_RETURN )
		return ACADOERROR(RET_OPTALG_INIT_FAILED);

    // SETUP OF THE NLP SOLVER:
    // ------------------------
    if ( allocateNlpSolver( objective,dynamicDiscretization,constraint ) != SUCCESSFUL_RETURN )
		return ACADOERROR(RET_OPTALG_INIT_FAILED);
	
    // DETERMINE THE DIMENSIONS OF THE OPTIMIZATION VARIABLES:
    // -------------------------------------------------------

    uint nx  = 0;
    uint nxa = 0;
    uint np  = 0;
    uint nu  = 0;
    uint nw  = 0;

	if ( determineDimensions( objective,differentialEquation,constraint, nx,nxa,np,nu,nw ) != SUCCESSFUL_RETURN )
	{
		if( differentialEquation  != 0 ) delete differentialEquation[0];

		if( objective             != 0 ) delete   objective            ;
		if( differentialEquation  != 0 ) delete[] differentialEquation ;
		if( constraint            != 0 ) delete   constraint           ;
		if( dynamicDiscretization != 0 ) delete   dynamicDiscretization;

		return ACADOERROR(RET_OPTALG_INIT_FAILED);
	}
	
	if ( initializeOCPiterate( constraint,unionGrid,nx,nxa,np,nu,nw ) != SUCCESSFUL_RETURN )
	{
		if( differentialEquation != 0 ) delete differentialEquation[0];

		if( objective             != 0 ) delete   objective            ;
		if( differentialEquation  != 0 ) delete[] differentialEquation ;
		if( constraint            != 0 ) delete   constraint           ;
		if( dynamicDiscretization != 0 ) delete   dynamicDiscretization;

		return ACADOERROR(RET_OPTALG_INIT_FAILED);
	}

    // ELIMINATE EQUALITY BOUNDS: ??
    // --------------------------

    // changes the dimensions again !


    // DEFINE MULTIPLE SHOOTING NOTES
    // OR COLLOCATION NOTES IF REQUESTED:
    // ----------------------------------

       // ADAPT INITIALIZATION OF X AND XA ?

       if( iter.p  != 0 ) iter.p ->disableAutoInit();
       if( iter.u  != 0 ) iter.u ->disableAutoInit();
       if( iter.w  != 0 ) iter.w ->disableAutoInit();


    // (COLLOCATION NOT IMPLEMENTED YET)

// 	printf("before!!!\n");
// 	iter.print();

    // INITIALIZE THE NLP-ALGORITHM:
    // -----------------------------
	if ( initializeNlpSolver( iter ) != SUCCESSFUL_RETURN )
		return ACADOERROR( RET_OPTALG_INIT_FAILED );

// 	printf("after!!!\n");
// 	iter.print();
	
    // GIVE THE TEMPORARY MEMORY FREE:
    // -------------------------------
    if( differentialEquation != 0 ) delete differentialEquation[0];

    if( objective             != 0 ) delete   objective            ;
    if( differentialEquation  != 0 ) delete[] differentialEquation ;
    if( constraint            != 0 ) delete   constraint           ;
    if( dynamicDiscretization != 0 ) delete   dynamicDiscretization;

    return SUCCESSFUL_RETURN;
}


BooleanType OptimizationAlgorithmBase::isLinearQuadratic(	Objective *F,
															DynamicDiscretization *G,
															Constraint *H
															) const
{

    // CONVEXITY DETECTION DOES NOT WORK FOR /examples/michaelis_menten.txt
    // AUTOMATIC CONVEXITY DETECTION IS DISABLED AT THE MOMENT.
    // (PLEASE FIX !!!).

//     return BT_FALSE;

	// ENABLES AGAIN, BUT MENTIONED BUG IS PROBABLY STILL NOT FIXED

    if( F != 0 ) if ( F->isQuadratic( ) == BT_FALSE ) return BT_FALSE;
    if( G != 0 ) if ( G->isAffine   ( ) == BT_FALSE ) return BT_FALSE;
    if( H != 0 ) if ( H->isAffine   ( ) == BT_FALSE ) return BT_FALSE;
    return BT_TRUE;
}


returnValue OptimizationAlgorithmBase::extractOCPdata(	Objective** objective,
														DifferentialEquation*** differentialEquation,
														Constraint** constraint,
														Grid& unionGrid
														)
{

	returnValue returnvalue;

    // EXTRACT INFORMATION PACKED IN THE DATA WRAPPER OCP:
    // ---------------------------------------------------

    if( ocp->hasObjective() == BT_TRUE )  *objective = new Objective( );
    else                    return ACADOERROR( RET_NO_VALID_OBJECTIVE );

    if( ocp->hasDifferentialEquation() == BT_TRUE ){
        *differentialEquation = new DifferentialEquation*[1];
    }
    else *differentialEquation = 0;

    if( ocp->hasConstraint() == BT_TRUE )
	{
		*constraint = new Constraint( );
		ocp->getConstraint( **constraint );
	}
    else
	{
		// CONSTRUCT A CONSTRAINT IF NOT ALLOCATED YET:
		// --------------------------------------------
		*constraint = new Constraint(); constraint[0]->init( unionGrid );
	}

    ocp->getObjective( **objective );
    returnvalue = initializeObjective( *objective );

    if( returnvalue != SUCCESSFUL_RETURN ) return ACADOERROR(returnvalue);

    if( ocp->hasDifferentialEquation() == BT_TRUE ){

        differentialEquation[0][0] = new DifferentialEquation();
        ocp->getModel( *differentialEquation[0][0] );
    }


    // OBTAIN THE UNION GRID:
    // -------------------------
    ocp->getGrid( unionGrid );

	return SUCCESSFUL_RETURN;
}


returnValue OptimizationAlgorithmBase::setupObjective(	Objective* objective,
														DifferentialEquation** differentialEquation,
														Constraint* constraint,
														Grid unionGrid
														)
{
	// note that even if the differentialEquation == 0
	// is satisfied it might still be that the objective
	// allocates a differentialEquation (e.g. for the case
	// that the objective is an integral expression.)
    return objective->init( 1,0,differentialEquation,0,constraint );
}


returnValue OptimizationAlgorithmBase::setupDifferentialEquation(	Objective* objective,
																	DifferentialEquation** differentialEquation,
																	Constraint* constraint,
																	Grid unionGrid
																	)
{
	if( differentialEquation != 0 ){
		if( differentialEquation[0]->getNumAlgebraicEquations() != 0 ){
			constraint->add( unionGrid.getNumPoints(), *differentialEquation[0] );
		}
	}

	return SUCCESSFUL_RETURN;
}


returnValue OptimizationAlgorithmBase::setupDynamicDiscretization(	UserInteraction* _userIteraction,
																	Objective* objective,
																	DifferentialEquation** differentialEquation,
																	Constraint* constraint,
																	Grid unionGrid,
																	DynamicDiscretization** dynamicDiscretization
																	)
{

    if( differentialEquation != 0 ){

        *dynamicDiscretization = new ShootingMethod( _userIteraction );

        int intType;
        _userIteraction->get( INTEGRATOR_TYPE, intType );

        if( differentialEquation[0]->getNumAlgebraicEquations() != 0 ) intType = (int) INT_BDF     ;
        if( differentialEquation[0]->isImplicit()    == BT_TRUE      ) intType = (int) INT_BDF     ;
        if( differentialEquation[0]->isDiscretized() == BT_TRUE      ) intType = (int) INT_DISCRETE;

        int sensType;
        (*dynamicDiscretization)->get( DYNAMIC_SENSITIVITY, sensType );

        if( differentialEquation[0]->isSymbolic() == BT_FALSE && sensType == (int) BACKWARD_SENSITIVITY ){
            (*dynamicDiscretization)->set( DYNAMIC_SENSITIVITY,  FORWARD_SENSITIVITY );
        }

        (*dynamicDiscretization)->addStage( *differentialEquation[0],
                                            unionGrid,
                                            (IntegratorType)intType );
    }

	return SUCCESSFUL_RETURN;
}


returnValue OptimizationAlgorithmBase::determineDimensions(	Objective* const _objective,
															DifferentialEquation** const _differentialEquation,
															Constraint* const _constraint,
															uint& _nx,
															uint& _nxa,
															uint& _np,
															uint& _nu,
															uint& _nw
															) const
{
	if( _objective != 0 )
	{
        _nx  = acadoMax( _objective->getNX (), _nx  );
        _nxa = acadoMax( _objective->getNXA(), _nxa );
        _np  = acadoMax( _objective->getNP (), _np  );
        _nu  = acadoMax( _objective->getNU (), _nu  );
        _nw  = acadoMax( _objective->getNW (), _nw  );
    }
    if( _differentialEquation != 0 )
	{
        _nx  = acadoMax( _differentialEquation[0]->getNX() , _nx  );
        _nxa = acadoMax( _differentialEquation[0]->getNXA(), _nxa );
        _np  = acadoMax( _differentialEquation[0]->getNP (), _np  );
        _nu  = acadoMax( _differentialEquation[0]->getNU (), _nu  );
        _nw  = acadoMax( _differentialEquation[0]->getNW (), _nw  );
    }
    if( _constraint != 0 )
	{
        _nx  = acadoMax( _constraint->getNX (), _nx  );
        _nxa = acadoMax( _constraint->getNXA(), _nxa );
        _np  = acadoMax( _constraint->getNP (), _np  );
        _nu  = acadoMax( _constraint->getNU (), _nu  );
        _nw  = acadoMax( _constraint->getNW (), _nw  );
    }

    if( _differentialEquation == 0 )
	{
        if( _nx > 0 )
			return RET_INCOMPATIBLE_DIMENSIONS;
    }
    else
	{
        _nx = _differentialEquation[0]->getNumDynamicEquations();
//         if( nxa != (uint) differentialEquation[0]->getNumAlgebraicEquations()){
// 
//             for( run1 = 0; run1 < numberOfStages; run1++ )
//                  if( differentialEquation != 0 ) delete differentialEquation[run1];
// 
//             if( objective             != 0 ) delete   objective            ;
//             if( differentialEquation  != 0 ) delete[] differentialEquation ;
//             if( transitions           != 0 ) delete[] transitions          ;
//             if( positions             != 0 ) delete[] positions            ;
//             if( constraint            != 0 ) delete   constraint           ;
//             if( dynamicDiscretization != 0 ) delete   dynamicDiscretization;
// 
//             return ACADOERROR(RET_INCOMPATIBLE_DIMENSIONS);
//         }
    }
	
	return SUCCESSFUL_RETURN;
}



returnValue OptimizationAlgorithmBase::initializeOCPiterate(	Constraint* const _constraint,
																const Grid& _unionGrid,
																uint nx,
																uint nxa,
																uint np,
																uint nu,
																uint nw
																)
{
  
	uint run1, run2;

    // CONSTRUCT THE OPTIMIZATION VARIABLES:
    // -------------------------------------

    iter.clear( );

    if( nx  > 0 ) iter.x  = new VariablesGrid( nx , _unionGrid, VT_DIFFERENTIAL_STATE );
    if( nxa > 0 ) iter.xa = new VariablesGrid( nxa, _unionGrid, VT_ALGEBRAIC_STATE );
    if( np  > 0 ) iter.p  = new VariablesGrid( np , _unionGrid, VT_PARAMETER );
    if( nu  > 0 ) iter.u  = new VariablesGrid( nu , _unionGrid, VT_CONTROL );
    if( nw  > 0 ) iter.w  = new VariablesGrid( nw , _unionGrid, VT_DISTURBANCE );

    // CHECK OF BOUND CONSISTENCY:
    // ---------------------------

    if( _constraint != 0 ){

        // the following routine initializes the bounds of the optimization
        // variables. If no bound is given, a lower bound will be initialized
        // with -INFTY while upper bounds are initialized with +INFTY. Note
        // that the class Constraint auto-detects bounds. Moreover, if there
        // are inconsistent bounds detected, i.e. if there is an upper bound
        // smaller that the corresponding lower bound, the routine
        // getBounds(...) will return with a corresponding error message which
        // should be captured and returned to the user.

        returnValue returnvalue = _constraint->getBounds( iter );
        if( returnvalue != SUCCESSFUL_RETURN ){
            return ACADOERROR(returnvalue);
        }
    }

    // LOAD A ROUGH INITIALIZATION FOR ALL VARIABLES FOR THE CASE
    // THAT NO FURTHER INITIALIZATION IS AVAILABLE:
    // ----------------------------------------------------------

    if( iter.x  != 0 ) iter.x ->initializeFromBounds();   // inititializes the variables with 0 if no bounds are
    if( iter.xa != 0 ) iter.xa->initializeFromBounds();   // given. if there is one bound specified this bound
    if( iter.p  != 0 ) iter.p ->initializeFromBounds();   // will be used as an initialization. If there is an
    if( iter.u  != 0 ) iter.u ->initializeFromBounds();   // upper as well as a lower bound specified, then the
    if( iter.w  != 0 ) iter.w ->initializeFromBounds();   // average, i.e. (lb+ub)/2, will be used as an initialization.

//     printf("OptimizAlg::1 nx = %d \n", nx );
//     printf("OptimizAlg::1 np = %d \n", np );
//     printf("OptimizAlg::1 nu = %d \n", nu );
	
    // LOAD THE GIVEN INITIALIZATION:
    // ------------------------------

    // For the case that the user provides an initialization for one or
    // the other variable this additional information from the user should
    // be taken into account. The following routine reads the user data in a
    // robust way. (Even if the user provides data with wrong dimensions
    // the code tries to use it as reasonable as possible.)

    // Added: Copy auto_initialize-values of user initialization

// 	iter.x->print("iter.x");
// 	userInit.x->print("userInit.x");

    BooleanType ai;
    if( nx > 0 && userInit.x->getNumPoints() > 0 ){
        ai=userInit.x->getAutoInit(0);
        for( run1 = 0; run1 < _unionGrid.getNumPoints(); run1++ ){
            DVector tmp = userInit.x->linearInterpolation( _unionGrid.getTime(run1) );
            uint nxx = tmp.getDim();
            if( nxx > nx ) nxx = nx;
            for( run2 = 0; run2 < nxx; run2++ )
                iter.x->operator()( run1, run2 ) = tmp(run2);
	        iter.x->setAutoInit(run1,ai);
        }
    }
    
    if( nxa > 0 && userInit.xa->getNumPoints() > 0 ){
        ai=userInit.xa->getAutoInit(0);
        for( run1 = 0; run1 < _unionGrid.getNumPoints(); run1++ ){
            DVector tmp = userInit.xa->linearInterpolation( _unionGrid.getTime(run1) );
			uint nxx = tmp.getDim();
            if( nxx > nxa ) nxx = nxa;
            for( run2 = 0; run2 < nxx; run2++ )
                iter.xa->operator()( run1, run2 ) = tmp(run2);
			iter.xa->setAutoInit(run1,ai);
        }
    }	
	
    if( nu > 0 && userInit.u->getNumPoints() > 0 ){
        for( run1 = 0; run1 < _unionGrid.getNumPoints(); run1++ ){
            DVector tmp = userInit.u->linearInterpolation( _unionGrid.getTime(run1) );
            uint nxx = tmp.getDim();
            if( nxx > nu ) nxx = nu;
            for( run2 = 0; run2 < nxx; run2++ )
                iter.u->operator()( run1, run2 ) = tmp(run2);
        }
    }


    // MAKE SURE THAT THE PARAMETER IS TIME-CONSTANT:
    // ----------------------------------------------
    
	if( np > 0 && userInit.p->getNumPoints() > 0 ){
		iter.p->setAllVectors( userInit.p->getVector(0) );
	}

//     if( np > 0 && userInit.p->getNumPoints() > 0 ){
//         for( run1 = 0; run1 < _unionGrid.getNumPoints(); run1++ ){
//             DVector tmp = userInit.p->getFirstVector( );
//             uint nxx = tmp.getDim();
//             if( nxx > np ) nxx = np;
//             for( run2 = 0; run2 < nxx; run2++ )
//                 iter.p->operator()( run1, run2 ) = tmp(run2);
//         }
//     }

    if( nw > 0 && userInit.w->getNumPoints() > 0 ){
        for( run1 = 0; run1 < _unionGrid.getNumPoints(); run1++ ){
            DVector tmp = userInit.w->linearInterpolation( _unionGrid.getTime(run1) );
            uint nxx = tmp.getDim();
            if( nxx > nw ) nxx = nw;
            for( run2 = 0; run2 < nxx; run2++ )
                iter.w->operator()( run1, run2 ) = tmp(run2);
        }
    }


	return SUCCESSFUL_RETURN;
}



CLOSE_NAMESPACE_ACADO


// end of file.
