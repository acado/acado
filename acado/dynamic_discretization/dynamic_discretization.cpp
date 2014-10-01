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
 *    \file src/dynamic_discretization/dynamic_discretization.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *
 */


#include <acado/dynamic_discretization/dynamic_discretization.hpp>



BEGIN_NAMESPACE_ACADO




//
// PUBLIC MEMBER FUNCTIONS:
//

DynamicDiscretization::DynamicDiscretization()
                      :AlgorithmicBase( ){

    setupOptions( );
    setupLogging( );

    initializeVariables();
}



DynamicDiscretization::DynamicDiscretization( UserInteraction* _userInteraction )
                      :AlgorithmicBase( _userInteraction ){

    // setup options and loggings for stand-alone instances
    if ( _userInteraction == 0 ){

         setupOptions( );
         setupLogging( );
    }
    initializeVariables();
}


DynamicDiscretization::DynamicDiscretization( const DynamicDiscretization& arg ) 
					  : AlgorithmicBase( arg ){

    DynamicDiscretization::copy(arg);
}


DynamicDiscretization::~DynamicDiscretization( ){ }


DynamicDiscretization& DynamicDiscretization::operator=( const DynamicDiscretization& arg ){

    if ( this != &arg ){
        AlgorithmicBase::operator=(arg);
        DynamicDiscretization::copy(arg);
    }
    return *this;
}


returnValue DynamicDiscretization::setForwardSeed( const BlockMatrix &xSeed_,
                                                   const BlockMatrix &pSeed_,
                                                   const BlockMatrix &uSeed_,
                                                   const BlockMatrix &wSeed_  ){

    xSeed = xSeed_;
    pSeed = pSeed_;
    uSeed = uSeed_;
    wSeed = wSeed_;

    return SUCCESSFUL_RETURN;
}


returnValue DynamicDiscretization::setUnitForwardSeed(){

    BlockMatrix xSeed_( N, 1 );
    BlockMatrix pSeed_( N, 1 );
    BlockMatrix uSeed_( N, 1 );
    BlockMatrix wSeed_( N, 1 );

    int run1;
    for( run1 = 0; run1 < N; run1++ ){
        xSeed_.setIdentity( run1, 0, nx );
        pSeed_.setIdentity( run1, 0, np );
        uSeed_.setIdentity( run1, 0, nu );
        wSeed_.setIdentity( run1, 0, nw );
    }
    return setForwardSeed( xSeed_, pSeed_, uSeed_, wSeed_ );
}



returnValue DynamicDiscretization::setBackwardSeed( const BlockMatrix &seed ){

    bSeed = seed;
    return SUCCESSFUL_RETURN;
}


returnValue DynamicDiscretization::setUnitBackwardSeed(){

    BlockMatrix seed( 1, N );
    int run1;
    for( run1 = 0; run1 < N; run1++ )
        seed.setIdentity( 0, run1, nx );

    return setBackwardSeed( seed );
}



returnValue DynamicDiscretization::getResiduum( BlockMatrix &residuum_ ) const{

    int  run1;
    uint run2;

    residuum_.init( N, 1 );

    for( run1 = 0; run1 < N; run1++ ){
        DMatrix tmp( residuum.getNumValues(), 1 );
        for( run2 = 0; run2 < residuum.getNumValues(); run2++ )
                tmp( run2, 0 ) = residuum(run1,run2);
        residuum_.setDense(run1,0,tmp);
    }

    return SUCCESSFUL_RETURN;
}


returnValue DynamicDiscretization::getForwardSensitivities( BlockMatrix &D ) const{

    D = dForward;
    return SUCCESSFUL_RETURN;
}


returnValue DynamicDiscretization::getBackwardSensitivities( BlockMatrix &D ) const{

    D = dBackward;
    return SUCCESSFUL_RETURN;
}


returnValue DynamicDiscretization::deleteAllSeeds(){

    BlockMatrix empty;
    setBackwardSeed( empty );
    setForwardSeed( empty, empty, empty, empty );
    return SUCCESSFUL_RETURN;
}



//
// PROTECTED MEMBER FUNCTIONS:
//

returnValue DynamicDiscretization::setupOptions( ){

	// add integration options
	addOption( FREEZE_INTEGRATOR           , defaultFreezeIntegrator        );
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

	return SUCCESSFUL_RETURN;
}


returnValue DynamicDiscretization::setupLogging( )
{
    LogRecord tmp( LOG_AT_END );

    tmp.addItem( LOG_DIFFERENTIAL_STATES      );
    tmp.addItem( LOG_ALGEBRAIC_STATES         );
    tmp.addItem( LOG_PARAMETERS               );
    tmp.addItem( LOG_CONTROLS                 );
    tmp.addItem( LOG_DISTURBANCES             );
    tmp.addItem( LOG_INTERMEDIATE_STATES      );

    tmp.addItem( LOG_DISCRETIZATION_INTERVALS );
    tmp.addItem( LOG_STAGE_BREAK_POINTS       );

    outputLoggingIdx = addLogRecord( tmp );

	return SUCCESSFUL_RETURN;
}

void DynamicDiscretization::initializeVariables(){

    N                = 0        ;
    printLevel       = LOW      ;

    nx               = 0        ;
    na               = 0        ;
    np               = 0        ;
    nu               = 0        ;
    nw               = 0        ;
}

void DynamicDiscretization::copy( const DynamicDiscretization& arg ){

    N  = arg.N ;
    nx = arg.nx;
    na = arg.na;
    np = arg.np;
    nu = arg.nu;
    nw = arg.nw;

    unionGrid     = arg.unionGrid ;
    printLevel    = arg.printLevel;
    residuum      = arg.residuum  ;

    xSeed  = arg.xSeed ;
    pSeed  = arg.pSeed ;
    uSeed  = arg.uSeed ;
    wSeed  = arg.wSeed ;

    bSeed  = arg.bSeed ;

    dForward  = arg.dForward ;
    dBackward = arg.dBackward;
}


uint DynamicDiscretization::getNumEvaluationPoints() const{

    uint nEvaluationPoints = 0;
    uint nShoot = unionGrid.getNumIntervals();

    int plotResolution;
    get( PLOT_RESOLUTION,plotResolution );

    switch ( (PrintLevel) plotResolution ) {

         case LOW   : nEvaluationPoints = (uint)(10.0  / sqrt((double)nShoot) + 2.0); break;
         case MEDIUM: nEvaluationPoints = (uint)(30.0  / sqrt((double)nShoot) + 2.0); break;
         case HIGH  : nEvaluationPoints = (uint)(100.0 / sqrt((double)nShoot) + 2.0); break;
         default    : nEvaluationPoints = (uint)(10.0  / sqrt((double)nShoot) + 2.0); break;
    }
    return nEvaluationPoints;
}



CLOSE_NAMESPACE_ACADO

// end of file.
