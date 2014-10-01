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
 *    \file src/optimization_algorithm/mhe_algorithm.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date 2009
 */


#include <acado/optimization_algorithm/mhe_algorithm.hpp>


BEGIN_NAMESPACE_ACADO



//
// PUBLIC MEMBER FUNCTIONS:
//


MHEalgorithm::MHEalgorithm()
             :OptimizationAlgorithm(){

    eta = 0;
    S   = 0;

//  set( HESSIAN_APPROXIMATION, GAUSS_NEWTON );
//  set( USE_REALTIME_ITERATIONS,BT_TRUE );
//  set( MAX_NUM_ITERATIONS,1 );

}


MHEalgorithm::MHEalgorithm( const OCP& ocp_ )
             :OptimizationAlgorithm( ocp_ ){

    eta = 0;
    S   = 0;
}


MHEalgorithm::MHEalgorithm( const MHEalgorithm& arg )
             :OptimizationAlgorithm( arg ){

    if( arg.eta != 0 ) eta = new DVector(*arg.eta);
    else               eta = 0                   ;

    if( arg.S  != 0 )  S   = new DMatrix(*arg.S)  ;
    else               S   = 0                   ;
}


MHEalgorithm::~MHEalgorithm( ){

    if( eta  != 0 ) delete eta;
    if( S    != 0 ) delete S  ;
}


MHEalgorithm& MHEalgorithm::operator=( const MHEalgorithm& arg ){

    if( this != &arg ){

        if( eta  != 0 ) delete eta;
        if( S    != 0 ) delete S  ;

        OptimizationAlgorithm::operator=(arg);

        if( arg.eta != 0 ) eta = new DVector(*arg.eta);
        else               eta = 0                   ;

        if( arg.S  != 0 )  S   = new DMatrix(*arg.S)  ;
        else               S   = 0                   ;
    }
    return *this;
}


returnValue MHEalgorithm::init( const DVector &eta_, const DMatrix &S_ ){

    if( eta  != 0 ) delete eta;
    if( S    != 0 ) delete S  ;

    eta = new DVector(eta_);
    S   = new DMatrix(S_  );

    return OptimizationAlgorithm::init( );
}


returnValue MHEalgorithm::step( const DVector &eta_, const DMatrix &S_ ){

    return ACADOERROR(RET_NOT_IMPLEMENTED_YET);
}


returnValue MHEalgorithm::shift( ){

    return ACADOERROR(RET_NOT_IMPLEMENTED_YET);
}


returnValue MHEalgorithm::solve( const DVector &eta_, const DMatrix &S_ ){

    if ( status == BS_NOT_INITIALIZED ){
         if( init( eta_, S_ ) != SUCCESSFUL_RETURN )
             return ACADOERROR( RET_OPTALG_INIT_FAILED );
    }

    if ( status != BS_READY )
        return ACADOERROR( RET_OPTALG_INIT_FAILED );


    returnValue returnvalue = nlpSolver->solve( );

//     if( ( returnvalue != SUCCESSFUL_RETURN ) &&
//         ( returnvalue != CONVERGENCE_ACHIEVED ) &&
//         ( returnvalue != RET_MAX_NUMBER_OF_STEPS_EXCEEDED ) ) return ACADOERROR(returnvalue);

    return returnvalue;
}



returnValue MHEalgorithm::initializeNlpSolver( 	const OCPiterate& _userInit )
{
   return OptimizationAlgorithm::initializeNlpSolver( _userInit );
}


returnValue MHEalgorithm::initializeObjective(	Objective* F
												)
{
	return SUCCESSFUL_RETURN;
}



CLOSE_NAMESPACE_ACADO

// end of file.
