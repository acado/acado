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
 *    \file src/optimization_algorithm/parameter_estimation_algorithm.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date 2009
 */


#include <acado/optimization_algorithm/parameter_estimation_algorithm.hpp>



BEGIN_NAMESPACE_ACADO



//
// PUBLIC MEMBER FUNCTIONS:
//


ParameterEstimationAlgorithm::ParameterEstimationAlgorithm()
                             :OptimizationAlgorithm(){

    set( HESSIAN_APPROXIMATION, GAUSS_NEWTON );
}


ParameterEstimationAlgorithm::ParameterEstimationAlgorithm( const OCP& ocp_ )
                             :OptimizationAlgorithm( ocp_ ){

    set( HESSIAN_APPROXIMATION, GAUSS_NEWTON );
}


ParameterEstimationAlgorithm::ParameterEstimationAlgorithm( const ParameterEstimationAlgorithm& arg )
                             :OptimizationAlgorithm( arg ){


}


ParameterEstimationAlgorithm::~ParameterEstimationAlgorithm( ){

}



ParameterEstimationAlgorithm& ParameterEstimationAlgorithm::operator=( const ParameterEstimationAlgorithm& arg ){

    if( this != &arg ){

        OptimizationAlgorithm::operator=(arg);
    }
    return *this;
}


returnValue ParameterEstimationAlgorithm::getParameterVarianceCovariance( DMatrix &pVar ){

    DMatrix              tmp;
    returnValue returnvalue;
    int              offset;
    int                  np;
    int           run1,run2;

    returnvalue = getVarianceCovariance( tmp );
    if( returnvalue != SUCCESSFUL_RETURN ) return ACADOERROR(returnvalue);

    if( iter.p == 0 ){
        pVar.init(0,0);
        return SUCCESSFUL_RETURN;
    }

    offset = 0;

    if( iter.x  != 0 ) offset += iter.x->getNumValues();
    if( iter.xa != 0 ) offset += iter.xa->getNumValues();

    np = iter.p->getNumValues();
    pVar.init( np, np );

    for( run1 = 0; run1 < np; run1++ )
        for( run2 = 0; run2 < np; run2++ )
            pVar( run1, run2 ) = tmp( offset+run1, offset+run2 );

    return SUCCESSFUL_RETURN;
}


returnValue ParameterEstimationAlgorithm::getDifferentialStateVarianceCovariance( DMatrix &xVar ){

    return ACADOERROR(RET_NOT_IMPLEMENTED_YET);
}


returnValue ParameterEstimationAlgorithm::getAlgebraicStateVarianceCovariance( DMatrix &xaVar ){

    return ACADOERROR(RET_NOT_IMPLEMENTED_YET);
}


returnValue ParameterEstimationAlgorithm::getControlCovariance( DMatrix &uVar ){

    return ACADOERROR(RET_NOT_IMPLEMENTED_YET);
}


returnValue ParameterEstimationAlgorithm::getDistubanceVarianceCovariance( DMatrix &wVar ){

    return ACADOERROR(RET_NOT_IMPLEMENTED_YET);
}


returnValue ParameterEstimationAlgorithm::getVarianceCovariance( DMatrix &var ){

    if( nlpSolver == 0 ) return ACADOERROR( RET_MEMBER_NOT_INITIALISED );
    return nlpSolver->getVarianceCovariance( var );
}




//
// PROTECTED MEMBER FUNCTIONS:
//

returnValue ParameterEstimationAlgorithm::initializeNlpSolver( const OCPiterate& _userInit )
{
	return OptimizationAlgorithm::initializeNlpSolver( _userInit );
}


returnValue ParameterEstimationAlgorithm::initializeObjective(	Objective* F
														)
{
	return SUCCESSFUL_RETURN;
}






CLOSE_NAMESPACE_ACADO

// end of file.
