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
 *    \file src/dynamic_discretization/collocation_method.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *
 */


#include <acado/dynamic_discretization/collocation_method.hpp>


BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//


CollocationMethod::CollocationMethod( ) : DynamicDiscretization( )
{
}


CollocationMethod::CollocationMethod( UserInteraction* _userInteraction ) : DynamicDiscretization( _userInteraction )
{
}


CollocationMethod::CollocationMethod( const CollocationMethod& rhs )
                     :DynamicDiscretization ( rhs ){


}


CollocationMethod::~CollocationMethod( ){

}



CollocationMethod& CollocationMethod::operator=( const CollocationMethod& rhs ){

    return *this;
}


DynamicDiscretization* CollocationMethod::clone() const{

    return new CollocationMethod(*this);
}



returnValue CollocationMethod::addStage( const DynamicSystem  &dynamicSystem_,
                                      const Grid           &stageIntervals,
                                      const IntegratorType &integratorType_ ){

    return ACADOERROR(RET_NOT_IMPLEMENTED_YET);
}


returnValue CollocationMethod::addTransition( const Transition& transition_ ){

    return ACADOERROR(RET_NOT_IMPLEMENTED_YET);
}



returnValue CollocationMethod::clear(){

    return ACADOERROR(RET_NOT_IMPLEMENTED_YET);
}



returnValue CollocationMethod::evaluate( OCPiterate &iter ){

    return ACADOERROR(RET_NOT_IMPLEMENTED_YET);
}



returnValue CollocationMethod::evaluateSensitivities( ){

    return ACADOERROR(RET_NOT_IMPLEMENTED_YET);
}


returnValue CollocationMethod::evaluateSensitivitiesLifted( ){

    return ACADOERROR(RET_NOT_IMPLEMENTED_YET);
}


returnValue CollocationMethod::evaluateSensitivities( const BlockMatrix &seed, BlockMatrix &hessian ){

    return ACADOERROR(RET_NOT_IMPLEMENTED_YET);
}


returnValue CollocationMethod::deleteAllSeeds(){

//     setForwardSeed(  0, 0, 0, 0, 1 );
//     setForwardSeed(  0, 0, 0, 0, 2 );
//     setBackwardSeed( 0, 1 );
//     setBackwardSeed( 0, 2 );

    // delete seeds of member classes ...

    return SUCCESSFUL_RETURN;
}



returnValue CollocationMethod::unfreeze( ){

    return ACADOERROR(RET_NOT_IMPLEMENTED_YET);
}



BooleanType CollocationMethod::isAffine( ) const
{
    return BT_FALSE;
}



CLOSE_NAMESPACE_ACADO

// end of file.
