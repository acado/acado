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
*    \file src/symbolic_operator/nonsmooth_operator.cpp
*    \author Boris Houska, Hans Joachim Ferreau
*    \date 2008
*/


#include <acado/symbolic_operator/symbolic_operator.hpp>


BEGIN_NAMESPACE_ACADO


NonsmoothOperator::NonsmoothOperator():Operator(){}

NonsmoothOperator::~NonsmoothOperator(){}

returnValue NonsmoothOperator::evaluate( EvaluationBase *x ){  return SUCCESSFUL_RETURN; }


SharedOperator NonsmoothOperator::AD_forward( SharedOperatorMap &seed ){

    return SharedOperator( new NonsmoothOperator() );
}


returnValue NonsmoothOperator::AD_backward( SharedOperator     &seed,
                                            SharedOperatorMap  &df  ,
                                            SharedOperatorMap2 &IS   ){

    return SUCCESSFUL_RETURN;
}


returnValue NonsmoothOperator::AD_symmetric( SharedOperator     &l  ,
                                             SharedOperatorMap  &ldf,
                                             SharedOperatorMap  &df ,
                                             SharedOperatorMap2 &H  ,
                                             SharedOperatorMap2 &LIS,
                                             SharedOperatorMap2 &SIS,
                                             SharedOperatorMap3 &HIS  ){

    return SUCCESSFUL_RETURN; 
}

SharedOperator NonsmoothOperator::substitute( SharedOperatorMap &sub ){

    return SharedOperator( new NonsmoothOperator(*this) );
}



NeutralElement NonsmoothOperator::isOneOrZero() const{

    return NE_NEITHER_ONE_NOR_ZERO;
}


BooleanType NonsmoothOperator::isSmooth( ) const{ return BT_FALSE; }

std::ostream& NonsmoothOperator::print( std::ostream &stream, StringMap &name ) const{ return stream; }

returnValue NonsmoothOperator::getArgumentList( DependencyMap &exists,
                                                SharedOperatorVector &list  ){ return SUCCESSFUL_RETURN; }


BooleanType NonsmoothOperator::isSymbolic() const{  return BT_TRUE; }

double NonsmoothOperator::getValue() const{ return INFTY; }

CLOSE_NAMESPACE_ACADO


// end of file.
