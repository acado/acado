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
*    \file src/symbolic_operator/doubleconstant.cpp
*    \author Boris Houska, Hans Joachim Ferreau
*    \date 2008
*/


#include <acado/utils/acado_utils.hpp>
#include <acado/symbolic_operator/symbolic_operator.hpp>



BEGIN_NAMESPACE_ACADO




DoubleConstant::DoubleConstant():SmoothOperator(){
    value = 0.;
    neutralElement = NE_ZERO;
}

DoubleConstant::DoubleConstant( double value_, NeutralElement neutralElement_ )
	: SmoothOperator( ), value( value_ ), neutralElement( neutralElement_ )
{
    if( acadoIsEqual(value,0.0) == BT_TRUE ) neutralElement = NE_ZERO;
    if( acadoIsEqual(value,1.0) == BT_TRUE ) neutralElement = NE_ONE;
}

DoubleConstant::DoubleConstant( const DoubleConstant &arg ){

    value           = arg.value          ;
    neutralElement  = arg.neutralElement ;
}

DoubleConstant::~DoubleConstant(){}


returnValue DoubleConstant::evaluate( EvaluationBase *x ){

    x->set(value);
    return SUCCESSFUL_RETURN;
}


SharedOperator DoubleConstant::AD_forward( SharedOperatorMap &seed ){

    return SharedOperator( new DoubleConstant( 0.0, NE_ZERO ));
}



returnValue DoubleConstant::AD_backward( SharedOperator     &seed,
                                         SharedOperatorMap  &df  ,
                                         SharedOperatorMap2 &IS   ){

    return SUCCESSFUL_RETURN;
}


returnValue DoubleConstant::AD_symmetric( SharedOperator     &l  ,
                                          SharedOperatorMap  &ldf,
                                          SharedOperatorMap  &df ,
                                          SharedOperatorMap2 &H  ,
                                          SharedOperatorMap2 &LIS,
                                          SharedOperatorMap2 &SIS,
                                          SharedOperatorMap3 &HIS  ){
  
    return SUCCESSFUL_RETURN; 
}

SharedOperator DoubleConstant::substitute( SharedOperatorMap &sub ){

    return SharedOperator( new DoubleConstant(*this) );
}

NeutralElement DoubleConstant::isOneOrZero() const{  return neutralElement; }


std::ostream& DoubleConstant::print( std::ostream &stream, StringMap &name ) const{

    return stream << "(real_t)(" << value << ")";
}

returnValue DoubleConstant::getArgumentList( DependencyMap &exists,
                                             SharedOperatorVector &list  ){ return SUCCESSFUL_RETURN; }

BooleanType DoubleConstant::isSymbolic() const{ return BT_TRUE; }

double DoubleConstant::getValue() const{  return value; }


CLOSE_NAMESPACE_ACADO

// end of file.
