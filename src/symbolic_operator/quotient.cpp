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
 *    \file src/symbolic_operator/quotient.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date 2008
 */


#include <acado/utils/acado_utils.hpp>
#include <acado/symbolic_operator/symbolic_operator.hpp>



BEGIN_NAMESPACE_ACADO



Quotient::Quotient():BinaryOperator(){ }

Quotient::Quotient( const SharedOperator &_argument1, const SharedOperator &_argument2 )
        :BinaryOperator( _argument1, _argument2 ){}

Quotient::Quotient( const Quotient &arg ):BinaryOperator( arg ){}

Quotient::~Quotient(){}

returnValue Quotient::evaluate( EvaluationBase *x ){
 
    x->quotient(*a1,*a2);
    return SUCCESSFUL_RETURN;
}

SharedOperator Quotient::substitute( SharedOperatorMap &sub ){

    return SharedOperator( new Quotient( a1->substitute(sub),
                                         a2->substitute(sub) ));
}

std::ostream& Quotient::print( std::ostream &stream, StringMap &name ) const{

	if ( ( acadoIsFinite( a1->getValue() ) == BT_FALSE ) ||
		 ( acadoIsFinite( a2->getValue() ) == BT_FALSE ) )
	{
		 stream << "(";
		 a1->print(stream,name);
		 stream << "/";
		 a2->print(stream,name);
		 return stream << ")";
	}
	else
	{
		return stream << "((real_t)(" << (a1->getValue() / a2->getValue()) << "))";
	}
}


returnValue Quotient::initDerivative() {

    if( d1 != 0 ) return SUCCESSFUL_RETURN;

    SharedOperator zero( new DoubleConstant(0.0,NE_ZERO) );
    SharedOperator one ( new DoubleConstant(1.0,NE_ONE ) );
    SharedOperator two ( new DoubleConstant(2.0,NE_NEITHER_ONE_NOR_ZERO ) );
    
    SharedOperator inv   = convert2TreeProjection( SharedOperator( new Quotient(one,a2) ) );
    SharedOperator inv2  = convert2TreeProjection( myProd(inv,inv) );
    SharedOperator xinv2 = convert2TreeProjection( myProd(inv2,a1) ); // x/y^2
    
    d1  = inv;  // 1/y
    d2  = convert2TreeProjection( mySubtract(zero,xinv2) );  // -x/y^2
    d11 = zero;
    d12 = convert2TreeProjection( mySubtract(zero, inv2) ); // -1/y^2
    d22 = SharedOperator( myProd( myProd(two,xinv2), inv ) ); // 2*x/y^3
    
    a1->initDerivative();
    a2->initDerivative();

    return SUCCESSFUL_RETURN;
}

CLOSE_NAMESPACE_ACADO

// end of file.
