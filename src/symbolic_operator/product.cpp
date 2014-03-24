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
 *    \file src/symbolic_operator/product.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date 2008
 */


#include <acado/utils/acado_utils.hpp>
#include <acado/symbolic_operator/symbolic_operator.hpp>



BEGIN_NAMESPACE_ACADO




Product::Product():BinaryOperator(){ }

Product::Product( const SharedOperator &_argument1, const SharedOperator &_argument2 )
        :BinaryOperator( _argument1, _argument2 ){}

Product::Product( const Product &arg ):BinaryOperator( arg ){}

Product::~Product(){}

returnValue Product::evaluate( EvaluationBase *x ){
 
    x->product(*a1,*a2);
    return SUCCESSFUL_RETURN;
}

SharedOperator Product::substitute( SharedOperatorMap &sub ){

    return SharedOperator( new Product( a1->substitute(sub),
                                        a2->substitute(sub) ));
}

std::ostream& Product::print( std::ostream &stream, StringMap &name ) const{

	if ( ( acadoIsFinite( a1->getValue() ) == BT_FALSE ) ||
		 ( acadoIsFinite( a2->getValue() ) == BT_FALSE ) )
	{
		 stream << "(";
		 a1->print(stream,name);
		 stream << "*";
		 a2->print(stream,name);
		 return stream << ")";
	}
	else
	{
		return stream << "((real_t)(" << (a1->getValue() * a2->getValue()) << "))";
	}
}


returnValue Product::initDerivative() {

    if( d1 != 0 ) return SUCCESSFUL_RETURN;

    d1  = a2;
    d2  = a1;
    d11 = SharedOperator( new DoubleConstant(0.0,NE_ZERO) );
    d12 = SharedOperator( new DoubleConstant(1.0,NE_ONE ) );
    d22 = SharedOperator( new DoubleConstant(0.0,NE_ZERO) );
    
    a1->initDerivative();
    a2->initDerivative();

    return SUCCESSFUL_RETURN;
}

CLOSE_NAMESPACE_ACADO

// end of file.
