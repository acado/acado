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
*    \file src/symbolic_operator/operator.cpp
*    \author Boris Houska, Hans Joachim Ferreau
*    \date 2008
*/


#include <acado/utils/acado_utils.hpp>
#include <acado/symbolic_operator/symbolic_operator.hpp>
#include <acado/symbolic_expression/symbolic_expression.hpp>


BEGIN_NAMESPACE_ACADO

Operator::Operator(){ }

Operator::~Operator(){ }

returnValue Operator::setArgument( const SharedOperator & arg ){ return SUCCESSFUL_RETURN; }


Operator& Operator::operator=( const ScalarExpression &arg ){ setArgument(arg.element); return *this; }

Operator& Operator::operator+=( const ScalarExpression & arg ){ return *this; }
Operator& Operator::operator-=( const ScalarExpression & arg ){ return *this; }
Operator& Operator::operator*=( const ScalarExpression & arg ){ return *this; }
Operator& Operator::operator/=( const ScalarExpression & arg ){ return *this; }

double Operator::getValue() const{ return INFTY; }

SharedOperator Operator::myProd(const SharedOperator &a, const SharedOperator &b) const{
  
    if( a->isOneOrZero() == NE_ZERO ) return SharedOperator( new DoubleConstant( 0.0 , NE_ZERO ) );
    if( b->isOneOrZero() == NE_ZERO ) return SharedOperator( new DoubleConstant( 0.0 , NE_ZERO ) );
    
    if( a->isOneOrZero() == NE_ONE  ) return b;
    if( b->isOneOrZero() == NE_ONE  ) return a;
        
    if( a == b ) return convert2TreeProjection( SharedOperator( new Power_Int(a,2) ));
    
    return convert2TreeProjection( SharedOperator( new Product(a,b) )); 
}
 
 
SharedOperator Operator::myQuotient(const SharedOperator &a, const SharedOperator &b) const{
  
    if( a->isOneOrZero() == NE_ZERO ) return SharedOperator( new DoubleConstant( 0.0 , NE_ZERO ) );
    if( b->isOneOrZero() == NE_ONE  ) return a;
    
    if( a == b ) return SharedOperator( new DoubleConstant( 1.0 , NE_ONE ) );
    
    return convert2TreeProjection( SharedOperator( new Quotient(a,b) )); 
}

SharedOperator Operator::myAdd( const SharedOperator &a, const SharedOperator &b) const{

    if( a == 0 && b == 0 ) return SharedOperator( new DoubleConstant( 0.0 , NE_ZERO ) );
    if( a == 0 && b != 0 ) return b;
    if( a != 0 && b == 0 ) return a;
  
    if( a->isOneOrZero() == NE_ZERO && b->isOneOrZero() == NE_ZERO )
        return SharedOperator( new DoubleConstant( 0.0 , NE_ZERO ) );

    if( a->isOneOrZero() == NE_ZERO ) return b;
    if( b->isOneOrZero() == NE_ZERO ) return a;
    
    if( a->isOneOrZero() == NE_ONE && b->isOneOrZero() == NE_ONE )
        return SharedOperator( new DoubleConstant( 2.0 , NE_NEITHER_ONE_NOR_ZERO ) );
    
    if( a == b ) return convert2TreeProjection( SharedOperator( new Product( a, SharedOperator( new DoubleConstant( 2.0 , NE_NEITHER_ONE_NOR_ZERO) ) ) ));
    
    return convert2TreeProjection( SharedOperator( new Addition(a,b) ));
}


SharedOperator Operator::mySubtract (const SharedOperator &a,const SharedOperator &b) const{

    if( a->isOneOrZero() == NE_ZERO && b->isOneOrZero() == NE_ZERO )
        return SharedOperator( new DoubleConstant( 0.0 , NE_ZERO ) );

    if( a->isOneOrZero() == NE_ZERO ) return convert2TreeProjection( SharedOperator( new Subtraction(SharedOperator( new DoubleConstant( 0.0 , NE_ZERO )), b)));
    if( b->isOneOrZero() == NE_ZERO ) return a;

    if( a->isOneOrZero() == NE_ONE && b->isOneOrZero() == NE_ONE )
        return SharedOperator( new DoubleConstant( 0.0 , NE_ZERO ) );

    if( a == b ) return SharedOperator( new DoubleConstant( 0.0 , NE_ZERO ) );

    return convert2TreeProjection( SharedOperator( new Subtraction(a,b) ));
}


SharedOperator Operator::myPower (const SharedOperator &a, const SharedOperator &b) const{

    if( a->isOneOrZero() == NE_ZERO && b->isOneOrZero() == NE_ONE ) return SharedOperator( new DoubleConstant( 0.0 , NE_ZERO ) );

    if( b->isOneOrZero() == NE_ZERO ) return SharedOperator( new DoubleConstant( 1.0 , NE_ONE ) );
    if( a->isOneOrZero() == NE_ONE ) return SharedOperator( new DoubleConstant( 1.0 , NE_ONE ) );

    if( b->isOneOrZero() == NE_ONE ) return a;

    return convert2TreeProjection( SharedOperator( new Power(a,b) ));
}


SharedOperator Operator::myPowerInt (const SharedOperator &a, const int &b) const{

	if( a->isOneOrZero() == NE_ZERO && b>0 ) 	return SharedOperator( new DoubleConstant( 0.0 , NE_ZERO ) );
	if( b == 0 ) 					return SharedOperator( new DoubleConstant( 1.0 , NE_ONE  ) );
	if( a->isOneOrZero() == NE_ONE ) 		return SharedOperator( new DoubleConstant( 1.0 , NE_ONE  ) );
	if( b == 1 ) 					return a;

    return convert2TreeProjection( SharedOperator( new Power_Int(a,b) ));
}


SharedOperator Operator::mySin(const SharedOperator &a) const{
  
    if( a->isOneOrZero() == NE_ZERO ) return SharedOperator( new DoubleConstant( 0.0 , NE_ZERO ) );
    return convert2TreeProjection( SharedOperator( new Sin(a) ));
}


SharedOperator Operator::myCos(const SharedOperator &a) const{

    if( a->isOneOrZero() == NE_ZERO ) return SharedOperator( new DoubleConstant( 1.0 , NE_ONE ) );
    return convert2TreeProjection( SharedOperator( new Cos(a) ));
}


SharedOperator Operator::myTan(const SharedOperator &a) const{

    if( a->isOneOrZero() == NE_ZERO ) return SharedOperator( new DoubleConstant( 0.0 , NE_ZERO ) );
    return convert2TreeProjection( SharedOperator( new Tan(a) ) );
}


SharedOperator Operator::myAsin(const SharedOperator &a) const{

    if( a->isOneOrZero() == NE_ZERO ) return SharedOperator( new DoubleConstant( 0.0 , NE_ZERO ) );
    return convert2TreeProjection( SharedOperator( new Asin(a) ) );
}


SharedOperator Operator::myAcos(const SharedOperator &a) const{

    if( a->isOneOrZero() == NE_ONE ) return SharedOperator( new DoubleConstant( 0.0 , NE_ZERO ) );
    return convert2TreeProjection( SharedOperator( new Acos(a) ) );
}


SharedOperator Operator::myAtan(const SharedOperator &a) const{

    if( a->isOneOrZero() == NE_ZERO ) return SharedOperator( new DoubleConstant( 0.0 , NE_ZERO ) );
    return convert2TreeProjection( SharedOperator( new Atan(a) ) );
}


SharedOperator Operator::myExp(const SharedOperator &a) const{

    if( a->isOneOrZero() == NE_ZERO ) return SharedOperator( new DoubleConstant( 1.0 , NE_ONE ) );
    return convert2TreeProjection( SharedOperator( new Exp(a) ));
}


SharedOperator Operator::mySqrt(const SharedOperator &a) const{

    return convert2TreeProjection( myPower(a, SharedOperator(new DoubleConstant(0.5,NE_NEITHER_ONE_NOR_ZERO))) );
}

SharedOperator Operator::myLogarithm (const SharedOperator &a) const{

    if( a->isOneOrZero() == NE_ONE ) return SharedOperator( new DoubleConstant( 0.0 , NE_ZERO ) );

    return convert2TreeProjection( SharedOperator( new Logarithm(a) ));
}

SharedOperator Operator::convert2TreeProjection( const SharedOperator &a ) const{
  
    // check whether  a  is a tree projection:
    TreeProjection *tp = dynamic_cast<TreeProjection *>(a.get());
    if( tp != 0 ) return a;
    
    // check whether  a  is a tree projection:
    Projection *p = dynamic_cast<Projection *>(a.get());
    if( p != 0 ) return a;
    
    // check whether  a  is a constant:
    DoubleConstant *d = dynamic_cast<DoubleConstant*>(a.get());
    if( d != 0 ) return a;
            
    // no special case: create a new TreeProjection:
     TreeProjection b;
     b.setArgument(a);
     return SharedOperator( new TreeProjection(b) );
}


SharedOperator Operator::checkForZero( const SharedOperator &x ){

    if( x == 0 ) return SharedOperator( new DoubleConstant(0.0,NE_ZERO) );
    return x;
}

returnValue Operator::initDerivative(){ return SUCCESSFUL_RETURN; }



CLOSE_NAMESPACE_ACADO


// end of file.
