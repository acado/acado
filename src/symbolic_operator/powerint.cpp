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
*    \file src/symbolic_operator/powerint.cpp
*    \author Boris Houska, Hans Joachim Ferreau
*    \date 2008
*/


#include <acado/utils/acado_utils.hpp>
#include <acado/symbolic_operator/symbolic_operator.hpp>



BEGIN_NAMESPACE_ACADO

Power_Int::Power_Int():UnaryOperator(){ }

Power_Int::Power_Int( const SharedOperator &_argument, const int &_exponent ):UnaryOperator(_argument," "){ exponent = _exponent; }

Power_Int::Power_Int( const Power_Int &arg ):UnaryOperator(arg){ exponent = arg.exponent; }

Power_Int::~Power_Int(){ }

returnValue Power_Int::evaluate( EvaluationBase *x ){
 
    x->powerInt(*argument,exponent);
    return SUCCESSFUL_RETURN;
}

    
SharedOperator Power_Int::substitute( SharedOperatorMap &sub ){

    return SharedOperator( new Power_Int( argument->substitute(sub), exponent ) );
}

returnValue Power_Int::initDerivative() {

    if( derivative != 0 && derivative2 != 0 ) return SUCCESSFUL_RETURN;

    
    
    switch( exponent ){
    
      case 0:
        derivative  = SharedOperator( new DoubleConstant(0.0,NE_ZERO) );
        derivative2 = SharedOperator( new DoubleConstant(0.0,NE_ZERO) );
     
      case 1:
        derivative  = SharedOperator( new DoubleConstant(1.0,NE_ONE ) );
        derivative2 = SharedOperator( new DoubleConstant(0.0,NE_ZERO) );
      
      case 2:
        derivative  = convert2TreeProjection(
                       myProd(
                          SharedOperator(new DoubleConstant(2.0,NE_NEITHER_ONE_NOR_ZERO)),
                          argument
                       )
                      );
        derivative2 = SharedOperator( new DoubleConstant(2.0,NE_NEITHER_ONE_NOR_ZERO) );

      case 3:
        derivative  = convert2TreeProjection(
                       myProd(
                           SharedOperator(new DoubleConstant(exponent,NE_NEITHER_ONE_NOR_ZERO)),
                          SharedOperator(new Power_Int(argument,exponent-1))
                       )
                     );
        derivative2 = convert2TreeProjection(
                       myProd(
                          SharedOperator(new DoubleConstant(6.0,NE_NEITHER_ONE_NOR_ZERO)),
                          argument
                       )
                      );

      default:
        derivative  = convert2TreeProjection(
                       myProd(
                           SharedOperator(new DoubleConstant(exponent,NE_NEITHER_ONE_NOR_ZERO)),
                          SharedOperator(new Power_Int(argument,exponent-1))
                       )
                     );
        derivative2 = convert2TreeProjection(
                       myProd(
                           SharedOperator(new DoubleConstant(exponent*(exponent-1),NE_NEITHER_ONE_NOR_ZERO)),
                           SharedOperator(new Power_Int(argument,exponent-2))
                        )
                      );
    }
    
    return argument->initDerivative();
}


std::ostream& Power_Int::print(std::ostream& stream, StringMap &name ) const{
  
   switch( exponent ){
    
     case 0:
        return stream << "1.";
     
     case 1:
        argument->print(stream,name);
        return stream;

     case 2:
        stream <<"(";
        argument->print(stream,name);
        stream <<"*";
        argument->print(stream,name);
        stream <<")";
        return stream;

     default:
        stream << "(pow(";
        argument->print(stream,name);
        stream <<","<< exponent <<"))";
   } 
   return stream;
}


CLOSE_NAMESPACE_ACADO

// end of file.
