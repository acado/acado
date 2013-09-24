/*
 *    This file is part of ACADO Toolkit.
 *
 *    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
 *    Copyright (C) 2008-2013 by Boris Houska, Hans Joachim Ferreau,
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
*    \file src/symbolic_operator/atan.cpp
*    \author Boris Houska, Hans Joachim Ferreau
*    \date 2008
*/


#include <acado/utils/acado_utils.hpp>
#include <acado/symbolic_operator/symbolic_operator.hpp>


double dAtan(double x){
  return 1/(1+x*x);
}


double ddAtan(double x){
  double v1 = 1+x*x;
  return -2*x/v1/v1;
}

BEGIN_NAMESPACE_ACADO


Atan::Atan():UnaryOperator(){
  cName = "atan";

  fcn = &atan;
  dfcn = &dAtan;
  ddfcn = &ddAtan;

  operatorName = ON_ATAN;

}

Atan::Atan( Operator *_argument ):UnaryOperator(_argument){
  cName = "atan";

  fcn = &atan;
  dfcn = &dAtan;
  ddfcn = &ddAtan;

  operatorName = ON_ATAN;
}


Atan::Atan( const Atan &arg ):UnaryOperator(arg){
  cName = "atan";

  fcn = &atan;
  dfcn = &dAtan;
  ddfcn = &ddAtan;

  operatorName = ON_ATAN;
}


Atan::~Atan(){

}


Atan& Atan::operator=( const Atan &arg ){

  UnaryOperator::operator=(arg);

  return *this;
}


returnValue Atan::evaluate( EvaluationBase *x ){
 
    x->Atan(*argument);
    return SUCCESSFUL_RETURN;
}


Operator* Atan::differentiate( int index ){

  dargument = argument->differentiate( index );
  if( dargument->isOneOrZero() == NE_ZERO ){
    return new DoubleConstant( 0.0 , NE_ZERO );
  }
  if( dargument->isOneOrZero() == NE_ONE ){
    return new Power_Int(
             new Addition(
               new DoubleConstant( 1.0 , NE_ONE ),
               new Power_Int(
                 argument->clone(),
                 2
               )
             ),
             -1
           );
  }
  return new Quotient(
                 dargument->clone(),
                 new Addition(
                         new DoubleConstant( 1.0 , NE_ONE ),
                         new Power_Int(
                                 argument->clone(),
                                 2
                             )
                     )

             );

}


Operator* Atan::ADforwardProtected( int dim,
                                      VariableType *varType,
                                      int *component,
                                      Operator **seed,
                                      int &nNewIS,
                                      TreeProjection ***newIS ){

    if( dargument != 0 )
        delete dargument;

    dargument = argument->AD_forward(dim,varType,component,seed,nNewIS,newIS);

    if( dargument->isOneOrZero() == NE_ZERO ){
        return new DoubleConstant( 0.0 , NE_ZERO );
    }
    if( dargument->isOneOrZero() == NE_ONE ){
        return new Power_Int(
                 new Addition(
                   new DoubleConstant( 1.0 , NE_ONE ),
                   new Power_Int(
                     argument->clone(),
                     2
                   )
                 ),
                 -1
             );
    }
    return new Quotient(
                 dargument->clone(),
                 new Addition(
                         new DoubleConstant( 1.0 , NE_ONE ),
                         new Power_Int(
                                 argument->clone(),
                                 2
                             )
                     )

         );
}



returnValue Atan::ADbackwardProtected( int dim,
                                       VariableType *varType,
                                       int *component,
                                       Operator *seed,
                                       Operator **df         ){


    if( seed->isOneOrZero() == NE_ZERO ){
            argument->AD_backward( dim,
                                          varType,
                                          component,
                                          new DoubleConstant( 0.0 , NE_ZERO ),
                                          df
            );

        delete seed;
        return SUCCESSFUL_RETURN;
    }
    if( seed->isOneOrZero() == NE_ONE ){
            argument->AD_backward( dim,
                                          varType,
                                          component,
             new Power_Int(
                 new Addition(
                   new DoubleConstant( 1.0 , NE_ONE ),
                   new Power_Int(
                     argument->clone(),
                     2
                   )
                 ),
                 -1
             ),
                              df
            );
        delete seed;
        return SUCCESSFUL_RETURN;
    }
    argument->AD_backward( dim,
                                  varType,
                                  component,
                                  new Quotient(
                                      seed->clone(),
                                      new Addition(
                                          new DoubleConstant( 1.0 , NE_ONE ),
                                          new Power_Int(
                                              argument->clone(),
                                              2
                                          )
                                      )
                                  ),
                                  df
            );

    delete seed;
    return SUCCESSFUL_RETURN;
}

Operator* Atan::substitute( int index, const Operator *sub ){

    return new Atan( argument->substitute( index , sub ) );

}

Operator* Atan::clone() const{

    return new Atan(*this);
}


CLOSE_NAMESPACE_ACADO

// end of file.
