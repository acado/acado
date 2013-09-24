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
 *    \file src/symbolic_operator/sin.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date 2008
 */


#include <acado/utils/acado_utils.hpp>
#include <acado/symbolic_operator/symbolic_operator.hpp>


double ddSin(double x){
  return -sin(x);
}


BEGIN_NAMESPACE_ACADO


Sin::Sin():UnaryOperator(){
  cName = "sin";

  fcn = &sin;
  dfcn = &cos;
  ddfcn = &ddSin;

  operatorName = ON_SIN;

}

Sin::Sin( Operator *_argument ):UnaryOperator(_argument){
  cName = "sin";

  fcn = &sin;
  dfcn = &cos;
  ddfcn = &ddSin;

  operatorName = ON_SIN;
}


Sin::Sin( const Sin &arg ):UnaryOperator(arg){
  cName = "sin";

  fcn = &sin;
  dfcn = &cos;
  ddfcn = &ddSin;

  operatorName = ON_SIN;
}


Sin::~Sin(){

}


Sin& Sin::operator=( const Sin &arg ){

  UnaryOperator::operator=(arg);

  return *this;
}

Operator* Sin::differentiate( int index ){

  dargument = argument->differentiate( index );
  if( dargument->isOneOrZero() == NE_ZERO ){
    return new DoubleConstant( 0.0 , NE_ZERO );
  }
  if( dargument->isOneOrZero() == NE_ONE ){
    return new Cos( argument->clone() );
  }
  return new Product( dargument->clone() , new Cos( argument->clone() ) );

}


returnValue Sin::evaluate( EvaluationBase *x ){

    x->Sin(*argument);
    return SUCCESSFUL_RETURN;
}


Operator* Sin::ADforwardProtected( int dim,
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
        return new Cos( argument->clone() );
    }
    return new Product( dargument->clone() , new Cos( argument->clone() ) );
}



returnValue Sin::ADbackwardProtected( int dim,
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
                                          new Cos( argument->clone() ),
                                          df
            );
        delete seed;
        return SUCCESSFUL_RETURN;
    }
    argument->AD_backward( dim,
                                  varType,
                                  component,
                                  new Product( seed->clone() , new Cos( argument->clone() ) ),
                                  df
            );

    delete seed;
    return SUCCESSFUL_RETURN;
}





Operator* Sin::substitute( int index, const Operator *sub ){

    return new Sin( argument->substitute( index , sub ) );

}

Operator* Sin::clone() const{

    return new Sin(*this);
}


CLOSE_NAMESPACE_ACADO

// end of file.
