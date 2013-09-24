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
 *    \file src/symbolic_operator/logarithm.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date 2008
 */


#include <acado/utils/acado_utils.hpp>
#include <acado/symbolic_operator/symbolic_operator.hpp>


double dLogarithm(double x){
  return 1/x;
}


double ddLogarithm(double x){
  return -1/x/x;
}

BEGIN_NAMESPACE_ACADO


Logarithm::Logarithm():UnaryOperator(){
  cName = "log";

  fcn = &log;
  dfcn = &dLogarithm;
  ddfcn = &ddLogarithm;

  operatorName = ON_LOGARITHM;

}

Logarithm::Logarithm( Operator *_argument ):UnaryOperator(_argument){
  cName = "log";

  fcn = &log;
  dfcn = &dLogarithm;
  ddfcn = &ddLogarithm;

  operatorName = ON_LOGARITHM;
}


Logarithm::Logarithm( const Logarithm &arg ):UnaryOperator(arg){
  cName = "log";

  fcn = &log;
  dfcn = &dLogarithm;
  ddfcn = &ddLogarithm;

  operatorName = ON_LOGARITHM;
}


Logarithm::~Logarithm(){

}


Logarithm& Logarithm::operator=( const Logarithm &arg ){

  UnaryOperator::operator=(arg);

  return *this;
}


returnValue Logarithm::evaluate( EvaluationBase *x ){
 
    x->Log(*argument);
    return SUCCESSFUL_RETURN;
}


Operator* Logarithm::differentiate( int index ){

  dargument = argument->differentiate( index );
  if( dargument->isOneOrZero() == NE_ONE ){
    return new Power_Int(
             argument->clone(),
             -1
           );
  }
  if( dargument->isOneOrZero() == NE_ZERO ){
    return new DoubleConstant( 0.0 , NE_ZERO );
  }
  return new Quotient(
                dargument->clone(),
                argument->clone()
             );
}


Operator* Logarithm::ADforwardProtected( int dim,
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
                 argument->clone(),
                 -1
             );
    }
    return new Quotient(
                dargument->clone(),
                argument->clone()
             );
}



returnValue Logarithm::ADbackwardProtected( int dim,
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
                                              argument->clone(),
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
                                      argument->clone()
                                  ),
                                  df
            );

    delete seed;
    return SUCCESSFUL_RETURN;
}

Operator* Logarithm::substitute( int index, const Operator *sub ){

    return new Logarithm( argument->substitute( index , sub ) );

}

Operator* Logarithm::clone() const{

    return new Logarithm(*this);
}


CurvatureType Logarithm::getCurvature( ){

    if( curvature != CT_UNKNOWN )  return curvature;

    const CurvatureType cc = argument->getCurvature();

    if( cc == CT_CONSTANT )  return CT_CONSTANT;
    if( cc == CT_AFFINE   )  return CT_CONCAVE ;
    if( cc == CT_CONCAVE  )  return CT_CONCAVE ;

    return CT_NEITHER_CONVEX_NOR_CONCAVE;
}

CLOSE_NAMESPACE_ACADO

// end of file.
