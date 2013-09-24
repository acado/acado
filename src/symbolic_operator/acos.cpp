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
*    \file src/symbolic_operator/acos.cpp
*    \author Boris Houska, Hans Joachim Ferreau
*    \date 2008
*/


#include <acado/utils/acado_utils.hpp>
#include <acado/symbolic_operator/symbolic_operator.hpp>

double dAcos(double x){
  return -1/sqrt(1-x*x);
}


double ddAcos(double x){
  double v1 = sqrt(1-x*x);
  return 2*x*(-0.5/v1/v1/v1);
}

BEGIN_NAMESPACE_ACADO


Acos::Acos():UnaryOperator(){
  cName = "acos";

  fcn = &acos;
  dfcn = &dAcos;
  ddfcn = &ddAcos;

  operatorName = ON_ACOS;

}

Acos::Acos( Operator *_argument ):UnaryOperator(_argument){
  cName = "acos";

  fcn = &acos;
  dfcn = &dAcos;
  ddfcn = &ddAcos;

  operatorName = ON_ACOS;
}


Acos::Acos( const Acos &arg ):UnaryOperator(arg){
  cName = "acos";

  fcn = &acos;
  dfcn = &dAcos;
  ddfcn = &ddAcos;

  operatorName = ON_ACOS;
}


Acos::~Acos(){

}


Acos& Acos::operator=( const Acos &arg ){

  UnaryOperator::operator=(arg);

  return *this;
}


returnValue Acos::evaluate( EvaluationBase *x ){
 
    x->Acos(*argument);
    return SUCCESSFUL_RETURN;
}



Operator* Acos::differentiate( int index ){

    dargument = argument->differentiate( index );
    if( dargument->isOneOrZero() == NE_ZERO ){
        return new DoubleConstant( 0.0 , NE_ZERO );
    }
    if( dargument->isOneOrZero() == NE_ONE ){
        return new Product(
             new DoubleConstant( -1.0, NE_NEITHER_ONE_NOR_ZERO ),
             new Power(
               new Addition(
                 new DoubleConstant(1.0 , NE_ONE),
                 new Product(
                   new DoubleConstant( -1.0, NE_NEITHER_ONE_NOR_ZERO),
                   new Power_Int(
                     argument->clone(),
                     2
                   )
                 )
               ),
               new DoubleConstant( -0.5 , NE_NEITHER_ONE_NOR_ZERO )
             )
           );
    }
    return new Product(
           new DoubleConstant( -1.0, NE_NEITHER_ONE_NOR_ZERO ),
           new Product(
             new Power(
               new Addition(
                 new DoubleConstant(1.0 , NE_ONE),
                 new Product(
                   new DoubleConstant( -1.0, NE_NEITHER_ONE_NOR_ZERO),
                   new Power_Int(
                     argument->clone(),
                     2
                   )
                 )
               ),
               new DoubleConstant( -0.5 , NE_NEITHER_ONE_NOR_ZERO )
             ),
             dargument->clone()
           )
     );

}


Operator* Acos::ADforwardProtected( int dim,
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
        return new Product(
                 new DoubleConstant( -1.0, NE_NEITHER_ONE_NOR_ZERO ),
                 new Power(
                     new Addition(
                         new DoubleConstant(1.0 , NE_ONE),
                             new Product(
                                 new DoubleConstant( -1.0, NE_NEITHER_ONE_NOR_ZERO),
                                 new Power_Int(
                                     argument->clone(),
                                     2
                                 )
                             )
                         ),
                         new DoubleConstant( -0.5 , NE_NEITHER_ONE_NOR_ZERO )
                     )
             );
    }
    return new Product(
           new DoubleConstant( -1.0, NE_NEITHER_ONE_NOR_ZERO ),
           new Product(
             new Power(
               new Addition(
                 new DoubleConstant(1.0 , NE_ONE),
                 new Product(
                   new DoubleConstant( -1.0, NE_NEITHER_ONE_NOR_ZERO),
                   new Power_Int(
                     argument->clone(),
                     2
                   )
                 )
               ),
               new DoubleConstant( -0.5 , NE_NEITHER_ONE_NOR_ZERO )
             ),
             dargument->clone()
           )
     );
}



returnValue Acos::ADbackwardProtected( int dim,
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
                              new Product(
                                new DoubleConstant( -1.0, NE_NEITHER_ONE_NOR_ZERO ),
                                new Power(
                                  new Addition(
                                    new DoubleConstant(1.0 , NE_ONE),
                                    new Product(
                                       new DoubleConstant( -1.0, NE_NEITHER_ONE_NOR_ZERO),
                                       new Power_Int(
                                         argument->clone(),
                                         2
                                       )
                                    )
                                  ),
                                  new DoubleConstant( -0.5 , NE_NEITHER_ONE_NOR_ZERO )
                                )
                              ),
                              df
            );
        delete seed;
        return SUCCESSFUL_RETURN;
    }
    argument->AD_backward( dim,
                                  varType,
                                  component,
                                  new Product(
                                      new DoubleConstant( -1.0, NE_NEITHER_ONE_NOR_ZERO ),
                                      new Product(
                                          new Power(
                                              new Addition(
                                                  new DoubleConstant(1.0 , NE_ONE),
                                                  new Product(
                                                      new DoubleConstant( -1.0, NE_NEITHER_ONE_NOR_ZERO),
                                                      new Power_Int(
                                                          argument->clone(),
                                                          2
                                                      )
                                                  )
                                              ),
                                              new DoubleConstant( -0.5 , NE_NEITHER_ONE_NOR_ZERO )
                                          ),
                                          seed->clone()
                                      )
                                  ),
                                  df
            );

    delete seed;
    return SUCCESSFUL_RETURN;
}



Operator* Acos::substitute( int index, const Operator *sub ){

    return new Acos( argument->substitute( index , sub ) );

}

Operator* Acos::clone() const{

    return new Acos(*this);
}


CLOSE_NAMESPACE_ACADO

// end of file.
