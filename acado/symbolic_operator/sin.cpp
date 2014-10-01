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


returnValue Sin::evaluate( EvaluationBase *x ){

    x->Sin(*argument);
    return SUCCESSFUL_RETURN;
}


Operator* Sin::substitute( int index, const Operator *sub ){

    return new Sin( argument->substitute( index , sub ) );

}

Operator* Sin::clone() const{

    return new Sin(*this);
}

returnValue Sin::initDerivative() {

	if( initialized ) return SUCCESSFUL_RETURN;
	initialized = BT_TRUE;

	derivative = convert2TreeProjection(new Cos(argument->clone()));
	derivative2 = convert2TreeProjection(new Product( new DoubleConstant( -1.0 , NE_NEITHER_ONE_NOR_ZERO ), new Sin(argument->clone()) ));

	return argument->initDerivative();
}


CLOSE_NAMESPACE_ACADO

// end of file.
