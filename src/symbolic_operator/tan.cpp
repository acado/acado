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
 *    \file src/symbolic_operator/tan.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date 2008
 */


#include <acado/utils/acado_utils.hpp>
#include <acado/symbolic_operator/symbolic_operator.hpp>


double dTan(double x){
  double v1 = tan(x);
  return 1+v1*v1;
}

double ddTan(double x){
  double v1 = tan(x);
  return 2*v1*(1+v1*v1);
}

BEGIN_NAMESPACE_ACADO




Tan::Tan():UnaryOperator(){
  cName = "tan";

  fcn = &tan;
  dfcn = &dTan;
  ddfcn = &ddTan;

  operatorName = ON_TAN;

}

Tan::Tan( const SharedOperator &_argument ):UnaryOperator(_argument){
  cName = "tan";

  fcn = &tan;
  dfcn = &dTan;
  ddfcn = &ddTan;

  operatorName = ON_TAN;
}


Tan::Tan( const Tan &arg ):UnaryOperator(arg){
  cName = "tan";

  fcn = &tan;
  dfcn = &dTan;
  ddfcn = &ddTan;

  operatorName = ON_TAN;
}


Tan::~Tan(){

}


Tan& Tan::operator=( const Tan &arg ){

  UnaryOperator::operator=(arg);

  return *this;
}


returnValue Tan::evaluate( EvaluationBase *x ){

    x->Tan(*argument);
    return SUCCESSFUL_RETURN;
}

SharedOperator Tan::substitute( int index, const SharedOperator &sub ){

    return SharedOperator( new Tan( argument->substitute( index , sub ) ));

}


returnValue Tan::initDerivative() {

	if( derivative != 0 && derivative2 != 0 ) return SUCCESSFUL_RETURN;

	derivative = convert2TreeProjection( SharedOperator( new Quotient( SharedOperator( new DoubleConstant( 1.0 , NE_ONE )), SharedOperator( new Power_Int( SharedOperator( new Cos( argument )), 2 )) )));
	derivative2 = convert2TreeProjection(SharedOperator( new Quotient(   SharedOperator(  new Product(
			SharedOperator( new DoubleConstant( 2.0 , NE_NEITHER_ONE_NOR_ZERO )),
			SharedOperator( new Tan(argument))
	)),
			SharedOperator( new Power_Int( SharedOperator( new Cos( argument )), 2 ))
	)));

	return argument->initDerivative();
}


CLOSE_NAMESPACE_ACADO

// end of file.
