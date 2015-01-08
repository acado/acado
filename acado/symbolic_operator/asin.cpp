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
*    \file src/symbolic_operator/asin.cpp
*    \author Boris Houska, Hans Joachim Ferreau
*    \date 2008
*/


#include <acado/utils/acado_utils.hpp>
#include <acado/symbolic_operator/symbolic_operator.hpp>


double dAsin(double x){
  return 1/sqrt(1-x*x);
}

double ddAsin(double x){
  double v1 = sqrt(1-x*x);
  return -2*x*(-0.5/v1/v1/v1);
}

BEGIN_NAMESPACE_ACADO


Asin::Asin():UnaryOperator(){
  cName = "asin";

  fcn = &asin;
  dfcn = &dAsin;
  ddfcn = &ddAsin;

  operatorName = ON_ASIN;

}

Asin::Asin( Operator *_argument ):UnaryOperator(_argument){
  cName = "asin";

  fcn = &asin;
  dfcn = &dAsin;
  ddfcn = &ddAsin;

  operatorName = ON_ASIN;
}


Asin::Asin( const Asin &arg ):UnaryOperator(arg){
  cName = "asin";

  fcn = &asin;
  dfcn = &dAsin;
  ddfcn = &ddAsin;

  operatorName = ON_ASIN;
}


Asin::~Asin(){

}


Asin& Asin::operator=( const Asin &arg ){

  UnaryOperator::operator=(arg);

  return *this;
}


returnValue Asin::evaluate( EvaluationBase *x ){
 
    x->Asin(*argument);
    return SUCCESSFUL_RETURN;
}



Operator* Asin::substitute( int index, const Operator *sub ){

    return new Asin( argument->substitute( index , sub ) );

}

Operator* Asin::clone() const{

    return new Asin(*this);
}

returnValue Asin::initDerivative() {

	if( initialized ) return SUCCESSFUL_RETURN;
	initialized = BT_TRUE;

	derivative = convert2TreeProjection(
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
			));
	derivative2 = convert2TreeProjection(
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
							new DoubleConstant( -1.5 , NE_NEITHER_ONE_NOR_ZERO )
					),
					argument->clone()
			));

	return argument->initDerivative();
}


CLOSE_NAMESPACE_ACADO

// end of file.
