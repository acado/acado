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
*    \file src/symbolic_operator/exp.cpp
*    \author Boris Houska, Hans Joachim Ferreau
*    \date 2008
*/


#include <acado/utils/acado_utils.hpp>
#include <acado/symbolic_operator/symbolic_operator.hpp>



BEGIN_NAMESPACE_ACADO


Exp::Exp():UnaryOperator(){
  cName = "exp";

  fcn = &exp;
  dfcn = &exp;
  ddfcn = &exp;

  operatorName = ON_EXP;

}

Exp::Exp( Operator *_argument ):UnaryOperator(_argument){
  cName = "exp";

  fcn = &exp;
  dfcn = &exp;
  ddfcn = &exp;

  operatorName = ON_EXP;
}


Exp::Exp( const Exp &arg ):UnaryOperator(arg){
  cName = "exp";

  fcn = &exp;
  dfcn = &exp;
  ddfcn = &exp;

  operatorName = ON_EXP;
}


Exp::~Exp(){

}


Exp& Exp::operator=( const Exp &arg ){

  UnaryOperator::operator=(arg);

  return *this;
}


returnValue Exp::evaluate( EvaluationBase *x ){
 
    x->Exp(*argument);
    return SUCCESSFUL_RETURN;
}


Operator* Exp::substitute( int index, const Operator *sub ){

    return new Exp( argument->substitute( index , sub ) );

}

Operator* Exp::clone() const{

    return new Exp(*this);
}

CurvatureType Exp::getCurvature( ){

    if( curvature != CT_UNKNOWN )  return curvature;

    const CurvatureType cc = argument->getCurvature();

    if( cc == CT_CONSTANT )  return CT_CONSTANT;
    if( cc == CT_AFFINE   )  return CT_CONVEX  ;
    if( cc == CT_CONVEX   )  return CT_CONVEX  ;

    return CT_NEITHER_CONVEX_NOR_CONCAVE;
}

returnValue Exp::initDerivative() {

	if( initialized ) return SUCCESSFUL_RETURN;
	initialized = BT_TRUE;

	derivative = convert2TreeProjection(new Exp(argument->clone()));
	derivative2 = derivative;

	return argument->initDerivative();
}

CLOSE_NAMESPACE_ACADO

// end of file.
