/*
 *    This file is part of ACADO Toolkit.
 *
 *    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
 *    Copyright (C) 2008-2009 by Boris Houska and Hans Joachim Ferreau, K.U.Leuven.
 *    Developed within the Optimization in Engineering Center (OPTEC) under
 *    supervision of Moritz Diehl. All rights reserved.
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
 *    \file include/acado/symbolic_operator/evaluation_template.hpp
 *    \author Boris Houska
 */


#ifndef ACADO_TOOLKIT_EVALUATION_TEMPLATE_HPP
#define ACADO_TOOLKIT_EVALUATION_TEMPLATE_HPP

#include <acado/symbolic_operator/symbolic_operator_fwd.hpp>


BEGIN_NAMESPACE_ACADO

/**
 *	\brief Templated class for operator evaluation.
 *
 *	\ingroup BasicDataStructures
 *
 *	\author Boris Houska
 */

template <typename T>
class EvaluationTemplate : public EvaluationBase{

public:

	/** Default constructor. */
	EvaluationTemplate();
	EvaluationTemplate( Tmatrix<T> *_val );
	
	virtual ~EvaluationTemplate();

	virtual returnValue addition   ( Operator &arg1, Operator &arg2 );
	virtual returnValue subtraction( Operator &arg1, Operator &arg2 );
	virtual returnValue product    ( Operator &arg1, Operator &arg2 );
	virtual returnValue quotient   ( Operator &arg1, Operator &arg2 );
	virtual returnValue power      ( Operator &arg1, Operator &arg2 );
	virtual returnValue powerInt   ( Operator &arg1, int      &arg2 );

	virtual returnValue project    ( int      &idx );
	virtual returnValue set        ( double   &arg );
	virtual returnValue Acos       ( Operator &arg );
	virtual returnValue Asin       ( Operator &arg );
	virtual returnValue Atan       ( Operator &arg );
	virtual returnValue Cos        ( Operator &arg );
	virtual returnValue Exp        ( Operator &arg );
	virtual returnValue Log        ( Operator &arg );
	virtual returnValue Sin        ( Operator &arg );
	virtual returnValue Tan        ( Operator &arg );
	
	Tmatrix<T> *val;
	T           res;
	
};



CLOSE_NAMESPACE_ACADO

#include <acado/symbolic_operator/operator.hpp>

BEGIN_NAMESPACE_ACADO



template <typename T> EvaluationTemplate<T>::EvaluationTemplate():EvaluationBase(){ val = 0; }
template <typename T> EvaluationTemplate<T>::EvaluationTemplate( Tmatrix<T> *_val ):EvaluationBase()
{ val = _val; }
template <typename T> EvaluationTemplate<T>::~EvaluationTemplate(){}

template <typename T> returnValue EvaluationTemplate<T>::addition( Operator &arg1, Operator &arg2 ){
	
	EvaluationTemplate<T> r1(val);
	EvaluationTemplate<T> r2(val);
	
	arg1.evaluate( &r1 );
	arg2.evaluate( &r2 );
	
	res = r1.res + r2.res;

	return SUCCESSFUL_RETURN;
}

template <typename T> returnValue EvaluationTemplate<T>::subtraction( Operator &arg1, Operator &arg2 ){
 
	EvaluationTemplate<T> r1(val);
	EvaluationTemplate<T> r2(val);
	
	arg1.evaluate( &r1 );
	arg2.evaluate( &r2 );
	
	res = r1.res - r2.res;
	
	return SUCCESSFUL_RETURN;
}

template <typename T> returnValue EvaluationTemplate<T>::product( Operator &arg1, Operator &arg2 ){
 
	EvaluationTemplate<T> r1(val);
	EvaluationTemplate<T> r2(val);
	
	arg1.evaluate( &r1 );
	arg2.evaluate( &r2 );
	
	res = r1.res*r2.res;
	
	return SUCCESSFUL_RETURN;
}

template <typename T> returnValue EvaluationTemplate<T>::quotient( Operator &arg1, Operator &arg2 ){
 
	EvaluationTemplate<T> r1(val);
	EvaluationTemplate<T> r2(val);
	
	arg1.evaluate( &r1 );
	arg2.evaluate( &r2 );
	
	res = r1.res/r2.res;
	
	return SUCCESSFUL_RETURN;
}

template <typename T> returnValue EvaluationTemplate<T>::power( Operator &arg1, Operator &arg2 ){
 
	EvaluationTemplate<T> r1(val);
	EvaluationTemplate<T> r2(val);
	
	arg1.evaluate( &r1 );
	arg2.evaluate( &r2 );
	
	res = pow(r1.res,r2.res);

	return SUCCESSFUL_RETURN;
}

template <typename T> returnValue EvaluationTemplate<T>::powerInt( Operator &arg1, int &arg2 ){
 
	EvaluationTemplate<T> r1(val);
	
	arg1.evaluate( &r1 );
	
	res = pow( r1.res, arg2 );

	return SUCCESSFUL_RETURN;
}


template <typename T> returnValue EvaluationTemplate<T>::project( int &idx ){

	res = val->operator()(idx);

	return SUCCESSFUL_RETURN;
}


template <typename T> returnValue EvaluationTemplate<T>::set( double &arg ){

	res = arg;

	return SUCCESSFUL_RETURN;
}


template <typename T> returnValue EvaluationTemplate<T>::Acos( Operator &arg ){

	EvaluationTemplate<T> r1(val);
	arg.evaluate( &r1 );
	res = acos( r1.res );

	return SUCCESSFUL_RETURN;
}

template <typename T> returnValue EvaluationTemplate<T>::Asin( Operator &arg ){

	EvaluationTemplate<T> r1(val);
	arg.evaluate( &r1 );
	res = asin( r1.res );

	return SUCCESSFUL_RETURN;
}

template <typename T> returnValue EvaluationTemplate<T>::Atan( Operator &arg ){

	EvaluationTemplate<T> r1(val);
	arg.evaluate( &r1 );
	res = atan( r1.res );

	return SUCCESSFUL_RETURN;
}

template <typename T> returnValue EvaluationTemplate<T>::Cos( Operator &arg ){

	EvaluationTemplate<T> r1(val);
	arg.evaluate( &r1 );
	res = cos( r1.res );

	return SUCCESSFUL_RETURN;
}

template <typename T> returnValue EvaluationTemplate<T>::Exp( Operator &arg ){

	EvaluationTemplate<T> r1(val);
	arg.evaluate( &r1 );
	res = exp( r1.res );

	return SUCCESSFUL_RETURN;
}

template <typename T> returnValue EvaluationTemplate<T>::Log( Operator &arg ){

	EvaluationTemplate<T> r1(val);
	arg.evaluate( &r1 );
	res = log( r1.res );

	return SUCCESSFUL_RETURN;
}

template <typename T> returnValue EvaluationTemplate<T>::Sin( Operator &arg ){

	EvaluationTemplate<T> r1(val);
	arg.evaluate( &r1 );
	res = sin( r1.res );

	return SUCCESSFUL_RETURN;
}

template <typename T> returnValue EvaluationTemplate<T>::Tan( Operator &arg ){

	EvaluationTemplate<T> r1(val);
	arg.evaluate( &r1 );
	res = tan( r1.res );

	return SUCCESSFUL_RETURN;
}

CLOSE_NAMESPACE_ACADO

#endif
