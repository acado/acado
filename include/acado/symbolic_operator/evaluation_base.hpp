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
 *    \file include/acado/symbolic_operator/evaluation_base.hpp
 *    \author Boris Houska
 */


#ifndef ACADO_TOOLKIT_EVALUATION_BASE_HPP
#define ACADO_TOOLKIT_EVALUATION_BASE_HPP


#include <acado/symbolic_operator/symbolic_operator_fwd.hpp>


BEGIN_NAMESPACE_ACADO


/**
 *	\brief Abstract base class for templated evaluation of operators.
 *
 *	\ingroup BasicDataStructures
 *
 *	\author Boris Houska
 */

class EvaluationBase{

public:

	/** Default constructor. */
	EvaluationBase(){};

	virtual ~EvaluationBase(){};

	virtual returnValue addition   ( Operator &arg1, Operator &arg2 ) = 0;
	virtual returnValue subtraction( Operator &arg1, Operator &arg2 ) = 0;
	virtual returnValue product    ( Operator &arg1, Operator &arg2 ) = 0;
	virtual returnValue quotient   ( Operator &arg1, Operator &arg2 ) = 0;
	virtual returnValue power      ( Operator &arg1, Operator &arg2 ) = 0;
	virtual returnValue powerInt   ( Operator &arg1, int      &arg2 ) = 0;

	virtual returnValue project    ( int      &idx ) = 0;
	virtual returnValue set        ( double   &arg ) = 0;
	virtual returnValue Acos       ( Operator &arg ) = 0;
	virtual returnValue Asin       ( Operator &arg ) = 0;
	virtual returnValue Atan       ( Operator &arg ) = 0;
	virtual returnValue Cos        ( Operator &arg ) = 0;
	virtual returnValue Exp        ( Operator &arg ) = 0;
	virtual returnValue Log        ( Operator &arg ) = 0;
	virtual returnValue Sin        ( Operator &arg ) = 0;
	virtual returnValue Tan        ( Operator &arg ) = 0;

};

CLOSE_NAMESPACE_ACADO

#endif
