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
 *    \file src/symbolic_operator/asin.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date 2008
 */

#include <acado/utils/acado_utils.hpp>
#include <acado/symbolic_operator/symbolic_operator.hpp>

double sign(double x) {
	if (x > 0)
		return 1;
	else if (x < 0)
		return -1;
	else
		return 0;
}

double dSign(double x) {
	return 0;
}

double ddSign(double x) {
	return 0;
}

BEGIN_NAMESPACE_ACADO

Sign::Sign() :
		UnaryOperator() {
	cName = "sign";

	fcn = &sign;
	dfcn = &dSign;
	ddfcn = &ddSign;

	operatorName = ON_ASIN;

}

Sign::Sign(Operator *_argument) :
		UnaryOperator(_argument) {
	cName = "sign";

	fcn = &sign;
	dfcn = &dSign;
	ddfcn = &ddSign;

	operatorName = ON_ASIN;
}

Sign::Sign(const Sign &arg) :
		UnaryOperator(arg) {
	cName = "sign";

	fcn = &sign;
	dfcn = &dSign;
	ddfcn = &ddSign;

	operatorName = ON_ASIN;
}

Sign::~Sign() {

}

Sign& Sign::operator=(const Sign &arg) {

	UnaryOperator::operator=(arg);

	return *this;
}

returnValue Sign::evaluate(EvaluationBase *x) {

	x->Sign(*argument);
	return SUCCESSFUL_RETURN;
}

Operator* Sign::differentiate(int index) {

	dargument = argument->differentiate(index);
	if (dargument->isOneOrZero() == NE_ZERO)
		return new DoubleConstant(0.0, NE_ZERO);
	else
		return new DoubleConstant(1.0, NE_ONE);
}

Operator* Sign::ADforwardProtected(int dim, VariableType *varType,
		int *component, Operator **seed, int &nNewIS, TreeProjection ***newIS) {

	if (dargument != 0)
		delete dargument;

	dargument = argument->AD_forward(dim, varType, component, seed, nNewIS,
			newIS);

	if (dargument->isOneOrZero() == NE_ZERO)
		return new DoubleConstant(0.0, NE_ZERO);
	else
		return new DoubleConstant(1.0, NE_ONE);
}

returnValue Sign::ADbackwardProtected(int dim, VariableType *varType,
		int *component, Operator *seed, Operator **df) {

	if (seed->isOneOrZero() == NE_ZERO) {
		argument->AD_backward(dim, varType, component,
				new DoubleConstant(0.0, NE_ZERO), df);
		delete seed;
		return SUCCESSFUL_RETURN;
	}

	argument->AD_backward(dim, varType, component,
			new DoubleConstant(1.0, NE_NEITHER_ONE_NOR_ZERO), df);
	delete seed;
	return SUCCESSFUL_RETURN;

}

Operator* Sign::substitute(int index, const Operator *sub) {

	return new Sign(argument->substitute(index, sub));

}

Operator* Sign::clone() const {

	return new Sign(*this);
}

CLOSE_NAMESPACE_ACADO

// end of file.
