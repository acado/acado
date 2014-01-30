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
 *    \file src/symbolic_operator/absolute.cpp
 *    \author Torstein Ingebrigtsen Bø
 *    \date 2014
 */

#include <acado/utils/acado_utils.hpp>
#include <acado/symbolic_operator/symbolic_operator.hpp>


BEGIN_NAMESPACE_ACADO

double dAbs(double x) {
	if (x >= 0)
		return 1;
	else
		return -1;

}

double ddAbs(double x) {
	return 0;
}

Absolute::Absolute() :
		UnaryOperator() {
	cName = "absolute";

	fcn = &fabs;
	dfcn = &dAbs;
	ddfcn = &ddAbs;

	operatorName = ON_ABSOLUTE;

}

Absolute::Absolute(Operator *_argument) :
		UnaryOperator(_argument) {
	cName = "absolute";

	fcn = &fabs;
	dfcn = &dAbs;
	ddfcn = &ddAbs;

	operatorName = ON_ABSOLUTE;
}

Absolute::Absolute(const Absolute &arg) :
		UnaryOperator(arg) {
	cName = "absolute";

	fcn = &fabs;
	dfcn = &dAbs;
	ddfcn = &ddAbs;

	operatorName = ON_ABSOLUTE;
}

Absolute::~Absolute() {

}

Absolute& Absolute::operator=(const Absolute &arg) {

	UnaryOperator::operator=(arg);

	return *this;
}

returnValue Absolute::evaluate(EvaluationBase *x) {

	x->Absolute(*argument);
	return SUCCESSFUL_RETURN;
}

Operator* Absolute::differentiate(int index) {

	dargument = argument->differentiate(index);
	if (dargument->isOneOrZero() == NE_ZERO) {
		return new DoubleConstant(0.0, NE_ZERO);
	}
	if (dargument->isOneOrZero() == NE_ONE) {
		return new Sign(argument->clone());
	}
	return new Product(new Sign(argument->clone()), dargument->clone());
}

Operator* Absolute::ADforwardProtected(int dim, VariableType *varType,
		int *component, Operator **seed, int &nNewIS, TreeProjection ***newIS) {

	if (dargument != 0)
		delete dargument;

	dargument = argument->AD_forward(dim, varType, component, seed, nNewIS,
			newIS);

	if (dargument->isOneOrZero() == NE_ZERO) {
		return new DoubleConstant(0.0, NE_ZERO);
	}
	if (dargument->isOneOrZero() == NE_ONE) {
		return new Sign(argument->clone());
	}
	return new Product(new Sign(argument->clone()), dargument->clone());
}

returnValue Absolute::ADbackwardProtected(int dim, VariableType *varType,
		int *component, Operator *seed, Operator **df) {

	if (seed->isOneOrZero() == NE_ZERO) {
		argument->AD_backward(dim, varType, component,
				new DoubleConstant(0.0, NE_ZERO), df);
		delete seed;
		return SUCCESSFUL_RETURN;
	}
	if (seed->isOneOrZero() == NE_ONE) {
		argument->AD_backward(dim, varType, component,
				new Sign(argument->clone()), df);
		delete seed;
		return SUCCESSFUL_RETURN;
	}
	argument->AD_backward(dim, varType, component,
			new Product(
					new Sign(argument->clone()),
					seed->clone()), df);

	delete seed;
	return SUCCESSFUL_RETURN;
}

Operator* Absolute::substitute(int index, const Operator *sub) {

	return new Absolute(argument->substitute(index, sub));

}

Operator * Absolute::clone() const {

	return new Absolute(*this);
}

CLOSE_NAMESPACE_ACADO

// end of file.
