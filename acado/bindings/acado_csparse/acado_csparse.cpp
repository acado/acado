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
 *    \file   external_packages/src/acado_csparse/acado_csparse.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date   2009
 */

#include <acado/bindings/acado_csparse/acado_csparse.hpp>

#ifndef __MATLAB__

#if defined( ACADO_CMAKE_BUILD ) && defined( __cplusplus )
extern "C"
{
#endif // ACADO_CMAKE_BUILD
#include "../../csparse/cs.h"

#if defined( ACADO_CMAKE_BUILD ) && defined( __cplusplus )
}
#endif // ACADO_CMAKE_BUILD

BEGIN_NAMESPACE_ACADO

//
// PUBLIC MEMBER FUNCTIONS:
//

ACADOcsparse::ACADOcsparse()
{
	dim = 0;
	nDense = 0;
	index1 = 0;
	index2 = 0;
	x = 0;
	S = 0;
	N = 0;
	TOL = 1e-14;
	printLevel = LOW;
}

ACADOcsparse::ACADOcsparse(const ACADOcsparse &arg)
{
	int run1;

	dim = arg.dim;
	nDense = arg.nDense;
	index1 = 0;
	index2 = 0;

	if (arg.x == 0)
		x = 0;
	else
	{
		x = new double[dim];
		for (run1 = 0; run1 < dim; run1++)
			x[run1] = arg.x[run1];
	}

	S = 0;
	N = 0;

	TOL = arg.TOL;
	printLevel = arg.printLevel;
}

ACADOcsparse::~ACADOcsparse()
{
	if (index1 != 0)
		delete[] index1;
	if (index2 != 0)
		delete[] index2;
	if (x != 0)
		delete[] x;

	if (S != 0)
		cs_free(S);
	if (N != 0)
	{

		if (N->L != 0)
			cs_spfree(N->L);
		if (N->U != 0)
			cs_spfree(N->U);
		if (N->pinv != 0)
			free(N->pinv);
		if (N->B != 0)
			free(N->B);
		free(N);
	}
}

ACADOcsparse* ACADOcsparse::clone() const
{

	return new ACADOcsparse(*this);
}

returnValue ACADOcsparse::solve(double *b)
{
	// CONSISTENCY CHECKS:
	// -------------------
	if (dim <= 0)
		return ACADOERROR(RET_MEMBER_NOT_INITIALISED);
	if (nDense <= 0)
		return ACADOERROR(RET_MEMBER_NOT_INITIALISED);
	if (S == 0)
		return ACADOERROR(RET_MEMBER_NOT_INITIALISED);

	// CASE: LU

	cs_ipvec(N->pinv, b, x, dim); /* x = b(p) */
	cs_lsolve(N->L, x); /* x = L\x  */
	cs_usolve(N->U, x); /* x = U\x  */
	cs_ipvec(S->q, x, b, dim); /* b(q) = x */

	return SUCCESSFUL_RETURN;
}

returnValue ACADOcsparse::solveTranspose(double *b)
{
	// CONSISTENCY CHECKS:
	// -------------------
	if (dim <= 0)
		return ACADOERROR(RET_MEMBER_NOT_INITIALISED);
	if (nDense <= 0)
		return ACADOERROR(RET_MEMBER_NOT_INITIALISED);
	if (S == 0)
		return ACADOERROR(RET_MEMBER_NOT_INITIALISED);

	// CASE: LU

	cs_ipvec(N->pinv, b, x, dim); /* x = b(p) */
	cs_utsolve(N->U, x); /* x = U'\x */
	cs_ltsolve(N->L, x); /* x = L'\x */
	cs_ipvec(S->q, x, b, dim); /* b(q) = x */

	return SUCCESSFUL_RETURN;
}

returnValue ACADOcsparse::setDimension(const int &n)
{
	dim = n;

	if (x != 0)
	{
		delete[] x;
		x = 0;
	}
	x = new double[dim];

	return SUCCESSFUL_RETURN;
}

returnValue ACADOcsparse::setNumberOfEntries(const int &nDense_)
{
	nDense = nDense_;
	return SUCCESSFUL_RETURN;
}

returnValue ACADOcsparse::setIndices(const int *rowIdx_, const int *colIdx_)
{
	if (index1 != 0)
		delete[] index1;
	if (index2 != 0)
		delete[] index2;

	int run1;

	index1 = new int[nDense];
	index2 = new int[nDense];

	for (run1 = 0; run1 < nDense; run1++)
	{
		index1[run1] = rowIdx_[run1];
		index2[run1] = colIdx_[run1];
	}
	return SUCCESSFUL_RETURN;
}

returnValue ACADOcsparse::setMatrix(double *A_)
{
	int run1;
	int order = 0;

	if (dim <= 0)
		return ACADOERROR(RET_MEMBER_NOT_INITIALISED);
	if (nDense <= 0)
		return ACADOERROR(RET_MEMBER_NOT_INITIALISED);

	cs *C, *D;
	C = cs_spalloc(0, 0, 1, 1, 1);

	for (run1 = 0; run1 < nDense; run1++)
		cs_entry(C, index1[run1], index2[run1], A_[run1]);

	D = cs_compress(C);
	S = cs_sqr(order, D, 0);
	N = cs_lu(D, S, TOL);

	cs_spfree(C);
	cs_spfree(D);

	return SUCCESSFUL_RETURN;
}

returnValue ACADOcsparse::getX(double *x_)
{
	int run1;
	for (run1 = 0; run1 < dim; run1++)
		x_[run1] = x[run1];

	return SUCCESSFUL_RETURN;
}

returnValue ACADOcsparse::setTolerance(double TOL_)
{
	TOL = TOL_;
	return SUCCESSFUL_RETURN;
}

returnValue ACADOcsparse::setPrintLevel(PrintLevel printLevel_)
{
	printLevel = printLevel_;
	return SUCCESSFUL_RETURN;
}

CLOSE_NAMESPACE_ACADO

#else // __MATLAB__

BEGIN_NAMESPACE_ACADO

//
// PUBLIC MEMBER FUNCTIONS:
//

ACADOcsparse::ACADOcsparse( )
{

}

ACADOcsparse::ACADOcsparse( const ACADOcsparse &arg )
{}

ACADOcsparse::~ACADOcsparse( )
{}

ACADOcsparse* ACADOcsparse::clone() const
{
	return new ACADOcsparse(*this);
}

returnValue ACADOcsparse::solve( double *b )
{
	return ACADOERROR(RET_NOT_IMPLEMENTED_YET);
}

returnValue ACADOcsparse::setDimension( const int &n )
{
	return ACADOERROR(RET_NOT_IMPLEMENTED_YET);
}

returnValue ACADOcsparse::setNumberOfEntries( const int &nDense_ )
{
	return ACADOERROR(RET_NOT_IMPLEMENTED_YET);
}

returnValue ACADOcsparse::setIndices( const int *rowIdx_, const int *colIdx_ )
{
	return ACADOERROR(RET_NOT_IMPLEMENTED_YET);
}

returnValue ACADOcsparse::setMatrix( double *A_ )
{
	return ACADOERROR(RET_NOT_IMPLEMENTED_YET);
}

returnValue ACADOcsparse::solveTranspose( double *b )
{
	return ACADOERROR(RET_NOT_IMPLEMENTED_YET);
}

returnValue ACADOcsparse::getX( double *x_ )
{
	return ACADOERROR(RET_NOT_IMPLEMENTED_YET);
}

returnValue ACADOcsparse::setTolerance( double TOL_ )
{
	return ACADOERROR(RET_NOT_IMPLEMENTED_YET);
}

returnValue ACADOcsparse::setPrintLevel( PrintLevel printLevel_ )
{
	return ACADOERROR(RET_NOT_IMPLEMENTED_YET);
}

CLOSE_NAMESPACE_ACADO

#endif // __MATLAB__


/*
 *   end of file
 */
