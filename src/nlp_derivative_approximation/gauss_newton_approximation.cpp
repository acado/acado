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
 *    \file src/nlp_derivative_approximation/gauss_newton_approximation.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *
 */


#include <acado/nlp_derivative_approximation/gauss_newton_approximation.hpp>



BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

GaussNewtonApproximation::GaussNewtonApproximation( ) : NLPderivativeApproximation( )
{
}


GaussNewtonApproximation::GaussNewtonApproximation( UserInteraction* _userInteraction ) : NLPderivativeApproximation( _userInteraction )
{
}


GaussNewtonApproximation::GaussNewtonApproximation( const GaussNewtonApproximation& rhs ) : NLPderivativeApproximation( rhs )
{
}


GaussNewtonApproximation::~GaussNewtonApproximation( )
{
}


GaussNewtonApproximation& GaussNewtonApproximation::operator=( const GaussNewtonApproximation& rhs )
{
	if ( this != &rhs )
	{
		NLPderivativeApproximation::operator=( rhs );
	}

	return *this;
}


NLPderivativeApproximation* GaussNewtonApproximation::clone( ) const
{
	return new GaussNewtonApproximation( *this );
}



returnValue GaussNewtonApproximation::initHessian(	BlockMatrix& B,
													uint N,
													const OCPiterate& iter
													)
{
	return SUCCESSFUL_RETURN;
}


returnValue GaussNewtonApproximation::initScaling(	BlockMatrix& B,
													const BlockMatrix& x,
													const BlockMatrix& y
													)
{
	return SUCCESSFUL_RETURN;
}



returnValue GaussNewtonApproximation::apply(	BlockMatrix &B,
												const BlockMatrix &x,
												const BlockMatrix &y
												)
{
	return SUCCESSFUL_RETURN;
}



//
// PROTECTED MEMBER FUNCTIONS:
//





CLOSE_NAMESPACE_ACADO

// end of file.
