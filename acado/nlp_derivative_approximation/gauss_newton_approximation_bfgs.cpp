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
 *    \file src/nlp_derivative_approximation/gauss_newton_approximation_bfgs.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *
 */


#include <acado/nlp_derivative_approximation/bfgs_update.hpp>
#include <acado/nlp_derivative_approximation/gauss_newton_approximation_bfgs.hpp>


BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

GaussNewtonApproximationWithBFGS::GaussNewtonApproximationWithBFGS( ) : GaussNewtonApproximation( )
{
	bfgsUpdate = new BFGSupdate;
}


GaussNewtonApproximationWithBFGS::GaussNewtonApproximationWithBFGS(	UserInteraction* _userInteraction,
																	uint _nBlocks
																	) : GaussNewtonApproximation( _userInteraction )
{
	bfgsUpdate = new BFGSupdate( _userInteraction,_nBlocks );
}


GaussNewtonApproximationWithBFGS::GaussNewtonApproximationWithBFGS( const GaussNewtonApproximationWithBFGS& rhs ) : GaussNewtonApproximation( rhs )
{
	if ( rhs.bfgsUpdate != 0 )
		bfgsUpdate = new BFGSupdate( *(rhs.bfgsUpdate) );
	else
		bfgsUpdate = 0;
}


GaussNewtonApproximationWithBFGS::~GaussNewtonApproximationWithBFGS( )
{
	if ( bfgsUpdate != 0 )
		delete bfgsUpdate;
}


GaussNewtonApproximationWithBFGS& GaussNewtonApproximationWithBFGS::operator=( const GaussNewtonApproximationWithBFGS& rhs )
{
	if ( this != &rhs )
	{
		if ( bfgsUpdate != 0 )
			delete bfgsUpdate;

		GaussNewtonApproximation::operator=( rhs );

		if ( rhs.bfgsUpdate != 0 )
			bfgsUpdate = new BFGSupdate( *(rhs.bfgsUpdate) );
		else
			bfgsUpdate = 0;
	}

	return *this;
}


NLPderivativeApproximation* GaussNewtonApproximationWithBFGS::clone( ) const
{
	return new GaussNewtonApproximationWithBFGS( *this );
}



returnValue GaussNewtonApproximationWithBFGS::initHessian(	BlockMatrix& B,
															uint N,
															const OCPiterate& iter
															)
{
	return GaussNewtonApproximation::initHessian( B,N,iter );
}


returnValue GaussNewtonApproximationWithBFGS::initScaling(	BlockMatrix& B,
															const BlockMatrix& x,
															const BlockMatrix& y
															)
{
	if ( bfgsUpdate == 0 )
		return ACADOERROR( RET_MEMBER_NOT_INITIALISED );

	bfgsUpdate->initScaling( B,x,y );

	return GaussNewtonApproximation::initScaling( B,x,y );
}



returnValue GaussNewtonApproximationWithBFGS::apply(	BlockMatrix &B,
														const BlockMatrix &x,
														const BlockMatrix &y
														)
{
	if ( bfgsUpdate == 0 )
		return ACADOERROR( RET_MEMBER_NOT_INITIALISED );

	bfgsUpdate->apply( B,x,y );

	return GaussNewtonApproximation::apply( B,x,y );
}



//
// PROTECTED MEMBER FUNCTIONS:
//





CLOSE_NAMESPACE_ACADO

// end of file.
