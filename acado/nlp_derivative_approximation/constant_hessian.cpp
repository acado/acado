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
 *    \file src/nlp_derivative_approximation/constant_hessian.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *
 */


#include <acado/nlp_derivative_approximation/constant_hessian.hpp>



BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

ConstantHessian::ConstantHessian( ) : NLPderivativeApproximation( )
{
}


ConstantHessian::ConstantHessian( UserInteraction* _userInteraction ) : NLPderivativeApproximation( _userInteraction )
{
}


ConstantHessian::ConstantHessian( const ConstantHessian& rhs ) : NLPderivativeApproximation( rhs )
{
}


ConstantHessian::~ConstantHessian( )
{
}


ConstantHessian& ConstantHessian::operator=( const ConstantHessian& rhs )
{
	if ( this != &rhs )
	{
		NLPderivativeApproximation::operator=( rhs );
	}

	return *this;
}


NLPderivativeApproximation* ConstantHessian::clone( ) const
{
	return new ConstantHessian( *this );
}



returnValue ConstantHessian::initHessian(	BlockMatrix& B,
											uint N,
											const OCPiterate& iter
											)
{
	if( N > 1 )
	{
		for( uint run1=0; run1<N; ++run1 )
		{
			if ( iter.getNX() != 0 )
				B.setIdentity( run1,run1, iter.getNX() );

			if ( iter.getNXA() != 0 )
				B.setIdentity( N+run1,N+run1, iter.getNXA() );

			if ( ( iter.getNP() != 0 ) && ( run1 != N-1 ) )
				B.setIdentity( 2*N+run1,2*N+run1, iter.getNP() );

			if ( ( iter.getNU() != 0 ) && ( run1 != N-1 ) )
				B.setIdentity( 3*N+run1,3*N+run1, iter.getNU() );

			if ( ( iter.getNW() != 0 ) && ( run1 != N-1 ) )
				B.setIdentity( 4*N+run1,4*N+run1, iter.getNW() );
		}
	}
	else
	{
		if ( iter.getNP() != 0 )
			B.setIdentity( 2,2, iter.getNP() );

		if ( iter.getNU() != 0 )
			B.setIdentity( 3,3, iter.getNU() );

		if ( iter.getNW() != 0 )
			B.setIdentity( 4,4, iter.getNW() );
	}

	return SUCCESSFUL_RETURN;
}


returnValue ConstantHessian::initScaling(	BlockMatrix& B,
											const BlockMatrix& x,
											const BlockMatrix& y
											)
{
    DMatrix scale1, scale2;

    (x^x).getSubBlock(0,0,scale1,1,1);
    (y^y).getSubBlock(0,0,scale2,1,1);

    if ( ( scale1(0,0) <= 1000.0*EPS ) || ( scale2(0,0) <= 1000.0*EPS ) )
	{
		hessianScaling = 1.0;
	}
	else
	{
		hessianScaling = sqrt( scale2(0,0)/scale1(0,0) );
	}

	B *= hessianScaling;

    return SUCCESSFUL_RETURN;
}


returnValue ConstantHessian::apply(	BlockMatrix &B,
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
