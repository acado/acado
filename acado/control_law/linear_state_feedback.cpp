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
 *    \file src/control_law/linear_state_feedback.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 20.08.2008
 */


#include <acado/control_law/linear_state_feedback.hpp>



BEGIN_NAMESPACE_ACADO



//
// PUBLIC MEMBER FUNCTIONS:
//

LinearStateFeedback::LinearStateFeedback( ) : ControlLaw( ), ClippingFunctionality(  )
{
	setStatus( BS_NOT_INITIALIZED );
}


LinearStateFeedback::LinearStateFeedback(	const DMatrix& _K,
											double _samplingTime
											) : ControlLaw( _samplingTime ), ClippingFunctionality( _K.getNumRows() )
{
	K = _K;
	setStatus( BS_READY );
}


LinearStateFeedback::LinearStateFeedback( const LinearStateFeedback& rhs ) : ControlLaw( rhs ), ClippingFunctionality( rhs )
{
	K = rhs.K;
}


LinearStateFeedback::~LinearStateFeedback( )
{
}


LinearStateFeedback& LinearStateFeedback::operator=( const LinearStateFeedback& rhs )
{
	if ( this != &rhs )
	{
		ControlLaw::operator=( rhs );
		ClippingFunctionality::operator=( rhs );

		K = rhs.K;
	}

    return *this;
}


ControlLaw* LinearStateFeedback::clone( ) const
{
	return new LinearStateFeedback( *this );
}


returnValue LinearStateFeedback::init(	double startTime,
										const DVector &x0_,
										const DVector &p_,
										const VariablesGrid& _yRef
										)
{
	setStatus( BS_READY );
	return SUCCESSFUL_RETURN;
}


returnValue LinearStateFeedback::step(	double currentTime,
										const DVector& _x,
										const DVector& _p,
										const VariablesGrid& _yRef
										)
{
	if ( getStatus( ) != BS_READY )
		return ACADOERROR( RET_BLOCK_NOT_READY );

	if ( _x.getDim( ) != getNX( ) )
		return ACADOERROR( RET_VECTOR_DIMENSION_MISMATCH );


	/* 1) Use reference trajectory if it is defined */
	// set default reference to zero
	DVector xRef( _x );
	
	if ( _yRef.getNumPoints( ) > 0 )
	{
		if ( _yRef.getNumValues( ) != getNX( ) )
			return ACADOERROR( RET_VECTOR_DIMENSION_MISMATCH );

		xRef = _yRef.getVector( 0 );
	}
	else
	{
		xRef.setZero( );
	}


	/* 2) Linear state feedback. */
	u = K * ( xRef - _x );
	p = _p;


	/* 3) Call output transformator. */
	if ( clipSignals( u,p ) != SUCCESSFUL_RETURN )
		return ACADOERROR( RET_CONTROLLAW_STEP_FAILED );
	
	return SUCCESSFUL_RETURN;
}



uint LinearStateFeedback::getNX( ) const
{
	return K.getNumCols( );
}


uint LinearStateFeedback::getNXA( ) const
{
	return 0;
}


uint LinearStateFeedback::getNU( ) const
{
	return K.getNumRows( );
}


uint LinearStateFeedback::getNP( ) const
{
	return 0;
}


uint LinearStateFeedback::getNW( ) const
{
	return 0;
}


uint LinearStateFeedback::getNY( ) const
{
	return getNX( );
}


BooleanType LinearStateFeedback::isDynamic( ) const
{
	return BT_FALSE;
}


BooleanType LinearStateFeedback::isStatic( ) const
{
	if ( isDynamic() == BT_TRUE )
		return BT_FALSE;
	else
		return BT_TRUE;
}



//
// PROTECTED MEMBER FUNCTIONS:
//




CLOSE_NAMESPACE_ACADO

// end of file.
