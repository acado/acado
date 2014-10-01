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
 *    \file src/control_law/clipping_functionality.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 13.06.2008
 */


#include <acado/control_law/clipping_functionality.hpp>



BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//


ClippingFunctionality::ClippingFunctionality( )
{
}


ClippingFunctionality::ClippingFunctionality(	uint _nU,
												uint _nP
												)
{
	lowerLimitControls.init( _nU );
	upperLimitControls.init( _nU );
	lowerLimitControls.setAll( -INFTY );
	upperLimitControls.setAll(  INFTY );

	lowerLimitParameters.init( _nP );
	upperLimitParameters.init( _nP );
	lowerLimitParameters.setAll( -INFTY );
	upperLimitParameters.setAll(  INFTY );
}


ClippingFunctionality::ClippingFunctionality( const ClippingFunctionality& rhs )
{
	lowerLimitControls = rhs.lowerLimitControls;
	upperLimitControls = rhs.upperLimitControls;

	lowerLimitParameters = rhs.lowerLimitParameters;
	upperLimitParameters = rhs.upperLimitParameters;
}


ClippingFunctionality::~ClippingFunctionality( )
{
}


ClippingFunctionality& ClippingFunctionality::operator=( const ClippingFunctionality& rhs )
{
	if ( this != &rhs )
	{
		lowerLimitControls = rhs.lowerLimitControls;
		upperLimitControls = rhs.upperLimitControls;
	
		lowerLimitParameters = rhs.lowerLimitParameters;
		upperLimitParameters = rhs.upperLimitParameters;
	}

    return *this;
}



returnValue ClippingFunctionality::setControlLowerLimits(	const DVector& _lowerLimit
															)
{
	if ( _lowerLimit.getDim( ) != getNumControlLimits( ) )
		return ACADOERROR( RET_VECTOR_DIMENSION_MISMATCH );

	lowerLimitControls = _lowerLimit;

	return SUCCESSFUL_RETURN;
}


returnValue ClippingFunctionality::setControlLowerLimit(	uint idx,
															double _lowerLimit
															)
{
	if ( idx >= getNumControlLimits( ) )
		return ACADOERROR( RET_INDEX_OUT_OF_BOUNDS );

	lowerLimitControls( idx ) = _lowerLimit;
	return SUCCESSFUL_RETURN;
}


returnValue ClippingFunctionality::setControlUpperLimits(	const DVector& _upperLimit
															)
{
	if ( _upperLimit.getDim( ) != getNumControlLimits( ) )
		return ACADOERROR( RET_VECTOR_DIMENSION_MISMATCH );

	upperLimitControls = _upperLimit;

	return SUCCESSFUL_RETURN;
}


returnValue ClippingFunctionality::setControlUpperLimit(	uint idx,
																double _upperLimit
																)
{
	if ( idx >= getNumControlLimits( ) )
		return ACADOERROR( RET_INDEX_OUT_OF_BOUNDS );

	upperLimitControls( idx ) = _upperLimit;
	return SUCCESSFUL_RETURN;
}



returnValue ClippingFunctionality::setParameterLowerLimits(	const DVector& _lowerLimit
															)
{
	if ( _lowerLimit.getDim( ) != getNumParameterLimits( ) )
		return ACADOERROR( RET_VECTOR_DIMENSION_MISMATCH );

	lowerLimitParameters = _lowerLimit;

	return SUCCESSFUL_RETURN;
}


returnValue ClippingFunctionality::setParameterLowerLimit(	uint idx,
															double _lowerLimit
															)
{
	if ( idx >= getNumParameterLimits( ) )
		return ACADOERROR( RET_INDEX_OUT_OF_BOUNDS );

	lowerLimitParameters( idx ) = _lowerLimit;
	return SUCCESSFUL_RETURN;
}


returnValue ClippingFunctionality::setParameterUpperLimits(	const DVector& _upperLimit
															)
{
	if ( _upperLimit.getDim( ) != getNumParameterLimits( ) )
		return ACADOERROR( RET_VECTOR_DIMENSION_MISMATCH );

	upperLimitParameters = _upperLimit;

	return SUCCESSFUL_RETURN;
}


returnValue ClippingFunctionality::setParameterUpperLimit(	uint idx,
															double _upperLimit
															)
{
	if ( idx >= getNumParameterLimits( ) )
		return ACADOERROR( RET_INDEX_OUT_OF_BOUNDS );

	upperLimitParameters( idx ) = _upperLimit;
	return SUCCESSFUL_RETURN;
}



//
// PROTECTED MEMBER FUNCTIONS:
//



returnValue ClippingFunctionality::clipSignals(	VariablesGrid& _u,
												VariablesGrid& _p
												)
{
	// consistency checks
	if ( _u.getNumValues( ) != getNumControlLimits( ) )
		return ACADOERROR( RET_VECTOR_DIMENSION_MISMATCH );

	if ( _p.getNumValues( ) != getNumParameterLimits( ) )
		return ACADOERROR( RET_VECTOR_DIMENSION_MISMATCH );


	// limit controls
	for( uint i=0; i<getNumControlLimits( ); ++i )
	{
		for( uint j=0; j<_u.getNumPoints( ); ++j )
		{
			if ( _u( j,i ) < lowerLimitControls( i ) )
				_u( j,i ) = lowerLimitControls( i );

			if ( _u( j,i ) > upperLimitControls( i ) )
				_u( j,i ) = upperLimitControls( i );
		}
	}


	// limit parameters
	for( uint i=0; i<getNumParameterLimits( ); ++i )
	{
		for( uint j=0; j<_p.getNumPoints( ); ++j )
		{
			if ( _p( j,i ) < lowerLimitParameters( i ) )
				_p( j,i ) = lowerLimitParameters( i );

			if ( _p( j,i ) > upperLimitParameters( i ) )
				_p( j,i ) = upperLimitParameters( i );
		}
	}

	return SUCCESSFUL_RETURN;
}


returnValue ClippingFunctionality::clipSignals(	DVector& _u,
												DVector& _p
												)
{
	// consistency checks
	if ( _u.getDim( ) != getNumControlLimits( ) )
		return ACADOERROR( RET_VECTOR_DIMENSION_MISMATCH );

	if ( _p.getDim( ) != getNumParameterLimits( ) )
		return ACADOERROR( RET_VECTOR_DIMENSION_MISMATCH );


	// limit controls
	for( uint i=0; i<getNumControlLimits( ); ++i )
	{
		if ( _u( i ) < lowerLimitControls( i ) )
			_u( i ) = lowerLimitControls( i );

		if ( _u( i ) > upperLimitControls( i ) )
			_u( i ) = upperLimitControls( i );
	}


	// limit parameters
	for( uint i=0; i<getNumParameterLimits( ); ++i )
	{
		if ( _p( i ) < lowerLimitParameters( i ) )
			_p( i ) = lowerLimitParameters( i );

		if ( _p( i ) > upperLimitParameters( i ) )
			_p( i ) = upperLimitParameters( i );
	}

	return SUCCESSFUL_RETURN;
}


CLOSE_NAMESPACE_ACADO

// end of file.
