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
 *    \file src/control_law/pid_controller.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 20.08.2008
 */


#include <acado/control_law/pid_controller.hpp>



BEGIN_NAMESPACE_ACADO



//
// PUBLIC MEMBER FUNCTIONS:
//

PIDcontroller::PIDcontroller( ) : ControlLaw( ), ClippingFunctionality( )
{
	nInputs  = 0;
	nOutputs = 0;

	setStatus( BS_NOT_INITIALIZED );
}


PIDcontroller::PIDcontroller(	uint _nInputs,
								uint _nOutputs,
								double _samplingTime
								) : ControlLaw( _samplingTime ), ClippingFunctionality( _nOutputs )
{
	if ( ( _nOutputs != _nInputs ) && ( _nOutputs != 1 ) )
	{
		_nOutputs = 1;
		ACADOERROR( RET_INVALID_PID_OUTPUT_DIMENSION );
	}

	nInputs  = _nInputs;
	nOutputs = _nOutputs;

	pWeights.init( nInputs );
	pWeights.setZero( );

	iWeights.init( nInputs );
	iWeights.setZero( );

	dWeights.init( nInputs );
	dWeights.setZero( );

	iValue.init( nInputs );
	iValue.setZero( );

	lastError.init( nInputs );
	lastError.setZero( );

	setStatus( BS_READY );
}


PIDcontroller::PIDcontroller( const PIDcontroller& rhs ) : ControlLaw( rhs ), ClippingFunctionality( rhs )
{
	nInputs  = rhs.nInputs;
	nOutputs = rhs.nOutputs;

	pWeights = rhs.pWeights;
	iWeights = rhs.iWeights;
	dWeights = rhs.dWeights;

	iValue    = rhs.iValue;
	lastError = rhs.lastError;
}


PIDcontroller::~PIDcontroller( )
{
}


PIDcontroller& PIDcontroller::operator=( const PIDcontroller& rhs )
{
	if ( this != &rhs )
	{
		ControlLaw::operator=( rhs );
		ClippingFunctionality::operator=( rhs );

		nInputs  = rhs.nInputs;
		nOutputs = rhs.nOutputs;
		
		pWeights = rhs.pWeights;
		iWeights = rhs.iWeights;
		dWeights = rhs.dWeights;

		iValue    = rhs.iValue;
		lastError = rhs.lastError;
	}

    return *this;
}


ControlLaw* PIDcontroller::clone( ) const
{
	return new PIDcontroller( *this );
}



returnValue PIDcontroller::setProportionalWeights(	const DVector& _pWeights
													)
{
	if ( _pWeights.getDim() != getNumInputs( ) )
		return ACADOERROR( RET_VECTOR_DIMENSION_MISMATCH );

	pWeights = _pWeights;
	
	return SUCCESSFUL_RETURN;
}


returnValue PIDcontroller::setIntegralWeights(		const DVector& _iWeights
													)
{
	if ( _iWeights.getDim() != getNumInputs( ) )
		return ACADOERROR( RET_VECTOR_DIMENSION_MISMATCH );

	iWeights = _iWeights;
	
	return SUCCESSFUL_RETURN;
}


returnValue PIDcontroller::setDerivativeWeights(	const DVector& _dWeights
													)

{
	if ( _dWeights.getDim() != getNumInputs( ) )
		return ACADOERROR( RET_VECTOR_DIMENSION_MISMATCH );

	dWeights = _dWeights;
	
	return SUCCESSFUL_RETURN;
}


returnValue PIDcontroller::init(	double startTime,
									const DVector &x0_,
									const DVector &p_,
									const VariablesGrid& _yRef
									)
{
	if ( x0_.getDim( ) != getNumInputs( ) )
		return ACADOERROR( RET_VECTOR_DIMENSION_MISMATCH );

	// Use reference trajectory if it is defined, 
	// otherwise set default reference to zero
	DVector xRef( x0_.getDim() );

	if ( _yRef.getNumPoints( ) > 0 )
	{
		if ( _yRef.getNumValues( ) != getNumInputs( ) )
			return ACADOERROR( RET_VECTOR_DIMENSION_MISMATCH );

		xRef = _yRef.getVector( 0 );
	}
	else
	{
		xRef.setZero( );
	}


	// initialize control and parameter signals
	u.init( getNumOutputs() );
	u.setZero( );
	
	p = p_;


	lastError = xRef - x0_;

	setStatus( BS_READY );

	return SUCCESSFUL_RETURN;
}


returnValue PIDcontroller::step(	double currentTime,
									const DVector& _x,
									const DVector& _p,
									const VariablesGrid& _yRef
									)
{
	if ( getStatus( ) != BS_READY )
		return ACADOERROR( RET_BLOCK_NOT_READY );

	if ( _x.getDim( ) != getNumInputs( ) )
		return ACADOERROR( RET_VECTOR_DIMENSION_MISMATCH );


	/* 1) Use reference trajectory if it is defined */
	// set default reference to zero
	DVector xRef( _x.getDim() );

	if ( _yRef.getNumPoints( ) > 0 )
	{
		if ( _yRef.getNumValues( ) != getNumInputs( ) )
			return ACADOERROR( RET_VECTOR_DIMENSION_MISMATCH );

		xRef = _yRef.getVector( 0 );
	}
	else
	{
		xRef.setZero( );
	}


	/* 2) Determine PID control action. */
	if ( getNumOutputs( ) > 0 )
	{
		if ( determineControlAction( xRef-_x,u ) != SUCCESSFUL_RETURN )
			return ACADOERROR( RET_CONTROLLAW_STEP_FAILED );
	}
	else
		u.init();

	p = _p;


	/* 3) Call output transformator. */
	if ( clipSignals( u,p ) != SUCCESSFUL_RETURN )
		return ACADOERROR( RET_OUTPUTTRANSFORMATOR_STEP_FAILED );

	return SUCCESSFUL_RETURN;
}



uint PIDcontroller::getNX( ) const
{
	return getNumInputs( );
}


uint PIDcontroller::getNXA( ) const
{
	return 0;
}


uint PIDcontroller::getNU( ) const
{
	return getNumOutputs( );
}


uint PIDcontroller::getNP( ) const
{
	return 0;
}


uint PIDcontroller::getNW( ) const
{
	return 0;
}


uint PIDcontroller::getNY( ) const
{
	return getNX( );
}


BooleanType PIDcontroller::isDynamic( ) const
{
	return BT_FALSE;
}


BooleanType PIDcontroller::isStatic( ) const
{
	if ( isDynamic() == BT_TRUE )
		return BT_FALSE;
	else
		return BT_TRUE;
}



//
// PROTECTED MEMBER FUNCTIONS:
//

returnValue PIDcontroller::determineControlAction(	const DVector& error,
													DVector& output
													)
{
	uint i;
	double tmp;
	
	output.init( getNumOutputs() );
	output.setZero( );

	// update integral value
	for( i=0; i<getNumInputs(); ++i )
		iValue(i) += error(i) * getSamplingTime( );

	// determine outputs
	for( i=0; i<getNumInputs(); ++i )
	{
		tmp  = pWeights(i) * error(i);
		tmp += iWeights(i) * iValue(i);
		tmp += dWeights(i) * (error(i) - lastError(i)) / getSamplingTime( );

		if ( getNumOutputs( ) > 1 )
			output(i) = tmp;
		else
			output(0) += tmp;
	}

	// update last error
	lastError = error;

	return SUCCESSFUL_RETURN;
}



CLOSE_NAMESPACE_ACADO

// end of file.
