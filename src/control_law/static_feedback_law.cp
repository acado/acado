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
 *    \file src/control_law/static_feedback_law.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 20.08.2008
 */


#include <acado/control_law/static_feedback_law.hpp>



BEGIN_NAMESPACE_ACADO



//
// PUBLIC MEMBER FUNCTIONS:
//

StaticFeedbackLaw::StaticFeedbackLaw( ) : ControlLaw( )
{
	staticFeedbackFcnU = 0;
	staticFeedbackFcnP = 0;
	
	setStatus( BS_NOT_INITIALIZED );
}


StaticFeedbackLaw::StaticFeedbackLaw(	const OutputFcn& _staticFeedbackFcnU,
										double _samplingTime
										) : ControlLaw( _samplingTime )
{
	staticFeedbackFcnU = new OutputFcn( _staticFeedbackFcnU );
	staticFeedbackFcnP = 0;
	
	setStatus( BS_NOT_INITIALIZED );
}


StaticFeedbackLaw::StaticFeedbackLaw(	const OutputFcn& _staticFeedbackFcnU,
										const OutputFcn& _staticFeedbackFcnP,
										double _samplingTime
										) : ControlLaw( _samplingTime )
{
	staticFeedbackFcnU = new OutputFcn( _staticFeedbackFcnU );
	staticFeedbackFcnP = new OutputFcn( _staticFeedbackFcnP );
	
	setStatus( BS_NOT_INITIALIZED );
}


StaticFeedbackLaw::StaticFeedbackLaw( const StaticFeedbackLaw& rhs ) : ControlLaw( rhs )
{
	if ( rhs.staticFeedbackFcnU != 0 )
		staticFeedbackFcnU = new OutputFcn( *(rhs.staticFeedbackFcnU) );
	else
		staticFeedbackFcnU = 0;

	if ( rhs.staticFeedbackFcnP != 0 )
		staticFeedbackFcnP = new OutputFcn( *(rhs.staticFeedbackFcnP) );
	else
		staticFeedbackFcnP = 0;
}


StaticFeedbackLaw::~StaticFeedbackLaw( )
{
	if ( staticFeedbackFcnU != 0 )
		delete staticFeedbackFcnU;

	if ( staticFeedbackFcnP != 0 )
		delete staticFeedbackFcnP;
}


StaticFeedbackLaw& StaticFeedbackLaw::operator=( const StaticFeedbackLaw& rhs )
{
	if ( this != &rhs )
	{
		ControlLaw::operator=( rhs );
	}

    return *this;
}


ControlLaw* StaticFeedbackLaw::clone( ) const
{
	return new StaticFeedbackLaw( *this );
}


returnValue StaticFeedbackLaw::init(	double startTime,
										const Vector &x0_,
										const Vector &p_,
										const VariablesGrid& _yRef
										)
{
	return THROWERROR( RET_NOT_YET_IMPLEMENTED );
}


returnValue StaticFeedbackLaw::step(	double currentTime,
										const Vector& _x,
										const Vector& _p,
										const VariablesGrid& _yRef,
										const VariablesGrid& _yRefNext
										)
{
	return THROWERROR( RET_NOT_YET_IMPLEMENTED );
}




uint StaticFeedbackLaw::getNX( ) const
{
	return staticFeedbackFcnU->getN( );
}


uint StaticFeedbackLaw::getNXA( ) const
{
	return staticFeedbackFcnU->getNXA( );
}


uint StaticFeedbackLaw::getNU( ) const
{
	return staticFeedbackFcnU->getDim( );
}


uint StaticFeedbackLaw::getNP( ) const
{
	return staticFeedbackFcnP->getDim( );
}


uint StaticFeedbackLaw::getNW( ) const
{
	return staticFeedbackFcnU->getNW( );
}


uint StaticFeedbackLaw::getNY( ) const
{
	return staticFeedbackFcnU->getNX( ); // CHANGE TO NY()!!!!!!
}


BooleanType StaticFeedbackLaw::isDynamic( ) const
{
	return BT_FALSE;
}


BooleanType StaticFeedbackLaw::isStatic( ) const
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
