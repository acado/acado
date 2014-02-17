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
 *    \file src/user_interaction/algorithmic_base.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 */


#include <acado/user_interaction/algorithmic_base.hpp>


BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//


AlgorithmicBase::AlgorithmicBase( )
{
	userInteraction = new UserInteraction;
	useModuleStandalone = BT_TRUE;
	
	outputLoggingIdx = -1;
}


AlgorithmicBase::AlgorithmicBase(	UserInteraction* _userInteraction
									)
{
	if ( _userInteraction == 0 )
	{
		userInteraction = new UserInteraction;
		useModuleStandalone = BT_TRUE;
	}
	else
	{
		userInteraction = _userInteraction;
		useModuleStandalone = BT_FALSE;
	}
	
	outputLoggingIdx = -1;
}


AlgorithmicBase::AlgorithmicBase( const AlgorithmicBase& rhs )
{
	if ( rhs.useModuleStandalone == BT_TRUE )
	{
		userInteraction = new UserInteraction( *(rhs.userInteraction) );
		useModuleStandalone = BT_TRUE;
	}
	else
	{
		userInteraction = rhs.userInteraction;
		useModuleStandalone = BT_FALSE;
	}
	
	outputLoggingIdx = rhs.outputLoggingIdx;
}


AlgorithmicBase::~AlgorithmicBase( )
{
	if ( useModuleStandalone == BT_TRUE )
	{
		if ( userInteraction != 0 )
		  delete userInteraction;
	}
}


AlgorithmicBase& AlgorithmicBase::operator=( const AlgorithmicBase& rhs )
{
	if ( this != &rhs )
	{
		if ( rhs.useModuleStandalone == BT_TRUE )
		{
			userInteraction = new UserInteraction( *(rhs.userInteraction) );
			useModuleStandalone = BT_TRUE;
		}
		else
		{
			userInteraction = rhs.userInteraction;
			useModuleStandalone = BT_FALSE;
		}
		
		outputLoggingIdx = rhs.outputLoggingIdx;
	}

	return *this;
}


returnValue AlgorithmicBase::addOptionsList( )
{
	return userInteraction->addOptionsList( );
}


returnValue AlgorithmicBase::set(	OptionsName name,
									int value
									)
{
	return userInteraction->set( name,value );
}


returnValue AlgorithmicBase::set(	OptionsName name,
									double value
									)
{	
	return userInteraction->set( name,value );
}

returnValue AlgorithmicBase::set(	OptionsName name,
									const std::string& value
									)
{
	return userInteraction->set( name,value );
}


returnValue AlgorithmicBase::set(	uint idx,
									OptionsName name,
									int value
									)
{
	return userInteraction->set( idx,name,value );
}


returnValue AlgorithmicBase::set(	uint idx,
									OptionsName name,
									double value
									)
{	
	return userInteraction->set( idx,name,value );
}


returnValue AlgorithmicBase::setOptions( const Options &arg )
{
	return userInteraction->setOptions( arg );
}


returnValue AlgorithmicBase::setOptions(	uint idx,
											const Options &arg
											)
{
	return userInteraction->setOptions( idx,arg );
}


Options AlgorithmicBase::getOptions(	uint idx
										) const
{
	return userInteraction->getOptions( idx );
}


//
// PROTECTED MEMBER FUNCTIONS:
//




CLOSE_NAMESPACE_ACADO


/*
 *	end of file
 */
