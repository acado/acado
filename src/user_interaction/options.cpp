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
 *    \file src/user_interaction/options.cpp
 *    \author Hans Joachim Ferreau, Boris Houska, Milan Vukov
 *    \date 2008 - 2013
 */

#include <acado/user_interaction/options.hpp>



BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//


Options::Options( )
{
	lists.push_back( OptionsList() );
}


Options::Options(	const OptionsList& _lists
					)
{
	lists.push_back( _lists );
}


Options::~Options( )
{}

returnValue Options::addOptionsList( )
{
	lists.push_back( lists[ 0 ] );
	
	return SUCCESSFUL_RETURN;
}


returnValue Options::get(	OptionsName name,
							int& value
							) const
{
	return lists[0].get( name,value );
}


returnValue Options::get(	OptionsName name,
							double& value
							) const
{
	return lists[0].get( name,value );
}

returnValue Options::get(	OptionsName name,
							std::string& value
							) const
{
	return lists[0].get( name,value );
}


returnValue Options::get(	uint idx,
							OptionsName name,
							int& value
							) const
{
	if ( idx >= getNumOptionsLists( ) )
		return ACADOERROR( RET_INDEX_OUT_OF_BOUNDS );

	return lists[idx].get( name,value );
}


returnValue Options::get(	uint idx,
							OptionsName name,
							double& value
							) const
{
	if ( idx >= getNumOptionsLists( ) )
		return ACADOERROR( RET_INDEX_OUT_OF_BOUNDS );
	
	return lists[idx].get( name,value );
}

returnValue Options::get(	uint idx,
							OptionsName name,
							std::string& value
							) const
{
	if ( idx >= getNumOptionsLists( ) )
		return ACADOERROR( RET_INDEX_OUT_OF_BOUNDS );

	return lists[idx].get( name,value );
}

returnValue Options::set(	OptionsName name,
							int value
							)
{
	return lists[0].set( name,value );
}


returnValue Options::set(	OptionsName name,
							double value
							)
{	
	return lists[0].set( name,value );
}

returnValue Options::set(	OptionsName name,
							const std::string& value
							)
{
	return lists[0].set( name,value );
}

returnValue Options::set(	uint idx,
							OptionsName name,
							int value
							)
{
	if ( idx >= getNumOptionsLists( ) )
		return ACADOERROR( RET_INDEX_OUT_OF_BOUNDS );

	return lists[idx].set( name,value );
}


returnValue Options::set(	uint idx,
							OptionsName name,
							double value
							)
{
	if ( idx >= getNumOptionsLists( ) )
		return ACADOERROR( RET_INDEX_OUT_OF_BOUNDS );
	
	return lists[idx].set( name,value );
}

returnValue Options::set(	uint idx,
							OptionsName name,
							const std::string& value
							)
{
	if ( idx >= getNumOptionsLists( ) )
		return ACADOERROR( RET_INDEX_OUT_OF_BOUNDS );

	return lists[idx].set( name,value );
}


returnValue Options::setOptions(	const Options &arg
									)
{
	operator=( arg );
	return SUCCESSFUL_RETURN;
}


returnValue Options::setOptions(	uint idx,
									const Options &arg
									)
{
	if ( ( idx >= getNumOptionsLists( ) ) || ( idx >= arg.getNumOptionsLists( ) ) )
		return ACADOERROR( RET_INDEX_OUT_OF_BOUNDS );

	lists[ idx ] = arg.lists[ idx ];

	return SUCCESSFUL_RETURN;
}


Options Options::getOptions(	uint idx
								) const
{
	if ( idx >= getNumOptionsLists( ) )
	{
		ACADOERROR( RET_INDEX_OUT_OF_BOUNDS );
		return Options();
	}

	return Options( lists[idx] );
}


returnValue Options::printOptionsList( ) const
{
	returnValue returnvalue;
	
	for( uint i=0; i<getNumOptionsLists( ); ++i )
	{
		returnvalue = lists[i].printOptionsList( );
		if ( returnvalue != SUCCESSFUL_RETURN )
			return returnvalue;
	}
	
	return SUCCESSFUL_RETURN;
}


returnValue Options::printOptionsList(	uint idx
										) const
{
	if ( idx >= getNumOptionsLists( ) )
		return ACADOERROR( RET_INDEX_OUT_OF_BOUNDS );

	return lists[idx].printOptionsList( );
}


//
// PROTECTED MEMBER FUNCTIONS:
//

returnValue Options::setupOptions( )
{
	return SUCCESSFUL_RETURN;
}

BooleanType Options::haveOptionsChanged( ) const
{
	for( uint i=0; i<getNumOptionsLists( ); ++i )
	{
		if ( lists[i].haveOptionsChanged( ) == BT_TRUE )
			return BT_TRUE;
	}
	
	return BT_FALSE;
}


BooleanType Options::haveOptionsChanged(	uint idx
											) const
{
	if ( idx >= getNumOptionsLists( ) )
	{
		ACADOERROR( RET_INDEX_OUT_OF_BOUNDS );
		return BT_FALSE;
	}

	return lists[idx].haveOptionsChanged( );
}


returnValue Options::declareOptionsUnchanged( )
{
	returnValue returnvalue;
	
	for( uint i=0; i<getNumOptionsLists( ); ++i )
	{
		returnvalue = lists[i].declareOptionsUnchanged( );
		if ( returnvalue != SUCCESSFUL_RETURN )
			return returnvalue;
	}
	
	return SUCCESSFUL_RETURN;
}


returnValue Options::declareOptionsUnchanged(	uint idx
												)
{
	if ( idx >= getNumOptionsLists( ) )
		return ACADOERROR( RET_INDEX_OUT_OF_BOUNDS );

	return lists[idx].declareOptionsUnchanged( );
}


returnValue Options::addOption(	OptionsName name,
								int value
								)
{
	returnValue returnvalue;
	
	for( uint i=0; i<getNumOptionsLists( ); ++i )
	{
		returnvalue = lists[i].add( name,value );
		if ( returnvalue != SUCCESSFUL_RETURN )
			return returnvalue;
	}
	
	return SUCCESSFUL_RETURN;
}


returnValue Options::addOption(	OptionsName name,
								double value
								)
{
	returnValue returnvalue;
	
	for( uint i=0; i<getNumOptionsLists( ); ++i )
	{
		returnvalue = lists[i].add( name,value );
		if ( returnvalue != SUCCESSFUL_RETURN )
			return returnvalue;
	}
	
	return SUCCESSFUL_RETURN;
}

returnValue Options::addOption(	OptionsName name,
								const std::string& value
								)
{
	returnValue returnvalue;

	for( uint i=0; i<getNumOptionsLists( ); ++i )
	{
		returnvalue = lists[i].add( name,value );
		if ( returnvalue != SUCCESSFUL_RETURN )
			return returnvalue;
	}

	return SUCCESSFUL_RETURN;
}



returnValue Options::addOption(	uint idx,
								OptionsName name,
								int value
								)
{
	if ( idx >= getNumOptionsLists( ) )
		return ACADOERROR( RET_INDEX_OUT_OF_BOUNDS );

	return lists[idx].add( name,value );
}


returnValue Options::addOption(	uint idx,
								OptionsName name,
								double value
								)
{
	if ( idx >= getNumOptionsLists( ) )
		return ACADOERROR( RET_INDEX_OUT_OF_BOUNDS );

	return lists[idx].add( name,value );
}

returnValue Options::addOption(	uint idx,
								OptionsName name,
								const std::string& value
								)
{
	if ( idx >= getNumOptionsLists( ) )
		return ACADOERROR( RET_INDEX_OUT_OF_BOUNDS );

	return lists[idx].add( name,value );
}

uint Options::getNumOptionsLists() const
{
	return lists.size();
}

CLOSE_NAMESPACE_ACADO


/*
 *	end of file
 */
