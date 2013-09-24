/*
 *    This file is part of ACADO Toolkit.
 *
 *    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
 *    Copyright (C) 2008-2013 by Boris Houska, Hans Joachim Ferreau,
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
 *    \file src/user_interaction/options_list.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 12.06.2008
 */


#include <acado/utils/acado_utils.hpp>

#include <acado/user_interaction/options_list.hpp>
#include <acado/user_interaction/options_item_int.hpp>
#include <acado/user_interaction/options_item_double.hpp>


BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//


OptionsList::OptionsList( )
{
	first = 0;
	last  = 0;

	number = 0;

	optionsHaveChanged = BT_FALSE;
}


OptionsList::OptionsList( const OptionsList& rhs )
{
	first = 0;
	last = 0;
	number = 0;

	OptionsItem* current = rhs.first;

	/* if rhs options list is not empty, add all option items... */
	if ( current == 0 )
		return;

	while ( current != 0 )
	{
		if ( current->getType( ) == OIT_INT )
		{
			int currentValue;
			current->getValue( currentValue );
			add( current->getName( ),currentValue );
		}
		else
		{
			double currentValue;
			current->getValue( currentValue );
			add( current->getName( ),currentValue );
		}

		current = current->getNext( );
	}

	optionsHaveChanged = rhs.optionsHaveChanged;
}


OptionsList::~OptionsList( )
{
	OptionsItem* current = first;
	OptionsItem* tmp;

	/* deallocate all OptionItems within list */
	while ( current != 0 )
	{
		tmp = current->getNext( );
		delete current;
		current = tmp;
	}
}


OptionsList& OptionsList::operator=( const OptionsList& rhs )
{
	if ( this != &rhs )
	{
		OptionsItem* current = first;
		OptionsItem* tmp;

		/* deallocate all OptionItems within list */
		while ( current != 0 )
		{
			tmp = current->getNext( );
			delete current;
			current = tmp;
		}


		first = 0;
		last = 0;
		number = 0;

		current = rhs.first;

		/* if rhs options list is not empty, add all option items... */
		if ( current == 0 )
			return *this;

		while ( current != 0 )
		{
			if ( current->getType( ) == OIT_INT )
			{
				int currentValue;
				current->getValue( currentValue );
				add( current->getName( ),currentValue );
			}
			else
			{
				double currentValue;
				current->getValue( currentValue );
				add( current->getName( ),currentValue );
			}

			current = current->getNext( );
		}

		optionsHaveChanged = rhs.optionsHaveChanged;
	}

	return *this;
}



returnValue OptionsList::add(	OptionsName name,
								int value
								)
{
	// checks if item already exists, and if so set it to new value instead of adding it
	if ( find( name,OIT_INT ) != 0 )
		return set( name,value );
	//	return ACADOERROR( RET_OPTION_ALREADY_EXISTS );

	// create new item
	OptionsItemInt* newItem = new OptionsItemInt( name,value );

	if ( number == 0 )
	{
		first = newItem;
		last = newItem;
	}
	else
	{
		if ( last->setNext( newItem ) != SUCCESSFUL_RETURN )
			return ACADOERROR( RET_OPTIONS_LIST_CORRUPTED );
		last = newItem;
	}

	++number;

	return SUCCESSFUL_RETURN;
}



returnValue OptionsList::add(	OptionsName name,
								double value
								)
{
	// checks if item already exists, and if so set it to new value instead of adding it
	if ( find( name,OIT_DOUBLE ) != 0 )
		return set( name,value );
//		return ACADOERROR( RET_OPTION_ALREADY_EXISTS );

	// create new item
	OptionsItemDouble* newItem = new OptionsItemDouble( name,value );

	if ( number == 0 )
	{
		first = newItem;
		last = newItem;
	}
	else
	{
		if ( last->setNext( newItem ) != SUCCESSFUL_RETURN )
			return ACADOERROR( RET_OPTIONS_LIST_CORRUPTED );
		last = newItem;
	}

	++number;

	return SUCCESSFUL_RETURN;
}


returnValue OptionsList::get(	OptionsName name,
								int& value
								) const
{
	OptionsItem* item = find( name,OIT_INT );

	// checks if item exists
	if ( item == 0 ) {
		return ACADOERROR( RET_OPTION_DOESNT_EXIST );
	}

	return item->getValue( value );
}


returnValue OptionsList::get(	OptionsName name,
								double& value
								) const
{
	OptionsItem* item = find( name,OIT_DOUBLE );

	// checks if item exists
	if ( item == 0 )
		return ACADOERROR( RET_OPTION_DOESNT_EXIST );

	return item->getValue( value );
}



returnValue OptionsList::set(	OptionsName name,
								int value
								)
{
	OptionsItem* item = find( name,OIT_INT );

	// checks if item exists
	if ( item == 0 )
		return ACADOERROR( RET_OPTION_DOESNT_EXIST );

	if ( item->setValue( value ) != SUCCESSFUL_RETURN )
		return ACADOERROR( RET_OPTIONS_LIST_CORRUPTED );

	optionsHaveChanged = BT_TRUE;

	return SUCCESSFUL_RETURN;
}


returnValue OptionsList::set(	OptionsName name,
								double value
								)
{
	OptionsItem* item = find( name,OIT_DOUBLE );

	// checks if item exists
	if ( item == 0 )
		return ACADOERROR( RET_OPTION_DOESNT_EXIST );

	if ( item->setValue( value ) != SUCCESSFUL_RETURN )
		return ACADOERROR( RET_OPTIONS_LIST_CORRUPTED );

	optionsHaveChanged = BT_TRUE;

	return SUCCESSFUL_RETURN;
}


returnValue OptionsList::setOptions( const OptionsList &arg )
{
	operator=( arg );
	return SUCCESSFUL_RETURN;
}



returnValue OptionsList::printOptionsList( ) const
{
	OptionsItem* current = first;

	acadoPrintf( "\nThis class provides the following %d user options:\n",number );

	while ( current != 0 )
	{
		if ( current->getType( ) == OIT_INT )
		{
			int curVal;
			current->getValue( curVal );
			acadoPrintf( "  --> set( \"%-22d\", <int>    );  current value: %d\n",current->getName(),curVal );
		}
		else
		{
			double curVal;
			current->getValue( curVal );
			acadoPrintf( "  --> set( \"%-22d\", <double> );  current value: %e\n",current->getName(),curVal );
		}

		current = current->getNext( );
	}

	acadoPrintf( "\n" );

	return SUCCESSFUL_RETURN;
}




//
// PROTECTED MEMBER FUNCTIONS:
//

OptionsItem* OptionsList::find(	OptionsName name,
								OptionsItemType type
								) const
{
	OptionsItem* item = first;

	uint i;
	for( i=0; i<number; ++i )
	{
		if ( item == 0 )
			return 0;

		if ( ( name == item->getName( ) ) && ( item->getType( ) == type ) )
			return item;

		item = item->getNext( );
	}

	return 0;
}



CLOSE_NAMESPACE_ACADO


/*
 *	end of file
 */
