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
 *    \file src/user_interaction/options_list.cpp
 *    \author Hans Joachim Ferreau, Boris Houska, Milan Vukov
 *    \date 2008 - 2013
 */


#include <acado/utils/acado_utils.hpp>

#include <acado/user_interaction/options_list.hpp>

using namespace std;

BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//


OptionsList::OptionsList( )
{
	optionsHaveChanged = BT_FALSE;
}


OptionsList::OptionsList( const OptionsList& rhs )
{
	optionsHaveChanged = rhs.optionsHaveChanged;
	items = rhs.items;
}


OptionsList::~OptionsList( )
{}


OptionsList& OptionsList::operator=( const OptionsList& rhs )
{
	if ( this != &rhs )
	{
		optionsHaveChanged = rhs.optionsHaveChanged;
		items = rhs.items;
	}

	return *this;
}


returnValue OptionsList::printOptionsList( ) const
{
	cout << "\nThis class provides the following" << items.size() << "user options:\n";

	OptionItems::const_iterator it;
	for (it = items.begin(); it != items.end(); ++it)
	{
		cout << "  --> set( \" ";
		switch (it->first.second)
		{
		case OIT_INT:
			cout << "\", <int>    );  current value: ";
			break;

		case OIT_DOUBLE:
			cout << "\", <double> );  current value: ";
			break;

		case OIT_STRING:
			cout << "\", <string> );  current value: ";
			break;

		default:
			return ACADOERROR(RET_UNKNOWN_BUG);

		}

		it->second->print( cout );
		cout << endl;
	}

	cout << endl;

	return SUCCESSFUL_RETURN;
}

CLOSE_NAMESPACE_ACADO

/*
 *	end of file
 */
