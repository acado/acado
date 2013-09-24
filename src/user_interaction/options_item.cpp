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
 *    \file src/user_interaction/options_item.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 12.06.2008
 */


#include <acado/utils/acado_utils.hpp>
#include <acado/user_interaction/options_item.hpp>


BEGIN_NAMESPACE_ACADO



//
// PUBLIC MEMBER FUNCTIONS:
//

OptionsItem::OptionsItem( )
{
	name = OPT_UNKNOWN;
	type = OIT_UNKNOWN;
		
	next = 0;
}


OptionsItem::OptionsItem(	OptionsName _name,
							OptionsItemType _type
							)
{
	name = _name;
	type = _type;

	next = 0;
}


OptionsItem::OptionsItem( const OptionsItem& rhs )
{
	name = rhs.name;
	type = rhs.type;
		
	next = rhs.next;
}


OptionsItem::~OptionsItem( )
{
}


OptionsItem& OptionsItem::operator=( const OptionsItem& rhs )
{
	if ( this != &rhs )
	{
		name = rhs.name;
		type = rhs.type;

		next = rhs.next;
	}

	return *this;
}



//
// PROTECTED MEMBER FUNCTIONS:
//




CLOSE_NAMESPACE_ACADO


/*
 *	end of file
 */
