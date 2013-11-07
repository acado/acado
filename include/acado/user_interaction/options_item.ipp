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
 *    \file include/acado/user_interaction/options_item.ipp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 12.06.2008
 */


//
// PUBLIC MEMBER FUNCTIONS:
//


BEGIN_NAMESPACE_ACADO



inline OptionsName OptionsItem::getName( ) const
{
	return name;
}


inline OptionsItemType OptionsItem::getType( ) const
{
	return type;
}



inline returnValue OptionsItem::setName( OptionsName _name )
{
	name = _name;
	return SUCCESSFUL_RETURN;
}


inline returnValue OptionsItem::setType( OptionsItemType _type )
{
	type = _type;
	return SUCCESSFUL_RETURN;
}



inline OptionsItem* OptionsItem::getNext( ) const
{
	return next;
}


inline returnValue OptionsItem::setNext( OptionsItem* const _next )
{
	next = _next;
	return SUCCESSFUL_RETURN;
}


CLOSE_NAMESPACE_ACADO


/*
 *	end of file
 */
