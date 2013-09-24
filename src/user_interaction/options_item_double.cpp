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
 *    \file src/user_interaction/options_item_double.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 12.06.2008
 */


#include <acado/utils/acado_utils.hpp>
#include <acado/user_interaction/options_item_double.hpp>



BEGIN_NAMESPACE_ACADO



//
// PUBLIC MEMBER FUNCTIONS:
//

OptionsItemDouble::OptionsItemDouble( ) : OptionsItem( )
{
	value = 0.0;
}


OptionsItemDouble::OptionsItemDouble(	OptionsName const _name,
										double _value
										) : OptionsItem( _name,OIT_DOUBLE )
{
	value = _value;
}


OptionsItemDouble::OptionsItemDouble( const OptionsItemDouble& rhs ) : OptionsItem( rhs )
{
	value = rhs.value;
}


OptionsItemDouble::~OptionsItemDouble( )
{
}


OptionsItemDouble& OptionsItemDouble::operator=( const OptionsItemDouble& rhs )
{
	if ( this != &rhs )
	{
		OptionsItem::operator=( rhs );

		value = rhs.value;
	}

	return *this;
}



returnValue OptionsItemDouble::getValue( int& _value ) const{

	_value = 0;
	return ACADOERROR( RET_OPTION_DOESNT_EXIST );
}


returnValue OptionsItemDouble::getValue( double& _value ) const
{
	_value = value;

	return SUCCESSFUL_RETURN;
}


returnValue OptionsItemDouble::setValue( int _value){

	_value = 0;
	return ACADOERROR( RET_OPTION_DOESNT_EXIST );
}


returnValue OptionsItemDouble::setValue(	double _value
											)
{
	value = _value;

	return SUCCESSFUL_RETURN;
}



//
// PROTECTED MEMBER FUNCTIONS:
//



CLOSE_NAMESPACE_ACADO


/*
 *	end of file
 */
