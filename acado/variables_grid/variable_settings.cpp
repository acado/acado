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
 *    \file src/variables_grid/variable_settings.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 20.08.2008
 */


#include <acado/variables_grid/variable_settings.hpp>



BEGIN_NAMESPACE_ACADO



//
// PUBLIC MEMBER FUNCTIONS:
//

VariableSettings::VariableSettings( )
{
	dim = 0;
	type = VT_UNKNOWN;

	names = 0;
	units = 0;

	autoInit = defaultAutoInit;
}


VariableSettings::VariableSettings(	uint _dim,
									VariableType _type,
									const char** const _names,
									const char** const _units,
									const DVector& _scaling,
									const DVector& _lb,
									const DVector& _ub,
									BooleanType _autoInit
									)
{
	names = 0;
	units = 0;
	
	init( _dim,_type,_names,_units,_scaling,_lb,_ub,_autoInit );
}


VariableSettings::VariableSettings( const VariableSettings& rhs )
{
	uint i;

	dim = rhs.dim;
	type = rhs.type;

	if ( rhs.names != 0 )
	{
		names = (char**) calloc( dim,sizeof(char*) );
		for( i=0; i<dim; ++i )
		{
			names[i] = new char[MAX_LENGTH_NAME+1];
			setName( i,rhs.names[i] );
		}
	}
	else
	{
		names = 0;
	}

	if ( rhs.units != 0 )
	{
		units = (char**) calloc( dim,sizeof(char*) );
		for( i=0; i<dim; ++i )
		{
			units[i] = new char[MAX_LENGTH_NAME+1];
			setUnit( i,rhs.units[i] );
		}
	}
	else
	{
		units = 0;
	}


	if ( rhs.scaling.isEmpty( ) == BT_FALSE )
	{
		scaling.init( dim );
		setScaling( rhs.scaling );
	}
	else
	{
		scaling.init( );
	}

	if ( rhs.lb.isEmpty( ) == BT_FALSE )
	{
		lb.init( dim );
		setLowerBounds( rhs.lb );
	}
	else
	{
		lb.init( );
	}

	if ( rhs.ub.isEmpty( ) == BT_FALSE )
	{
		ub.init( dim );
		setUpperBounds( rhs.ub );
	}
	else
	{
		ub.init( );
	}

	autoInit = rhs.autoInit;
}


VariableSettings::~VariableSettings( )
{
	clear( );
}


VariableSettings& VariableSettings::operator=( const VariableSettings& rhs )
{
	uint i;

    if ( this != &rhs )
    {
		clear( );

		dim = rhs.dim;
		type = rhs.type;

		if ( rhs.names != 0 )
		{
			names = (char**) calloc( dim,sizeof(char*) );
			for( i=0; i<dim; ++i )
			{
				names[i] = new char[MAX_LENGTH_NAME+1];
				setName( i,rhs.names[i] );
			}
		}
		else
		{
			names = 0;
		}

		if ( rhs.units != 0 )
		{
			units = (char**) calloc( dim,sizeof(char*) );
			for( i=0; i<dim; ++i )
			{
				units[i] = new char[MAX_LENGTH_NAME+1];
				setUnit( i,rhs.units[i] );
			}
		}
		else
		{
			units = 0;
		}

		if ( rhs.scaling.isEmpty( ) == BT_FALSE )
		{
			scaling.init( dim );
			setScaling( rhs.scaling );
		}
		else
		{
			scaling.init( );
		}

		if ( rhs.lb.isEmpty( ) == BT_FALSE )
		{
			lb.init( dim );
			setLowerBounds( rhs.lb );
		}
		else
		{
			lb.init( );
		}

		if ( rhs.ub.isEmpty( ) == BT_FALSE )
		{
			ub.init( dim );
			setUpperBounds( rhs.ub );
		}
		else
		{
			ub.init( );
		}

		autoInit = rhs.autoInit;
    }

    return *this;
}



returnValue VariableSettings::init( )
{
	clear( );

	dim = 0;
	type = VT_UNKNOWN;

	names = 0;
	units = 0;

    autoInit = defaultAutoInit;

    return SUCCESSFUL_RETURN;
}


returnValue VariableSettings::init(	uint _dim,
									VariableType _type,
									const char** const _names,
									const char** const _units,
									const DVector& _scaling,
									const DVector& _lb,
									const DVector& _ub,
									BooleanType _autoInit
									)
{
	init( );

	type = _type;
	autoInit = _autoInit;

	return appendSettings( _dim,_names,_units,_scaling,_lb,_ub );
}



returnValue VariableSettings::appendSettings(	const VariableSettings& rhs
												)
{
	return appendSettings( rhs.dim,(const char**)rhs.names,(const char**)rhs.units,rhs.scaling,rhs.lb,rhs.ub );
}


returnValue VariableSettings::appendSettings(	uint _dim,
												const char** const _names,
												const char** const _units,
												const DVector& _scaling,
												const DVector& _lb,
												const DVector& _ub
												)
{
	uint i;

	uint newDim = dim + _dim;

	if ( names != 0 )
	{
		names = (char**) realloc( names,newDim*sizeof(char*) );

		if ( _names != 0 )
		{
			for( i=0; i<_dim; ++i )
			{
				names[dim+i] = new char[MAX_LENGTH_NAME+1];
				setName( dim+i,_names[i] );
			}
		}
		else
		{
			for( i=0; i<_dim; ++i )
			{
				names[dim+i] = new char[MAX_LENGTH_NAME+1];
				setName( dim+i,defaultName );
			}
		}
	}
	else
	{
		if ( _names != 0 )
		{
			names = (char**) realloc( names,newDim*sizeof(char*) );

			for( i=0; i<dim; ++i )
			{
				names[i] = new char[MAX_LENGTH_NAME+1];
				setName( i,defaultName );
			}

			for( i=0; i<_dim; ++i )
			{
				names[dim+i] = new char[MAX_LENGTH_NAME+1];
				setName( dim+i,_names[i] );
			}
		}
	}

	if ( units != 0 )
	{
		units = (char**) realloc( units,newDim*sizeof(char*) );

		if ( _units != 0 )
		{
			for( i=0; i<_dim; ++i )
			{
				units[dim+i] = new char[MAX_LENGTH_NAME+1];
				setUnit( dim+i,_units[i] );
			}
		}
		else
		{
			for( i=0; i<_dim; ++i )
			{
				units[dim+i] = new char[MAX_LENGTH_NAME+1];
				setUnit( dim+i,defaultUnit );
			}
		}
	}
	else
	{
		if ( _units != 0 )
		{
			units = (char**) realloc( units,newDim*sizeof(char*) );

			for( i=0; i<dim; ++i )
			{
				units[i] = new char[MAX_LENGTH_NAME+1];
				setUnit( i,defaultUnit );
			}

			for( i=0; i<_dim; ++i )
			{
				units[dim+i] = new char[MAX_LENGTH_NAME+1];
				setUnit( dim+i,_units[i] );
			}
		}
	}

	if ( scaling.isEmpty( ) == BT_FALSE )
	{
		if ( _scaling.isEmpty( ) == BT_FALSE )
		{
			scaling.append( _scaling );
		}
		else
		{
			DVector tmp( _dim );
			tmp.setAll( defaultScaling );
			scaling.append( tmp );
		}
	}
	else
	{
		if ( _scaling.isEmpty( ) == BT_FALSE )
		{
			scaling.init( dim );
			scaling.setAll( defaultScaling );
			scaling.append( _scaling );
		}
	}

	if ( lb.isEmpty( ) == BT_FALSE )
	{
		if ( _lb.isEmpty( ) == BT_FALSE )
		{
			lb.append( _lb );
		}
		else
		{
			DVector tmp( _dim );
			tmp.setAll( defaultLowerBound );
			lb.append( tmp );
		}
	}
	else
	{
		if ( _lb.isEmpty( ) == BT_FALSE )
		{
			lb.init( dim );
			lb.setAll( defaultLowerBound );
			lb.append( _lb );
		}
	}

	if ( ub.isEmpty( ) == BT_FALSE )
	{
		if ( _ub.isEmpty( ) == BT_FALSE )
		{
			ub.append( _ub );
		}
		else
		{
			DVector tmp( _dim );
			tmp.setAll( defaultUpperBound );
			ub.append( tmp );
		}
	}
	else
	{
		if ( _ub.isEmpty( ) == BT_FALSE )
		{
			ub.init( dim );
			ub.setAll( defaultUpperBound );
			ub.append( _ub );
		}
	}

	dim = newDim;

    return SUCCESSFUL_RETURN;
}



returnValue VariableSettings::getName(	uint idx,
												char* _name
												) const
{
	if ( hasNames( ) != BT_TRUE )
		return ACADOERROR( RET_MEMBER_NOT_INITIALISED );

	if ( idx >= dim )
		return ACADOERROR( RET_INDEX_OUT_OF_BOUNDS );

	strncpy( _name,names[idx],MAX_LENGTH_NAME );
	_name[MAX_LENGTH_NAME] = '\0';

	return SUCCESSFUL_RETURN;
}


returnValue VariableSettings::setName(	uint idx,
												const char* const _name
												)
{
	if ( idx >= dim )
		return ACADOERROR( RET_INDEX_OUT_OF_BOUNDS );

	if ( _name == 0 )
		return ACADOERROR( RET_INVALID_ARGUMENTS );

	if ( hasNames( ) != BT_TRUE )
	{
		/* allocate names if necessary */
		names = new char*[dim];

		for( uint i=0; i<dim; ++i )
		{
			names[i] = new char[MAX_LENGTH_NAME+1];
			strncpy( names[i],defaultName,MAX_LENGTH_NAME );
		}
	}

	strncpy( names[idx],_name,MAX_LENGTH_NAME );
	names[MAX_LENGTH_NAME] = NULL;

	return SUCCESSFUL_RETURN;
}



returnValue VariableSettings::getUnit(	uint idx,
												char* _unit
												) const
{
	if ( idx >= dim )
		return ACADOERROR( RET_INDEX_OUT_OF_BOUNDS );

	if ( hasUnits( ) != BT_TRUE )
		return ACADOERROR( RET_MEMBER_NOT_INITIALISED );

	strncpy( _unit,units[idx],MAX_LENGTH_NAME );
	_unit[MAX_LENGTH_NAME] = '\0';

	return SUCCESSFUL_RETURN;
}


returnValue VariableSettings::setUnit(	uint idx,
												const char* const _unit
												)
{
	if ( idx >= dim )
		return ACADOERROR( RET_INDEX_OUT_OF_BOUNDS );

	if ( _unit == 0 )
		return ACADOERROR( RET_INVALID_ARGUMENTS );

	if ( hasUnits( ) != BT_TRUE )
	{
		/* allocate units if necessary */
		units = new char*[dim];

		for( uint i=0; i<dim; ++i )
		{
			units[i] = new char[MAX_LENGTH_NAME+1];
			strncpy( units[i],defaultUnit,MAX_LENGTH_NAME );
		}
	}

	strncpy( units[idx],_unit,MAX_LENGTH_NAME );
	units[MAX_LENGTH_NAME] = NULL;

	return SUCCESSFUL_RETURN;
}



//
// PROTECTED MEMBER FUNCTIONS:
//

returnValue VariableSettings::clear( )
{
	uint i;

	if ( names != 0 )
	{
		for( i=0; i<dim; ++i )
		{
			if ( names[i] != 0 )
				delete[] (names[i]);
		}

		free( names );
		names = 0;
	}

	if ( units != 0 )
	{
		for( i=0; i<dim; ++i )
		{
			if ( units[i] != 0 )
				delete[] (units[i]);
		}

		free( units );
		units = 0;
	}

	return SUCCESSFUL_RETURN;
}



CLOSE_NAMESPACE_ACADO

/*
 *	end of file
 */
