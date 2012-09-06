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
 *    \file src/code_generation/export_index.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 2011
 */


#include <acado/code_generation/export_index.hpp>
#include <acado/code_generation/export_argument.hpp>


BEGIN_NAMESPACE_ACADO


static const int undefinedValue = -16182;
static char exportDataString[1024];

static const uint maxLengthName = 20;


//
// PUBLIC MEMBER FUNCTIONS:
//


ExportIndex::ExportIndex(	const String& _name,
							ExportType _type,
							const int* const _value,
							const int* const _factor,
							const int* const _offset
							) : ExportData( )
{
	value  = 0;
	factor = 0;
	offset = 0;

	init( _name,_type,_value,_factor,_offset );
}


ExportIndex::ExportIndex(	int _value
							) : ExportData( )
{
	value  = 0;
	factor = 0;
	offset = 0;

	init( "run1",INT,&_value );
}


ExportIndex::ExportIndex( const ExportIndex& arg ) : ExportData( )
{
	value  = 0;
	factor = 0;
	offset = 0;

	init( arg.name, arg.type,arg.value, arg.factor, arg.offset );
}


ExportIndex::~ExportIndex( )
{
	clear( );
}


ExportIndex& ExportIndex::operator=( const ExportIndex& arg )
{
	if( this != &arg )
	{
		init( arg.name, arg.type, arg.value, arg.factor, arg.offset );
	}

	return *this;
}


ExportIndex& ExportIndex::operator=(	int arg
										)
{
	init("run1", INT, &arg, 0, 0);

	return *this;
}


ExportData* ExportIndex::clone( ) const
{
	return new ExportIndex(*this);
}



returnValue ExportIndex::init(	const String& _name,
								ExportType _type,
								const int* const _value,
								const int* const _factor,
								const int* const _offset
								)
{
	clear( );

	if ( _value != 0 )
	{
		if ( value != 0 )
			value[0] = _value[0];
		else
			value = new int( _value[0] );
	}
	
	if ( _factor != 0 )
	{
		if ( factor != 0 )
			factor[0] = _factor[0];
		else
			factor = new int( _factor[0] );
	}
	
	if ( _offset != 0 )
	{
		if ( offset != 0 )
			offset[0] = _offset[0];
		else
			offset = new int( _offset[0] );
	}
	
	ExportData::init( _name,_type );

	return SUCCESSFUL_RETURN;
}



returnValue ExportIndex::exportDataDeclaration(	FILE *file,
												const String& _realString,
												const String& _intString,
												int _precision
												) const
{
	// _realString, _intString, precision not used!
	if ( isGiven() == BT_FALSE )
		return acadoFPrintf( file,"%s %s;\n", getTypeString( _realString,_intString ).getName(),name.getName() );
	else
		return acadoFPrintf( file,"%s %s=%d;\n", getTypeString( _realString,_intString ).getName(),name.getName(),getGivenValue() );
}



const char* ExportIndex::get( ) const
{
	if ( isGiven( ) == BT_TRUE )
	{
		sprintf( exportDataString,"%d",getGivenValue() );
	}
	else
	{
		char tmp[60];
		
		sprintf( exportDataString,"%s",name.getName() );
		
		if ( factor != 0 )
		{
			if ( factor[0] < 0 )
			{
				sprintf( tmp,"*(%d)",factor[0] );
				strcat( exportDataString,tmp );
			}
			else
			{
				sprintf( tmp,"*%d",factor[0] );
				strcat( exportDataString,tmp );
			}
		}
		
		if ( offset != 0 )
		{
			if ( offset[0] > 0 )
			{
				sprintf( tmp,"+%d",offset[0] );
				strcat( exportDataString,tmp );
			}
			
			if ( offset[0] < 0 )
			{
				sprintf( tmp,"%d",offset[0] );
				strcat( exportDataString,tmp );
			}
		}
	}

	return exportDataString;
}


const int ExportIndex::getGivenValue( ) const
{
	int totalValue = undefinedValue;

	if ( isGiven( ) == BT_TRUE )
	{
		if ( value != 0 )
			totalValue = value[0];
		else
			totalValue = 0;
		
		if ( factor != 0 )
			totalValue *= factor[0];
		
		if ( offset != 0 )
			totalValue += offset[0];
	}

	return totalValue;
}



ExportIndex ExportIndex::operator+(	const ExportIndex& arg
									) const
{
// 	ASSERT( name == arg.name );
	ExportIndex tmp;

	if ( arg.isGiven( ) == BT_TRUE )
	{
		tmp = (*this) + arg.getGivenValue( );
	}
	else
	{
		if ( isGiven() == BT_TRUE )
		{
			tmp = arg + getGivenValue();
		}
		else
		{
			// both summand not given
			tmp = (*this);

			int addFactor = 1;
			if ( arg.factor != 0 )
				addFactor = arg.factor[0];
				
			if ( tmp.factor != 0 )
				tmp.factor[0] += addFactor;
			else
				tmp.factor = new int( 1+addFactor );
			
			if ( arg.offset != 0 )
			{
				if ( tmp.offset != 0 )
					tmp.offset[0] += arg.offset[0];
				else
					tmp.offset = new int( arg.offset[0] );
			}
		}
	}
	
	return tmp;
}


ExportIndex ExportIndex::operator-(	const ExportIndex& arg
									) const
{
// 	ASSERT( name == arg.name );
	ExportIndex tmp;

	if ( arg.isGiven( ) == BT_TRUE )
	{
		tmp = (*this) - arg.getGivenValue( );
	}
	else
	{
		if ( isGiven() == BT_TRUE )
		{
			tmp = (arg*(-1)) + getGivenValue();
		}
		else
		{
			// both summand not given
			tmp = (*this);
			
			int addFactor = -1;
			if ( arg.factor != 0 )
				addFactor = -arg.factor[0];
				
			if ( tmp.factor != 0 )
				tmp.factor[0] += addFactor;
			else
				tmp.factor = new int( 1+addFactor );
			
			if ( arg.offset != 0 )
			{
				if ( tmp.offset != 0 )
					tmp.offset[0] -= arg.offset[0];
				else
					tmp.offset = new int( -arg.offset[0] );
			}
		}
	}
	
	return tmp;
}


ExportIndex ExportIndex::operator+(	int _offset
									) const
{
	ExportIndex tmp = *this;

	if ( tmp.offset != 0 )
		tmp.offset[0] += _offset;
	else
		tmp.offset = new int( _offset );

	return tmp;
}


ExportIndex ExportIndex::operator-(	int _offset
									) const
{
	ExportIndex tmp = *this;
	
	if ( tmp.offset != 0 )
		tmp.offset[0] -= _offset;
	else
		tmp.offset = new int( -_offset );

	return tmp;
}


ExportIndex ExportIndex::operator*(	int _factor
									) const
{
	ASSERT( _factor != 0 );

	ExportIndex tmp = *this;
	
	if ( tmp.factor != 0 )
		tmp.factor[0] *= _factor;
	else
		tmp.factor = new int( _factor );
	
	if ( tmp.offset != 0 )
		tmp.offset[0] *= _factor;

	return tmp;
}



BooleanType ExportIndex::isGiven( ) const
{
	if ( value == 0 )
	{
		if ( ( factor != 0 ) && ( factor[0] == 0 ) )
			return BT_TRUE;
		else
			return BT_FALSE;
	}
	else
	{
		return BT_TRUE;
	}
}



ExportArgument ExportIndex::makeArgument( ) const
{
	ExportArgument tmp( name,1,1, type,ACADO_LOCAL,BT_TRUE,emptyConstExportIndex );
	return tmp;
}



//
// PROTECTED MEMBER FUNCTIONS:
//

returnValue ExportIndex::clear( )
{
	if ( value != 0 )
	{
		delete value;
		value = 0;
	}

	if ( factor != 0 )
	{
		delete factor;
		factor = 0;
	}
	
	if ( offset != 0 )
	{
		delete offset;
		offset = 0;
	}

	return SUCCESSFUL_RETURN;
}



CLOSE_NAMESPACE_ACADO

// end of file.
