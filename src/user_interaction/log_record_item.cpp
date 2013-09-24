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
 *    \file src/user_interaction/log_record_item.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 12.06.2008
 */


#include <acado/user_interaction/log_record_item.hpp>



BEGIN_NAMESPACE_ACADO



//
// PUBLIC MEMBER FUNCTIONS:
//

LogRecordItem::LogRecordItem( )
{
	name = -1;
	type = LRT_UNKNOWN;
	
	label = 0;
	startString = 0;
	endString = 0;

	width = 0;
	precision = 0;

	colSeparator = 0;
	rowSeparator = 0;

	writeProtected = BT_FALSE;

	next = 0;
}


LogRecordItem::LogRecordItem(	LogName _name,
								const char* const _label,
								const char* const _startString,
								const char* const _endString,
								uint _width,
								uint _precision,
								const char* const _colSeparator,
								const char* const _rowSeparator
								)
{
	name = (int) _name;
	type  = LRT_ENUM;

	label = 0;
	startString = 0;
	endString = 0;

	colSeparator = 0;
	rowSeparator = 0;

	acadoAssignString( &label,_label,DEFAULT_LABEL );
	acadoAssignString( &startString,_startString,DEFAULT_START_STRING );
	acadoAssignString( &endString,_endString,DEFAULT_END_STRING );

	assignDigits( width,_width,DEFAULT_WIDTH );
	assignDigits( precision,_precision,DEFAULT_PRECISION );

	acadoAssignString( &colSeparator,_colSeparator,DEFAULT_COL_SEPARATOR );
	acadoAssignString( &rowSeparator,_rowSeparator,DEFAULT_ROW_SEPARATOR );

	writeProtected = BT_FALSE;

	next = 0;
}


LogRecordItem::LogRecordItem(	const Expression& _name,
								const char* const _label,
								const char* const _startString,
								const char* const _endString,
								uint _width,
								uint _precision,
								const char* const _colSeparator,
								const char* const _rowSeparator
								)
{
	name = _name.getComponent( 0 );
	type  = LRT_VARIABLE;

	label = 0;
	startString = 0;
	endString = 0;

	colSeparator = 0;
	rowSeparator = 0;

	acadoAssignString( &label,_label,DEFAULT_LABEL );
	acadoAssignString( &startString,_startString,DEFAULT_START_STRING );
	acadoAssignString( &endString,_endString,DEFAULT_END_STRING );

	assignDigits( width,_width,DEFAULT_WIDTH );
	assignDigits( precision,_precision,DEFAULT_PRECISION );

	acadoAssignString( &colSeparator,_colSeparator,DEFAULT_COL_SEPARATOR );
	acadoAssignString( &rowSeparator,_rowSeparator,DEFAULT_ROW_SEPARATOR );

	writeProtected = BT_FALSE;

	next = 0;
}


LogRecordItem::LogRecordItem( const LogRecordItem& rhs )
{
	values = rhs.values;

	name  = rhs.name;
	type  = rhs.type;

	label = 0;
	startString = 0;
	endString = 0;

	colSeparator = 0;
	rowSeparator = 0;

	acadoAssignString( &label,rhs.label,DEFAULT_LABEL );
	acadoAssignString( &startString,rhs.startString,DEFAULT_START_STRING );
	acadoAssignString( &endString,rhs.endString,DEFAULT_END_STRING );

	assignDigits( width,rhs.width,DEFAULT_WIDTH );
	assignDigits( precision,rhs.precision,DEFAULT_PRECISION );

	acadoAssignString( &colSeparator,rhs.colSeparator,DEFAULT_COL_SEPARATOR );
	acadoAssignString( &rowSeparator,rhs.rowSeparator,DEFAULT_ROW_SEPARATOR );

	writeProtected = rhs.writeProtected;

	next  = 0;
}


LogRecordItem::~LogRecordItem( )
{
	if ( label != 0 )
		delete[] label;

	if ( startString != 0 )
		delete[] startString;

	if ( endString != 0 )
		delete[] endString;

	if ( colSeparator != 0 )
		delete[] colSeparator;

	if ( rowSeparator != 0 )
		delete[] rowSeparator;
}


LogRecordItem& LogRecordItem::operator=( const LogRecordItem& rhs )
{
	if ( this != &rhs )
	{
		if ( label != 0 )
			delete[] label;

		if ( startString != 0 )
			delete[] startString;
	
		if ( endString != 0 )
			delete[] endString;

		if ( colSeparator != 0 )
			delete[] colSeparator;
	
		if ( rowSeparator != 0 )
			delete[] rowSeparator;


		values = rhs.values;
	
		name  = rhs.name;
		type  = rhs.type;

		label = 0;
		startString = 0;
		endString = 0;
	
		colSeparator = 0;
		rowSeparator = 0;

		acadoAssignString( &label,rhs.label,DEFAULT_LABEL );
		acadoAssignString( &startString,rhs.startString,DEFAULT_START_STRING );
		acadoAssignString( &endString,rhs.endString,DEFAULT_END_STRING );
	
		assignDigits( width,rhs.width,DEFAULT_WIDTH );
		assignDigits( precision,rhs.precision,DEFAULT_PRECISION );
	
		acadoAssignString( &colSeparator,rhs.colSeparator,DEFAULT_COL_SEPARATOR );
		acadoAssignString( &rowSeparator,rhs.rowSeparator,DEFAULT_ROW_SEPARATOR );

		writeProtected = rhs.writeProtected;
	
		next  = 0;
	}

	return *this;
}


BooleanType LogRecordItem::operator==(	const LogRecordItem& rhs
										) const
{
	if ( name != rhs.name )
		return BT_FALSE;

	if ( type != rhs.type )
		return BT_FALSE;

	if ( acadoIsEqual( label,rhs.label ) == BT_FALSE )
		return BT_FALSE;

	if ( acadoIsEqual( startString,rhs.startString ) == BT_FALSE )
		return BT_FALSE;

	if ( acadoIsEqual( endString,rhs.endString ) == BT_FALSE )
		return BT_FALSE;

	if ( width != rhs.width )
		return BT_FALSE;

	if ( precision != rhs.precision )
		return BT_FALSE;

	if ( acadoIsEqual( colSeparator,rhs.colSeparator ) == BT_FALSE )
		return BT_FALSE;

	if ( acadoIsEqual( rowSeparator,rhs.rowSeparator ) == BT_FALSE )
		return BT_FALSE;

	// no check for writeProtected and next

	return BT_TRUE;
}


BooleanType LogRecordItem::operator!=(	const LogRecordItem& rhs
										) const
{
	if ( operator==( rhs ) == BT_TRUE )
		return BT_FALSE;
	else
		return BT_TRUE;
}


returnValue LogRecordItem::setAllValues(	LogFrequency _frequency,
											const MatrixVariablesGrid& _values
											)
{
	if ( _values.getNumPoints( ) <= 0 )
		return values.init( );

	switch( _frequency )
	{
		case LOG_AT_START:
			values.init();
			values.addMatrix( _values.getFirstMatrix( ),_values.getFirstTime( ) );
			break;

		case LOG_AT_END:
			values.init();
			values.addMatrix( _values.getLastMatrix( ),_values.getFirstTime( ) );
			break;

		case LOG_AT_EACH_ITERATION:
			values = _values;
			break;
	}
	return SUCCESSFUL_RETURN;
}


returnValue LogRecordItem::setValue(	LogFrequency _frequency,
										const Matrix& _value,
										double _time
										)
{
	double logTime = _time;
	
	switch( _frequency )
	{
		case LOG_AT_START:
			// only log if no matrix has been logged so far
			if ( getNumPoints( ) == 0 )
			{
				if ( acadoIsEqual( logTime,-INFTY ) == BT_TRUE )
					logTime = 0.0;
				
				values.addMatrix( _value,logTime );
			}
			break;

		case LOG_AT_END:
			// always overwrite existing matrices in order to keep only the last one
			values.init();
			
			if ( acadoIsEqual( logTime,-INFTY ) == BT_TRUE )
				logTime = 0.0;
			
			values.addMatrix( _value,logTime );
			break;

		case LOG_AT_EACH_ITERATION:
			// add matrix to list
			if ( acadoIsEqual( logTime,-INFTY ) == BT_TRUE )
				logTime = (double)getNumPoints() + 1.0;

			values.addMatrix( _value,logTime );
			break;
	}
	return SUCCESSFUL_RETURN;
}




//
// PROTECTED MEMBER FUNCTIONS:
//

returnValue LogRecordItem::assignDigits(	uint& toDigit,
											uint fromDigit,
											uint defaultDigit
											)
{
//	if ( fromDigit >= 0 )
		toDigit = fromDigit; 
//	else
//		toDigit = defaultDigit;

	return SUCCESSFUL_RETURN;
}



CLOSE_NAMESPACE_ACADO


/*
 *	end of file
 */
