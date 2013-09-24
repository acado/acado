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
 *    \file src/user_interaction/log_record.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 12.06.2008
 */


#include <acado/user_interaction/log_record_item.hpp>

#include <acado/user_interaction/log_record.hpp>



BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//


LogRecord::LogRecord( )
{
	next     =  0;
	aliasIdx = -1;

	frequency      = LOG_AT_EACH_ITERATION;
	outputFile     = stdout;
	outputFileName = 0;
	printScheme    = PS_DEFAULT;

	first = 0;
	last  = 0;

	number = 0;
}


LogRecord::LogRecord(	LogFrequency _frequency,
						FILE* _outputFile,
						PrintScheme _printScheme
						)
{
	next     =  0;
	aliasIdx = -1;

	frequency      = _frequency;
	outputFile     = _outputFile;
	outputFileName = 0;
	printScheme    = _printScheme;

	first = 0;
	last  = 0;

	number = 0;
}


LogRecord::LogRecord(	LogFrequency _frequency,
						const char* _outputFileName,
						PrintScheme _printScheme
						)
{
	next     =  0;
	aliasIdx = -1;

	frequency      = _frequency;
	outputFile     = 0;
	outputFileName = 0;
	acadoAssignString( &outputFileName,_outputFileName,"default.log" );
	printScheme    = _printScheme;

	first = 0;
	last  = 0;

	number = 0;
}


LogRecord::LogRecord( const LogRecord& rhs )
{
	next     =  0;
	aliasIdx = rhs.aliasIdx; //-1

	frequency      = rhs.frequency;
	outputFile     = rhs.outputFile;
	outputFileName = 0;
	acadoAssignString( &outputFileName,rhs.outputFileName,"default.log" );
	printScheme    = rhs.printScheme;

	first = 0;
	last  = 0;

	number = 0;

	/* if rhs log_record list is not empty, add all option items... */
	LogRecordItem* current = rhs.first;

	while ( current != 0 )
	{
		addItem( *current );
		current = current->getNext( );
	}
}


LogRecord::~LogRecord( )
{
	clearAllItems( );

	if ( outputFileName != 0 )
		delete[] outputFileName;
}


LogRecord& LogRecord::operator=( const LogRecord& rhs )
{
	if ( this != &rhs )
	{
		clearAllItems( );

		if ( outputFileName != 0 )
			delete[] outputFileName;

		next     =  0;
		aliasIdx = rhs.aliasIdx; //-1

		frequency      = rhs.frequency;
		outputFile     = rhs.outputFile;
		outputFileName = 0;
		acadoAssignString( &outputFileName,rhs.outputFileName,"default.log" );
		printScheme    = rhs.printScheme;

		/* if rhs log_record list is not empty, add all option items... */
		LogRecordItem* current = rhs.first;

		while ( current != 0 )
		{
			addItem( *current );
			current = current->getNext( );
		}
	}

	return *this;
}


BooleanType LogRecord::operator==(	const LogRecord& rhs
									) const
{
	// no check for next
	// no check for aliasIdx

	if ( frequency != rhs.frequency )
		return BT_FALSE;

	if ( outputFile != rhs.outputFile )
		return BT_FALSE;

	if ( acadoIsEqual( outputFileName,rhs.outputFileName ) == BT_FALSE )
		return BT_FALSE;

	if ( printScheme != rhs.printScheme )
		return BT_FALSE;


	if ( number != rhs.number )
		return BT_FALSE;

	LogRecordItem* current = first;
	LogRecordItem* rhsCurrent = rhs.first;

	while ( ( current != 0 ) && ( rhsCurrent != 0 ) )
	{
		if ( (*current) != (*rhsCurrent) )
			return BT_FALSE;

		current = current->getNext( );
		rhsCurrent = rhsCurrent->getNext( );
	}

	return BT_TRUE;
}


BooleanType LogRecord::operator!=(	const LogRecord& rhs
										)
{
	if ( operator==( rhs ) == BT_TRUE )
		return BT_FALSE;
	else
		return BT_TRUE;
}



returnValue LogRecord::operator<<(	LogRecordItem& _item
									)
{
	return addItem( _item );
}

returnValue LogRecord::operator<<(	LogName _name
									)
{
	return addItem( _name );
}


returnValue LogRecord::operator<<(	const Expression& _name
									)
{
	return addItem( _name );
}



returnValue LogRecord::addItem(	LogRecordItem& _item
								)
{
	// checks if item already exists
	if ( hasItem( _item.getName( ),_item.getType( ) ) == BT_TRUE )
		return SUCCESSFUL_RETURN;

	// create new item
	LogRecordItem* newItem = new LogRecordItem( _item );

	if ( number == 0 )
	{
		first = newItem;
		last = newItem;
	}
	else
	{
		if ( last->setNext( newItem ) != SUCCESSFUL_RETURN )
			return ACADOERROR( RET_LOG_RECORD_CORRUPTED );
		last = newItem;
	}

	++number;

	return SUCCESSFUL_RETURN;
}



returnValue LogRecord::addItem(	LogName _name,
								const char* const _label
								)
{
	return addItem( _name,printScheme,_label );
}


returnValue LogRecord::addItem(	LogName _name,
								PrintScheme _printScheme,
								const char* const _label
								)
{
	char* _startString = 0;
	char* _endString = 0;
	uint _width = 0;
	uint _precision = 0;
	char* _colSeparator = 0;
	char* _rowSeparator = 0;

	returnValue returnvalue;
	returnvalue = getGlobalStringDefinitions( _printScheme,&_startString,&_endString,
											  _width,_precision,&_colSeparator,&_rowSeparator );

	if ( returnvalue == SUCCESSFUL_RETURN )
	{
		returnvalue = addItem( _name,_label,_startString,_endString,_width,_precision,_colSeparator,_rowSeparator );
	}

	if ( _startString != 0 )   delete[] _startString;
	if ( _endString != 0 )     delete[] _endString;
	if ( _colSeparator != 0 )  delete[] _colSeparator;
	if ( _rowSeparator != 0 )  delete[] _rowSeparator;

	return returnvalue;
}


returnValue LogRecord::addItem(	LogName _name,
								const char* const _label,
								const char* const _startString,
								const char* const _endString,
								uint _width,
								uint _precision,
								const char* const _colSeparator,
								const char* const _rowSeparator
								)
{
	// checks if item already exists
	if ( hasItem( _name ) == BT_TRUE )
		return SUCCESSFUL_RETURN;

	// create new item
	LogRecordItem* newItem = new LogRecordItem( _name,_label,_startString,_endString,
												_width,_precision,_colSeparator,_rowSeparator );

	if ( number == 0 )
	{
		first = newItem;
		last = newItem;
	}
	else
	{
		if ( last->setNext( newItem ) != SUCCESSFUL_RETURN )
			return ACADOERROR( RET_LOG_RECORD_CORRUPTED );
		last = newItem;
	}

	++number;

	return SUCCESSFUL_RETURN;
}



returnValue LogRecord::addItem(	const Expression& _name,
								const char* const _label
								)
{
	return addItem( _name,printScheme,_label );
}


returnValue LogRecord::addItem(	const Expression& _name,
								PrintScheme _printScheme,
								const char* const _label
								)
{
	char* _startString = 0;
	char* _endString = 0;
	uint _width = 0;
	uint _precision = 0;
	char* _colSeparator = 0;
	char* _rowSeparator = 0;

	returnValue returnvalue;
	returnvalue = getGlobalStringDefinitions( _printScheme,&_startString,&_endString,
											  _width,_precision,&_colSeparator,&_rowSeparator );

	if ( returnvalue == SUCCESSFUL_RETURN )
	{
		returnvalue = addItem( _name,_label,_startString,_endString,_width,_precision,_colSeparator,_rowSeparator );
	}

	if ( _startString != 0 )   delete[] _startString;
	if ( _endString != 0 )     delete[] _endString;
	if ( _colSeparator != 0 )  delete[] _colSeparator;
	if ( _rowSeparator != 0 )  delete[] _rowSeparator;

	return returnvalue;
}


returnValue LogRecord::addItem(	const Expression& _name,
								const char* const _label,
								const char* const _startString,
								const char* const _endString,
								uint _width,
								uint _precision,
								const char* const _colSeparator,
								const char* const _rowSeparator
								)
{
	// checks if item already exists
	if ( hasItem( _name ) == BT_TRUE )
		return SUCCESSFUL_RETURN;

	// create new item
	LogRecordItem* newItem = new LogRecordItem( _name,_label,_startString,_endString,
												_width,_precision,_colSeparator,_rowSeparator );

	if ( number == 0 )
	{
		first = newItem;
		last = newItem;
	}
	else
	{
		if ( last->setNext( newItem ) != SUCCESSFUL_RETURN )
			return ACADOERROR( RET_LOG_RECORD_CORRUPTED );
		last = newItem;
	}

	++number;

	return SUCCESSFUL_RETURN;
}



returnValue LogRecord::clearAllItems( )
{
	LogRecordItem* current = first;
	LogRecordItem* tmp;

	/* deallocate all LogRecordItems within list */
	while ( current != 0 )
	{
		tmp = current->getNext( );
		delete current;
		current = tmp;
	}

	/* ... and initialise an empty list. */
	first = 0;
	last  = 0;

	number = 0;

	return SUCCESSFUL_RETURN;
}


returnValue LogRecord::print(	LogPrintMode _mode
								) const
{
	uint stringLength = determineRecordStringLength( )+1;

	char* string      = new char[stringLength];
	for( uint i=0; i<stringLength; ++i )
		string[i] = '\0';
	char* valueString = 0;

	LogRecordItem* current = first;

	switch( _mode )
	{
		case PRINT_ITEM_BY_ITEM:
			while ( current != 0 )
			{
				if ( current->getValueString( &valueString ) != SUCCESSFUL_RETURN )
				{
					delete[] valueString; delete[] string;
					return ACADOERROR( RET_UNKNOWN_BUG );
				}

				if ( valueString != 0 )
				{
					strcat( string,valueString );

					delete[] valueString;
					valueString = 0;
				}

				current = current->getNext( );
			}
			break;

		case PRINT_ITER_BY_ITER:
			for( uint i=0; i<getMaxNumMatrices( ); ++i )
			{
				current = first;
				while ( current != 0 )
				{
					if ( current->getValueString( &valueString,i ) != SUCCESSFUL_RETURN )
					{
						delete[] valueString; delete[] string;
						return ACADOERROR( RET_UNKNOWN_BUG );
					}

					if ( valueString != 0 )
					{
						strcat( string,valueString );

						delete[] valueString;
						valueString = 0;
					}

					current = current->getNext( );
				}
			}
			break;

		case PRINT_LAST_ITER:
			while ( current != 0 )
			{
				if ( current->getValueString( &valueString,getMaxNumMatrices( )-1 ) != SUCCESSFUL_RETURN )
				{
					delete[] valueString; delete[] string;
					return ACADOERROR( RET_UNKNOWN_BUG );
				}

				if ( valueString != 0 )
				{
					strcat( string,valueString );

					delete[] valueString;
					valueString = 0;
				}

				current = current->getNext( );
			}
			strcat( string,"\n" );
			break;
	}

	if ( outputFile != 0 )
	{
		acadoFPrintf( outputFile,string );
	}
	else
	{
		if ( outputFileName != 0 )
		{
			FILE* outputFileFromName = fopen( outputFileName,"w" );
			acadoFPrintf( outputFileFromName,string );
			fclose( outputFileFromName );
		}
		else
		{
			delete[] string;
			return ACADOERROR( RET_INVALID_ARGUMENTS );
		}
	}

	delete[] string;

	return SUCCESSFUL_RETURN;
}


returnValue LogRecord::printInfo( ) const
{
	printf( "LogRecord having the following items:  " );

	LogRecordItem* current = first;

	while ( current != 0 )
	{
		printf( "%d, ",current->getName( ) );
		current = current->getNext( );
	}
	printf( "\n" );

	return SUCCESSFUL_RETURN;
}


uint LogRecord::getMaxNumMatrices( ) const
{
	uint maxNumMatrices = 0;

	LogRecordItem* current = first;

	while ( current != 0 )
	{
//		if ( current->getNumPoints( ) > maxNumMatrices );
		maxNumMatrices = current->getNumPoints( );

		current = current->getNext( );
	}

	return maxNumMatrices;
}


//
// PROTECTED MEMBER FUNCTIONS:
//


returnValue LogRecord::getAll(	uint _name,
								LogRecordItemType _type,
								MatrixVariablesGrid& values
								) const
{
	LogRecordItem* item = find( _name,_type );

	// checks if item exists
	if ( item == 0 )
		return ACADOERROR( RET_LOG_ENTRY_DOESNT_EXIST );

	values = item->getAllValues( );

	return SUCCESSFUL_RETURN;
}


returnValue LogRecord::getFirst(	uint _name,
									LogRecordItemType _type,
									Matrix& lastValue
									) const
{
	LogRecordItem* item = find( _name,_type );

	// checks if item exists
	if ( item == 0 )
		return ACADOERROR( RET_LOG_ENTRY_DOESNT_EXIST );

	lastValue = item->getFirstValue( );

	return SUCCESSFUL_RETURN;
}


returnValue LogRecord::getLast(	uint _name,
								LogRecordItemType _type,
								Matrix& lastValue
								) const
{
	LogRecordItem* item = find( _name,_type );

	// checks if item exists
	if ( item == 0 )
		return ACADOERROR( RET_LOG_ENTRY_DOESNT_EXIST );

	lastValue = item->getLastValue( );

	return SUCCESSFUL_RETURN;
}



returnValue LogRecord::setAll(	uint _name,
								LogRecordItemType _type,
								const MatrixVariablesGrid& values
								)
{
	LogRecordItem* item = find( _name,_type );

	// checks if item exists
	if ( ( item != 0 ) && ( item->isWriteProtected( ) == BT_FALSE ) )
	{
		if ( item->setAllValues( frequency,values ) != SUCCESSFUL_RETURN )
			return ACADOERROR( RET_LOG_RECORD_CORRUPTED );
	}

	return SUCCESSFUL_RETURN;
}


returnValue LogRecord::setLast(	uint _name,
								LogRecordItemType _type,
								const Matrix& value,
								double time
								)
{
	LogRecordItem* item = find( _name,_type );

	// checks if item exists
	if ( ( item != 0 ) && ( item->isWriteProtected( ) == BT_FALSE ) )
	{
		if ( item->setValue( frequency,value,time ) != SUCCESSFUL_RETURN )
			return ACADOERROR( RET_LOG_RECORD_CORRUPTED );
	}

	return SUCCESSFUL_RETURN;
}



LogRecordItem* LogRecord::find(	LogName _name
								) const
{
	return find( (uint)_name,LRT_ENUM );
}


LogRecordItem* LogRecord::find(	const Expression& _name
								) const
{
	return find( _name.getComponent( 0 ),LRT_VARIABLE );
}


LogRecordItem* LogRecord::find(	uint _name,
								LogRecordItemType _type
								) const
{
	LogRecordItem* item = first;

	while ( item != 0 )
	{
		if ( ( item->getName( ) == (int)_name ) && ( item->getType( ) == _type ) )
			return item;

		item = item->getNext( );
	}

	return 0;
}


LogRecordItem* LogRecord::findNonEmpty(	uint _name,
										LogRecordItemType _type
										) const
{
	LogRecordItem* item = first;

	while ( item != 0 )
	{
		if ( ( item->getName( ) == (int)_name ) && 
			 ( item->getType( ) == _type )      && 
			 ( item->isEmpty( ) == BT_FALSE )   )
			return item;

		item = item->getNext( );
	}

	return 0;
}


uint LogRecord::determineRecordStringLength( ) const
{
	uint stringLength = 0;

	LogRecordItem* current = first;

	while ( current != 0 )
	{
		stringLength += current->determineStringLength( );
		current = current->getNext( );
	}

	return stringLength;
}



CLOSE_NAMESPACE_ACADO


/*
 *	end of file
 */
