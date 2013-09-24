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
 *    \file src/user_interaction/log_collection.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 12.05.2009
 */


#include <acado/user_interaction/log_collection.hpp>

//#include <acado/user_interaction/log_record.hpp>


BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//


LogCollection::LogCollection( )
{
	first = 0;
	last  = 0;

	number = 0;
}


LogCollection::LogCollection( const LogCollection& rhs )
{
	first = 0;
	last  = 0;

	number = 0;

	/* if rhs logging list is not empty, add all logging records... */
	LogRecord* current = rhs.first;

	while ( current != 0 )
	{
		addLogRecord( *current );
		current = current->getNext( );
	}
}


LogCollection::~LogCollection( )
{
	clearAllRecords( );
}


LogCollection& LogCollection::operator=( const LogCollection& rhs )
{
	if ( this != &rhs )
	{
		clearAllRecords( );

		/* if rhs logging list is not empty, add all option items... */
		LogRecord* current = rhs.first;

		while ( current != 0 )
		{
			addLogRecord( *current );
			current = current->getNext( );
		}
	}

	return *this;
}


int LogCollection::operator<<(	LogRecord& record
								)
{
	return addLogRecord( record );
}


int LogCollection::addLogRecord(	LogRecord& record
									)
{
	// checks whether record already exists
	int existingIdx = findRecord( record );
	if ( existingIdx >= 0 )
		return  operator()( existingIdx ).getAliasIdx( );

	record.setAliasIdx( number );

	// create new record
	LogRecord* newRecord = new LogRecord( record );

	if ( number == 0 )
	{
		first = newRecord;
		last  = newRecord;
	}
	else
	{
		if ( last->setNext( newRecord ) != SUCCESSFUL_RETURN )
			return -ACADOERROR( RET_LOG_COLLECTION_CORRUPTED );
		last = newRecord;
	}

	++number;

	return (number-1);
}


returnValue LogCollection::clearAllRecords( )
{
	LogRecord* current = first;
	LogRecord* tmp;

	/* deallocate all LogginRecords within list... */
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


returnValue LogCollection::print(	LogPrintMode _mode
									) const
{
	LogRecord* record = first;

	while ( record != 0 )
	{
		record->print( _mode );
		record = record->getNext( );
	}

	return SUCCESSFUL_RETURN;
}


returnValue LogCollection::updateLogRecord(	LogRecord& _record
											) const
{
	LogRecord currentRecord;

	uint name;
	LogRecordItemType type;

	MatrixVariablesGrid newValues;

	for( uint i=0; i<_record.getNumItems( ); ++i )
	{
		if ( _record( i ).isWriteProtected( ) == BT_FALSE )
		{
			name = _record( i ).getName();
			type = _record( i ).getType();

			for( uint j=0; j<getNumLogRecords( ); ++j )
			{
				currentRecord = operator()( j );
	
				if ( ( currentRecord.hasItem( name,type ) == BT_TRUE ) && 
					( currentRecord.getLogFrequency( ) == _record.getLogFrequency( ) ) )
				{
					currentRecord.getAll( name,type,newValues );
	
					if ( newValues.getNumPoints( ) > 0 )
						_record.setAll( name,type,newValues );
	
					break;
				}
			}
		}
	}

	return SUCCESSFUL_RETURN;
}



//
// PROTECTED MEMBER FUNCTIONS:
//



returnValue LogCollection::getAll(	uint _name,
									LogRecordItemType _type,
									MatrixVariablesGrid& values
									) const
{
	for( uint i=0; i<getNumLogRecords( ); ++i )
	{
		if ( operator()( i ).hasItem( _name,_type ) == BT_TRUE )
		{
			operator()( i ).getAll( _name,_type,values );
			return SUCCESSFUL_RETURN;
		}
	}

	return ACADOERROR( RET_LOG_ENTRY_DOESNT_EXIST );
}


returnValue LogCollection::getFirst(	uint _name,
										LogRecordItemType _type,
										Matrix& value
										) const
{
	for( uint i=0; i<getNumLogRecords( ); ++i )
	{
		if ( operator()( i ).hasItem( _name,_type ) == BT_TRUE )
		{
			operator()( i ).getFirst( _name,_type,value );
			return SUCCESSFUL_RETURN;
		}
	}

	return ACADOERROR( RET_LOG_ENTRY_DOESNT_EXIST );
}


returnValue LogCollection::getLast(	uint _name,
									LogRecordItemType _type,
									Matrix& value
									) const
{
	for( uint i=0; i<getNumLogRecords( ); ++i )
	{
		if ( operator()( i ).hasItem( _name,_type ) == BT_TRUE )
		{
			operator()( i ).getLast( _name,_type,value );
			return SUCCESSFUL_RETURN;
		}
	}

	return ACADOERROR( RET_LOG_ENTRY_DOESNT_EXIST );
}


returnValue LogCollection::setAll(	uint _name,
									LogRecordItemType _type,
									const MatrixVariablesGrid& values
									)
{
	LogRecord* record = first;

	while ( record != 0 )
	{
		if ( record->hasItem( _name,_type ) == BT_TRUE )
		{
			if ( record->setAll( _name,_type,values ) != SUCCESSFUL_RETURN )
				return ACADOERROR( RET_LOG_COLLECTION_CORRUPTED );
		}

		record = record->getNext( );
	}

	return SUCCESSFUL_RETURN;
}


returnValue LogCollection::setLast(	uint _name,
									LogRecordItemType _type,
									const Matrix& value,
									double time
									)
{
	LogRecord* record = first;

	while ( record != 0 )
	{
		if ( record->hasItem( _name,_type ) == BT_TRUE )
		{
			if ( record->setLast( _name,_type,value,time ) != SUCCESSFUL_RETURN )
				return ACADOERROR( RET_LOG_COLLECTION_CORRUPTED );
		}

		record = record->getNext( );
	}

	return SUCCESSFUL_RETURN;
}



LogRecord* LogCollection::find(	LogName _name
								) const
{
	return find( (int)_name,LRT_ENUM );
}


LogRecord* LogCollection::find(	const Expression& _name
								) const
{
	return find( _name.getComponent( 0 ),LRT_VARIABLE );
}


LogRecord* LogCollection::find(	uint _name,
								LogRecordItemType _type
								) const
{
	LogRecord* record = first;

	while ( record != 0 )
	{
		if ( record->hasItem( _name,_type ) == BT_TRUE )
			return record;

		record = record->getNext( );
	}

	return 0;
}


LogRecord* LogCollection::findNonEmpty(	uint _name,
										LogRecordItemType _type
										) const
{
	LogRecord* record = first;

	while ( record != 0 )
	{
		if ( record->findNonEmpty( _name,_type ) != 0 )
			return record;

		record = record->getNext( );
	}

	return 0;
}


CLOSE_NAMESPACE_ACADO


/*
 *	end of file
 */
