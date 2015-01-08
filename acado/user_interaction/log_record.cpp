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
 *    \file src/user_interaction/log_record.cpp
 *    \author Hans Joachim Ferreau, Boris Houska, Milan Vukov
 *    \date 2008 - 2013
 */

#include <acado/user_interaction/log_record.hpp>

using namespace std;

BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

LogRecord::LogRecord(	LogFrequency _frequency,
						PrintScheme _printScheme
						)
{
	aliasIdx = -1;

	frequency      = _frequency;
	printScheme    = _printScheme;
}

LogRecord::~LogRecord( )
{}

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


returnValue LogRecord::addItem(	LogName _name,
								const char* const _label
								)
{
	if (items.count(make_pair(_name, LRT_ENUM)))
	{
//		LOG( LVL_DEBUG ) << "Log record "<< _name << " exists!" << endl;
		return SUCCESSFUL_RETURN;
	}
	items[ make_pair(_name, LRT_ENUM) ] = LogRecordData( _label );

	return SUCCESSFUL_RETURN;
}

returnValue LogRecord::addItem(	const Expression& _name,
								const char* const _label
								)
{
	if (items.count(make_pair(_name.getComponent( 0 ), LRT_VARIABLE)))
		return SUCCESSFUL_RETURN;
	items[ make_pair(_name.getComponent( 0 ), LRT_VARIABLE) ] = LogRecordData( _label );

	return SUCCESSFUL_RETURN;
}

returnValue LogRecord::print(	std::ostream& _stream,
								LogPrintMode _mode
								) const
{
	LogRecordItems::const_iterator it;
	returnValue status;

	char* startString = 0;
	char* endString = 0;
	uint width = 0;
	uint precision = 0;
	char* colSeparator = 0;
	char* rowSeparator = 0;

	getGlobalStringDefinitions(printScheme, &startString, &endString, width,
			precision, &colSeparator, &rowSeparator);

	switch (_mode)
	{
	case PRINT_ITEM_BY_ITEM:
		for (it = items.begin(); it != items.end(); ++it)
		{
			DMatrix tmp;

			for (uint i = 0; i < it->second.values.getNumPoints(); ++i)
				tmp.appendRows(it->second.values.getMatrix(i));

			 status = tmp.print(
					_stream, it->second.label.c_str(),
					startString, endString, width, precision,
					colSeparator, rowSeparator);
			 if (status != SUCCESSFUL_RETURN)
				 goto LogRecord_print_exit;
		}

		break;

	case PRINT_ITER_BY_ITER:
		for (unsigned i = 0; i < getMaxNumMatrices(); ++i)
			for (it = items.begin(); it != items.end(); ++it)
			{
				if (i >= it->second.values.getNumPoints()
						|| it->second.values.getNumPoints() == 0)
					break;
				if (it->second.values.getMatrix( i ).print(
						_stream, it->second.label.c_str(),
						startString, endString, width, precision,
						colSeparator, rowSeparator)
						!= SUCCESSFUL_RETURN)
					goto LogRecord_print_exit;
			}

	break;

	case PRINT_LAST_ITER:
		for (it = items.begin(); it != items.end(); ++it)
		{
			if (it->second.values.getNumPoints() == 0)
				continue;
			if (it->second.values.getMatrix(getMaxNumMatrices() - 1).print(
					_stream, it->second.label.c_str(),
					startString, endString, width, precision,
					colSeparator, rowSeparator) != SUCCESSFUL_RETURN)
				goto LogRecord_print_exit;
		}

	break;
	}

	LogRecord_print_exit:

	if ( startString != 0 )   delete[] startString;
	if ( endString != 0 )     delete[] endString;
	if ( colSeparator != 0 )  delete[] colSeparator;
	if ( rowSeparator != 0 )  delete[] rowSeparator;

	return status;
}

returnValue LogRecord::printInfo( ) const
{
	cout << "LogRecord having the following items: ";

	LogRecordItems::const_iterator it;
	for (it = items.begin(); it != items.end(); ++it)
		cout << it->first.first << ", ";

	cout << endl;

	return SUCCESSFUL_RETURN;
}


uint LogRecord::getMaxNumMatrices( ) const
{
	unsigned maxNumMatrices = 0;

	LogRecordItems::const_iterator it;
	for (it = items.begin(); it != items.end(); ++it)
	{
//		if ( items[ i ].getNumPoints( ) > maxNumMatrices );
		maxNumMatrices = it->second.values.getNumPoints( );
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
	LogRecordItems::const_iterator it = items.find(make_pair(_name, _type));
	if (it == items.end())
		return ACADOERROR( RET_LOG_ENTRY_DOESNT_EXIST );

	values = it->second.values;

	return SUCCESSFUL_RETURN;
}


returnValue LogRecord::getFirst(	uint _name,
									LogRecordItemType _type,
									DMatrix& firstValue
									) const
{
	LogRecordItems::const_iterator it = items.find(make_pair(_name, _type));
	if (it == items.end())
		return ACADOERROR( RET_LOG_ENTRY_DOESNT_EXIST );

	if (it->second.values.getNumPoints() == 0)
		firstValue = DMatrix();
	else
		firstValue = it->second.values.getMatrix( 0 );

	return SUCCESSFUL_RETURN;
}


returnValue LogRecord::getLast(	uint _name,
								LogRecordItemType _type,
								DMatrix& lastValue
								) const
{
	LogRecordItems::const_iterator it = items.find(make_pair(_name, _type));
	if (it == items.end())
		return ACADOERROR( RET_LOG_ENTRY_DOESNT_EXIST );

	if (it->second.values.getNumPoints() == 0)
		lastValue = DMatrix();
	else
		lastValue = it->second.values.getMatrix(it->second.values.getNumPoints() - 1);

	return SUCCESSFUL_RETURN;
}



returnValue LogRecord::setAll(	uint _name,
								LogRecordItemType _type,
								const MatrixVariablesGrid& values
								)
{
	LogRecordItems::iterator it = items.find(make_pair(_name, _type));
	if (it == items.end())
		return ACADOERROR( RET_LOG_ENTRY_DOESNT_EXIST );

	if (it->second.values.getNumPoints( ) == 0)
		return it->second.values.init( );

	switch( frequency )
	{
	case LOG_AT_START:
		it->second.values.init();
		it->second.values.addMatrix(values.getFirstMatrix( ), values.getFirstTime( ));
		break;

	case LOG_AT_END:
		it->second.values.init();
		it->second.values.addMatrix(values.getLastMatrix( ), values.getFirstTime( ));
		break;

	case LOG_AT_EACH_ITERATION:
		it->second.values = values;
		break;
	}

	return SUCCESSFUL_RETURN;
}

returnValue LogRecord::setLast(	uint _name,
								LogRecordItemType _type,
								const DMatrix& value,
								double time
								)
{
	LogRecordItems::iterator it = items.find(make_pair(_name, _type));
	if (it == items.end())
		return ACADOERROR( RET_LOG_ENTRY_DOESNT_EXIST );

	double logTime = time;

	switch( frequency )
	{
	case LOG_AT_START:
		// only log if no matrix has been logged so far
		if (it->second.values.getNumPoints() == 0)
		{
			if (acadoIsEqual(logTime, -INFTY) == BT_TRUE)
				logTime = 0.0;

			it->second.values.addMatrix(value, logTime);
		}
		break;

	case LOG_AT_END:
		// always overwrite existing matrices in order to keep only the last one
		it->second.values.init();

		if (acadoIsEqual(logTime, -INFTY) == BT_TRUE)
			logTime = 0.0;

		it->second.values.addMatrix(value, logTime);
		break;

	case LOG_AT_EACH_ITERATION:
		// add matrix to list
		if (acadoIsEqual(logTime, -INFTY) == BT_TRUE)
			logTime = (double)it->second.values.getNumPoints() + 1.0;

		it->second.values.addMatrix(value, logTime);
		break;
	}
	return SUCCESSFUL_RETURN;
}

returnValue LogRecord::updateLogRecord( LogRecord& _record
										) const
{
	LogRecordItems::iterator it;

	for (it = _record.items.begin(); it != _record.items.end(); ++it)
	{
		if (it->second.writeProtection == true)
			continue;

		LogRecordItems::const_iterator cit = items.find( it->first );

		if (cit != items.end())
			it->second = cit->second;
	}

	return SUCCESSFUL_RETURN;
}

CLOSE_NAMESPACE_ACADO

/*
 *	end of file
 */
