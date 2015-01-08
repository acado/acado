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
 *    \file src/user_interaction/logging.cpp
 *    \author Hans Joachim Ferreau, Boris Houska, Milan Vukov
 *    \date 2009 - 2013
 */

#include <acado/user_interaction/logging.hpp>

using namespace std;

BEGIN_NAMESPACE_ACADO

//
// PUBLIC MEMBER FUNCTIONS:
//


Logging::Logging( )
{
  	logIdx = -1;
}

Logging::~Logging( )
{}

int Logging::operator<<(	LogRecord& _record
							)
{
	return addLogRecord( _record );
}

int Logging::addLogRecord(	LogRecord& _record
							)
{
	unsigned newIndex;
	if (_record.aliasIdx >= 0)
	{
		logCollection[ _record.aliasIdx ] = _record;
		newIndex = _record.aliasIdx;
	}
	else
	{
		logCollection.push_back( _record );
		newIndex = logCollection.size() - 1;
		logCollection.back().aliasIdx = _record.aliasIdx = newIndex;
	}

	return newIndex;
}

returnValue Logging::getLogRecord(	LogRecord& _record
									) const
{
	if (_record.aliasIdx < 0  || _record.aliasIdx >= ((int)getNumLogRecords( ) - 1))
		return ACADOERROR( RET_INDEX_OUT_OF_BOUNDS );

	_record = logCollection[ _record.aliasIdx ];

	return SUCCESSFUL_RETURN;
}


returnValue Logging::updateLogRecord(	LogRecord& _record
										) const
{
	for (unsigned ind = 0; ind < logCollection.size(); ++ind)
	{
		if (logCollection[ ind ].getLogFrequency() != _record.getLogFrequency())
			continue;

		logCollection[ ind ].updateLogRecord( _record );
	}

	return SUCCESSFUL_RETURN;
}

uint Logging::getNumLogRecords( ) const
{
	return logCollection.size();
}

returnValue Logging::printLoggingInfo( ) const
{
	cout << "LogCollection having the following records: \n";
	for (uint i = 0; i < logCollection.size(); ++i)
		logCollection[ i ].printInfo();

	return SUCCESSFUL_RETURN;
}

returnValue Logging::printNumDoubles( ) const
{
	unsigned nDoubles = 0;

	for (unsigned i = 0; i < logCollection.size(); ++i)
		nDoubles += logCollection[ i ].getNumDoubles();

	return nDoubles;

	cout << "LogCollection contains " << nDoubles <<  "doubles.\n";
	return SUCCESSFUL_RETURN;
}

//
// PROTECTED MEMBER FUNCTIONS:
//

returnValue Logging::setupLogging( )
{
	return SUCCESSFUL_RETURN;
}

CLOSE_NAMESPACE_ACADO

/*
 *	end of file
 */
