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
 *    \file src/user_interaction/logging.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 12.05.2009
 */


#include <acado/user_interaction/log_collection.hpp>
#include <acado/user_interaction/logging.hpp>


BEGIN_NAMESPACE_ACADO


// define static member
//LogCollection Logging::logCollection;


//
// PUBLIC MEMBER FUNCTIONS:
//


Logging::Logging( )
{
  	logIdx = -1;
}


Logging::Logging( const Logging& rhs )
{
	logCollection = rhs.logCollection;
	logIdx = rhs.logIdx;
}


Logging::~Logging( )
{
}


Logging& Logging::operator=( const Logging& rhs )
{
	if ( this != &rhs )
	{
		logCollection = rhs.logCollection;
		logIdx = rhs.logIdx;
	}

	return *this;
}


int Logging::operator<<(	LogRecord& _record
							)
{
	return addLogRecord( _record );
}


int Logging::addLogRecord(	LogRecord& _record
							)
{
	return logCollection.addLogRecord( _record );
}


returnValue Logging::getLogRecord(	LogRecord& _record
									) const
{
	return getLogRecord( _record.getAliasIdx( ),_record );
}


returnValue Logging::getLogRecord(	uint idx,
									LogRecord& _record
									) const
{
	if (idx >= getNumLogRecords( ))
		return ACADOERROR( RET_INDEX_OUT_OF_BOUNDS );

	_record = logCollection( idx );

	return SUCCESSFUL_RETURN;
}



returnValue Logging::updateLogRecord(	LogRecord& _record
										) const
{
	return logCollection.updateLogRecord( _record );
}



uint Logging::getNumLogRecords( ) const
{
	return logCollection.getNumLogRecords( );
}



returnValue Logging::printLoggingInfo( ) const
{
	printf( "LogCollection having the following records: \n" );
	for( uint i=0; i<logCollection.getNumLogRecords( ); ++i )
		logCollection( i ).printInfo( );

	return SUCCESSFUL_RETURN;
}


returnValue Logging::printNumDoubles( ) const
{
	printf( "LogCollection contains  %d  doubles.\n",logCollection.getNumDoubles( ) );
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
