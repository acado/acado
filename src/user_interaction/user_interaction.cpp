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
 *    \file src/user_interaction/user_interaction.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 */


#include <acado/user_interaction/user_interaction.hpp>


BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//


UserInteraction::UserInteraction( ) : Options( ), Logging( ), Plotting( )
{
	setStatus( BS_UNDEFINED );
}


UserInteraction::UserInteraction( const UserInteraction& rhs ) : Options( rhs ), Logging( rhs ), Plotting( rhs )
{
	setStatus( rhs.status );
}


UserInteraction::~UserInteraction( )
{
}


UserInteraction& UserInteraction::operator=( const UserInteraction& rhs )
{
	if ( this != &rhs )
	{
		Options::operator=( rhs );
		Logging::operator=( rhs );
		Plotting::operator=( rhs );
		
		setStatus( rhs.status );
	}

	return *this;
}



int UserInteraction::operator<<(	PlotWindow& _window
									)
{
	return UserInteraction::addPlotWindow( _window );
}


int UserInteraction::addPlotWindow(	PlotWindow& _window
									)
{
	LogRecord tmp;

	_window.getPlotDataRecord( tmp );
	Logging::addLogRecord( tmp );
	_window.setPlotDataRecord( tmp ); // do not forget to update _window's alias index!

	return Plotting::addPlotWindow( _window );
}


int UserInteraction::operator<<(	LogRecord& _record
									)
{
	return Logging::operator<<( _record );
}




//
// PROTECTED MEMBER FUNCTIONS:
//


returnValue UserInteraction::getPlotDataFromMemberLoggings(	PlotWindow& _window
															) const
{
	LogRecord tmp;

	_window.getPlotDataRecord( tmp );
	Logging::updateLogRecord( tmp );
	_window.setPlotDataRecord( tmp );

	return SUCCESSFUL_RETURN;
}



BlockStatus UserInteraction::getStatus( ) const
{
	return status;
}


returnValue UserInteraction::setStatus(	BlockStatus _status
										)
{
	status = _status;
	return SUCCESSFUL_RETURN;
}


CLOSE_NAMESPACE_ACADO


/*
 *	end of file
 */
