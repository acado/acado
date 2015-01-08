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
 *    \file src/user_interaction/plotting.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 12.05.2009
 */


#include <acado/user_interaction/plot_collection.hpp>
#include <acado/user_interaction/plotting.hpp>


BEGIN_NAMESPACE_ACADO



//
// PUBLIC MEMBER FUNCTIONS:
//


Plotting::Plotting( )
{
}


Plotting::Plotting( const Plotting& rhs )
{
	plotCollection = rhs.plotCollection;
}


Plotting::~Plotting( )
{
}


Plotting& Plotting::operator=( const Plotting& rhs )
{
	if ( this != &rhs )
	{
		plotCollection = rhs.plotCollection;
	}

	return *this;
}


returnValue Plotting::plot(	PlotFrequency _frequency
							)
{
	if ( _frequency == PLOT_NEVER )
		return SUCCESSFUL_RETURN;

	PlotWindow* window = plotCollection.first;

	while ( window != 0 )
	{
		// update plot data and ...
		getPlotWindow( *window );

		// ... plot the window
		if ( window->plot( _frequency ) != SUCCESSFUL_RETURN )
			return ACADOERROR( RET_PLOTTING_FAILED );

		window = window->getNext( );
	}

	return SUCCESSFUL_RETURN;
}


returnValue Plotting::replot(	PlotFrequency _frequency
								)
{
	if ( _frequency == PLOT_NEVER )
		return SUCCESSFUL_RETURN;

	PlotWindow* window = plotCollection.first;

	while ( window != 0 )
	{
		// update plot data and ...
		getPlotWindow( *window );

		// ... plot the window
		if ( window->replot( _frequency ) != SUCCESSFUL_RETURN )
			return ACADOERROR( RET_PLOTTING_FAILED );

		window = window->getNext( );
	}

	return SUCCESSFUL_RETURN;
}



//
// PROTECTED MEMBER FUNCTIONS:
//


returnValue Plotting::getPlotDataFromMemberLoggings(	PlotWindow& _window
														) const
{
	return SUCCESSFUL_RETURN;
}



CLOSE_NAMESPACE_ACADO


/*
 *	end of file
 */
