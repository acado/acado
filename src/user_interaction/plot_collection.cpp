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
 *    \file src/user_interaction/plot_collection.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 12.05.2009
 */


#include <acado/user_interaction/plot_collection.hpp>



BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//


PlotCollection::PlotCollection( )
{
	first = 0;
	last  = 0;

	number = 0;
}


PlotCollection::PlotCollection( const PlotCollection& rhs )
{
	first = 0;
	last  = 0;

	number = 0;

	/* if rhs logging list is not empty, add all logging windows... */
	PlotWindow* current = rhs.first;

	while ( current != 0 )
	{
		addPlotWindow( *current );
		current = current->getNext( );
	}
}


PlotCollection::~PlotCollection( )
{
	clearAllWindows( );
}


PlotCollection& PlotCollection::operator=( const PlotCollection& rhs )
{
	if ( this != &rhs )
	{
		clearAllWindows( );

		/* if rhs logging list is not empty, add all option items... */
		PlotWindow* current = rhs.first;

		while ( current != 0 )
		{
			addPlotWindow( *current );
			current = current->getNext( );
		}
	}

	return *this;
}


int PlotCollection::operator<<(	PlotWindow& window
								)
{
	return addPlotWindow( window );
}


int PlotCollection::addPlotWindow(	PlotWindow& window
									)
{
	window.setAliasIdx( number );

	// create new window
	PlotWindow* newWindow = window.clone( );

	if ( number == 0 )
	{
		first = newWindow;
		last  = newWindow;
	}
	else
	{
		if ( last->setNext( newWindow ) != SUCCESSFUL_RETURN )
			return -ACADOERROR( RET_PLOT_COLLECTION_CORRUPTED );
		last = newWindow;
	}

	++number;

	return (number-1);
}


returnValue PlotCollection::clearAllWindows( )
{
	PlotWindow* current = first;
	PlotWindow* tmp;

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



returnValue PlotCollection::enableNominalControls( )
{
	PlotWindow* current = first;

	while ( current != 0 )
	{
		current->enableNominalControls( );
		current = current->getNext( );
	}
	
	return SUCCESSFUL_RETURN;
}


returnValue PlotCollection::disableNominalControls( )
{
	PlotWindow* current = first;

	while ( current != 0 )
	{
		current->disableNominalControls( );
		current = current->getNext( );
	}
	
	return SUCCESSFUL_RETURN;
}



returnValue PlotCollection::enableNominalParameters( )
{
	PlotWindow* current = first;

	while ( current != 0 )
	{
		current->enableNominalParameters( );
		current = current->getNext( );
	}
	
	return SUCCESSFUL_RETURN;
}


returnValue PlotCollection::disableNominalParameters( )
{
	PlotWindow* current = first;

	while ( current != 0 )
	{
		current->disableNominalParameters( );
		current = current->getNext( );
	}
	
	return SUCCESSFUL_RETURN;
}



returnValue PlotCollection::enableNominalOutputs( )
{
	PlotWindow* current = first;

	while ( current != 0 )
	{
		current->enableNominalOutputs( );
		current = current->getNext( );
	}
	
	return SUCCESSFUL_RETURN;
}


returnValue PlotCollection::disableNominalOutputs( )
{
	PlotWindow* current = first;

	while ( current != 0 )
	{
		current->disableNominalOutputs( );
		current = current->getNext( );
	}
	
	return SUCCESSFUL_RETURN;
}


//
// PROTECTED MEMBER FUNCTIONS:
//




CLOSE_NAMESPACE_ACADO


/*
 *	end of file
 */
