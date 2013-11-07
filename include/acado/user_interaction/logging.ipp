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
 *    \file include/acado/user_interaction/logging.ipp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 12.05.2009
 */



BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

inline returnValue Logging::getAll(	LogName _name,
									MatrixVariablesGrid& _values
									) const
{
	if ( logCollection.hasNonEmptyItem( _name ) == BT_TRUE )
		return logCollection.getAll( _name,_values );
	else
		return ACADOERROR( RET_LOG_ENTRY_DOESNT_EXIST );
}


inline returnValue Logging::getFirst(	LogName _name,
										Matrix& _firstValue
										) const
{
	if ( logCollection.hasNonEmptyItem( _name ) == BT_TRUE )
		return logCollection.getFirst( _name,_firstValue );
	else
		return ACADOERROR( RET_LOG_ENTRY_DOESNT_EXIST );
}


inline returnValue Logging::getFirst(	LogName _name,
										VariablesGrid& _firstValue
										) const
{
	if ( logCollection.hasNonEmptyItem( _name ) == BT_TRUE )
		return logCollection.getFirst( _name,_firstValue );
	else
		return ACADOERROR( RET_LOG_ENTRY_DOESNT_EXIST );
}


inline returnValue Logging::getLast(	LogName _name,
										Matrix& _lastValue
										) const
{
	if ( logCollection.hasNonEmptyItem( _name ) == BT_TRUE )
		return logCollection.getLast( _name,_lastValue );
	else
		return ACADOERROR( RET_LOG_ENTRY_DOESNT_EXIST );
}


inline returnValue Logging::getLast(	LogName _name,
										VariablesGrid& _lastValue
										) const
{
	if ( logCollection.hasNonEmptyItem( _name ) == BT_TRUE )
		return logCollection.getLast( _name,_lastValue );
	else
		return ACADOERROR( RET_LOG_ENTRY_DOESNT_EXIST );
}


CLOSE_NAMESPACE_ACADO


/*
 *	end of file
 */
