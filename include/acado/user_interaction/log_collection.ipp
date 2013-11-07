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
 *    \file include/acado/user_interaction/log_collection.ipp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 12.05.2009
 */



BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

inline LogRecord& LogCollection::operator()(	uint idx
												)
{
	ASSERT( idx < getNumLogRecords( ) );

	LogRecord* current = first;
	for( uint i=0; i<idx; ++i )
		current = current->getNext( );

	return *current;
}

inline const LogRecord& LogCollection::operator()( uint idx ) const{

	ASSERT( idx < getNumLogRecords( ) );

	LogRecord* current = first;
	for( uint i=0; i<idx; ++i )
		current = current->getNext( );

	return *current;
}



inline returnValue LogCollection::getAll(	LogName _name,
											MatrixVariablesGrid& values
											) const
{
	return getAll( (uint)_name,LRT_ENUM,values );
}


inline returnValue LogCollection::getAll(	const Expression& _name,
											MatrixVariablesGrid& values
											) const
{
	return getAll( _name.getComponent( 0 ),LRT_VARIABLE,values );
}



inline returnValue LogCollection::getFirst(	LogName _name,
											Matrix& firstValue
											) const
{
	return getFirst( (uint)_name,LRT_ENUM,firstValue );
}


inline returnValue LogCollection::getFirst(	const Expression& _name,
											Matrix& firstValue
											) const
{
	return getFirst( _name.getComponent( 0 ),LRT_VARIABLE,firstValue );
}


inline returnValue LogCollection::getFirst(	LogName _name,
											VariablesGrid& firstValue
											) const
{
	Matrix tmp;
	getFirst( (uint)_name,LRT_ENUM,tmp );
	firstValue = tmp;

	return SUCCESSFUL_RETURN;
}


inline returnValue LogCollection::getFirst(	const Expression& _name,
											VariablesGrid& firstValue
											) const
{
	Matrix tmp;
	getFirst( _name.getComponent( 0 ),LRT_VARIABLE,tmp );
	firstValue = tmp;

	return SUCCESSFUL_RETURN;
}



inline returnValue LogCollection::getLast(	LogName _name,
											Matrix& lastValue
											) const
{
	return getLast( (uint)_name,LRT_ENUM,lastValue );
}


inline returnValue LogCollection::getLast(	const Expression& _name,
											Matrix& lastValue
											) const
{
	return getLast( _name.getComponent( 0 ),LRT_VARIABLE,lastValue );
}


inline returnValue LogCollection::getLast(	LogName _name,
											VariablesGrid& lastValue
											) const
{
	Matrix tmp;
	getLast( (uint)_name,LRT_ENUM,tmp );
	lastValue = tmp;

	return SUCCESSFUL_RETURN;
}


inline returnValue LogCollection::getLast(	const Expression& _name,
											VariablesGrid& lastValue
											) const
{
	Matrix tmp;
	getLast( _name.getComponent( 0 ),LRT_VARIABLE,tmp );
	lastValue = tmp;

	return SUCCESSFUL_RETURN;
}



inline returnValue LogCollection::setAll(	LogName _name,
											const MatrixVariablesGrid& values
											)
{
	return setAll( (uint)_name,LRT_ENUM,values );
}


inline returnValue LogCollection::setAll(	const Expression& _name,
											const MatrixVariablesGrid& values
											)
{
	return setAll( _name.getComponent( 0 ),LRT_VARIABLE,values );
}



inline returnValue LogCollection::setLast(	LogName _name,
											int lastValue,
											double time
											)
{
	Matrix tmp( lastValue );
	return setLast( _name,tmp,time );
}


inline returnValue LogCollection::setLast(	const Expression& _name,
											int lastValue,
											double time
											)
{
	Matrix tmp( lastValue );
	return setLast( _name,tmp,time );
}


inline returnValue LogCollection::setLast(	LogName _name,
											double lastValue,
											double time
											)
{
	Matrix tmp( lastValue );
	return setLast( _name,tmp,time );
}


inline returnValue LogCollection::setLast(	const Expression& _name,
											double lastValue,
											double time
											)
{
	Matrix tmp( lastValue );
	return setLast( _name,tmp,time );
}


inline returnValue LogCollection::setLast(	LogName _name,
											const Vector& lastValue,
											double time
											)
{
	Matrix tmp( lastValue );
	return setLast( (int)_name,LRT_ENUM,tmp,time );
}


inline returnValue LogCollection::setLast(	const Expression& _name,
											const Vector& lastValue,
											double time
											)
{
	Matrix tmp( lastValue );
	return setLast( _name.getComponent( 0 ),LRT_VARIABLE,tmp,time );
}


inline returnValue LogCollection::setLast(	LogName _name,
											const Matrix& lastValue,
											double time
											)
{
	return setLast( (int)_name,LRT_ENUM,lastValue,time );
}


inline returnValue LogCollection::setLast(	const Expression& _name,
											const Matrix& lastValue,
											double time
											)
{
	return setLast( _name.getComponent( 0 ),LRT_VARIABLE,lastValue,time );
}


inline returnValue LogCollection::setLast(	LogName _name,
											const VariablesGrid& lastValue,
											double time
											)
{
	Matrix tmp( lastValue );
	return setLast( _name,tmp,time );
}


inline returnValue LogCollection::setLast(	const Expression& _name,
											const VariablesGrid& lastValue,
											double time
											)
{
	Matrix tmp( lastValue );
	return setLast( _name,tmp,time );
}



inline uint LogCollection::getNumLogRecords( ) const
{
	return number;
}


inline BooleanType LogCollection::hasRecord(	const LogRecord& _record
												) const
{
	if ( findRecord( _record ) >= 0 )
		return BT_TRUE;
	else
		return BT_FALSE;
}


inline BooleanType LogCollection::hasItem(	LogName _name
											) const
{
	if ( find( _name ) != 0 )
		return BT_TRUE;
	else
		return BT_FALSE;
}


inline BooleanType LogCollection::hasItem(	const Expression& _name
											) const
{
	if ( find( _name ) != 0 )
		return BT_TRUE;
	else
		return BT_FALSE;
}



inline BooleanType LogCollection::hasNonEmptyItem(	LogName _name
													) const
{
	if ( findNonEmpty( (int)_name,LRT_ENUM ) != 0 )
		return BT_TRUE;
	else
		return BT_FALSE;
}


inline BooleanType LogCollection::hasNonEmptyItem(	const Expression& _name
													) const
{
	if ( findNonEmpty( _name.getComponent( 0 ),LRT_VARIABLE ) != 0 )
		return BT_TRUE;
	else
		return BT_FALSE;
}


inline uint LogCollection::getNumDoubles( ) const
{
	uint nDoubles = 0;

	LogRecord* current = first;
	for( uint i=0; i<number; ++i )
	{
		nDoubles += current->getNumDoubles();
		current = current->getNext( );
	}

	return nDoubles;
}



//
// PROTECTED MEMBER FUNCTIONS:
//

inline int LogCollection::findRecord(	const LogRecord& _record
										) const
{
	for( uint i=0; i<getNumLogRecords(); ++i )
		if ( operator()( i ) == _record )
			return i;

	return -1;
}


CLOSE_NAMESPACE_ACADO


/*
 *	end of file
 */
