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
 *    \file include/acado/user_interaction/log_record.ipp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 12.06.2008
 */


//
// PUBLIC MEMBER FUNCTIONS:
//


BEGIN_NAMESPACE_ACADO


inline LogRecordItem& LogRecord::operator()(	uint idx
												)
{
	ASSERT( idx < getNumItems( ) );

	LogRecordItem* current = first;
	for( uint i=0; i<idx; ++i )
		current = current->getNext( );

	return *current;
}


inline LogRecordItem LogRecord::operator()(	uint idx
											) const
{
	ASSERT( idx < getNumItems( ) );

	LogRecordItem* current = first;
	for( uint i=0; i<idx; ++i )
		current = current->getNext( );	

	return *current;
}


inline LogRecordItem& LogRecord::operator()(	LogName _name
												)
{
	LogRecordItem* tmp = find( _name );
	ASSERT( tmp != 0 );

	return *tmp;
}


inline LogRecordItem LogRecord::operator()(	LogName _name
											) const
{
	LogRecordItem* tmp = find( _name );
	ASSERT( tmp != 0 );

	return *tmp;
}


inline LogRecordItem& LogRecord::operator()(	const Expression& _name
												)
{
	LogRecordItem* tmp = find( _name );
	ASSERT( tmp != 0 );

	return *tmp;
}


inline LogRecordItem LogRecord::operator()(	const Expression& _name
											) const
{
	LogRecordItem* tmp = find( _name );
	ASSERT( tmp != 0 );

	return *tmp;
}


inline returnValue LogRecord::getAll(	LogName _name,
										MatrixVariablesGrid& values
										) const
{
	return getAll( (uint)_name,LRT_ENUM,values );
}


inline returnValue LogRecord::getAll(	const Expression& _name,
										MatrixVariablesGrid& values
										) const
{
	return getAll( _name.getComponent( 0 ),LRT_VARIABLE,values );
}



inline returnValue LogRecord::getFirst(	LogName _name,
										Matrix& firstValue
										) const
{
	return getFirst( (uint)_name,LRT_ENUM,firstValue );
}


inline returnValue LogRecord::getFirst(	const Expression& _name,
										Matrix& firstValue
										) const
{
	return getFirst( _name.getComponent( 0 ),LRT_VARIABLE,firstValue );
}


inline returnValue LogRecord::getFirst(	LogName _name,
										VariablesGrid& firstValue
										) const
{
	Matrix tmp;
	getFirst( (uint)_name,LRT_ENUM,tmp );
	firstValue = tmp;

	return SUCCESSFUL_RETURN;
}


inline returnValue LogRecord::getFirst(	const Expression& _name,
										VariablesGrid& firstValue
										) const
{
	Matrix tmp;
	getFirst( _name.getComponent( 0 ),LRT_VARIABLE,tmp );
	firstValue = tmp;
	firstValue.setType( _name.getVariableType() );

	return SUCCESSFUL_RETURN;
}



inline returnValue LogRecord::getLast(	LogName _name,
										Matrix& lastValue
										) const
{
	return getLast( (uint)_name,LRT_ENUM,lastValue );
}


inline returnValue LogRecord::getLast(	const Expression& _name,
										Matrix& lastValue
										) const
{
	return getLast( _name.getComponent( 0 ),LRT_VARIABLE,lastValue );
}


inline returnValue LogRecord::getLast(	LogName _name,
										VariablesGrid& lastValue
										) const
{
	Matrix tmp;
	getLast( (uint)_name,LRT_ENUM,tmp );
	lastValue = tmp;

	return SUCCESSFUL_RETURN;
}


inline returnValue LogRecord::getLast(	const Expression& _name,
										VariablesGrid& lastValue
										) const
{
	Matrix tmp;
	getLast( _name.getComponent( 0 ),LRT_VARIABLE,tmp );
	lastValue = tmp;

	return SUCCESSFUL_RETURN;
}



inline returnValue LogRecord::setAll(	LogName _name,
										const MatrixVariablesGrid& values
										)
{
	return setAll( (uint)_name,LRT_ENUM,values );
}


inline returnValue LogRecord::setAll(	const Expression& _name,
										const MatrixVariablesGrid& values
										)
{	
	return setAll( _name.getComponent( 0 ),LRT_VARIABLE,values );
}



inline returnValue LogRecord::setLast(	LogName _name,
										Matrix& value,
										double time
										)
{
	return setLast( (uint)_name,LRT_ENUM,value,time );
}


inline returnValue LogRecord::setLast(	const Expression& _name,
										Matrix& value,
										double time
										)
{	
	return setLast( _name.getComponent( 0 ),LRT_VARIABLE,value,time );
}


inline returnValue LogRecord::setLast(	LogName _name,
										VariablesGrid& value,
										double time
										)
{
	Matrix tmp( value );
	return setLast( _name,tmp,time );
}


inline returnValue LogRecord::setLast(	const Expression& _name,
										VariablesGrid& value,
										double time
										)
{
	Matrix tmp( value );
	return setLast( _name,tmp,time );
}


inline returnValue LogRecord::enableWriteProtection(	LogName _name
														)
{
	LogRecordItem* item = find( _name );

	if ( item != 0 )
		item->enableWriteProtection( );

	return SUCCESSFUL_RETURN;
}


inline returnValue LogRecord::enableWriteProtection(	const Expression& _name
														)
{
	LogRecordItem* item = find( _name );

	if ( item != 0 )
		item->enableWriteProtection( );

	return SUCCESSFUL_RETURN;
}


inline returnValue LogRecord::disableWriteProtection(	LogName _name
														)
{
	LogRecordItem* item = find( _name );

	if ( item != 0 )
		item->disableWriteProtection( );

	return SUCCESSFUL_RETURN;
}


inline returnValue LogRecord::disableWriteProtection(	const Expression& _name
														)
{
	LogRecordItem* item = find( _name );

	if ( item != 0 )
		item->disableWriteProtection( );

	return SUCCESSFUL_RETURN;
}


inline uint LogRecord::getNumItems( ) const
{
	return number;
}


inline BooleanType LogRecord::isEmpty( ) const
{
	if ( getNumItems( ) == 0 )
		return BT_TRUE;
	else
		return BT_FALSE;
}


inline LogFrequency LogRecord::getLogFrequency( ) const
{
	return frequency;
}


inline PrintScheme LogRecord::getPrintScheme( ) const
{
	return printScheme;
}


inline returnValue LogRecord::setLogFrequency(	LogFrequency _frequency
												)
{
	frequency = _frequency;
	return SUCCESSFUL_RETURN;
}


inline returnValue LogRecord::setPrintScheme(	PrintScheme _printScheme
												)
{
	printScheme = _printScheme;
	return SUCCESSFUL_RETURN;
}



inline BooleanType LogRecord::hasItem(	LogName _name
										) const
{
	if ( find( _name ) != 0 )
		return BT_TRUE;
	else
		return BT_FALSE;
}


inline BooleanType LogRecord::hasItem(	const Expression& _name
										) const
{
	if ( find( _name ) != 0 )
		return BT_TRUE;
	else
		return BT_FALSE;
}


inline BooleanType LogRecord::hasNonEmptyItem(	LogName _name
												) const
{
	if ( findNonEmpty( (uint)_name,LRT_ENUM ) != 0 )
		return BT_TRUE;
	else
		return BT_FALSE;
}


inline BooleanType LogRecord::hasNonEmptyItem(	const Expression& _name
												) const
{
	if ( findNonEmpty( _name.getComponent( 0 ),LRT_VARIABLE ) != 0 )
		return BT_TRUE;
	else
		return BT_FALSE;
}


inline BooleanType LogRecord::isAlias( ) const
{
	if ( aliasIdx < 0 )
		return BT_FALSE;
	else
		return BT_TRUE;
}


inline int LogRecord::getAliasIdx( ) const
{
	return aliasIdx;
}


inline uint LogRecord::getNumDoubles( ) const
{
	uint nDoubles = 0;

	LogRecordItem* current = first;
	for( uint i=0; i<number; ++i )
	{
		nDoubles += current->getNumDoubles();
		current = current->getNext( );
	}

	return nDoubles;
}



//
// PROTECTED INLINED MEMBER FUNCTIONS:
//


inline returnValue LogRecord::setNext( LogRecord* const _next )
{
	next = _next;
	return SUCCESSFUL_RETURN;
}


inline LogRecord* LogRecord::getNext( ) const
{
	return next;
}


inline BooleanType LogRecord::hasItem(	uint _name,
										LogRecordItemType _type
										) const
{
	if ( find( _name,_type ) != 0 )
		return BT_TRUE;
	else
		return BT_FALSE;
}


inline returnValue LogRecord::setAliasIdx(	int _aliasIdx
											)
{
	aliasIdx = _aliasIdx;
	return SUCCESSFUL_RETURN;
}


CLOSE_NAMESPACE_ACADO


/*
 *	end of file
 */
