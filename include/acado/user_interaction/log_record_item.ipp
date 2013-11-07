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
 *    \file include/acado/user_interaction/log_record_item.ipp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 12.06.2008
 */


//
// PUBLIC MEMBER FUNCTIONS:
//


BEGIN_NAMESPACE_ACADO

// To be returned by reference
const Matrix emptyMatrix_;

inline const MatrixVariablesGrid& LogRecordItem::getAllValues( ) const
{
	return values;
}


inline Matrix LogRecordItem::getValue(	uint idx
										) const
{
	if (idx >= getNumPoints( ))
	{
		return emptyMatrix_;
	}

	return values.getMatrix( idx );
}


inline Matrix LogRecordItem::getFirstValue( ) const
{
	if ( getNumPoints( ) == 0 )
	{
		return emptyMatrix_;
	}

	return getValue( 0 );
}


inline Matrix LogRecordItem::getLastValue( ) const
{
	if ( getNumPoints( ) == 0 )
	{
		return emptyMatrix_;
	}

	return getValue( getNumPoints( )-1 );
}


inline returnValue LogRecordItem::getValueString( char** valueString ) const
{
	Matrix tmp;

	for( uint i=0; i<getNumPoints( ); ++i )
		tmp.appendRows( values.getMatrix(i) );

	return tmp.printToString( valueString, label,startString,endString,
							  width,precision,colSeparator,rowSeparator );
}


inline returnValue LogRecordItem::getValueString(	char** valueString,
													uint idx
													) const
{
	if (idx >= getNumPoints( ))
		return SUCCESSFUL_RETURN;

	return values.getMatrix( idx ).printToString( valueString, label,startString,endString,
												  width,precision,colSeparator,rowSeparator );
}



inline uint LogRecordItem::determineStringLength( ) const
{
	Matrix tmp;

	for( uint i=0; i<getNumPoints( ); ++i )
		tmp.appendRows( values.getMatrix(i) );

	return tmp.determineStringLength( label,startString,endString,
									  width,precision,colSeparator,rowSeparator );
}


inline uint LogRecordItem::determineStringLength(	uint idx
													) const
{
	if (idx >= getNumPoints( ))
		return SUCCESSFUL_RETURN;

	return values.getMatrix( idx ).determineStringLength( label,startString,endString,
														  width,precision,colSeparator,rowSeparator );
}



inline int LogRecordItem::getName( ) const
{
	return name;
}

inline LogRecordItemType LogRecordItem::getType( ) const
{
	return type;
}


inline returnValue LogRecordItem::setNext( LogRecordItem* const _next )
{
	next = _next;
	return SUCCESSFUL_RETURN;
}


inline LogRecordItem* LogRecordItem::getNext( ) const
{
	return next;
}


inline uint LogRecordItem::getNumPoints( ) const
{
	return values.getNumPoints( );
}


inline BooleanType LogRecordItem::isEmpty( ) const
{
	if ( getNumPoints( ) == 0 )
		return BT_TRUE;
	else
		return BT_FALSE;
}



inline returnValue LogRecordItem::enableWriteProtection( )
{
	writeProtected = BT_TRUE;
	return SUCCESSFUL_RETURN;
}


inline returnValue LogRecordItem::disableWriteProtection( )
{
	writeProtected = BT_FALSE;
	return SUCCESSFUL_RETURN;
}


inline BooleanType LogRecordItem::isWriteProtected( ) const
{
	return writeProtected;
}


inline uint LogRecordItem::getNumDoubles( ) const
{
	return values.getDim();
}


CLOSE_NAMESPACE_ACADO


/*
 *	end of file
 */
