/*
 *    This file is part of ACADO Toolkit.
 *
 *    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
 *    Copyright (C) 2008-2009 by Boris Houska and Hans Joachim Ferreau, K.U.Leuven.
 *    Developed within the Optimization in Engineering Center (OPTEC) under
 *    supervision of Moritz Diehl. All rights reserved.
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
 *    \file include/acado/matrix_vector/vectorspace_data.ipp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 31.05.2008
 */


//
// PUBLIC MEMBER FUNCTIONS:
//


BEGIN_NAMESPACE_ACADO



inline double& VectorspaceElement::operator()(	uint idx
									)
{
	ASSERT( idx < getDim( ) );
	return data[idx];
}


inline double VectorspaceElement::operator()(	uint idx 
									) const
{
	ASSERT( idx < getDim( ) );
	return data[idx];
}


inline BooleanType VectorspaceElement::operator==(	const VectorspaceElement& arg
													) const
{
	uint i;

	for( i=0; i<getDim( ); ++i )
		if ( fabs( data[i] - arg.data[i] ) > 10*EPS )
			return BT_FALSE;

	return BT_TRUE;
}


inline BooleanType VectorspaceElement::operator!=(	const VectorspaceElement& arg
													) const
{
	ASSERT( getDim( ) == arg.getDim( ) );

	if ( operator==( arg ) == BT_TRUE )
		return BT_FALSE;
	else
		return BT_TRUE;
}


inline BooleanType VectorspaceElement::operator<(	const VectorspaceElement& arg
													) const
{
	ASSERT( getDim( ) == arg.getDim( ) );

	uint i;

	for( i=0; i<getDim( ); ++i )
		if ( data[i] >= arg.data[i] - EPS )
			return BT_FALSE;

	return BT_TRUE;
}


inline BooleanType VectorspaceElement::operator<=(	const VectorspaceElement& arg
													) const
{
	ASSERT( getDim( ) == arg.getDim( ) );

	uint i;

	for( i=0; i<getDim( ); ++i )
		if ( data[i] > arg.data[i] + EPS )
			return BT_FALSE;

	return BT_TRUE;
}


inline BooleanType VectorspaceElement::operator>(	const VectorspaceElement& arg
													) const
{
	ASSERT( getDim( ) == arg.getDim( ) );

	uint i;

	for( i=0; i<getDim( ); ++i )
		if ( data[i] <= arg.data[i] + EPS )
			return BT_FALSE;

	return BT_TRUE;
}


inline BooleanType VectorspaceElement::operator>=(	const VectorspaceElement& arg
													) const
{
	ASSERT( getDim( ) == arg.getDim( ) );

	uint i;

	for( i=0; i<getDim( ); ++i )
		if ( data[i] < arg.data[i] - EPS )
			return BT_FALSE;

	return BT_TRUE;
}



inline unsigned int VectorspaceElement::getDim( ) const
{
	return data.size();
}


inline BooleanType VectorspaceElement::isEmpty( ) const
{
	if ( getDim( ) == 0 )
		return BT_TRUE;
	else
		return BT_FALSE;
}


inline BooleanType VectorspaceElement::isEqualTo(	double _value
													) const
{
	for( uint i=0; i<getDim(); ++i )
	{
		if ( acadoIsEqual( operator()( i ),_value ) == BT_FALSE )
			return BT_FALSE;
	}

	return BT_TRUE;
}


inline BooleanType VectorspaceElement::isGreaterThan(	double _value
														) const
{
	for( uint i=0; i<getDim( ); ++i )
	{
		if ( data[i] < _value )
			return BT_FALSE;
	}

	return BT_TRUE;
}


inline BooleanType VectorspaceElement::isSmallerThan(	double _value
														) const
{
	for( uint i=0; i<getDim( ); ++i )
	{
		if ( data[i] > _value )
			return BT_FALSE;
	}

	return BT_TRUE;
}



inline BooleanType VectorspaceElement::hasEqualComponents( ) const
{
	if ( getDim() == 0 )
		return BT_TRUE;

	double tmp = operator()( 0 );

	for( uint i=0; i<getDim(); ++i )
	{
		if ( acadoIsEqual( operator()( i ),tmp ) == BT_FALSE )
			return BT_FALSE;
	}

	return BT_TRUE;
}


inline BooleanType VectorspaceElement::isZero( ) const
{
	return isEqualTo( 0.0 );
}


inline BooleanType VectorspaceElement::isPositive( ) const
{
	return isGreaterThan( 0.0 );
}


inline BooleanType VectorspaceElement::isNegative( ) const
{
	return isSmallerThan( 0.0 );
}


inline BooleanType VectorspaceElement::isFinite( ) const
{
	for( uint i=0; i<getDim( ); ++i )
	{
		if ( acadoIsFinite( data[i] ) == BT_TRUE )
			return BT_TRUE;
	}

	return BT_FALSE;
}


inline BooleanType VectorspaceElement::hasNaN( ) const
{
	for( uint i=0; i<getDim( ); ++i )
	{
		if ( acadoIsNaN( data[i] ) == BT_TRUE )
			return BT_TRUE;
	}
	
	return BT_FALSE;
}



inline returnValue VectorspaceElement::setZero( )
{
	return setAll( 0.0 );
}


inline returnValue VectorspaceElement::setAll(	double _value
												)
{
	uint i;

	for( i=0; i<getDim( ); ++i )
		data[i] = _value;

	return SUCCESSFUL_RETURN;
}



inline double VectorspaceElement::getMax( ) const
{
	double value = -INFTY;

	for( uint i=0; i<getDim( ); ++i )
		if ( data[i] > value )
			value = data[i];

	return value;
}


inline double VectorspaceElement::getMin( ) const
{
	double value = INFTY;

	for( uint i=0; i<getDim( ); ++i )
		if ( data[i] < value )
			value = data[i];

	return value;
}


inline double VectorspaceElement::getMean( ) const
{
	double value = 0.0;

	if ( getDim( ) == 0 )
		return value;

	for( uint i=0; i<getDim( ); ++i )
		value += data[i];

	return ( value / (double)getDim() );
}

CLOSE_NAMESPACE_ACADO

/*
 *	end of file
 */
