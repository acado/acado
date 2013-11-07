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
 *    \file include/acado/matrix_vector/matrix.ipp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 31.05.2008
 */


//
// PUBLIC MEMBER FUNCTIONS:
//


BEGIN_NAMESPACE_ACADO



inline double& Matrix::operator()(	uint rowIdx,
									uint colIdx
									)
{
	ASSERT( rowIdx < getNumRows( ) );
	ASSERT( colIdx < getNumCols( ) );
	return element[rowIdx*getNumCols( ) + colIdx];
}


inline double Matrix::operator()(	uint rowIdx,
									uint colIdx
									) const
{
	ASSERT( rowIdx < getNumRows( ) );
	ASSERT( colIdx < getNumCols( ) );
	return element[rowIdx*getNumCols( ) + colIdx];
}


inline Matrix Matrix::operator+(	const Matrix& arg
									) const
{
	ASSERT( ( getNumRows( ) == arg.getNumRows( ) ) && ( getNumCols( ) == arg.getNumCols( ) ) );

	uint i;

	Matrix tmp( arg );

	for( i=0; i<getDim( ); ++i )
		tmp.element[i] += element[i];

	return tmp;
}

inline BooleanType Matrix::operator==(	const Matrix& arg
										) const
{
	if( ( getNumRows( ) == arg.getNumRows( ) ) && ( getNumCols( ) == arg.getNumCols( ) ) )
	{
		bool equal = true;
		uint i;
		for( i=0; i<getDim( ); ++i )
		{
			equal = equal && (acadoIsEqual(element[i],arg.element[i]) == BT_TRUE);	
		}
		if(equal)
		{
			return BT_TRUE;
		}
		else
		{
			return BT_FALSE;
		}
	}
	else
	{
		return BT_FALSE;
	}
}


inline Matrix& Matrix::operator+=(	const Matrix& arg
									)
{
	ASSERT( ( getNumRows( ) == arg.getNumRows( ) ) && ( getNumCols( ) == arg.getNumCols( ) ) );

	uint i;

	for( i=0; i<getDim( ); ++i )
		element[i] += arg.element[i];

	return *this;
}


inline Matrix Matrix::operator-(	const Matrix& arg
									) const
{
	ASSERT( ( getNumRows( ) == arg.getNumRows( ) ) && ( getNumCols( ) == arg.getNumCols( ) ) );

	uint i;

	Matrix tmp( getNumRows(), getNumCols() );

	for( i=0; i<getDim( ); ++i )
		tmp.element[i] = element[i] - arg.element[i];

	return tmp;
}


inline Matrix& Matrix::operator-=(	const Matrix& arg
									)
{
	ASSERT( ( getNumRows( ) == arg.getNumRows( ) ) && ( getNumCols( ) == arg.getNumCols( ) ) );

	uint i;

	for( i=0; i<getDim( ); ++i )
		element[i] -= arg.element[i];

	return *this;
}


inline Matrix& Matrix::operator*=(	double scalar
									)
{
	uint i;

	if ( fabs(scalar) >= ZERO_EPS )
	{
		for( i=0; i<getDim( ); ++i )
			element[i] *= scalar;
	}
	else
	{
		setZero( );
	}

	return *this;
}


inline Matrix& Matrix::operator/=(	double scalar
									)
{
	uint i;

	if ( fabs(scalar) >= ZERO_EPS )
	{
		for( i=0; i<getDim( ); ++i )
			element[i] /= scalar;
	}

	return *this;
}


inline Matrix Matrix::operator*(	const Matrix& arg
									) const
{
	ASSERT( getNumCols( ) == arg.getNumRows( ) );

	uint i,j,k;

	uint newNumRows = getNumRows( );
	uint newNumCols = arg.getNumCols( );
	Matrix result( newNumRows,newNumCols );
	result.setZero( );

	for( i=0; i<newNumRows; ++i )
		for( j=0; j<newNumCols; ++j )
			for( k=0; k<getNumCols( ); ++k )
				result( i,j ) += operator()( i,k ) * arg( k,j );

	return result;
}


inline Matrix Matrix::operator^(	const Matrix& arg
									) const
{
	ASSERT( getNumRows( ) == arg.getNumRows( ) );

	uint i,j,k;

	uint newNumRows = getNumCols( );
	uint newNumCols = arg.getNumCols( );
	Matrix result( newNumRows,newNumCols );
	result.setZero( );

	for( i=0; i<newNumRows; ++i )
		for( j=0; j<newNumCols; ++j )
			for( k=0; k<getNumRows( ); ++k )
				result( i,j ) += operator()( k,i ) * arg( k,j );

	return result;
}


inline Vector Matrix::operator*(	const Vector& arg
									) const
{
	ASSERT( getNumCols( ) == arg.getDim( ) );

	uint i,j;

	uint newNumRows = getNumRows( );
	Vector result( newNumRows );
	result.setZero( );

	for( i=0; i<newNumRows; ++i )
		for( j=0; j<getNumCols( ); ++j )
			result( i ) += operator()( i,j ) * arg( j );

	return result;
}



inline Vector Matrix::operator^(	const Vector& arg
									) const
{
	ASSERT( getNumRows( ) == arg.getDim( ) );

	uint i,j;

	uint newNumRows = getNumCols( );
	Vector result( newNumRows );
	result.setZero( );

	for( j=0; j<getNumRows( ); ++j )
		for( i=0; i<newNumRows; ++i )
			result( i ) += operator()( j,i ) * arg( j );

	return result;
}


inline Matrix Matrix::transpose() const{

     Matrix result( getNumCols(), getNumRows() );

     uint i,j;

     for( i = 0; i < getNumRows(); i++ )
         for( j = 0; j < getNumCols(); j++ )
             result(j,i) = operator()( i,j );

     return result;
}

inline Matrix Matrix::negativeTranspose() const{

     Matrix result( getNumCols(), getNumRows() );

     uint i,j;

     for( i = 0; i < getNumRows(); i++ )
         for( j = 0; j < getNumCols(); j++ )
             result(j,i) = -operator()( i,j );

     return result;
}


inline Matrix& Matrix::makeVector( )
{
	nRows = getDim( );
	nCols = 1;
	
	return *this;
}


inline Matrix Matrix::minus(){

     Matrix result( getNumRows(), getNumCols() );

     uint i,j;

     for( i = 0; i < getNumRows(); i++ )
         for( j = 0; j < getNumCols(); j++ )
             result(i,j) = -operator()( i,j );

     return result;
}


// inline returnValue Matrix::print( FILE *file ){
// 
//     return writeDoublePointerToFile(element,getNumRows(),getNumCols(),file);
// }



inline uint Matrix::getNumRows( ) const
{
	return nRows;
}

inline uint Matrix::getNumCols( ) const
{
	return nCols;
}


inline Vector Matrix::getRow(	uint idx
								) const
{
	ASSERT( idx < getNumRows( ) );

	uint i;

	Vector result( getNumCols( ) );

	for( i=0; i<getNumCols( ); ++i )
		result( i ) = operator()( idx,i );

	return result;
}


inline Vector Matrix::getCol(	uint idx
								) const
{
	ASSERT( idx < getNumCols( ) );

	uint i;

	Vector result( getNumRows( ) );

	for( i=0; i<getNumRows( ); ++i )
		result( i ) = operator()( i,idx );

	return result;
}


inline returnValue Matrix::setRow(	uint idx,
									const Vector& arg
									)
{
	ASSERT( idx < getNumRows( ) );
	ASSERT( getNumCols( ) == arg.getDim( ) );

	uint i;

	for( i=0; i<getNumCols( ); ++i )
		operator()( idx,i ) = arg( i );

	return SUCCESSFUL_RETURN;
}


inline returnValue Matrix::setCol(	uint idx,
									const Vector& arg
									)
{
	ASSERT( idx < getNumCols( ) );
	ASSERT( getNumRows( ) == arg.getDim( ) );

	uint i;

	for( i=0; i<getNumRows( ); ++i )
		operator()( i,idx ) = arg( i );

	return SUCCESSFUL_RETURN;
}



inline Matrix Matrix::getRows(	uint idx1,
								uint idx2
								) const
{
	Matrix newMatrix;

	if (idx1 >= getNumRows( ))
		return newMatrix;

	if (idx2 >= getNumRows( ))
		return newMatrix;

	if ( idx1 > idx2 )
		return newMatrix;

	newMatrix.init( idx2-idx1+1,getNumCols( ) );

	for( uint i=idx1; i<=idx2; ++i )
		for( uint j=0; j<getNumCols( ); ++j )
			newMatrix( i-idx1,j ) = operator()( i,j );

	return newMatrix;
}


inline Matrix Matrix::getCols(	uint idx1,
								uint idx2
								) const
{
	Matrix newMatrix;

	if (idx1 >= getNumCols( ))
		return newMatrix;

	if (idx2 >= getNumCols( ))
		return newMatrix;

	if ( idx1 > idx2 )
		return newMatrix;

	newMatrix.init( getNumRows( ),idx2-idx1+1 );

	for( uint i=0; i<getNumRows( ); ++i )
		for( uint j=idx1; j<=idx2; ++j )
			newMatrix( i,j-idx1 ) = operator()( i,j );

	return newMatrix;
}


inline Vector Matrix::getDiag( ) const
{
	ASSERT( isSquare( ) == BT_TRUE );

	Vector result( getNumRows( ) );

	for( uint i=0; i<getNumRows( ); ++i )
		result( i ) = operator()( i,i );

	return result;
}


inline returnValue Matrix::setIdentity( )
{
	ASSERT( isSquare( ) == BT_TRUE );

	uint i;

	setZero( );
	for( i=0; i<getNumRows( ); ++i )
		operator()( i,i ) = 1.0;

	return SUCCESSFUL_RETURN;
}


inline BooleanType Matrix::isSquare( ) const
{
	if ( getNumRows( ) == getNumCols( ) )
		return BT_TRUE;
	else
		return BT_FALSE;
}


inline Matrix Matrix::absolute(){

    uint i;
    Matrix result( nRows, nCols );

    for( i = 0; i < getDim(); i++ )
        result.element[i] = fabs(element[i]);

    return result;
}


inline Matrix Matrix::positive(){

    uint i;
    Matrix result( nRows, nCols );

    for( i = 0; i < getDim(); i++ ){
        if( element[i] >= 0.0 )
            result.element[i] = element[i];
        else
            result.element[i] = 0.0;
    }
    return result;
}


inline Matrix Matrix::negative(){

    uint i;
    Matrix result( nRows, nCols );

    for( i = 0; i < getDim(); i++ ){
        if( element[i] <= 0.0 )
            result.element[i] = element[i];
        else
            result.element[i] = 0.0;
    }
    return result;
}

inline Vector sumRow(const Matrix& arg){
	Vector result(arg.getNumRows());
	result.setAll(1.0);
	return arg.transpose()*result;
}

inline Vector sumCol(const Matrix& arg) {
	Vector result( arg.getNumCols());
	result.setAll(1.0);
	return arg*result;
}

CLOSE_NAMESPACE_ACADO


/*
 *	end of file
 */
