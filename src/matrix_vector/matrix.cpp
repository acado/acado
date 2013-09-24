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
 *    \file src/matrix_vector/matrix.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 31.05.2008
 */


#include <acado/matrix_vector/matrix_vector.hpp>
#include <acado/variables_grid/variables_grid.hpp>
#include <acado/sparse_solver/sparse_solver.hpp>
#include <include/acado_csparse/acado_csparse.hpp>



BEGIN_NAMESPACE_ACADO



//
// PUBLIC MEMBER FUNCTIONS:
//

Matrix::Matrix( ) : VectorspaceElement( )
{
	nRows = 0;
	nCols = 0;
	solver = 0;
}


Matrix::Matrix( uint _nRows, uint _nCols ) : VectorspaceElement( _nRows * _nCols )
{
	nRows = _nRows;
	nCols = _nCols;
	solver = 0;
}


Matrix::Matrix( uint _nRows, uint _nCols, const double* const _values ) : VectorspaceElement( _nRows * _nCols, _values )
{
	nRows = _nRows;
	nCols = _nCols;
	solver = 0;
}


Matrix::Matrix( FILE *file ) : VectorspaceElement( )
{
	operator=( file );
	solver = 0;
}


Matrix::Matrix( int value ) : VectorspaceElement( 1 )
{
	nRows = 1;
	nCols = 1;
	solver = 0;

	operator()( 0,0 ) = (double) value;
}


Matrix::Matrix( double value ) : VectorspaceElement( 1 )
{
	nRows = 1;
	nCols = 1;
	solver = 0;

	operator()( 0,0 ) = value;
}


Matrix::Matrix(	const Vector& value,
				BooleanType doTranspose
				) : VectorspaceElement( value )
{
	if ( doTranspose == BT_FALSE )
	{
		nRows = value.getDim( );
		nCols = 1;
	}
	else
	{
		nRows = 1;
		nCols = value.getDim( );
	}

	solver = 0;
}


Matrix::Matrix( const VariablesGrid& value ) : VectorspaceElement( value.getNumPoints( )*( value.getNumValues( )+1 ) )
{
	nRows = value.getNumPoints( );
	nCols = value.getNumValues( )+1;

	for( uint run1=0; run1<value.getNumPoints( ); ++run1 )
	{
		operator()( run1,0 ) = value.getTime( run1 );

		for( uint run2=0; run2<value.getNumValues( ); ++run2 )
			operator()( run1,1+run2 ) = value( run1,run2 );
	}

	solver = 0;
}


Matrix::Matrix( const Matrix& rhs ) : VectorspaceElement( rhs )
{
	nRows = rhs.nRows;
	nCols = rhs.nCols;

    if( rhs.solver == 0 )
         solver = 0;
    else solver = rhs.solver->clone(); 
}


Matrix::~Matrix( ){

    if( solver != 0 )
        delete solver;
}


Matrix& Matrix::operator=( const Matrix& rhs ){

    if ( this != &rhs ){

        if( solver != 0 )
            delete solver;

		VectorspaceElement::operator=( rhs );

		nRows = rhs.nRows;
		nCols = rhs.nCols;

        if( rhs.solver == 0 )
             solver = 0;
        else solver = rhs.solver->clone(); 
    }
    return *this;
}


Matrix& Matrix::operator=( FILE *rhs ){

    int     nR, nC;
    double *x  ;
    returnValue returnvalue;

    x = 0;
    returnvalue = allocateDoublePointerFromFile(rhs, &x, nR, nC);

    if( returnvalue == SUCCESSFUL_RETURN && nR > 0 && nC > 0 ){
        init( nR, nC, x );
        if( x != 0 ) free(x);
    }
    else{
        if( x != 0 ) free(x);
        ACADOINFO(returnvalue);
    }
    return *this;
}


Matrix& Matrix::operator^=( const double *rhs ){

    uint i,j;

    for( i = 0; i < nRows; i++ )
        for( j = 0; j < nCols; j++ )
            operator()(i,j) = rhs[j*nRows+i];

    return *this;
}


double* operator^=( double *lhs, Matrix &rhs ){

    uint i,j;

    for( i = 0; i < rhs.getNumRows(); i++ )
        for( j = 0; j < rhs.getNumCols(); j++ )
            lhs[j*rhs.getNumRows()+i] = rhs.operator()(i,j);

    return lhs;
}


returnValue Matrix::init( uint _nRows, uint _nCols ){

	VectorspaceElement::init( _nRows*_nCols );
	nRows = _nRows;
	nCols = _nCols;
	solver = 0;

	return SUCCESSFUL_RETURN;
}



returnValue Matrix::init( uint _nRows, uint _nCols, double* _values )
{
	VectorspaceElement::init( _nRows*_nCols,_values );
	nRows = _nRows;
	nCols = _nCols;
	solver = 0;

	return SUCCESSFUL_RETURN;
}


returnValue Matrix::appendRows( const Matrix& arg )
{
	if( isEmpty() == BT_TRUE ){
        operator=( arg );
        return SUCCESSFUL_RETURN;
    }

	if ( arg.getNumRows( ) == 0 )
		return SUCCESSFUL_RETURN;

	if ( ( getNumCols( ) != 0 ) && ( arg.getNumCols( ) != getNumCols( ) ) )
		return ACADOERROR( RET_VECTOR_DIMENSION_MISMATCH );

	Matrix oldMatrix = *this;

	if ( init( oldMatrix.getNumRows()+arg.getNumRows(),arg.getNumCols() ) != SUCCESSFUL_RETURN )
		return ACADOERROR( RET_UNKNOWN_BUG );

	for ( uint j=0; j<arg.getNumCols(); ++j )
	{
		for( uint i=0; i<oldMatrix.getNumRows(); ++i )
			operator()( i,j ) = oldMatrix( i,j );
		for( uint i=0; i<arg.getNumRows(); ++i )
			operator()( oldMatrix.getNumRows()+i,j ) = arg( i,j );
	}

	return SUCCESSFUL_RETURN;
}


returnValue Matrix::appendCols( const Matrix& arg )
{
	if( isEmpty() == BT_TRUE ){
        operator=( arg );
        return SUCCESSFUL_RETURN;
    }

	if ( arg.getNumCols( ) == 0 )
		return SUCCESSFUL_RETURN;

	if ( ( getNumRows( ) != 0 ) && ( arg.getNumRows( ) != getNumRows( ) ) )
		return ACADOERROR( RET_VECTOR_DIMENSION_MISMATCH );

	Matrix oldMatrix = *this;

	if ( init( arg.getNumRows(),oldMatrix.getNumCols()+arg.getNumCols() ) != SUCCESSFUL_RETURN )
		return ACADOERROR( RET_UNKNOWN_BUG );

	for ( uint i=0; i<arg.getNumRows(); ++i )
	{
		for( uint j=0; j<oldMatrix.getNumCols(); ++j )
			operator()( i,j ) = oldMatrix( i,j );
		for( uint j=0; j<arg.getNumCols(); ++j )
			operator()( i,oldMatrix.getNumCols()+j ) = arg( i,j );
	}

	return SUCCESSFUL_RETURN;
}



returnValue operator<<( FILE *file, Matrix& arg ){

    return arg.printToFile(file);
}


returnValue Matrix::printToFile(	const char* const filename,
									const char* const name,
									const char* const startString,
									const char* const endString,
									uint width,
									uint precision,
									const char* const colSeparator,
									const char* const rowSeparator
									) const
{
	return VectorspaceElement::printToFile( filename,name,startString,endString,width,precision,colSeparator,rowSeparator );
}


returnValue Matrix::printToFile(	FILE* file,
									const char* const name,
									const char* const startString,
									const char* const endString,
									uint width,
									uint precision,
									const char* const colSeparator,
									const char* const rowSeparator
									) const
{
	return VectorspaceElement::printToFile( file,name,startString,endString,width,precision,colSeparator,rowSeparator );
}



returnValue Matrix::printToFile(	const char* const filename,
									const char* const name,
									PrintScheme printScheme
									) const
{
	FILE* file = 0;
	MatFile* matFile = 0;
	
	switch ( printScheme )
	{
		case PS_MATLAB_BINARY:
			matFile = new MatFile;
			
			matFile->open( filename );
			matFile->write( *this,name );
			matFile->close( );
			
			delete matFile;
			return SUCCESSFUL_RETURN;

		default:
			file = fopen( filename,"w+" );

			if ( file == 0 )
				return ACADOERROR( RET_FILE_CAN_NOT_BE_OPENED );

			printToFile( file, name,printScheme );

			fclose( file );
			return SUCCESSFUL_RETURN;
	}
}


returnValue Matrix::printToFile(	FILE* file,
									const char* const name,
									PrintScheme printScheme
									) const
{
	return VectorspaceElement::printToFile( file,name,printScheme );
}


returnValue Matrix::printToString(	char** string,
									const char* const name,
									const char* const startString,
									const char* const endString,
									uint width,
									uint precision,
									const char* const colSeparator,
									const char* const rowSeparator,
									BooleanType allocateMemory
									) const
{
	uint i,j;

	/* determine length of single component */
	uint componentLength = width;

	// 0.e-0000
	if ( componentLength < (9 + (uint)precision) )
		componentLength = 9 + precision;

	char* componentString = new char[componentLength];

	/* determine length of whole string */
	if ( allocateMemory == BT_TRUE )
	{
		uint stringLength = determineStringLength( name,startString,endString,
												   width,precision,colSeparator,rowSeparator );

		*string = new char[stringLength];

		for( i=0; i<stringLength; ++i )
			(*string)[i] = '\0';
	}

	if ( getStringLength(name) > 0 )
	{
		strcat( *string,name );
		strcat( *string," = " );
	}

	if ( getStringLength(startString) > 0 )
		strcat( *string,startString );

	int writtenChars;

	for( i=0; i<getNumRows( ); ++i )
	{
		for( j=0; j<getNumCols( ); ++j )
		{
			if ( precision > 0 )
				writtenChars = ::sprintf( componentString,"%*.*e",width,precision,operator()( i,j ) );
			else
			{
				writtenChars = ::sprintf( componentString,"%*.0d",width,(int)operator()( i,j ) );

				// spritf does not print zeros, that's a hack to circumvent this!
				if ( ( (int)operator()( i,j ) == 0 ) && ( writtenChars > 0 ) )
					componentString[writtenChars-1] = '0';
			}

			if ( ( writtenChars < 0 ) || ( (uint)writtenChars+1 > componentLength ) )
			{
				delete[] componentString;
				return ACADOERROR( RET_UNKNOWN_BUG );
			}

			strcat( *string,componentString );

			if ( j < getNumCols( )-1 )
				if ( getStringLength(colSeparator) > 0 )
					strcat( *string,colSeparator );
		}
		
		if ( i < getNumRows( )-1 )
			if ( getStringLength(rowSeparator) > 0 )
				strcat( *string,rowSeparator );
	}

	if ( getStringLength(endString) > 0 )
		strcat( *string,endString );

	delete[] componentString;

	return SUCCESSFUL_RETURN;
}


returnValue Matrix::printToString(	char** string,
									const char* const name,
									PrintScheme printScheme,
									BooleanType allocateMemory
									) const
{
	return VectorspaceElement::printToString( string,name,printScheme,allocateMemory );
}


uint Matrix::determineStringLength(	const char* const name,
									const char* const startString,
									const char* const endString,
									uint width,
									uint precision,
									const char* const colSeparator,
									const char* const rowSeparator
									) const
{
	uint componentLength = width;

	// 0.e-0000
	if ( componentLength < (9 + (uint)precision) )
		componentLength = 9 + precision;
	

	// allocate string of sufficient size (being quite conservative)
	uint stringLength;

	if ( getNumRows( ) * getNumCols( ) > 0 )
	{
		stringLength =	1
						+ getNumRows( ) * getStringLength(startString)
						+ ( getNumRows( )*getNumCols( ) ) * componentLength
						+ ( (getNumCols( )-1)*getNumRows( ) ) * getStringLength(colSeparator)
						+ ( getNumRows( )-1 ) * getStringLength(rowSeparator)
						+ getNumRows( ) * getStringLength(endString);
	}
	else
	{
		stringLength =	1
						+ getStringLength(startString) 
						+ getStringLength(endString);
	}

	if ( getStringLength(name) > 0 )
		stringLength += getStringLength(name)+3;

	return stringLength; 
}



BooleanType Matrix::isSymmetric( ) const
{
	if ( isSquare( ) == BT_FALSE )
		return BT_FALSE;
	
	for( uint i=0; i<getNumRows( ); ++i )
		for( uint j=i+1; j<getNumRows( ); ++j )
			if ( acadoIsEqual( operator()( i,j ),operator()( j,i ) ) == BT_FALSE )
				return BT_FALSE;
			
	return BT_TRUE;
}

BooleanType Matrix::isDiagonal( ) const
{
	if (isSquare() == BT_FALSE)
		return BT_FALSE;

	for (unsigned i = 0; i < getNumRows( ); ++i)
		for (unsigned j = 0; j < getNumCols( ); ++j)
			if ((i != j) && (acadoIsZero( operator()(i, j) ) == BT_FALSE))
				return BT_FALSE;

	return BT_TRUE;
}


returnValue Matrix::symmetrize( )
{
	if ( isSquare( ) == BT_FALSE )
		return ACADOERROR( RET_MATRIX_NOT_SQUARE );
	
	double mean;

	for( uint i=0; i<getNumRows( ); ++i )
		for( uint j=i+1; j<getNumRows( ); ++j )
		{
			mean = ( operator()( i,j ) + operator()( j,i ) ) / 2.0;
			operator()( i,j ) = mean;
			operator()( j,i ) = mean;
		}

	return SUCCESSFUL_RETURN;
}


BooleanType Matrix::isPositiveSemiDefinite( ) const
{
	if ( isSquare( ) == BT_FALSE )
		return BT_FALSE;

	Vector eigenValues = getEigenvalues( );

	if ( eigenValues.getMin() >= 0.0 )
		return BT_TRUE;
	else
		return BT_FALSE;
}


BooleanType Matrix::isPositiveDefinite( ) const
{
	if ( isSquare( ) == BT_FALSE )
		return BT_FALSE;

	Vector eigenValues = getEigenvalues( );

	return acadoIsPositive( eigenValues.getMin() );
}


double Matrix::getNorm(	MatrixNorm norm
						) const
{
	double value = 0.0;
	double tmp;

	switch( norm )
	{
		case MN_COLUMN_SUM:
			for( uint i=0; i<getNumCols(); ++i )
			{
				tmp = 0.0;
				for( uint j=0; j<getNumRows(); ++j )
					tmp += fabs( operator()( i,j ) );

				if ( tmp > value )
					value = tmp;
			}
			break;

		case MN_ROW_SUM:
			for( uint j=0; j<getNumRows(); ++j )
			{
				tmp = 0.0;
				for( uint i=0; i<getNumCols(); ++i )
					tmp += fabs( operator()( i,j ) );

				if ( tmp > value )
					value = tmp;
			}
			break;

		case MN_FROBENIUS:
			for( uint i=0; i<getNumCols(); ++i )
				for( uint j=0; j<getNumRows(); ++j )
				value += operator()( i,j ) * operator()( i,j );
			value = sqrt( value );
			break;

		default:
			value = -1.0;
	}

	return value;
}


double Matrix::getTrace( ) const
{
	if ( isSquare( ) == BT_FALSE )
		return -INFTY;

	double value = 0.0;
	for( uint i=0; i<getNumCols(); ++i )
		value += operator()( i,i );

	return value;
}


Vector Matrix::getEigenvalues( ) const{

    Matrix Q;
    return getEigenvalues( Q );
}


Vector Matrix::getEigenvalues( Matrix &Q ) const{

    int n = getNumRows();
    ASSERT( n == (int) getNumCols() );

    double *eigenvalues  = new double[n  ];
    double *eigenvectors = new double[dim];
    double *d_element    = new double[dim];

    int i, j, k, mm;
    double *pAk, *pAm, *p_r, *p_e;
    double threshold_norm;
    double threshold;
    double tan_phi, sin_phi, cos_phi, tan2_phi, sin2_phi, cos2_phi;
    double sin_2phi, cot_2phi;
    double dum1;
    double dum2;
    double dum3;
    double max;

    for(i = 0; i < (int) dim; i++)
        d_element[i] = element[i];

    for(p_e = eigenvectors, i = 0; i < n; i++)
        for(j = 0; j < n; p_e++, j++)
            if(i == j) *p_e = 1.0; else *p_e = 0.0;

    for(threshold = 0.0, pAk = d_element, i = 0; i < ( n - 1 ); pAk += n, i++) 
        for(j = i + 1; j < n; j++) threshold += *(pAk + j) * *(pAk + j);
    threshold = sqrt(threshold + threshold);
    threshold_norm = threshold * EPS;
    max = threshold + 1.0;
    while (threshold > threshold_norm) {
        threshold /= 10.0;
        if (max < threshold) continue;
        max = 0.0;
        for (pAk = d_element, k = 0; k < (n-1); pAk += n, k++) {
            for (pAm = pAk + n, mm = k + 1; mm < n; pAm += n, mm++) {
                if ( fabs(*(pAk + mm)) < threshold ) continue;

                cot_2phi = 0.5 * ( *(pAk + k) - *(pAm + mm) ) / *(pAk + mm);
                dum1 = sqrt( cot_2phi * cot_2phi + 1.0);
                if (cot_2phi < 0.0) dum1 = -dum1;
                tan_phi = -cot_2phi + dum1;
                tan2_phi = tan_phi * tan_phi;
                sin2_phi = tan2_phi / (1.0 + tan2_phi);
                cos2_phi = 1.0 - sin2_phi;
                sin_phi = sqrt(sin2_phi);
                if (tan_phi < 0.0) sin_phi = - sin_phi;
                cos_phi = sqrt(cos2_phi); 
                sin_2phi = 2.0 * sin_phi * cos_phi;

                p_r = d_element;
                dum1 = *(pAk + k);
                dum2 = *(pAm + mm);
                dum3 = *(pAk + mm);
                *(pAk + k) = dum1 * cos2_phi + dum2 * sin2_phi + dum3 * sin_2phi;
                *(pAm + mm) = dum1 * sin2_phi + dum2 * cos2_phi - dum3 * sin_2phi;
                *(pAk + mm) = 0.0;
                *(pAm + k) = 0.0;
                for (i = 0; i < n; p_r += n, i++) {
                   if ( (i == k) || (i == mm) ) continue;
                   if ( i < k ) dum1 = *(p_r + k); else dum1 = *(pAk + i);
                   if ( i < mm ) dum2 = *(p_r + mm); else dum2 = *(pAm + i);
                   dum3 = dum1 * cos_phi + dum2 * sin_phi;
                   if ( i < k ) *(p_r + k) = dum3; else *(pAk + i) = dum3;
                   dum3 = - dum1 * sin_phi + dum2 * cos_phi;
                   if ( i < mm ) *(p_r + mm) = dum3; else *(pAm + i) = dum3;
                }
                for (p_e = eigenvectors, i = 0; i < n; p_e += n, i++) {
                   dum1 = *(p_e + k);
                   dum2 = *(p_e + mm);
                   *(p_e + k) = dum1 * cos_phi + dum2 * sin_phi;
                   *(p_e + mm) = - dum1 * sin_phi + dum2 * cos_phi;
                }
            }
            for (i = 0; i < n; i++)
                if ( i == k ) continue;
                else if ( max < fabs(*(pAk + i))) max = fabs(*(pAk + i));
        }
    }
    for (pAk = d_element, k = 0; k < n; pAk += n, k++) eigenvalues[k] = *(pAk + k);

    Q.init(n,n);
    for(i = 0; i < (int) dim; i++)
        Q.element[i] = eigenvectors[i];
    Vector tmp(n,eigenvalues);

    delete[] eigenvalues ;
    delete[] eigenvectors;
    delete[] d_element   ;

    return tmp;
}


returnValue Matrix::printEigenvalues( ) const{

    Vector tmp;
    tmp = getEigenvalues();
    return tmp.print();
}


returnValue Matrix::getSingularValueDecomposition( Matrix &U_, Vector &D, Matrix &V_ ) const{

    if( getNumRows() >= getNumCols() ){

        int run1;

        double *A = new double[dim];
        for( run1 = 0; run1 < (int) dim; run1++ )
            A[run1] = element[run1];

        double *U = new double[dim];
        double *V = new double[getNumCols()*getNumCols()];

        double *singular_values = new double[getNumCols()];
        double *dummy_array     = new double[getNumCols()];

        householdersReductionToBidiagonalForm( A, getNumRows(), getNumCols(), U, V,
                                                singular_values, dummy_array);
        givensReductionToDiagonalForm( getNumRows(), getNumCols(), U, V, singular_values, dummy_array );
        sortByDecreasingSingularValues( getNumRows(), getNumCols(), singular_values, U, V);

        U_.init(getNumRows(),getNumCols());
        for( run1 = 0; run1 < (int) dim; run1++ )
            U_.element[run1] = U[run1];

        V_.init(getNumCols(),getNumCols());
        for( run1 = 0; run1 < (int) (getNumCols()*getNumCols()); run1++ )
            V_.element[run1] = V[run1];

        D.init(getNumCols());

        for( run1 = 0; run1 < (int) getNumCols(); run1++ )
            D.operator()(run1) = singular_values[run1];

        delete[] A;
        delete[] U;
        delete[] V;
        delete[] singular_values;
        delete[] dummy_array    ;
    }
    else{

        int run1, run2;

        double *A = new double[dim];
        for( run1 = 0; run1 < (int) getNumRows(); run1++ )
            for( run2 = 0; run2 < (int) getNumCols(); run2++ )
                A[run2*getNumRows()+run1] = element[run1*getNumCols()+run2];

        double *U = new double[dim];
        double *V = new double[getNumRows()*getNumRows()];

        double *singular_values = new double[getNumRows()];
        double *dummy_array     = new double[getNumRows()];

        householdersReductionToBidiagonalForm( A, getNumCols(), getNumRows(), U, V,
                                                singular_values, dummy_array);
        givensReductionToDiagonalForm( getNumCols(), getNumRows(), U, V, singular_values, dummy_array );
        sortByDecreasingSingularValues( getNumCols(), getNumRows(), singular_values, U, V);

        U_.init(getNumRows(),getNumRows());
        for( run1 = 0; run1 < (int) getNumRows(); run1++ )
            for( run2 = 0; run2 < (int) getNumRows(); run2++ )
                U_.element[run1+run2*getNumRows()] = V[run2+run1*getNumRows()];

        V_.init(getNumRows(),getNumCols());
        for( run1 = 0; run1 < (int) getNumRows(); run1++ )
            for( run2 = 0; run2 < (int) getNumCols(); run2++ )
                V_.element[run1*getNumCols()+run2] = U[run2*getNumRows()+run1];

        D.init(getNumRows());

        for( run1 = 0; run1 < (int) getNumRows(); run1++ )
            D.operator()(run1) = singular_values[run1];

        delete[] A;
        delete[] U;
        delete[] V;
        delete[] singular_values;
        delete[] dummy_array    ;
    }

    return SUCCESSFUL_RETURN;
}



Matrix Matrix::getCholeskyDecomposition() const{

    int n = getNumRows();
    ASSERT( n == (int) getNumCols() );

    int i, k, p;
    double *p_Lk0;
    double *p_Lkp;
    double *p_Lkk;
    double *p_Li0;
    double reciprocal;

    double *A = new double[dim];
    for( k = 0; k < (int) dim; k++ )
        A[k] = element[k];

    for (k = 0, p_Lk0 = A; k < n; p_Lk0 += n, k++) {
       p_Lkk = p_Lk0 + k;
       for (p = 0, p_Lkp = p_Lk0; p < k; p_Lkp += 1,  p++)
          *p_Lkk -= *p_Lkp * *p_Lkp;
       *p_Lkk = sqrt( *p_Lkk );
       ASSERT( *p_Lkk >= EPS );
       reciprocal = 1.0 / *p_Lkk;
       p_Li0 = p_Lk0 + n;
       for (i = k + 1; i < n; p_Li0 += n, i++) {
          for (p = 0; p < k; p++)
             *(p_Li0 + k) -= *(p_Li0 + p) * *(p_Lk0 + p);
          *(p_Li0 + k) *= reciprocal;
          *(p_Lk0 + i) = *(p_Li0 + k);
       }
    }

    Matrix tmp(n,n);
    tmp.setZero();

    for( i = 0; i < n; i++ )
        for( k = 0; k <= i; k++ )
             tmp(i,k) = A[i*n+k];

    delete[] A;

    return tmp;
}


Matrix Matrix::getCholeskyDecomposition( Vector &D ) const{

   int n = getNumRows();
   ASSERT( n == (int) getNumCols() );

   int i, j, k;
   double *p_i;
   double *p_j;
   double *p_k;
   double ld;

    double *A = new double[dim];
    for( k = 0; k < (int) dim; k++ ){
        A[k] = element[k];
    }

   for (i = 1, p_i = A + n; i < n; p_i += n, i++) {
      for (j = 0, p_j = A; j < i; j++, p_j += n) 
         for (k = 0; k < j; k++)
            *(p_i + j) -= *(p_i + k) * *(p_j + k);
      for (k = 0, p_k = A; k < i; p_k += n, k++) {
            ld = *(p_i + k) / *(p_k + k);
            *(p_i + i) -= *(p_i + k) * ld;
            *(p_i + k) = ld;
            *(p_k + i) = ld;
      }
      ASSERT( *(p_i + i) >= EPS );
   }

    Matrix tmp(n,n);
    tmp.setIdentity();
    D.init(n);

    for( i = 0; i < n; i++ )
        for( k = 0; k < i; k++ )
             tmp(i,k) = A[i*n+k];

    for( i = 0; i < n; i++ )
        D(i) = A[i*n+i];

    delete[] A;
    return tmp;
}





Matrix Matrix::getCholeskyInverse() const{

   int n = getNumRows();
   ASSERT( n == (int) getNumCols() );

   int i, j, k;
   double *p_i, *p_j, *p_k;
   double sum;

   Matrix cholesky;
   cholesky = getCholeskyDecomposition();

   double *LU = new double[n*n];

   for( i = 0; i < n; i++ ){
       for( k = 0; k <= i; k++ ){
           LU[i+n*k] = cholesky(i,k);
           LU[i*n+k] = cholesky(i,k);
       }
   }

   lowerTriangularInverse(LU, n);

   for (i = 0, p_i = LU; i < n; i++, p_i += n) {
      for (j = 0, p_j = LU; j <= i; j++, p_j += n) {
         sum = 0.0;
         for (k = i, p_k = p_i; k < n; k++, p_k += n)
            sum += *(p_k + i) * *(p_k + j);
         *(p_i + j) = sum;
         *(p_j + i) = sum;
      }
   }

   Matrix tmp(n,n,LU);
   delete[] LU;

   return tmp;
}




Matrix Matrix::getInverse() const{

    ASSERT( getNumCols() == getNumRows() );

    Matrix U,V;
    Vector D;

    getSingularValueDecomposition( U, D, V );

    uint run1, run2;

    for( run1 = 0; run1 < getNumRows(); run1++ )
        for( run2 = 0; run2 < getNumCols(); run2++ )
            V(run1,run2) = V(run1,run2)/D(run2);

    return V*U.transpose();
}


returnValue Matrix::computeQRdecomposition(){

    int run1, run2, run3;

    if( solver != 0 ){
        delete solver;
        solver = 0;
    }

    ASSERT( getNumRows() == getNumCols() );

    double r    ;
    double h_s  ;
    double kappa;
    double ll   ;
    double ttt  ;
    int nnn = getNumRows();
    Matrix ddd(1,nnn);

    for(run2 = 0; run2 < nnn; run2++){
        r = 0.0;
        for(run1 = run2; run1 < nnn; run1++){
            r = r + operator()(run1,run2) * operator()(run1,run2);
        }
        if( r < EPS )
            return ACADOERROR(RET_DIV_BY_ZERO);
        if( operator()(run2,run2) < 0.0 ){
            h_s         = sqrt(r);
            ddd(0,run2) = h_s      ;
        }
        else{
            h_s         = -sqrt(r);
            ddd(0,run2) = h_s     ;
        }
        ttt = h_s * operator()(run2,run2) - r;
        kappa = 1.0/ttt;
        operator()(run2,run2) -= h_s;
        for(run3 = run2+1; run3 < nnn; run3++){
            ll = 0;
            for(run1 = run2; run1 < nnn; run1++)
                ll += operator()(run1,run2) * operator()(run1,run3);
            ll = kappa * ll;
            for(run1 = run2; run1 < nnn; run1++)
                operator()(run1,run3) = operator()(run1,run3) + operator()(run1,run2) * ll;
        }
    }
    appendRows(ddd);
    return SUCCESSFUL_RETURN;
}


Vector Matrix::solveQR( const Vector &b ) const{

    int run1, run2;
    double dotp, vv, cc;
    const int m = getNumCols();
    Vector x(b);

    ASSERT( b.getDim() == getNumCols() );

    for( run1 = 0; run1 < m; run1++ ){

        dotp = 0.0;
        vv   = 0.0;
        for( run2 = run1; run2 < m; run2++ ){
            vv   = vv   + operator()(run2,run1)*operator()(run2,run1);
            dotp = dotp + operator()(run2,run1)*x(run2);
        }

        cc = 2.0*dotp/vv;

        for( run2 = run1; run2 < m; run2++ ){
            x(run2) -= cc*operator()(run2,run1);
        }
    }

    for( run1 = m-1; run1 >= 0; run1-- ){
        for( run2 = run1+1; run2 < m; run2++ ){
            x(run1) -= operator()(run1,run2)*x(run2);
        }
        x(run1) /= operator()(m,run1);
    }
    return x;
}

Vector Matrix::solveTransposeQR( const Vector &b ) const{

    int run1, run2;
    double dotp, vv, cc;
    const int m = getNumCols();
    Vector x(b);

    ASSERT( b.getDim() == getNumCols() );

    for( run1 = 0; run1 < m; run1++ ){
        x(run1) = b(run1);
        for( run2 = 0; run2 < run1; run2++ ){
            x(run1) -= operator()(run2,run1)*x(run2);
        }
        x(run1) /= operator()(m,run1);
    }
    for( run1 = m-1; run1 >= 0; run1-- ){
        dotp = 0.0;
        vv   = 0.0;
        for( run2 = run1; run2 < m; run2++ ){
            vv   = vv   + operator()(run2,run1)*operator()(run2,run1);
            dotp = dotp + operator()(run2,run1)*x(run2);
        }
        cc = 2.0*dotp/vv;
        for( run2 = run1; run2 < m; run2++ ){
            x(run2) -= cc*operator()(run2,run1);
        }
    }
    return x;
}


returnValue Matrix::computeSparseLUdecomposition(){

    ASSERT( solver == 0 );
    ASSERT( getNumRows() == getNumCols() );

    solver = new ACADOcsparse();

    int run1,run2;
    double ZERO_TOL = 1.e-12;
    const int n = getNumRows();

    double *A    = new double[dim];
    int    *idx1 = new int   [dim];
    int    *idx2 = new int   [dim];

    int nDense = 0;

    for( run1 = 0; run1 < n; run1++ ){
        for( run2 = 0; run2 < n; run2++ ){
            if( fabs( operator()(run1,run2) ) > ZERO_TOL ){
                A  [nDense]  = operator()(run1,run2);
                idx1[nDense] = run1;
                idx2[nDense] = run2;
                nDense++;
            }
        }
    }

    solver->setDimension      ( n          );
    solver->setNumberOfEntries( nDense     );
    solver->setIndices        ( idx1, idx2 );
    solver->setMatrix         ( A          );

    delete[] A;
    delete[] idx1;
    delete[] idx2;

    return SUCCESSFUL_RETURN;
}


Vector Matrix::solveSparseLU( const Vector &b ) const{

    ASSERT( solver != 0 );

    int run1;
    const int n = getNumRows();

    double *bb = new double[n];
    double *xx = new double[n];

    Vector x(n);

    for( run1 = 0; run1 < n; run1++ )
        bb[run1] = b(run1);

    solver->solve( bb );
    solver->getX ( xx );

    for( run1 = 0; run1 < n; run1++ )
        x(run1) = xx[run1];

    delete[] bb;
    delete[] xx;

    return x;
}

Vector Matrix::solveTransposeSparseLU( const Vector &b ) const{

    ASSERT( solver != 0 );

    int run1;
    const int n = getNumRows();

    double *bb = new double[n];
    double *xx = new double[n];

    Vector x(n);

    for( run1 = 0; run1 < n; run1++ )
        bb[run1] = b(run1);

    solver->solveTranspose( bb );
    solver->getX          ( xx );

    for( run1 = 0; run1 < n; run1++ )
        x(run1) = xx[run1];

    delete[] bb;
    delete[] xx;

    return x;
}



//
// PROTECTED MEMBER FUNCTIONS:
//


void Matrix::householdersReductionToBidiagonalForm( double* A, int nrows,
                                                    int ncols, double* U,
                                                    double* V, double* diagonal,
                                                    double* superdiagonal ) const{

   int i,j,k,ip1;
   double s, s2, si, scale;
   double *pu, *pui, *pv, *pvi;
   double half_norm_squared;

   memcpy(U,A, sizeof(double) * nrows * ncols);

   diagonal[0] = 0.0;
   s = 0.0;
   scale = 0.0;
   for ( i = 0, pui = U, ip1 = 1; i < ncols; pui += ncols, i++, ip1++ ) {
      superdiagonal[i] = scale * s;

      for (j = i, pu = pui, scale = 0.0; j < nrows; j++, pu += ncols)
         scale += fabs( *(pu + i) );
      if (scale > 0.0) {
         for (j = i, pu = pui, s2 = 0.0; j < nrows; j++, pu += ncols) {
            *(pu + i) /= scale;
            s2 += *(pu + i) * *(pu + i);
         }
         s = ( *(pui + i) < 0.0 ) ? sqrt(s2) : -sqrt(s2);
         half_norm_squared = *(pui + i) * s - s2;
         *(pui + i) -= s;
         for (j = ip1; j < ncols; j++) {
            for (k = i, si = 0.0, pu = pui; k < nrows; k++, pu += ncols)
               si += *(pu + i) * *(pu + j);
            si /= half_norm_squared;
            for (k = i, pu = pui; k < nrows; k++, pu += ncols) {
               *(pu + j) += si * *(pu + i);
            }
         }
      }
      for (j = i, pu = pui; j < nrows; j++, pu += ncols) *(pu + i) *= scale;
      diagonal[i] = s * scale;
      s = 0.0;
      scale = 0.0;
      if (i >= nrows || i == (ncols - 1) ) continue;
      for (j = ip1; j < ncols; j++) scale += fabs ( *(pui + j) );
      if ( scale > 0.0 ) {
         for (j = ip1, s2 = 0.0; j < ncols; j++) {
            *(pui + j) /= scale;
            s2 += *(pui + j) * *(pui + j);
         }
         s = ( *(pui + ip1) < 0.0 ) ? sqrt(s2) : -sqrt(s2);
         half_norm_squared = *(pui + ip1) * s - s2;
         *(pui + ip1) -= s;
         for (k = ip1; k < ncols; k++)
            superdiagonal[k] = *(pui + k) / half_norm_squared;
         if ( i < (nrows - 1) ) {
            for (j = ip1, pu = pui + ncols; j < nrows; j++, pu += ncols) {
               for (k = ip1, si = 0.0; k < ncols; k++) 
                  si += *(pui + k) * *(pu + k);
               for (k = ip1; k < ncols; k++) { 
                  *(pu + k) += si * superdiagonal[k];
               }
            }
         }
         for (k = ip1; k < ncols; k++) *(pui + k) *= scale;
      }
   }
   pui = U + ncols * (ncols - 2);
   pvi = V + ncols * (ncols - 1);
   *(pvi + ncols - 1) = 1.0;
   s = superdiagonal[ncols - 1];
   pvi -= ncols;
   for (i = ncols - 2, ip1 = ncols - 1; i >= 0; i--, pui -= ncols,
                                                      pvi -= ncols, ip1-- ) {
      if ( fabs(s) >= EPS ) {
         pv = pvi + ncols;
         for (j = ip1; j < ncols; j++, pv += ncols)
            *(pv + i) = ( *(pui + j) / *(pui + ip1) ) / s;
         for (j = ip1; j < ncols; j++) { 
            si = 0.0;
            for (k = ip1, pv = pvi + ncols; k < ncols; k++, pv += ncols)
               si += *(pui + k) * *(pv + j);
            for (k = ip1, pv = pvi + ncols; k < ncols; k++, pv += ncols)
               *(pv + j) += si * *(pv + i);
         }
      }
      pv = pvi + ncols;
      for ( j = ip1; j < ncols; j++, pv += ncols ) {
         *(pvi + j) = 0.0;
         *(pv + i) = 0.0;
      }
      *(pvi + i) = 1.0;
      s = superdiagonal[i];
   }
   pui = U + ncols * (ncols - 1);
   for (i = ncols - 1, ip1 = ncols; i >= 0; ip1 = i, i--, pui -= ncols ) {
      s = diagonal[i];
      for ( j = ip1; j < ncols; j++) *(pui + j) = 0.0;
      if ( fabs(s) >= EPS ) {
         for (j = ip1; j < ncols; j++) { 
            si = 0.0;
            pu = pui + ncols;
            for (k = ip1; k < nrows; k++, pu += ncols)
               si += *(pu + i) * *(pu + j);
            si = (si / *(pui + i) ) / s;
            for (k = i, pu = pui; k < nrows; k++, pu += ncols)
               *(pu + j) += si * *(pu + i);                  
         }
         for (j = i, pu = pui; j < nrows; j++, pu += ncols){
            *(pu + i) /= s;
         }
      }
      else 
         for (j = i, pu = pui; j < nrows; j++, pu += ncols) *(pu + i) = 0.0;
      *(pui + i) += 1.0;
   }
}


int Matrix::givensReductionToDiagonalForm( int nrows, int ncols,
                                           double* U, double* V,
                                           double* diagonal,
                                           double* superdiagonal ) const{

   double epsilon;
   double c, s;
   double f,g,h;
   double x,y,z;
   double *pu, *pv;
   int i,j,k,m;
   int rotation_test;
   int iteration_count;
   int MAX_ITERATION_COUNT = 3000;

   for (i = 0, x = 0.0; i < ncols; i++) {
      y = fabs(diagonal[i]) + fabs(superdiagonal[i]);
      if ( x < y ) x = y;
   }
   epsilon = x*2.0*EPS;
   for (k = ncols - 1; k >= 0; k--) {
      iteration_count = 0;
      while(1) {
         rotation_test = 1;
         for (m = k; m >= 0; m--) { 
            if (fabs(superdiagonal[m]) <= epsilon) {rotation_test = 0; break;}
            if (fabs(diagonal[m-1]) <= epsilon) break;
         }
         if (rotation_test) {
            c = 0.0;
            s = 1.0;
            for (i = m; i <= k; i++) {  
               f = s * superdiagonal[i];
               superdiagonal[i] *= c;
               if (fabs(f) <= epsilon) break;
               g = diagonal[i];
               h = sqrt(f*f + g*g);
               diagonal[i] = h;
               c = g / h;
               s = -f / h; 
               for (j = 0, pu = U; j < nrows; j++, pu += ncols) { 
                  y = *(pu + m - 1);
                  z = *(pu + i);
                  *(pu + m - 1 ) = y * c + z * s;
                  *(pu + i) = -y * s + z * c;
               }
            }
         }
         z = diagonal[k];
         if (m == k ) {
            if ( z < 0.0 ) {
               diagonal[k] = -z;
               for ( j = 0, pv = V; j < ncols; j++, pv += ncols) 
                  *(pv + k) = - *(pv + k);
            }
            break;
         }
         else {
            if ( iteration_count >= MAX_ITERATION_COUNT ) return -1;
            iteration_count++;
            x = diagonal[m];
            y = diagonal[k-1];
            g = superdiagonal[k-1];
            h = superdiagonal[k];
            f = ( (y - z) * ( y + z ) + (g - h) * (g + h) )/(2.0 * h * y);
            g = sqrt( f * f + 1.0 );
            if ( f < 0.0 ) g = -g;
            f = ( (x - z) * (x + z) + h * (y / (f + g) - h) ) / x;
            c = 1.0;
            s = 1.0;
            for (i = m + 1; i <= k; i++) {
               g = superdiagonal[i];
               y = diagonal[i];
               h = s * g;
               g *= c;
               z = sqrt( f * f + h * h );
               superdiagonal[i-1] = z;
               c = f / z;
               s = h / z;
               f =  x * c + g * s;
               g = -x * s + g * c;
               h = y * s;
               y *= c;
               for (j = 0, pv = V; j < ncols; j++, pv += ncols) {
                  x = *(pv + i - 1);
                  z = *(pv + i);
                  *(pv + i - 1) = x * c + z * s;
                  *(pv + i) = -x * s + z * c;
               }
               z = sqrt( f * f + h * h );
               diagonal[i - 1] = z;
               if ( fabs(z) >= 0.0) {
                  c = f / z;
                  s = h / z;
               } 
               f = c * g + s * y;
               x = -s * g + c * y;
               for (j = 0, pu = U; j < nrows; j++, pu += ncols) {
                  y = *(pu + i - 1);
                  z = *(pu + i);
                  *(pu + i - 1) = c * y + s * z;
                  *(pu + i) = -s * y + c * z;
               }
            }
            superdiagonal[m] = 0.0;
            superdiagonal[k] = f;
            diagonal[k] = x;
         }
      }
   }
   return 0;
}



void Matrix::sortByDecreasingSingularValues( int nrows, int ncols,
                                             double* singular_values,
                                             double* U, double* V ) const{

   int i,j,max_index;
   double temp;
   double *p1, *p2;

   for (i = 0; i < ncols - 1; i++) {
      max_index = i;
      for (j = i + 1; j < ncols; j++)
         if (singular_values[j] > singular_values[max_index] ) 
            max_index = j;
      if (max_index == i) continue;
      temp = singular_values[i];
      singular_values[i] = singular_values[max_index];
      singular_values[max_index] = temp;
      p1 = U + max_index;
      p2 = U + i;
      for (j = 0; j < nrows; j++, p1 += ncols, p2 += ncols) {
         temp = *p1;
         *p1 = *p2;
         *p2 = temp;
      } 
      p1 = V + max_index;
      p2 = V + i;
      for (j = 0; j < ncols; j++, p1 += ncols, p2 += ncols) {
         temp = *p1;
         *p1 = *p2;
         *p2 = temp;
      }
   } 
}


void Matrix::lowerTriangularInverse(double *L, int n) const{

   int i, j, k;
   double *p_i, *p_j, *p_k;
   double sum;

   for (k = 0, p_k = L; k < n; p_k += (n + 1), k++) {
      ASSERT(*p_k >= EPS);
      *p_k = 1.0 / *p_k;
   }
   for (i = 1, p_i = L + n; i < n; i++, p_i += n) {
      for (j = 0, p_j = L; j < i; p_j += n, j++) {
         sum = 0.0;
         for (k = j, p_k = p_j; k < i; k++, p_k += n)
            sum += *(p_i + k) * *(p_k + j);
         *(p_i + j) = - *(p_i + i) * sum;
      }
   }
   return;
}


Matrix operator-(const Matrix &arg){

    uint i,j;

    Matrix tmp(arg.getNumRows(),arg.getNumCols());

    for( i = 0; i < arg.getNumRows(); i++ )
        for( j = 0; j < arg.getNumCols(); j++ )
            tmp(i,j) = -arg(i,j);

    return tmp;
}



CLOSE_NAMESPACE_ACADO

/*
 *    end of file
 */
