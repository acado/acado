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
 *    \file src/variables_grid/matrix_variables_grid.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 20.08.2008
 */


#include <acado/variables_grid/matrix_variables_grid.hpp>
#include <acado/variables_grid/variables_grid.hpp>


BEGIN_NAMESPACE_ACADO



//
// PUBLIC MEMBER FUNCTIONS:
//

MatrixVariablesGrid::MatrixVariablesGrid( ) : Grid( )
{
	values = 0;
}


MatrixVariablesGrid::MatrixVariablesGrid(	uint _nRows,
											uint _nCols,
											const Grid& _grid,
											VariableType _type,
											const char** const _names,
											const char** const _units,
											const VectorspaceElement* const _scaling,
											const VectorspaceElement* const _lb,
											const VectorspaceElement* const _ub,
											const BooleanType* const  _autoInit
											) : Grid( )
{
	values = 0;
	init( _nRows,_nCols,_grid,_type,_names,_units,_scaling,_lb,_ub,_autoInit );
}


MatrixVariablesGrid::MatrixVariablesGrid(	uint _nRows,
											uint _nCols,
											uint _nPoints,
											VariableType _type,
											const char** const _names,
											const char** const _units,
											const VectorspaceElement* const _scaling,
											const VectorspaceElement* const _lb,
											const VectorspaceElement* const _ub,
											const BooleanType* const  _autoInit
											) : Grid( )
{
	values = 0;
	init( _nRows,_nCols,_nPoints,_type,_names,_units,_scaling,_lb,_ub,_autoInit );
}


MatrixVariablesGrid::MatrixVariablesGrid(	uint _nRows,
											uint _nCols,
											double _firstTime,
											double _lastTime,
											uint _nPoints,
											VariableType _type,
											const char** const _names,
											const char** const _units,
											const VectorspaceElement* const _scaling,
											const VectorspaceElement* const _lb,
											const VectorspaceElement* const _ub,
											const BooleanType* const  _autoInit
											) : Grid( )
{
	values = 0;
	init( _nRows,_nCols,_firstTime,_lastTime,_nPoints,_type,_names,_units,_scaling,_lb,_ub,_autoInit );
}


MatrixVariablesGrid::MatrixVariablesGrid(	const Matrix& arg,
											const Grid& _grid,
											VariableType _type
											) : Grid( )
{
	values = 0;
	init( arg,_grid,_type );
}



MatrixVariablesGrid::MatrixVariablesGrid(	FILE *file
											) : Grid( )
{
	values = 0;
	operator=( file );
}


MatrixVariablesGrid::MatrixVariablesGrid(	const char* filename
											) : Grid( )
{
	values = 0;

	FILE* file = fopen( filename,"r" );
	
	if ( file == 0 )
		ACADOERRORTEXT( RET_FILE_CAN_NOT_BE_OPENED,filename );

	operator=( file );

	/** Closing the file throws a "glibc detected : double free or corruption (!prev)" error */
	/** Someone already closed the file!?*/
	/** Indeed, a subcall to allocateDoublePointerFromFile is made
	"The file is closed at the end of the routine" **/
	//fclose(file); 
}


MatrixVariablesGrid::MatrixVariablesGrid(	const MatrixVariablesGrid& rhs
											) : Grid( rhs )
{
	if ( nPoints > 0 )
		values = (MatrixVariable**) calloc( nPoints,sizeof(MatrixVariable*) );
	else
		values = 0;

	for( uint i=0; i<nPoints; ++i )
		values[i] = new MatrixVariable( *(rhs.values[i]) );
}


MatrixVariablesGrid::~MatrixVariablesGrid( )
{
	clearValues( );
}



MatrixVariablesGrid& MatrixVariablesGrid::operator=(	const MatrixVariablesGrid& rhs
														)
{
    if ( this != &rhs )
    {
		clearValues( );

		Grid::operator=( rhs );

		if ( nPoints > 0 )
			values = (MatrixVariable**) calloc( nPoints,sizeof(MatrixVariable*) );
		else
			values = 0;
	
		for( uint i=0; i<nPoints; ++i )
			values[i] = new MatrixVariable( *(rhs.values[i]) );
    }

    return *this;
}


MatrixVariablesGrid& MatrixVariablesGrid::operator=(	FILE *rhs
														)
{
    int     nR, nC, run1, run2;
    double *x  ;
    returnValue returnvalue;

    x = 0; 
    returnvalue = allocateDoublePointerFromFile(rhs, &x, nR, nC); 

    if ( ( returnvalue == SUCCESSFUL_RETURN ) && ( nR > 0 ) && ( nC > 0 ) )
	{
		init( nC-1,1,nR,getType() );
        for( run1 = 0; run1 < nR; run1++ )
            setTime( x[run1*nC] );

        for( run1 = 0; run1 < nR; run1++ )
            for( run2 = 0; run2 < nC-1; run2++ )
                operator()( run1,run2,0 ) = x[run1*nC+1+run2];
        if( x != 0 ) free(x);
    }
    else{
        if( x != 0 ) free(x);
//         ACADOINFO(returnvalue);
    }

    return *this;
}


MatrixVariablesGrid& MatrixVariablesGrid::operator=(	const Matrix& rhs
														)
{
	init( rhs.getNumCols()-1,1,rhs.getNumRows( ),getType() );

	for( uint i=0; i<nPoints; ++i )
	{
		setTime( rhs( i,0 ) );
	
		for( uint j=0; j<rhs.getNumCols( )-1; ++j )
			operator()( i,j,0 ) = rhs( i,j+1 );
	}

    return *this;
}


returnValue MatrixVariablesGrid::init( )
{
	clearValues( );
	Grid::init( );

	return SUCCESSFUL_RETURN;
}


returnValue MatrixVariablesGrid::init(	uint _nRows,
										uint _nCols,
										const Grid& _grid,
										VariableType _type,
										const char** const _names,
										const char** const _units,
										const VectorspaceElement* const _scaling,
										const VectorspaceElement* const _lb,
										const VectorspaceElement* const _ub,
										const BooleanType* const _autoInit
										)
{
	clearValues( );
	Grid::init( _grid );

	if ( nPoints > 0 )
		values = (MatrixVariable**) calloc( nPoints,sizeof(MatrixVariable*) );
	else
		values = 0;

	return initMatrixVariables( _nRows,_nCols,_type,_names,_units,_scaling,_lb,_ub,_autoInit );
}


returnValue MatrixVariablesGrid::init(	uint _nRows,
										uint _nCols,
										uint _nPoints,
										VariableType _type,
										const char** const _names,
										const char** const _units,
										const VectorspaceElement* const _scaling,
										const VectorspaceElement* const _lb,
										const VectorspaceElement* const _ub,
										const BooleanType* const _autoInit
										)
{
	clearValues( );
	Grid::init( _nPoints );

	if ( nPoints > 0 )
		values = (MatrixVariable**) calloc( nPoints,sizeof(MatrixVariable*) );
	else
		values = 0;

	return initMatrixVariables( _nRows,_nCols,_type,_names,_units,_scaling,_lb,_ub,_autoInit );
}


returnValue MatrixVariablesGrid::init(	uint _nRows,
										uint _nCols,
										double _firstTime,
										double _lastTime,
										uint _nPoints,
										VariableType _type,
										const char** const _names,
										const char** const _units,
										const VectorspaceElement* const _scaling,
										const VectorspaceElement* const _lb,
										const VectorspaceElement* const _ub,
										const BooleanType* const  _autoInit
										)
{
	clearValues( );
	Grid::init( _firstTime,_lastTime,_nPoints );
	
	if ( nPoints > 0 )
		values = (MatrixVariable**) calloc( nPoints,sizeof(MatrixVariable*) );
	else
		values = 0;

	return initMatrixVariables( _nRows,_nCols,_type,_names,_units,_scaling,_lb,_ub,_autoInit );
}


returnValue MatrixVariablesGrid::init(	const Matrix& arg,
										const Grid& _grid,
										VariableType _type
										)
{
	clearValues( );
	Grid::operator=( _grid );

	if ( nPoints > 0 )
		values = (MatrixVariable**) calloc( nPoints,sizeof(MatrixVariable*) );
	else
		values = 0;

	for( uint i=0; i<nPoints; ++i )
		values[i] = new MatrixVariable( arg );

    return SUCCESSFUL_RETURN;
}



returnValue MatrixVariablesGrid::addMatrix(	const Matrix& newMatrix,
											double newTime
											)
{
	return addMatrix( MatrixVariable(newMatrix),newTime );
}



returnValue MatrixVariablesGrid::setMatrix(	uint pointIdx,
											const Matrix& _value
											) const
{
	ASSERT( values != 0 );

	if ( pointIdx >= getNumPoints( ) )
		return ACADOERROR( RET_INDEX_OUT_OF_BOUNDS );

	*(values[pointIdx]) = _value;

	return SUCCESSFUL_RETURN;
}


returnValue MatrixVariablesGrid::setAllMatrices(	const Matrix& _values
													)
{
	for( uint i = 0; i < getNumPoints(); i++ )
		ACADO_TRY( setMatrix( i,_values ) );

	return SUCCESSFUL_RETURN;
}


Matrix MatrixVariablesGrid::getMatrix(	uint pointIdx
										) const
{
	ASSERT( values != 0 );

	if ( pointIdx >= getNumPoints( ) )
		return emptyMatrix;

	return values[pointIdx]->getMatrix( );
}


Matrix MatrixVariablesGrid::getFirstMatrix( ) const
{
	if ( getNumPoints( ) <= 0 )
		return emptyMatrix;

	return getMatrix( 0 );
}


Matrix MatrixVariablesGrid::getLastMatrix( ) const
{
	if ( getNumPoints( ) <= 0 )
		return emptyMatrix;

	return getMatrix( getNumPoints( )-1 );
}



returnValue MatrixVariablesGrid::appendTimes(	const MatrixVariablesGrid& arg,
												MergeMethod _mergeMethod
												)
{
	if ( arg.getNumPoints( ) == 0 )
		return SUCCESSFUL_RETURN;

	if ( acadoIsGreater( getLastTime( ),arg.getFirstTime( ) ) == BT_TRUE )
		return ACADOERROR( RET_INVALID_ARGUMENTS );

	if ( acadoIsEqual( getLastTime( ),arg.getFirstTime( ) ) == BT_FALSE )
	{
		// simply append
		for( uint i=0; i<arg.getNumPoints( ); ++i )
			addMatrix( *(arg.values[i]),arg.getTime( i ) );
	}
	else
	{
		// if last and first time point coincide, merge as specified
		switch ( _mergeMethod )
		{
			case MM_KEEP:
				break;

			case MM_REPLACE:
				setMatrix( getLastIndex(),arg.getFirstMatrix( ) );
				break;

			case MM_DUPLICATE:
				addMatrix( *(arg.values[0]),arg.getTime( 0 ) );
				break;
		}

		// simply append all remaining points
		for( uint i=1; i<arg.getNumPoints( ); ++i )
			addMatrix( *(arg.values[i]),arg.getTime( i ) );
	}

	return SUCCESSFUL_RETURN;
}


returnValue MatrixVariablesGrid::appendValues( const MatrixVariablesGrid& arg )
{
	if ( getNumPoints( ) != arg.getNumPoints( ) )
		return ACADOERROR( RET_INVALID_ARGUMENTS );

	Vector tmp1,tmp2;

	for( uint i=0; i<getNumPoints(); ++i )
	{
		values[i]->appendRows( *(arg.values[i]) );
		values[i]->appendSettings( *(arg.values[i]) );
	}

	return SUCCESSFUL_RETURN;
}


// uses a simple O(n^2) algorithm for sorting
returnValue MatrixVariablesGrid::merge(	const MatrixVariablesGrid& arg,
										MergeMethod _mergeMethod,
										BooleanType keepOverlap
										)
{
	if ( ( keepOverlap == BT_FALSE ) && ( _mergeMethod == MM_DUPLICATE ) )
		return ACADOERROR( RET_INVALID_ARGUMENTS );

	// nothing to do if arg or object itself is empty
	if ( arg.getNumPoints( ) == 0 )
		return SUCCESSFUL_RETURN;

	if ( getNumPoints( ) == 0 )
	{
		*this = arg;
		return SUCCESSFUL_RETURN;
	}

	// use append if grids do not overlap
	if ( acadoIsSmaller( getLastTime( ),arg.getFirstTime( ) ) == BT_TRUE )
		return appendTimes( arg,_mergeMethod );


	// construct merged grid
	MatrixVariablesGrid mergedGrid;
	uint j = 0;
	BooleanType overlapping = BT_FALSE;

	for( uint i=0; i<getNumPoints( ); ++i )
	{
		if ( keepOverlap == BT_FALSE )
			overlapping = arg.isInInterval( getTime(i) );

		// add all grid points of argument grid that are smaller 
		// then current one of original grid
		while ( ( j < arg.getNumPoints( ) ) && 
				( acadoIsStrictlySmaller( arg.getTime( j ),getTime( i ) ) == BT_TRUE ) )
		{
			if ( ( overlapping == BT_FALSE ) ||
				 ( ( overlapping == BT_TRUE ) && ( _mergeMethod == MM_REPLACE ) ) )
			{
				mergedGrid.addMatrix( *(arg.values[j]),arg.getTime( j ) );
			}

			++j;
		}

		// merge current grid points if they are at equal times
		if ( acadoIsEqual( arg.getTime( j ),getTime( i ) ) == BT_TRUE )
		{
			switch ( _mergeMethod )
			{
				case MM_KEEP:
					mergedGrid.addMatrix( *(values[i]),getTime( i ) );
					break;
	
				case MM_REPLACE:
					mergedGrid.addMatrix( *(arg.values[j]),arg.getTime( j ) );
					break;
	
				case MM_DUPLICATE:
					mergedGrid.addMatrix( *(values[i]),getTime( i ) );
					mergedGrid.addMatrix( *(arg.values[j]),arg.getTime( j ) );
					break;
			}
			++j;
		}
		else
		{
			// add current grid point of original grid
			if ( ( overlapping == BT_FALSE ) ||
				 ( ( overlapping == BT_TRUE ) && ( _mergeMethod == MM_KEEP ) ) )
			{
				mergedGrid.addMatrix( *(values[i]),getTime( i ) );//arg.
			}
		}
	}

	// add all remaining grid points of argument grid
	while ( j < arg.getNumPoints( ) )
	{
		if ( acadoIsStrictlyGreater( arg.getTime(j),getLastTime() ) == BT_TRUE )
			mergedGrid.addMatrix( *(arg.values[j]),arg.getTime( j ) );

		++j;
	}

	// merged grid becomes current grid
	*this = mergedGrid;

	return SUCCESSFUL_RETURN;
}



MatrixVariablesGrid MatrixVariablesGrid::getTimeSubGrid(	uint startIdx,
															uint endIdx
															) const
{
	MatrixVariablesGrid newVariablesGrid;

	if ( ( startIdx >= getNumPoints( ) ) || ( endIdx >= getNumPoints( ) ) )
		return newVariablesGrid;

	if ( startIdx > endIdx )
		return newVariablesGrid;

	for( uint i=startIdx; i<=endIdx; ++i )
		newVariablesGrid.addMatrix( *(values[i]),getTime( i ) );

    return newVariablesGrid;
}


MatrixVariablesGrid MatrixVariablesGrid::getValuesSubGrid(	uint startIdx,
															uint endIdx
															) const
{
	MatrixVariablesGrid newVariablesGrid;

	if ( ( startIdx >= getNumValues( ) ) || ( endIdx >= getNumValues( ) ) )
		return newVariablesGrid;

	if ( startIdx > endIdx )
		return newVariablesGrid;

	for( uint i=0; i<getNumPoints( ); ++i )
		newVariablesGrid.addMatrix( values[i]->getRows( startIdx,endIdx ),getTime( i ) );

    return newVariablesGrid;
}



returnValue MatrixVariablesGrid::refineGrid(	const Grid& arg,
												InterpolationMode mode
												)
{
	// nothing to do
	if ( this->isEmpty( ) == BT_TRUE )
		return SUCCESSFUL_RETURN;

	*this = getRefinedGrid( arg,mode );

	if ( this->isEmpty( ) == BT_TRUE )
		return ACADOERROR( RET_INVALID_ARGUMENTS );
	else
		return SUCCESSFUL_RETURN;
}


returnValue MatrixVariablesGrid::coarsenGrid(	const Grid& arg
												)
{
	// nothing to do
	if ( this->isEmpty( ) == BT_TRUE )
		return SUCCESSFUL_RETURN;

	*this = getCoarsenedGrid( arg );

	if ( this->isEmpty( ) == BT_TRUE)
		return ACADOERROR( RET_INVALID_ARGUMENTS );
	else
		return SUCCESSFUL_RETURN;
}


MatrixVariablesGrid MatrixVariablesGrid::getRefinedGrid(	const Grid& arg,
															InterpolationMode mode
															) const
{
	MatrixVariablesGrid tmp;

	// nothing to do
	if ( this->isEmpty( ) == BT_TRUE )
		return tmp;

	int count = -1;

	// checks for equality and superset
	if ( Grid::operator==( arg ) == BT_TRUE )
		return *this;

// 	if ( Grid::operator<=( arg ) == BT_FALSE )
// 		return tmp;

	if ( mode != IM_CONSTANT )
		return tmp;

	for( uint i=0; i<arg.getNumPoints( ); ++i )
	{
		if ( hasTime( arg.getTime( i ) ) == BT_TRUE )
			count = acadoMin( count+1,(int)getNumPoints()-1 );

		if ( count < 0 )
			tmp.addMatrix( *(values[0]),arg.getTime( i ) );
		else
			tmp.addMatrix( *(values[count]),arg.getTime( i ) );
	}

	return tmp;
}


MatrixVariablesGrid MatrixVariablesGrid::getCoarsenedGrid(	const Grid& arg
															) const
{
	MatrixVariablesGrid tmp;
	
	// nothing to do
	if ( this->isEmpty( ) == BT_TRUE )
		return tmp;

	// checks for equality and subset
	if ( Grid::operator==( arg ) == BT_TRUE )
		return *this;

	if ( Grid::operator>=( arg ) == BT_FALSE )
		return tmp;

	for( uint i=0; i<arg.getNumPoints( ); ++i )
	{
		int idx = findLastTime( arg.getTime( i ) );

		if ( idx >= 0 )
			tmp.addMatrix( *(values[idx]),arg.getTime( i ) );
		else
		{
			tmp.init( );
			return tmp;
		}
	}

	return tmp;
}



MatrixVariablesGrid& MatrixVariablesGrid::shiftTimes(	double timeShift
														)
{
	Grid::shiftTimes( timeShift );
	return *this;
}


MatrixVariablesGrid& MatrixVariablesGrid::shiftBackwards( Matrix lastValue )
{
	if ( getNumPoints() < 2 ){
        if( lastValue.isEmpty() == BT_FALSE )
             *(values[getNumIntervals()]) = lastValue;	
		return *this;	
    }

	for( uint i=1; i<getNumPoints( ); ++i )
		*(values[i-1]) = *(values[i]);
		
    if( lastValue.isEmpty() == BT_FALSE )
        *(values[getNumIntervals()]) = lastValue;

	return *this;
}



Vector MatrixVariablesGrid::linearInterpolation( double time ) const
{
    uint idx1 = getFloorIndex( time );
    uint idx2 = getCeilIndex ( time );

	ASSERT( values != 0 );
	ASSERT( idx1 < getNumPoints( ) );
	ASSERT( idx2 < getNumPoints( ) );

    Vector tmp1( values[idx1]->getCol( 0 ) );
    Vector tmp2( values[idx2]->getCol( 0 ) );

    double t1 = getTime( idx1 );
    double t2 = getTime( idx2 );

    if( fabs( t2 - t1 ) < SQRT_EPS ) return tmp1;

    tmp1 *= (t2 - time);
    tmp2 *= (time - t1);

    Vector tmp = tmp1 + tmp2;
    tmp /= (t2 - t1);

    return tmp;
}



returnValue operator<<( FILE *file, MatrixVariablesGrid &arg )
{
	return arg.printToFile(file);
}


returnValue MatrixVariablesGrid::print(	const char* const name,
										const char* const startString,
										const char* const endString,
										uint width,
										uint precision,
										const char* const colSeparator,
										const char* const rowSeparator
										) const
{
	char* string = 0;

	printToString( &string, name,startString,endString,width,precision,colSeparator,rowSeparator );
	acadoPrintf( "%s",string );

	if ( string != 0 )
	  delete[] string;

	return SUCCESSFUL_RETURN;
}


returnValue MatrixVariablesGrid::print(	const char* const name,
										PrintScheme printScheme
										) const
{
	char* string = 0;

	printToString( &string, name,printScheme );
	acadoPrintf( "%s",string );

	if ( string != 0 )
	  delete[] string;

	return SUCCESSFUL_RETURN;
}


returnValue MatrixVariablesGrid::printToFile(	const char* const filename,
												const char* const name,
												const char* const startString,
												const char* const endString,
												uint width,
												uint precision,
												const char* const colSeparator,
												const char* const rowSeparator
												) const
{
	FILE* file = fopen( filename,"w+" );

	if ( file == 0 )
		return ACADOERROR( RET_FILE_CAN_NOT_BE_OPENED );

	printToFile( file, name,startString,endString,width,precision,colSeparator,rowSeparator );
	fclose( file );

	return SUCCESSFUL_RETURN;
}


returnValue MatrixVariablesGrid::printToFile(	FILE* file,
												const char* const name,
												const char* const startString,
												const char* const endString,
												uint width,
												uint precision,
												const char* const colSeparator,
												const char* const rowSeparator
												) const
{
	char* string = 0;

	printToString( &string, name,startString,endString,width,precision,colSeparator,rowSeparator );
	acadoFPrintf( file,"%s",string );

	if ( string != 0 )
	  delete[] string;

	return SUCCESSFUL_RETURN;
}


returnValue MatrixVariablesGrid::printToFile(	const char* const filename,
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
			matFile->write( (const VariablesGrid)*this,name );
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


returnValue MatrixVariablesGrid::printToFile(	FILE* file,
												const char* const name,
												PrintScheme printScheme
												) const
{
	char* string = 0;

	printToString( &string, name,printScheme );
	acadoFPrintf( file,"%s",string );

	if ( string != 0 )
	  delete[] string;

	return SUCCESSFUL_RETURN;
}


returnValue MatrixVariablesGrid::printToString(	char** string,
												const char* const name,
												const char* const startString,
												const char* const endString,
												uint width,
												uint precision,
												const char* const colSeparator,
												const char* const rowSeparator
												) const
{ 
	uint i,k;

	/* determine length of time */
	uint timeLength = width;

	// 0.e-0000
	if ( timeLength < (9 + (uint)precision) )
		timeLength = 9 + precision;

	char* timeString = new char[timeLength];

	char* matrixString = 0;
	
	/* determine length of whole string */
	uint stringLength = determineStringLength( name,startString,endString,
											   width,precision,colSeparator,rowSeparator );

	if ( *string != 0 )
		return ACADOERROR( RET_INVALID_ARGUMENTS );

	*string = new char[stringLength];

	for( i=0; i<stringLength; ++i )
		(*string)[i] = '\0';

	if ( getStringLength(name) > 0 )
	{
		strcat( *string,name );
		strcat( *string," = " );
	}

	if ( getStringLength(startString) > 0 )
		strcat( *string,startString );

	int writtenChars = 0;

	for( k=0; k<getNumPoints( ); ++k )
	{
		// write time
		if ( precision > 0 )
			writtenChars = ::sprintf( timeString,"%*.*e",width,precision,getTime( k ) );
		else
			writtenChars = ::sprintf( timeString,"%*.d",width,(int)getTime( k ) );

		if ( ( writtenChars < 0 ) || ( (uint)writtenChars+1 > timeLength ) )
		{
			delete[] timeString;
			return ACADOERROR( RET_UNKNOWN_BUG );
		}

		strcat( *string,timeString );
		strcat( *string,colSeparator );

		// write matrix string
		//matrixString = new char[ values[k]->determineStringLength( 0,0,0,width,precision,colSeparator,colSeparator ) ];
		values[k]->printToString( &matrixString,0,0,0,width,precision,colSeparator,colSeparator );

		strcat( *string,matrixString );

		if ( matrixString != 0 )
			delete[] matrixString;

		// write separator
		if ( k < getNumPoints( )-1 )
			if ( getStringLength(rowSeparator) > 0 )
				strcat( *string,rowSeparator );
	}

	if ( getStringLength(endString) > 0 )
		strcat( *string,endString );

	delete[] timeString;

	return SUCCESSFUL_RETURN;
}


returnValue MatrixVariablesGrid::printToString(	char** string,
												const char* const name,
												PrintScheme printScheme
												) const
{
	char* startString = 0;
	char* endString = 0;
	uint width = 0;
	uint precision = 0;
	char* colSeparator = 0;
	char* rowSeparator = 0;

	returnValue returnvalue;
	returnvalue = getGlobalStringDefinitions( printScheme,&startString,&endString,
											  width,precision,&colSeparator,&rowSeparator );

	if ( returnvalue == SUCCESSFUL_RETURN )
	{
		returnvalue = printToString( string,name,startString,endString,width,precision,colSeparator,rowSeparator );
	}

	if ( startString != 0 )   delete[] startString;
	if ( endString != 0 )     delete[] endString;
	if ( colSeparator != 0 )  delete[] colSeparator;
	if ( rowSeparator != 0 )  delete[] rowSeparator;

	return returnvalue;
}


uint MatrixVariablesGrid::determineStringLength(	const char* const name,
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
	uint stringLength = 1 
						+ getStringLength(startString) 
						+ getStringLength(endString)
						+ getNumPoints( ) * ( componentLength + getStringLength(colSeparator) + getStringLength(rowSeparator) );

	if ( getStringLength(name) > 0 )
		stringLength += getStringLength(name)+3;

	for( uint k=0; k<getNumPoints(); ++k )
		stringLength += values[k]->determineStringLength( 0,0,0,width,precision,colSeparator,colSeparator );

	return stringLength; 
}


int MatrixVariablesGrid::sprintf( char* buffer )
{
    int returnvalue;
    uint run1, run2, run3;

    double *tmp = new double[getNumPoints()*(getNumRows()+1)];

    if( times == 0 ) return ACADOERROR(RET_MEMBER_NOT_INITIALISED);

    for( run1 = 0; run1 < getNumPoints(); run1++ ){
        tmp[run1*(getNumValues()+1)] = getTime(run1);

        for( run2 = 0; run2 < getNumRows(); run2++ ){
			for( run3 = 0; run3 < getNumCols(); run3++ ){
	            tmp[run1*(getNumValues()+1)+1+run2*getNumCols() + run3] = operator()( run1,run2,run3 );
			}
        }
    }

    returnvalue = writeDoublePointerToString( tmp,getNumPoints(),getNumValues()+1,buffer );
    delete[] tmp;
    return returnvalue;
}



//
// PROTECTED MEMBER FUNCTIONS:
//


returnValue MatrixVariablesGrid::clearValues( )
{
	if ( values != 0 )
	{
		for( uint i=0; i<nPoints; ++i )
		{
			if ( values[i] != 0 )
				delete values[i];
		}

		free( values );
		values = 0;
	}

	return SUCCESSFUL_RETURN;
}


returnValue MatrixVariablesGrid::initMatrixVariables(	uint _nRows,
														uint _nCols,
														VariableType _type,
														const char** const _names,
														const char** const _units,
														const VectorspaceElement* const _scaling,
														const VectorspaceElement* const _lb,
														const VectorspaceElement* const _ub,
														const BooleanType* const _autoInit
														)
{
	VectorspaceElement currentScaling,currentLb,currentUb;

	for( uint i=0; i<nPoints; ++i )
	{
		if ( _scaling != 0 )
			currentScaling = _scaling[i];
		else
			currentScaling.init( );

		if ( _lb != 0 )
			currentLb = _lb[i];
		else
			currentLb.init( );

		if ( _ub != 0 )
			currentUb = _ub[i];
		else
			currentUb.init( );

		values[i] = new MatrixVariable( _nRows,_nCols,_type,_names,_units,currentScaling,currentLb,currentUb );
	}
	
	return SUCCESSFUL_RETURN;
}


returnValue MatrixVariablesGrid::addMatrix(	const MatrixVariable& newMatrix,
											double newTime
											)
{
	if ( ( isInfty( newTime ) == BT_TRUE ) && ( getNumPoints( ) > 0 ) )
		newTime = getLastTime( ) + 1.0;

	if ( Grid::addTime( newTime ) != SUCCESSFUL_RETURN )
		return RET_INVALID_ARGUMENTS;

	values = (MatrixVariable**) realloc( values,nPoints*sizeof(MatrixVariable*) );
	values[nPoints-1] = new MatrixVariable( newMatrix );

	return SUCCESSFUL_RETURN;
}



CLOSE_NAMESPACE_ACADO


/*
 *	end of file
 */
