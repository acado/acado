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
 *    \file src/variables_grid/matrix_variables_grid.cpp
 *    \author Hans Joachim Ferreau, Boris Houska, Milan Vukov
 *    \date 2008 - 2013
 */


#include <acado/variables_grid/matrix_variables_grid.hpp>
#include <acado/variables_grid/matrix_variable.hpp>
#include <acado/variables_grid/variables_grid.hpp>

#include <iomanip>

using namespace std;

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
											const DVector* const _scaling,
											const DVector* const _lb,
											const DVector* const _ub,
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
											const DVector* const _scaling,
											const DVector* const _lb,
											const DVector* const _ub,
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
											const DVector* const _scaling,
											const DVector* const _lb,
											const DVector* const _ub,
											const BooleanType* const  _autoInit
											) : Grid( )
{
	values = 0;
	init( _nRows,_nCols,_firstTime,_lastTime,_nPoints,_type,_names,_units,_scaling,_lb,_ub,_autoInit );
}


MatrixVariablesGrid::MatrixVariablesGrid(	const DMatrix& arg,
											const Grid& _grid,
											VariableType _type
											) : Grid( )
{
	values = 0;
	init( arg,_grid,_type );
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


MatrixVariablesGrid& MatrixVariablesGrid::operator=(	const DMatrix& rhs
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
										const DVector* const _scaling,
										const DVector* const _lb,
										const DVector* const _ub,
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
										const DVector* const _scaling,
										const DVector* const _lb,
										const DVector* const _ub,
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
										const DVector* const _scaling,
										const DVector* const _lb,
										const DVector* const _ub,
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


returnValue MatrixVariablesGrid::init(	const DMatrix& arg,
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



returnValue MatrixVariablesGrid::addMatrix(	const DMatrix& newMatrix,
											double newTime
											)
{
	return addMatrix( MatrixVariable(newMatrix),newTime );
}



returnValue MatrixVariablesGrid::setMatrix(	uint pointIdx,
											const DMatrix& _value
											) const
{
	ASSERT( values != 0 );

	if ( pointIdx >= getNumPoints( ) )
		return ACADOERROR( RET_INDEX_OUT_OF_BOUNDS );

	*(values[pointIdx]) = _value;

	return SUCCESSFUL_RETURN;
}


returnValue MatrixVariablesGrid::setAllMatrices(	const DMatrix& _values
													)
{
	for( uint i = 0; i < getNumPoints(); i++ )
		ACADO_TRY( setMatrix( i,_values ) );

	return SUCCESSFUL_RETURN;
}


DMatrix MatrixVariablesGrid::getMatrix(	uint pointIdx
										) const
{
	ASSERT( values != 0 );

	if ( pointIdx >= getNumPoints( ) )
		return emptyMatrix;

	return values[pointIdx]->getMatrix( );
}


DMatrix MatrixVariablesGrid::getFirstMatrix( ) const
{
	if ( getNumPoints( ) <= 0 )
		return emptyMatrix;

	return getMatrix( 0 );
}


DMatrix MatrixVariablesGrid::getLastMatrix( ) const
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

	DVector tmp1,tmp2;

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


MatrixVariablesGrid& MatrixVariablesGrid::shiftBackwards( DMatrix lastValue )
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



DVector MatrixVariablesGrid::linearInterpolation( double time ) const
{
    uint idx1 = getFloorIndex( time );
    uint idx2 = getCeilIndex ( time );

	ASSERT( values != 0 );
	ASSERT( idx1 < getNumPoints( ) );
	ASSERT( idx2 < getNumPoints( ) );

    DVector tmp1( values[idx1]->getCol( 0 ) );
    DVector tmp2( values[idx2]->getCol( 0 ) );

    double t1 = getTime( idx1 );
    double t2 = getTime( idx2 );

    if( fabs( t2 - t1 ) < SQRT_EPS ) return tmp1;

    tmp1 *= (t2 - time);
    tmp2 *= (time - t1);

    DVector tmp = tmp1 + tmp2;
    tmp /= (t2 - t1);

    return tmp;
}

returnValue MatrixVariablesGrid::print(	std::ostream& stream,
										const char* const name,
										const char* const startString,
										const char* const endString,
										uint width,
										uint precision,
										const char* const colSeparator,
										const char* const rowSeparator
										) const
{
	if (name != NULL && strlen(name) > 0)
		stream << name << " = ";
	if (startString != NULL && strlen(startString) > 0)
		stream << startString;

	if ( precision > 0 )
		stream << setw( width ) << setprecision( precision ) << scientific;
	else
		stream << setw( width );

	for (unsigned k = 0; k < getNumPoints(); ++k)
	{
		if ( precision > 0 )
			stream << getTime( k );
		else
			stream << (int)getTime( k );

		if (colSeparator != NULL && strlen(colSeparator) > 0)
			stream << colSeparator;

		values[k]->print(stream, "", "", "", width, precision, colSeparator, colSeparator);

		if (k < (getNumPoints() - 1) && rowSeparator != NULL && strlen(rowSeparator) > 0)
			stream << rowSeparator;
	}
	if (endString != NULL && strlen(endString) > 0)
		stream << endString;

	return SUCCESSFUL_RETURN;
}

returnValue MatrixVariablesGrid::print(	const char* const filename,
										const char* const name,
										const char* const startString,
										const char* const endString,
										uint width,
										uint precision,
										const char* const colSeparator,
										const char* const rowSeparator
										) const
{
	ofstream stream( filename );
	returnValue status;

	if (stream.is_open() == true)
		status = print(stream, name, startString, endString, width, precision,
				colSeparator, rowSeparator);
	else
		return ACADOERROR( RET_FILE_CAN_NOT_BE_OPENED );

	stream.close();

	return status;
}

returnValue MatrixVariablesGrid::print(	const char* const filename,
										const char* const name,
										PrintScheme printScheme
										) const
{
	ofstream stream(filename);
	returnValue status;

	if (stream.is_open())
		status = print(stream, name, printScheme);
	else
		return ACADOERROR( RET_FILE_CAN_NOT_BE_OPENED );

	stream.close();

	return status;
}

returnValue MatrixVariablesGrid::print(	std::ostream& stream,
										const char* const name,
										PrintScheme printScheme
										) const
{
//	MatFile* matFile = 0;

	switch (printScheme) {
	case PS_MATLAB_BINARY:

		ACADOFATAL( RET_NOT_IMPLEMENTED_YET );

		break;

//		matFile = new MatFile;
//
//		matFile->write(stream, (const VariablesGrid) *this, name);
//
//		delete matFile;
//
//		return SUCCESSFUL_RETURN;

	default:
		char* startString = 0;
		char* endString = 0;
		uint width = 0;
		uint precision = 0;
		char* colSeparator = 0;
		char* rowSeparator = 0;

		returnValue ret = getGlobalStringDefinitions(printScheme, &startString,
				&endString, width, precision, &colSeparator, &rowSeparator);

		returnValue status;
		if (status == SUCCESSFUL_RETURN)
			status = print(stream, name, startString, endString, width, precision,
					colSeparator, rowSeparator);

		if (startString != 0)
			delete[] startString;
		if (endString != 0)
			delete[] endString;
		if (colSeparator != 0)
			delete[] colSeparator;
		if (rowSeparator != 0)
			delete[] rowSeparator;

		return status;
	}

	return SUCCESSFUL_RETURN;
}

returnValue MatrixVariablesGrid::read( std::istream& stream )
{
	vector< vector< double > > data;
	stream >> data;

	if (data.size() == 0)
		return SUCCESSFUL_RETURN;

	// Sanity check
	unsigned nc = data[ 0 ].size();
	unsigned nr = data.size();
	for (unsigned row = 0; row < nr; ++row)
		if (data[ row ].size() != nc)
			return ACADOERROR( RET_INVALID_ARGUMENTS );

	// Data conversions and initialization
	init(nc - 1, 1, nr, getType());

	for (unsigned row = 0; row < nr; ++row)
		setTime( data[ row ][ 0 ] );

	for (unsigned row = 0; row < nr; ++row)
		for (unsigned col = 0; col < nc - 1; ++col)
			operator()(row, col, 0) = data[ row ][col + 1];

	return SUCCESSFUL_RETURN;
}

returnValue MatrixVariablesGrid::read( const char* const filename )
{
	ifstream stream( filename );
	returnValue status;

	if (stream.is_open())
		status = read( stream );
	else
		return ACADOERROR( RET_FILE_CAN_NOT_BE_OPENED );

	stream.close();

	return status;
}

std::ostream& operator<<(std::ostream& stream, const MatrixVariablesGrid& arg)
{
	if (arg.print( stream ) != SUCCESSFUL_RETURN)
		ACADOERRORTEXT(RET_INVALID_ARGUMENTS, "Cannot write to output stream.");

	return stream;
}

std::istream& operator>>(std::istream& stream, MatrixVariablesGrid& arg)
{
	if (arg.read( stream ) != SUCCESSFUL_RETURN)
		ACADOERRORTEXT(RET_INVALID_ARGUMENTS, "Cannot read from input stream.");

	return stream;
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
														const DVector* const _scaling,
														const DVector* const _lb,
														const DVector* const _ub,
														const BooleanType* const _autoInit
														)
{
	DVector currentScaling,currentLb,currentUb;

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

returnValue MatrixVariablesGrid::sprint(	std::ostream& stream
											)
{
	uint run1, run2, run3;

	unsigned tmpSize = getNumPoints() * (getNumRows() + 1);
	double *tmp = new double[tmpSize];

	if( times == 0 ) return ACADOERROR(RET_MEMBER_NOT_INITIALISED);

	for( run1 = 0; run1 < getNumPoints(); run1++ ){
		tmp[run1*(getNumValues()+1)] = getTime(run1);

		for( run2 = 0; run2 < getNumRows(); run2++ ){
			for( run3 = 0; run3 < getNumCols(); run3++ ){
				tmp[run1*(getNumValues()+1)+1+run2*getNumCols() + run3] = operator()( run1,run2,run3 );
			}
		}
	}

	unsigned nRows = getNumPoints();
	unsigned nCols = getNumValues() + 1;
	for (run1 = 0; run1 < nRows; run1++) {
		for (run2 = 0; run2 < nCols; run2++) {
			if (tmp[nCols * run1 + run2] <= ACADO_NAN - 1.0)
				stream << scientific << tmp[nCols * run1 + run2] << TEXT_SEPARATOR;
			else
				stream << NOT_A_NUMBER << TEXT_SEPARATOR;
		}
		stream << LINE_SEPARATOR;
	}
	stream << LINE_SEPARATOR;

	delete[] tmp;

	return SUCCESSFUL_RETURN;
}

#ifdef WIN32
#pragma warning( disable : 4390 )
#endif

//
// PUBLIC MEMBER FUNCTIONS:
//


double& MatrixVariablesGrid::operator()( uint pointIdx, uint rowIdx, uint colIdx )
{
	ASSERT( values != 0 );
	ASSERT( pointIdx < getNumPoints( ) );

    return values[pointIdx]->operator()( rowIdx,colIdx );
}


double MatrixVariablesGrid::operator()( uint pointIdx, uint rowIdx, uint colIdx ) const
{
	ASSERT( values != 0 );
	ASSERT( pointIdx < getNumPoints( ) );

    return values[pointIdx]->operator()( rowIdx,colIdx );
}



MatrixVariablesGrid MatrixVariablesGrid::operator()(	const uint rowIdx
															) const
{
    ASSERT( values != 0 );
	if ( rowIdx >= getNumRows( ) )
	{
		ACADOERROR( RET_INVALID_ARGUMENTS );
		return MatrixVariablesGrid();
	}

	Grid tmpGrid;
	getGrid( tmpGrid );

	MatrixVariablesGrid rowGrid( 1,1,tmpGrid,getType( ) );

    for( uint run1 = 0; run1 < getNumPoints(); run1++ )
         rowGrid( run1,0,0 ) = values[run1]->operator()( rowIdx,0 );

    return rowGrid;
}


MatrixVariablesGrid MatrixVariablesGrid::operator[](	const uint pointIdx
															) const
{
    ASSERT( values != 0 );
	if ( pointIdx >= getNumPoints( ) )
	{
		ACADOERROR( RET_INVALID_ARGUMENTS );
		return MatrixVariablesGrid();
	}

	MatrixVariablesGrid pointGrid;
	pointGrid.addMatrix( *(values[pointIdx]),getTime( pointIdx ) );

    return pointGrid;
}



MatrixVariablesGrid MatrixVariablesGrid::operator+(	const MatrixVariablesGrid& arg
															) const
{
	ASSERT( getNumPoints( ) == arg.getNumPoints( ) );

	MatrixVariablesGrid tmp( *this );

	for( uint i=0; i<getNumPoints( ); ++i )
		*(tmp.values[i]) += *(arg.values[i]);

	return tmp;
}





MatrixVariablesGrid& MatrixVariablesGrid::operator+=(	const MatrixVariablesGrid& arg
																)
{
	ASSERT( getNumPoints( ) == arg.getNumPoints( ) );

	for( uint i=0; i<getNumPoints( ); ++i )
		*(values[i]) += *(arg.values[i]);

	return *this;
}



MatrixVariablesGrid MatrixVariablesGrid::operator-(	const MatrixVariablesGrid& arg
															) const
{
	ASSERT( getNumPoints( ) == arg.getNumPoints( ) );

	MatrixVariablesGrid tmp( *this );

	for( uint i=0; i<getNumPoints( ); ++i )
		*(tmp.values[i]) -= *(arg.values[i]);

	return tmp;
}


MatrixVariablesGrid& MatrixVariablesGrid::operator-=(	const MatrixVariablesGrid& arg
																)
{
	ASSERT( getNumPoints( ) == arg.getNumPoints( ) );

	for( uint i=0; i<getNumPoints( ); ++i )
		*(values[i]) -= *(arg.values[i]);

	return *this;
}



uint MatrixVariablesGrid::getDim( ) const
{
	uint totalDim = 0;

	for( uint i=0; i<getNumPoints( ); ++i )
		totalDim += values[i]->getDim( );

	return totalDim;
}



uint MatrixVariablesGrid::getNumRows( ) const
{
	if ( values == 0 )
		return 0;

	return getNumRows( 0 );
}


uint MatrixVariablesGrid::getNumCols( ) const
{
	if ( values == 0 )
		return 0;

	return getNumCols( 0 );
}


uint MatrixVariablesGrid::getNumValues( ) const
{
	if ( values == 0 )
		return 0;

	return getNumValues( 0 );
}


uint MatrixVariablesGrid::getNumRows(	uint pointIdx
												) const
{
	if( values == 0 )
		return 0;

	ASSERT( pointIdx < getNumPoints( ) );

    return values[pointIdx]->getNumRows( );
}


uint MatrixVariablesGrid::getNumCols(	uint pointIdx
												) const
{
	if( values == 0 )
		return 0;

	ASSERT( pointIdx < getNumPoints( ) );

    return values[pointIdx]->getNumCols( );
}


uint MatrixVariablesGrid::getNumValues(	uint pointIdx
												) const
{
	if( values == 0 )
		return 0;

	ASSERT( pointIdx < getNumPoints( ) );

    return values[pointIdx]->getDim( );
}



VariableType MatrixVariablesGrid::getType( ) const
{
	if ( getNumPoints() == 0 )
		return VT_UNKNOWN;

	return getType( 0 );
}


returnValue MatrixVariablesGrid::setType(	VariableType _type
													)
{
	for( uint i=0; i<getNumPoints( ); ++i )
		setType( i,_type );

	return SUCCESSFUL_RETURN;
}


VariableType MatrixVariablesGrid::getType(	uint pointIdx
													) const
{
	if ( pointIdx >= getNumPoints( ) )
		return VT_UNKNOWN;

	return values[pointIdx]->getType( );
}


returnValue MatrixVariablesGrid::setType(	uint pointIdx,
													VariableType _type
													)
{
	if ( pointIdx >= getNumPoints( ) )
		return ACADOERROR( RET_INDEX_OUT_OF_BOUNDS );

	return values[pointIdx]->setType( _type );
}



returnValue MatrixVariablesGrid::getName(	uint pointIdx,
													uint idx,
													char* const _name
													) const
{
	if( pointIdx >= getNumPoints( ) )
		return ACADOERROR( RET_INDEX_OUT_OF_BOUNDS );

	return values[pointIdx]->getName( idx,_name );
}


returnValue MatrixVariablesGrid::setName(	uint pointIdx,
													uint idx,
													const char* const _name
													)
{
	if( pointIdx >= getNumPoints( ) )
		return ACADOERROR( RET_INDEX_OUT_OF_BOUNDS );

	return values[pointIdx]->setName( idx,_name );
}



returnValue MatrixVariablesGrid::getUnit(	uint pointIdx,
													uint idx,
													char* const _unit
													) const
{
	if( pointIdx >= getNumPoints( ) )
		return ACADOERROR( RET_INDEX_OUT_OF_BOUNDS );

	return values[pointIdx]->getUnit( idx,_unit );
}


returnValue MatrixVariablesGrid::setUnit(	uint pointIdx,
													uint idx,
													const char* const _unit
													)
{
	if( pointIdx >= getNumPoints( ) )
		return ACADOERROR( RET_INDEX_OUT_OF_BOUNDS );

	return values[pointIdx]->setUnit( idx,_unit );
}



DVector MatrixVariablesGrid::getScaling(	uint pointIdx
															) const
{
	if( pointIdx >= getNumPoints( ) )
		return emptyVector;

	return values[pointIdx]->getScaling( );
}


returnValue MatrixVariablesGrid::setScaling(	uint pointIdx,
													const DVector& _scaling
													)
{
    if ( pointIdx >= getNumPoints( ) )
        return ACADOERROR(RET_INDEX_OUT_OF_BOUNDS);

    return values[pointIdx]->setScaling( _scaling );
}


double MatrixVariablesGrid::getScaling(	uint pointIdx,
												uint valueIdx
												) const
{
    if( pointIdx >= getNumPoints( ) )
        return -1.0;

	return values[pointIdx]->getScaling( valueIdx );
}


returnValue MatrixVariablesGrid::setScaling(	uint pointIdx,
													uint valueIdx,
													double _scaling
													)
{
	if( pointIdx >= getNumPoints( ) )
		return ACADOERROR( RET_INDEX_OUT_OF_BOUNDS );

	if( valueIdx >= values[pointIdx]->getDim( ) )
		return ACADOERROR( RET_INDEX_OUT_OF_BOUNDS );

    values[pointIdx]->setScaling( valueIdx,_scaling );
    return SUCCESSFUL_RETURN;
}



DVector MatrixVariablesGrid::getLowerBounds(	uint pointIdx
																) const
{
	if( pointIdx >= getNumPoints( ) )
		return emptyVector;

	return values[pointIdx]->getLowerBounds( );
}


returnValue MatrixVariablesGrid::setLowerBounds(	uint pointIdx,
														const DVector& _lb
														)
{
    if( pointIdx >= nPoints )
        return ACADOERROR(RET_INDEX_OUT_OF_BOUNDS);

    return values[pointIdx]->setLowerBounds( _lb );
}


double MatrixVariablesGrid::getLowerBound(	uint pointIdx,
													uint valueIdx
													) const
{
    if( pointIdx >= getNumPoints( ) )
        return -INFTY;

	return values[pointIdx]->getLowerBound( valueIdx );
}


returnValue MatrixVariablesGrid::setLowerBound(	uint pointIdx,
														uint valueIdx,
														double _lb
														)
{
	if( pointIdx >= getNumPoints( ) )
		return ACADOERROR( RET_INDEX_OUT_OF_BOUNDS );

	if( valueIdx >= values[pointIdx]->getDim( ) )
		return ACADOERROR( RET_INDEX_OUT_OF_BOUNDS );

	values[pointIdx]->setLowerBound( valueIdx,_lb );
    return SUCCESSFUL_RETURN;
}



DVector MatrixVariablesGrid::getUpperBounds(	uint pointIdx
																) const
{
	if( pointIdx >= getNumPoints( ) )
		return emptyVector;

	return values[pointIdx]->getUpperBounds( );
}


returnValue MatrixVariablesGrid::setUpperBounds(	uint pointIdx,
														const DVector& _ub
														)
{
    if( pointIdx >= getNumPoints( ) )
        return ACADOERROR(RET_INDEX_OUT_OF_BOUNDS);

    return values[pointIdx]->setUpperBounds( _ub );
}


double MatrixVariablesGrid::getUpperBound(	uint pointIdx,
													uint valueIdx
													) const
{
    if( pointIdx >= getNumPoints( ) )
        return INFTY;

	return values[pointIdx]->getUpperBound( valueIdx );
}


returnValue MatrixVariablesGrid::setUpperBound(	uint pointIdx,
														uint valueIdx,
														double _ub
														)
{
	if( pointIdx >= getNumPoints( ) )
		return ACADOERROR( RET_INDEX_OUT_OF_BOUNDS );

	if( valueIdx >= values[pointIdx]->getDim( ) )
		return ACADOERROR( RET_INDEX_OUT_OF_BOUNDS );

    values[pointIdx]->setUpperBound( valueIdx,_ub );
    return SUCCESSFUL_RETURN;
}



BooleanType MatrixVariablesGrid::getAutoInit(	uint pointIdx
														) const
{
	if ( pointIdx >= getNumPoints( ) )
	{
		ACADOERROR( RET_INDEX_OUT_OF_BOUNDS );
		return defaultAutoInit;
	}

	return values[pointIdx]->getAutoInit( );
}


returnValue MatrixVariablesGrid::setAutoInit(	uint pointIdx,
														BooleanType _autoInit
														)
{
	if ( pointIdx >= getNumPoints( ) )
		return ACADOERROR( RET_INDEX_OUT_OF_BOUNDS );

	return values[pointIdx]->setAutoInit( _autoInit );
}


returnValue MatrixVariablesGrid::disableAutoInit( )
{
	for( uint i=0; i<getNumPoints( ); ++i )
		values[i]->setAutoInit( BT_FALSE );

	return SUCCESSFUL_RETURN;
}


returnValue MatrixVariablesGrid::enableAutoInit( )
{
	for( uint i=0; i<getNumPoints( ); ++i )
		values[i]->setAutoInit( BT_TRUE );

	return SUCCESSFUL_RETURN;
}



BooleanType MatrixVariablesGrid::hasNames( ) const
{
	for( uint i=0; i<getNumPoints( ); ++i )
	{
		if ( values[i]->hasNames( ) == BT_TRUE )
			return BT_TRUE;
	}

	return BT_FALSE;
}


BooleanType MatrixVariablesGrid::hasUnits( ) const
{
	for( uint i=0; i<getNumPoints( ); ++i )
	{
		if ( values[i]->hasUnits( ) == BT_TRUE )
			return BT_TRUE;
	}

	return BT_FALSE;
}


BooleanType MatrixVariablesGrid::hasScaling( ) const
{
	for( uint i=0; i<getNumPoints( ); ++i )
	{
		if ( values[i]->hasScaling( ) == BT_TRUE )
			return BT_TRUE;
	}

	return BT_FALSE;
}


BooleanType MatrixVariablesGrid::hasLowerBounds( ) const
{
	for( uint i=0; i<getNumPoints( ); ++i )
	{
		if ( values[i]->hasLowerBounds( ) == BT_TRUE )
			return BT_TRUE;
	}

	return BT_FALSE;
}


BooleanType MatrixVariablesGrid::hasUpperBounds( ) const
{
	for( uint i=0; i<getNumPoints( ); ++i )
	{
		if ( values[i]->hasUpperBounds( ) == BT_TRUE )
			return BT_TRUE;
	}

	return BT_FALSE;
}



double MatrixVariablesGrid::getMax( ) const
{
	double maxValue = -INFTY;

	for( uint i=0; i<getNumPoints( ); ++i )
	{
		if ( values[i]->getMax( ) > maxValue )
			maxValue = values[i]->getMax( );
	}

	return maxValue;
}


double MatrixVariablesGrid::getMin( ) const
{
	double minValue = INFTY;

	for( uint i=0; i<getNumPoints( ); ++i )
	{
		if ( values[i]->getMin( ) < minValue )
			minValue = values[i]->getMin( );
	}

	return minValue;
}


double MatrixVariablesGrid::getMean( ) const
{
	double meanValue = 0.0;

	if ( getNumPoints( ) == 0 )
		return meanValue;

	for( uint i=0; i<getNumPoints( ); ++i )
		meanValue += values[i]->getMean( );

	return ( meanValue / (double)getNumPoints( ) );
}



returnValue MatrixVariablesGrid::setZero( )
{
	for( uint i=0; i<getNumPoints( ); ++i )
		values[i]->setZero( );

	return SUCCESSFUL_RETURN;
}


returnValue MatrixVariablesGrid::setAll(	double _value
												)
{
    for( uint i = 0; i<getNumPoints( ); ++i )
		values[i]->setAll( _value );

    return SUCCESSFUL_RETURN;
}



returnValue MatrixVariablesGrid::getGrid(	Grid& _grid
													) const
{
	return _grid.init( getNumPoints(),times );
}


Grid MatrixVariablesGrid::getTimePoints( ) const
{
    Grid tmp;
    getGrid( tmp );
    return tmp;
}


CLOSE_NAMESPACE_ACADO


/*
 *	end of file
 */
