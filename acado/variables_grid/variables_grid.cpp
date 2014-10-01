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
 *    \file src/variables_grid/variables_grid.cpp
 *    \author Hans Joachim Ferreau, Boris Houska, Milan Vukov
 *    \date 2008 - 2013
 */


#include <acado/variables_grid/variables_grid.hpp>
#include <acado/variables_grid/matrix_variable.hpp>


BEGIN_NAMESPACE_ACADO



//
// PUBLIC MEMBER FUNCTIONS:
//

VariablesGrid::VariablesGrid( ) : MatrixVariablesGrid( )
{
}


VariablesGrid::VariablesGrid(	uint _dim,
								const Grid& _grid,
								VariableType _type,
								const char** const _names,
								const char** const _units,
								const DVector* const _scaling,
								const DVector* const _lb,
								const DVector* const _ub,
								const BooleanType* const  _autoInit
								) : MatrixVariablesGrid( _dim,1,_grid,_type,_names,_units,_scaling,_lb,_ub,_autoInit )
{
}


VariablesGrid::VariablesGrid(	uint _dim,
								uint _nPoints,
								VariableType _type,
								const char** const _names,
								const char** const _units,
								const DVector* const _scaling,
								const DVector* const _lb,
								const DVector* const _ub,
								const BooleanType* const  _autoInit
								) : MatrixVariablesGrid( _dim,1,_nPoints,_type,_names,_units,_scaling,_lb,_ub,_autoInit )
{
}


VariablesGrid::VariablesGrid(	uint _dim,
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
								) : MatrixVariablesGrid( _dim,1,_firstTime,_lastTime,_nPoints,_type,_names,_units,_scaling,_lb,_ub,_autoInit )
{
}


VariablesGrid::VariablesGrid(	const DVector& arg,
								const Grid& _grid,
								VariableType _type
								) : MatrixVariablesGrid( DMatrix(arg),_grid,_type )
{
}


VariablesGrid::VariablesGrid(	const DMatrix& arg,
								VariableType _type
								) : MatrixVariablesGrid( arg.getNumCols()-1,1,arg.getNumRows(),_type )
{
	uint i,j;

	for( i=0; i<arg.getNumRows(); ++i )
	{
		setTime( i,arg( i,0 ) );

		for( j=1; j<arg.getNumCols(); ++j )
			operator()( i,j-1 ) = arg( i,j );
	}
}


VariablesGrid::VariablesGrid(	const VariablesGrid& rhs
								) : MatrixVariablesGrid( rhs )
{
}


VariablesGrid::VariablesGrid(	const MatrixVariablesGrid& rhs
								) : MatrixVariablesGrid( rhs )
{
	ASSERT( rhs.getNumCols() <= 1 );
}



VariablesGrid::~VariablesGrid( )
{
}


VariablesGrid& VariablesGrid::operator=( const VariablesGrid& rhs )
{
    if ( this != &rhs )
    {
		MatrixVariablesGrid::operator=( rhs );
    }

    return *this;
}


VariablesGrid& VariablesGrid::operator=( const MatrixVariablesGrid& rhs )
{
	ASSERT( rhs.getNumCols() <= 1 );

    if ( this != &rhs )
    {
		MatrixVariablesGrid::operator=( rhs );
    }

    return *this;
}


VariablesGrid& VariablesGrid::operator=( const DMatrix& rhs )
{
	MatrixVariablesGrid::operator=( rhs );
	return *this;
}


VariablesGrid VariablesGrid::operator()(	const uint rowIdx
											) const
{
    ASSERT( values != 0 );
	if ( rowIdx >= getNumRows( ) )
	{
		ACADOERROR( RET_INDEX_OUT_OF_BOUNDS );
		return VariablesGrid();
	}

	Grid tmpGrid;
	getGrid( tmpGrid );

	VariablesGrid rowGrid( 1,tmpGrid,getType( ) );

    for( uint run1 = 0; run1 < getNumPoints(); run1++ )
         rowGrid( run1,0 ) = values[run1]->operator()( rowIdx,0 );

    return rowGrid;
}


VariablesGrid VariablesGrid::operator[](	const uint pointIdx
												) const
{
    ASSERT( values != 0 );
	if ( pointIdx >= getNumPoints( ) )
	{
		ACADOERROR( RET_INVALID_ARGUMENTS );
		return VariablesGrid();
	}

	VariablesGrid pointGrid;
	pointGrid.addMatrix( *(values[pointIdx]),getTime( pointIdx ) );

    return pointGrid;
}


returnValue VariablesGrid::init( )
{
	return MatrixVariablesGrid::init( );
}


returnValue VariablesGrid::init(	uint _dim,
									const Grid& _grid,
									VariableType _type,
									const char** const _names,
									const char** const _units,
									const DVector* const _scaling,
									const DVector* const _lb,
									const DVector* const _ub,
									const BooleanType* const  _autoInit
									)
{
	return MatrixVariablesGrid::init( _dim,1,_grid,_type,_names,_units,_scaling,_lb,_ub,_autoInit );
}


returnValue VariablesGrid::init(	uint _dim,
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
	return MatrixVariablesGrid::init( _dim,1,_nPoints,_type,_names,_units,_scaling,_lb,_ub,_autoInit );
}


returnValue VariablesGrid::init(	uint _dim,
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
	return MatrixVariablesGrid::init( _dim,1,_firstTime,_lastTime,_nPoints,_type,_names,_units,_scaling,_lb,_ub,_autoInit );
}


returnValue VariablesGrid::init(	const DVector& arg,
									const Grid& _grid,
									VariableType _type
									)
{
	return MatrixVariablesGrid::init( DMatrix(arg),_grid,_type );
}



returnValue VariablesGrid::addVector(	const DVector& newVector,
										double newTime
										)
{
	return MatrixVariablesGrid::addMatrix( DMatrix(newVector),newTime );
}


returnValue VariablesGrid::setVector(	uint pointIdx,
										const DVector& _values
										)
{
	if ( pointIdx >= getNumPoints( ) )
		return ACADOERROR( RET_INDEX_OUT_OF_BOUNDS );

	if ( _values.getDim( ) != getNumRows( pointIdx ) )
		return ACADOERROR( RET_VECTOR_DIMENSION_MISMATCH );
	
	for( uint j=0; j<getNumRows( ); ++j )
		operator()( pointIdx,j ) = _values( j );
	
	return SUCCESSFUL_RETURN;
}


returnValue VariablesGrid::setAllVectors(	const DVector& _values
											)
{
	for( uint i = 0; i < getNumPoints(); i++ )
		ACADO_TRY( setVector( i,_values ) );

	return SUCCESSFUL_RETURN;
}


DVector VariablesGrid::getVector(	uint pointIdx
									) const
{
	if ( ( values == 0 ) || ( pointIdx >= getNumPoints() ) )
		return emptyVector;

	return values[pointIdx]->getCol( 0 );
}


DVector VariablesGrid::getFirstVector( ) const
{
	if ( getNumPoints( ) <= 0 )
		return emptyVector;

	return getVector( 0 );
}


DVector VariablesGrid::getLastVector( ) const
{
	if ( getNumPoints( ) <= 0 )
		return emptyVector;

	return getVector( getNumPoints( )-1 );
}



VariablesGrid& VariablesGrid::shiftTimes(	double timeShift
											)
{
	Grid::shiftTimes( timeShift );
	return *this;
}


VariablesGrid& VariablesGrid::shiftBackwards( DVector lastValue )
{

    if( lastValue.isEmpty() == BT_FALSE ){
    
        DMatrix aux( lastValue.getDim(), 1 );
        aux.setCol( 0, lastValue );
    
     	MatrixVariablesGrid::shiftBackwards( aux );
	    return *this;    
    }


	MatrixVariablesGrid::shiftBackwards( );
	return *this;
}



returnValue VariablesGrid::appendTimes(	const VariablesGrid& arg,
										MergeMethod _mergeMethod
										)
{
	// nothing to do for empty grids
	if ( arg.getNumPoints( ) == 0 )
		return SUCCESSFUL_RETURN;

	if ( getNumPoints( ) == 0 )
	{
		*this = arg;
		return SUCCESSFUL_RETURN;
	}

	// consistency check
	if ( acadoIsGreater( arg.getFirstTime( ),getLastTime( ) ) == BT_FALSE )
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
				setVector( getLastIndex(),arg.getFirstVector( ) );
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


returnValue VariablesGrid::appendTimes(	const DMatrix& arg,
										MergeMethod _mergeMethod
										)
{
	VariablesGrid tmp = arg;
	return appendTimes( tmp,_mergeMethod );
}


returnValue VariablesGrid::appendValues( const VariablesGrid& arg )
{
	// setup new grid if current grid is empty
	if ( getNumPoints( ) == 0 )
	{
		*this = arg;
		return SUCCESSFUL_RETURN;
	}

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
returnValue VariablesGrid::merge(	const VariablesGrid& arg,
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
	VariablesGrid mergedGrid;
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




VariablesGrid VariablesGrid::getTimeSubGrid(	uint startIdx,
												uint endIdx
												) const
{
	VariablesGrid newVariablesGrid;

	if ( ( startIdx >= getNumPoints( ) ) || ( endIdx >= getNumPoints( ) ) )
		return newVariablesGrid;

	if ( startIdx > endIdx )
		return newVariablesGrid;

	for( uint i=startIdx; i<=endIdx; ++i )
		newVariablesGrid.addMatrix( *(values[i]),getTime( i ) );

    return newVariablesGrid;
}


VariablesGrid VariablesGrid::getTimeSubGrid(	double startTime,
												double endTime
												) const
{
    uint startIdx = getCeilIndex( startTime );
	uint endIdx   = getFloorIndex( endTime );

	VariablesGrid newVariablesGrid;

	if ( ( isInInterval( startTime ) == BT_FALSE ) || ( isInInterval( endTime ) == BT_FALSE ) )
		return newVariablesGrid;
	
	if ( ( startIdx >= getNumPoints( ) ) || ( endIdx >= getNumPoints( ) ) )
		return newVariablesGrid;

// 	if ( startIdx > endIdx )
// 		return newVariablesGrid;
	
	// add all matrices in interval (constant interpolation)
	if ( ( hasTime( startTime ) == BT_FALSE ) && ( startIdx > 0 ) )
		newVariablesGrid.addMatrix( *(values[ startIdx-1 ]),startTime );
	
	for( uint i=startIdx; i<=endIdx; ++i )
		newVariablesGrid.addMatrix( *(values[i]),getTime( i ) );
	
	if ( hasTime( endTime ) == BT_FALSE )
		newVariablesGrid.addMatrix( *(values[ endIdx ]),endTime );

    return newVariablesGrid;
}


VariablesGrid VariablesGrid::getValuesSubGrid(	uint startIdx,
												uint endIdx
												) const
{
	VariablesGrid newVariablesGrid;

	if ( ( startIdx >= getNumValues( ) ) || ( endIdx >= getNumValues( ) ) )
		return newVariablesGrid;

	if ( startIdx > endIdx )
		return newVariablesGrid;

	for( uint i=0; i<getNumPoints( ); ++i )
		newVariablesGrid.addMatrix( values[i]->getRows( startIdx,endIdx ),getTime( i ) );

    return newVariablesGrid;
}



returnValue VariablesGrid::getSum(	DVector& sum
									) const
{
	sum.setZero();
	
	for( uint i=0; i<getNumPoints( ); ++i )
		sum += getVector( i );

	return SUCCESSFUL_RETURN;
}


returnValue VariablesGrid::getIntegral(	InterpolationMode mode,
										DVector& value
										) const
{
	value.setZero();
	
	switch( mode )
	{
		case IM_CONSTANT:
			for( uint i=0; i<getNumIntervals( ); ++i )
			{
				for( uint j=0; j<getNumValues( ); ++j )
				{
					//value(j) += ( getIntervalLength( i ) / getIntervalLength( ) ) * operator()( i,j );
					value(j) +=  getIntervalLength( i ) * operator()( i,j );
				}
			}
			break;

		case IM_LINEAR:
			for( uint i=0; i<getNumIntervals( ); ++i )
			{
				for( uint j=0; j<getNumValues( ); ++j )
				{
					//value(j) += ( getIntervalLength( i ) / getIntervalLength( ) ) * ( operator()( i,j ) + operator()( i+1,j ) ) / 2.0;
					value(j) += getIntervalLength( i ) * ( operator()( i,j ) + operator()( i+1,j ) ) / 2.0;
				}
			}
			break;
		
		default: 
			return ACADOERROR( RET_NOT_YET_IMPLEMENTED );
	}

	return SUCCESSFUL_RETURN;
}

//
// PROTECTED MEMBER FUNCTIONS:
//

returnValue VariablesGrid::initializeFromBounds( )
{
    uint run1, run2;

    for( run1 = 0; run1 < getNumPoints(); run1++ ){
        for( run2 = 0; run2 < getNumValues(); run2++ ){

            if( fabs( getLowerBound(run1,run2) ) <  0.999*INFTY &&
                fabs( getUpperBound(run1,run2) ) <  0.999*INFTY ){

                operator()(run1,run2) = 0.5*( getLowerBound(run1,run2) + getUpperBound(run1,run2) );

            }

            if( fabs( getLowerBound(run1,run2) ) >= 0.999*INFTY &&
                fabs( getUpperBound(run1,run2) ) <  0.999*INFTY ){

                operator()(run1,run2) = getUpperBound(run1,run2);
            }

            if( fabs( getLowerBound(run1,run2) ) <  0.999*INFTY &&
                fabs( getUpperBound(run1,run2) ) >= 0.999*INFTY ){

                operator()(run1,run2) = getLowerBound(run1,run2);
            }

            if( fabs( getLowerBound(run1,run2) ) >= 0.999*INFTY &&
                fabs( getUpperBound(run1,run2) ) >= 0.999*INFTY ){

                operator()(run1,run2) = 0.0;
            }
        }
    }

	return SUCCESSFUL_RETURN;
}

VariablesGrid::operator DMatrix() const
{
	DMatrix tmp(getNumPoints( ), getNumValues( ) + 1);

	for (uint run1 = 0; run1 < getNumPoints(); ++run1)
	{
		tmp(run1, 0) = getTime(run1);

		for (uint run2 = 0; run2 < getNumValues(); ++run2)
			tmp(run1, 1 + run2) = operator()(run1, run2);
	}

	return tmp;
}


CLOSE_NAMESPACE_ACADO


/*
 *	end of file
 */
