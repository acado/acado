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
 *    \file src/variables_grid/grid.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 26.08.2008
 */


#include <acado/variables_grid/grid.hpp>
#include <iomanip>

using namespace std;

BEGIN_NAMESPACE_ACADO



//
// PUBLIC MEMBER FUNCTIONS:
//

Grid::Grid( )
{
	nPoints = 0;
	times   = 0;
}


Grid::Grid(	uint nPoints_,
			double *times_
			)
{
	times = 0;
	init( nPoints_,times_ );
}


Grid::Grid(	const DVector& times_
			)
{
	times = 0;
	init( times_ );
}


Grid::Grid(	double _firstTime,
			double _lastTime,
			uint _nPoints
			)
{
	times = 0;
	init( _firstTime,_lastTime,_nPoints );
}



Grid::Grid( const Grid& rhs )
{
	nPoints = rhs.nPoints;

	if ( rhs.times != 0 )
	{
		times = (double*) calloc( nPoints,sizeof(double) );

		for( uint i=0; i<nPoints; ++i )
			times[i] = rhs.times[i];
	}
	else
	{
		times = 0;
	}
}


Grid::~Grid( )
{
	if ( times != 0 )
		free( times );
}


returnValue Grid::init( uint _nPoints,
						const double* const _times
						)
{
	nPoints = _nPoints;

	if ( times != 0 )
		free( times );

	if ( nPoints > 0 )
		times = (double*) calloc( nPoints,sizeof(double) );
	else
		times = 0;

	if ( _times != 0 )
	{
		for( uint i=0; i<nPoints; ++i )
			times[i] = _times[i];
	}
	else
	{
		for( uint i=0; i<nPoints; ++i )
			times[i] = -INFTY;
	}

	return SUCCESSFUL_RETURN;
}


returnValue Grid::init(	const DVector& times_
						)
{
	nPoints = times_.getDim();

	if ( times != 0 )
		free( times );

	if ( nPoints > 0 )
		times = (double*) calloc( nPoints,sizeof(double) );
	else
		times = 0;

	for( uint i=0; i<nPoints; ++i )
		times[i] = times_(i);

	return SUCCESSFUL_RETURN;
}


returnValue Grid::init(	double _firstTime,
						double _lastTime,
						uint _nPoints
						)
{
	nPoints = _nPoints;

	if ( times != 0 )
		free( times );

	if ( nPoints > 0 )
		times = (double*) calloc( nPoints,sizeof(double) );
	else
		times = 0;

	return setupEquidistant( _firstTime,_lastTime );
}


returnValue Grid::init(	const Grid& rhs
						)
{
	operator=( rhs );

	return SUCCESSFUL_RETURN;
}



Grid& Grid::operator=( const Grid& rhs )
{
    if ( this != &rhs )
    {
		if ( times != 0 )
			free( times );


		nPoints = rhs.nPoints;

		if ( rhs.times != 0 )
		{
			times = (double*) calloc( nPoints,sizeof(double) );

			for( uint i=0; i<nPoints; ++i )
				times[i] = rhs.times[i];
		}
		else
		{
			times = 0;
		}
	}

	return *this;
}


Grid& Grid::operator&( const Grid& arg )
{
 	merge( arg,MM_KEEP,BT_TRUE );

//     if( times     == 0 ) return Grid::operator=( arg );
//     if( arg.times == 0 ) return *this;
// 
//     uint max_n = nPoints + arg.nPoints;
//     uint count  = 0;
//     uint count1 = 0;
//     uint count2 = 0;
// 
//     double* new_times = (double*) calloc( max_n,sizeof(double) );
//     double tt = getTime(0);
// 
//     if( arg.getTime(0) < tt ){
//         tt = arg.getTime(0);
//     }
// 
//     double tt2 = tt;
// 
//     new_times[count] = tt2;
//     count++;
// 
//     while( count < max_n ){
// 
//         count1 = 0;
//         while( count1 < nPoints ){
// 
//             if( acadoIsStrictlyGreater( getTime(count1) , tt ) == BT_TRUE ){
//                 tt2 = getTime(count1);
//                 break;
//             }
//             count1++;
//         }
// 
//         if( count1 < nPoints ){
//             count2 = 0;
//             while( count2 < arg.nPoints ){
//                 if( acadoIsStrictlyGreater( arg.getTime(count2) , tt  ) == BT_TRUE &&
//                     acadoIsStrictlySmaller( arg.getTime(count2) , tt2 ) == BT_TRUE   ){
//                     tt2 = arg.getTime(count2);
//                     break;
//                 }
//                 count2++;
//             }
//         }
//         else{
//             count2 = 0;
//             while( count2 < arg.nPoints ){
//                 if( acadoIsStrictlyGreater( arg.getTime(count2), tt ) == BT_TRUE ){
//                     tt2 = arg.getTime(count2);
//                     break;
//                 }
//                 count2++;
//             }
//         }
//         if( count1 == nPoints && count2 == arg.nPoints ){
//             break;
//         }
// 
//         new_times[count] = tt2;
//         tt = tt2;
// 
//         count++;
//     }
// 
//     free( times );
// 
//     nPoints = count;
//     times = (double*) calloc( count,sizeof(double) );
//     for( count = 0; count < nPoints; count++ ){
//         times[count] = new_times[count];
//     }
// 
//     free( new_times );

    return *this;
}


returnValue Grid::equalizeGrids(	Grid& arg
									)
{
	if ( *this == arg )
		return SUCCESSFUL_RETURN;

	// construct common grid and assigns it to both grids
	*this & arg;
	arg = *this;

	return SUCCESSFUL_RETURN;
}


returnValue Grid::setTime(	double _time
							)
{
	int idx = findNextIndex( );

	if ( idx < 0 )
		return ACADOERROR( RET_GRIDPOINT_SETUP_FAILED );

	if ( idx > 0 )
	{
		if ( acadoIsStrictlyGreater( times[idx-1] , _time ) == BT_TRUE )
			return ACADOERROR( RET_GRIDPOINT_HAS_INVALID_TIME );
	}

	times[idx] = _time;
	return SUCCESSFUL_RETURN;
}


returnValue Grid::setTime(	uint pointIdx,
							double _time
							)
{
	if ( pointIdx >= getNumPoints( ) )
		return ACADOERROR( RET_INDEX_OUT_OF_BOUNDS );

	times[pointIdx] = _time;

	return SUCCESSFUL_RETURN;
}


returnValue Grid::addTime(	double _time
							)
{
	if ( ( getNumPoints( ) > 0 ) && ( acadoIsGreater( _time,getLastTime() ) == BT_FALSE ) )
	{
		//ASSERT( 1==0 );
		return ACADOERROR( RET_INVALID_ARGUMENTS );
	}

	++nPoints;

	times = (double*) realloc( times,nPoints*sizeof(double) );
	times[nPoints-1] = _time;

	return SUCCESSFUL_RETURN;
}


// uses a simple O(n^2) algorithm for sorting
returnValue Grid::merge(	const Grid& arg,
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


	// construct merged grid
	Grid mergedGrid;
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
				mergedGrid.addTime( arg.getTime( j ) );
			}

			++j;
		}

		// merge current grid points if they are at equal times
		if ( acadoIsEqual( arg.getTime( j ),getTime( i ) ) == BT_TRUE )
		{
			switch ( _mergeMethod )
			{
				case MM_KEEP:
					mergedGrid.addTime( getTime( i ) );
					break;
	
				case MM_REPLACE:
					mergedGrid.addTime( arg.getTime( j ) );
					break;
	
				case MM_DUPLICATE:
					mergedGrid.addTime( getTime( i ) );
					mergedGrid.addTime( arg.getTime( j ) );
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
				mergedGrid.addTime( getTime( i ) );
			}
		}
	}

	// add all remaining grid points of argument grid
	while ( j < arg.getNumPoints( ) )
	{
		if ( acadoIsStrictlyGreater( arg.getTime(j),getLastTime() ) == BT_TRUE )
			mergedGrid.addTime( arg.getTime( j ) );

		++j;
	}

	// merged grid becomes current grid
	*this = mergedGrid;

	return SUCCESSFUL_RETURN;
}



Grid& Grid::shiftTimes(	double timeShift
						)
{
	for( uint i=0; i<nPoints; ++i )
		times[i] += timeShift;

	return *this;
}



returnValue Grid::scaleTimes(	double scaling
								)
{
	if ( acadoIsStrictlyGreater( scaling,0.0 ) == BT_FALSE )
	{
		ACADOWARNING( RET_INVALID_ARGUMENTS );
		scaling = 1.0;
	}

	for( uint i=0; i<nPoints; ++i )
		times[i] *= scaling;

	return SUCCESSFUL_RETURN;
}



returnValue Grid::refineGrid(	uint factor
								)
{
	if ( factor == 0 )
		return ACADOERROR( RET_INVALID_ARGUMENTS );

	/* nothing to do */
	if ( ( factor == 1 ) || ( getNumIntervals( ) == 0 ) )
		return SUCCESSFUL_RETURN;

	/* setup <factor>-times finer grid */
	double* newTimes = (double*) calloc( getNumIntervals( )*factor+1,sizeof(double) );

	for( uint i=0; i<getNumIntervals( ); ++i )
		for( uint j=0; j<factor; ++j )
			newTimes[i*factor + j] = getTime( i ) + ((double) j) / ((double) factor) * getIntervalLength( i );
	newTimes[ getNumIntervals( )*factor ] = getLastTime( );

	/* assign new time array and deallocate old one */
	nPoints = getNumIntervals( )*factor + 1;

	double* tmp = times;
	times = newTimes;
	free( tmp );

	return SUCCESSFUL_RETURN;
}


returnValue Grid::coarsenGrid(	uint factor
								)
{
	if ( factor == 0 )
		return ACADOERROR( RET_INVALID_ARGUMENTS );

	/* nothing to do */
	if ( ( factor == 1 ) || ( getNumIntervals( ) == 0 ) )
		return SUCCESSFUL_RETURN;

	return RET_NOT_YET_IMPLEMENTED;
}


BooleanType Grid::hasTime(	double _time
							) const
{
	if ( findTime( _time ) < 0 )
		return BT_FALSE;
	else
		return BT_TRUE;
}



int Grid::findTime(	double _time,
					uint startIdx
					) const
{
	return findFirstTime( _time,startIdx );
}


int Grid::findFirstTime(	double _time,
							uint startIdx
							) const
{
	if ( times == 0 )
		return -1;

	for( uint i=startIdx; i<getNumPoints( ); ++i )
	{
		if ( acadoIsEqual( times[i] ,_time ) == BT_TRUE )
			return i;

		/* stop here as grid point times are ordered! */
		if ( times[i] > _time )
			return -1;
	}

	/* no grid point with given time found */
	return -1;
}


int Grid::findLastTime(	double _time,
						uint startIdx
						) const
{
	if ( times == 0 )
		return -1;

	for( uint i=startIdx; i<getNumPoints( ); ++i )
	{
		if ( acadoIsEqual( times[i] ,_time ) == BT_TRUE )
		{
			uint j = i;

			while( ( j<getNumPoints( ) ) && ( acadoIsEqual( times[j] , _time ) == BT_TRUE ) )
			{
				++j;
			}

			return j-1;
		}

		/* stop here as grid point times are ordered! */
		if ( times[i] > _time )
			return -1;
	}

	/* no grid point with given time found */
	return -1;
}



uint Grid::getFloorIndex(	double time_
							) const
{
	uint lowerIdx = 0;
	uint upperIdx = getLastIndex( );
	uint idx = 0;

	/* ensure that time lies within range */
	if ( acadoIsGreater( getTime( lowerIdx ) , time_ ) == BT_TRUE )
		return lowerIdx;

	if ( acadoIsSmaller( getTime( upperIdx ) , time_ ) == BT_TRUE )
		return upperIdx;

	/* if so, perform binary search */
	while ( lowerIdx < upperIdx )
	{
		idx = (uint)floor( 0.5*( (double)(upperIdx + lowerIdx) ) );

		if ( isInUpperHalfOpenInterval( idx,time_ ) == BT_TRUE )
			break;

		if ( acadoIsStrictlyGreater( getTime( idx ) , time_ ) == BT_TRUE )
			upperIdx = idx;
		else
			lowerIdx = idx;
	}

    return idx;
}


uint Grid::getCeilIndex ( double time_ ) const
{
	uint lowerIdx = 0;
	uint upperIdx = getLastIndex( );
	uint idx = 0;

	/* ensure that time lies within range */
	if ( acadoIsGreater( getTime( lowerIdx ) , time_ ) == BT_TRUE )
		return lowerIdx;

	if ( acadoIsSmaller( getTime( upperIdx ) , time_ ) == BT_TRUE )
		return upperIdx;

	/* if so, perform binary search */
	while ( lowerIdx < upperIdx )
	{
		idx = (uint)ceil(0.5*( (double)( upperIdx + lowerIdx) ) );

		if ( isInLowerHalfOpenInterval( idx,time_ ) == BT_TRUE )
			break;

		if ( acadoIsGreater( getTime( idx ) ,  time_ ) == BT_TRUE )
			upperIdx = idx;
		else
			lowerIdx = idx;
    }
    return idx;
}


returnValue Grid::getSubGrid(	double tStart,
								double tEnd,
								Grid& _subGrid
								) const
{
	if ( ( isInInterval( tStart ) == BT_FALSE ) ||
		 ( isInInterval( tEnd ) == BT_FALSE ) )
		return ACADOERROR( RET_INVALID_ARGUMENTS );


	// determine number of subpoints
	uint nSubPoints = 0;

	if ( hasTime( tStart ) == BT_FALSE )
		++nSubPoints;

	for( uint i=0; i<getNumPoints( ); ++i )
	{
		if ( acadoIsGreater( getTime( i ) , tStart ) == BT_TRUE  &&
			 acadoIsSmaller( getTime( i ) , tEnd   ) == BT_TRUE     ) ++nSubPoints;
	}

	if ( hasTime( tEnd ) == BT_FALSE )
		++nSubPoints;

	// setup subgrid with subpoints
	_subGrid.init( nSubPoints );

	if ( hasTime( tStart ) == BT_FALSE )
		_subGrid.setTime( tStart );

	for( uint i=0; i<getNumPoints( ); ++i )
	{
		if ( acadoIsGreater( getTime( i ) , tStart ) == BT_TRUE &&
			 acadoIsSmaller( getTime( i ) , tEnd   ) == BT_TRUE     )
			_subGrid.setTime( getTime( i ) );
	}

	if ( hasTime( tEnd ) == BT_FALSE )
		_subGrid.setTime( tEnd );

	return SUCCESSFUL_RETURN;
}


returnValue Grid::print( ) const
{
	for (unsigned t = 0; t < getNumPoints(); t++)
		cout << setprecision( 8 ) << getTime( t ) << "  ";
	cout << endl;
	
	return SUCCESSFUL_RETURN;
}



//
// PROTECTED MEMBER FUNCTIONS:
//

returnValue Grid::setupEquidistant(	double _firstTime,
									double _lastTime
									)
{
	if ( getNumPoints( ) == 0 )
		return SUCCESSFUL_RETURN;

	if ( getNumPoints( ) == 1 )
	{
		times[0] = _firstTime;
		return SUCCESSFUL_RETURN;
	}

	if ( _firstTime > _lastTime )
		return ACADOERROR( RET_INVALID_ARGUMENTS );


	double horizon = _lastTime - _firstTime;

	for( uint i=0; i<nPoints; ++i )
		times[i] = _firstTime + ((double) i) / ((double) getNumPoints( )-1) * horizon;

	return SUCCESSFUL_RETURN;
}


int Grid::findNextIndex( ) const
{
	if ( times == 0 )
		return -1;

	for( uint i=0; i<getNumPoints( ); ++i )
		if ( times[i] < -INFTY+1.0 )
			return i;

	/* no uninitialised grid point found */
	return -1;
}



CLOSE_NAMESPACE_ACADO


/*
 *	end of file
 */
