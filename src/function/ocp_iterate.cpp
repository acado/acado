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
 *    \file src/conic_program/ocp_iterate.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *
 */

#include <acado/function/ocp_iterate.hpp>


BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

OCPiterate::OCPiterate( )
{
	x  = 0;
	xa = 0;
	p  = 0;
	u  = 0;
	w  = 0;
	
	init( 0,0,0,0,0 );
}


OCPiterate::OCPiterate(	const VariablesGrid* const _x,
						const VariablesGrid* const _xa,
						const VariablesGrid* const _p,
						const VariablesGrid* const _u,
						const VariablesGrid* const _w
						)
{

    x = 0; xa = 0; p = 0; u = 0; w = 0;

	init( _x,_xa,_p,_u,_w );
}


OCPiterate::OCPiterate( const OCPiterate& rhs ){

    copy(rhs);
}


OCPiterate::~OCPiterate( ){

    clear( );
}


OCPiterate& OCPiterate::operator=( const OCPiterate& rhs ){

    if ( this != &rhs ){

        clear( ) ;
        copy(rhs);
    }
    return *this;
}


returnValue OCPiterate::allocateAll( )
{
    if ( x == 0 )
		x = new VariablesGrid;

    if ( xa == 0 )
		xa = new VariablesGrid;

    if ( p == 0 )
		p = new VariablesGrid;

	if ( u == 0 )
		u = new VariablesGrid;
	
    if ( w == 0 )
		w = new VariablesGrid;

	return SUCCESSFUL_RETURN;
}

returnValue OCPiterate::init(	const VariablesGrid* const _x,
								const VariablesGrid* const _xa,
								const VariablesGrid* const _p,
								const VariablesGrid* const _u,
								const VariablesGrid* const _w
								)
{
	clear( );

	if ( _x != 0 )
		x = new VariablesGrid( *_x );

	if ( _xa != 0 )
		xa = new VariablesGrid( *_xa );

	if ( _p != 0 )
		p = new VariablesGrid( *_p );

	if ( _u != 0 )
		u = new VariablesGrid( *_u );

	if ( _w != 0 )
		w = new VariablesGrid( *_w );
	
	return SUCCESSFUL_RETURN;
}


returnValue OCPiterate::clear( ){

    if( x  != 0 ){ delete x ; x  = 0; }
    if( xa != 0 ){ delete xa; xa = 0; }
    if( p  != 0 ){ delete p ; p  = 0; }
    if( u  != 0 ){ delete u ; u  = 0; }
    if( w  != 0 ){ delete w ; w  = 0; }

	inSimulationMode = BT_FALSE;

    return SUCCESSFUL_RETURN;
}



uint OCPiterate::getNumPoints( ) const
{
	int N = 0;

	if( x  != 0 ) N = acadoMax( N, x ->getNumPoints( ) );
	if( xa != 0 ) N = acadoMax( N, xa->getNumPoints( ) );
	if( p  != 0 ) N = acadoMax( N, p ->getNumPoints( ) );
	if( u  != 0 ) N = acadoMax( N, u ->getNumPoints( ) );
	if( w  != 0 ) N = acadoMax( N, w ->getNumPoints( ) );

	return N;
}


returnValue OCPiterate::print( ) const{

    if( x  != 0 ) x ->print( "iter.x " );
    if( xa != 0 ) xa->print( "iter.xa" );
    if( p  != 0 ) p ->print( "iter.p " );
    if( u  != 0 ) u ->print( "iter.u " );
    if( w  != 0 ) w ->print( "iter.w " );

    return SUCCESSFUL_RETURN;
}



Grid OCPiterate::getUnionGrid() const{

    Grid tmp, unionGrid;

    if( x  != 0 ){ x ->getGrid(tmp);  unionGrid = unionGrid & tmp; }
    if( xa != 0 ){ xa->getGrid(tmp);  unionGrid = unionGrid & tmp; }
    if( p  != 0 ){ p ->getGrid(tmp);  unionGrid = unionGrid & tmp; }
    if( u  != 0 ){ u ->getGrid(tmp);  unionGrid = unionGrid & tmp; }
    if( w  != 0 ){ w ->getGrid(tmp);  unionGrid = unionGrid & tmp; }

    return unionGrid;
}



BooleanType OCPiterate::areGridsConsistent( )
{
	double startTime = 0.0, endTime = 0.0;
	BooleanType timeDefined = BT_FALSE;

	if ( ( x != 0 ) && ( x->isEmpty( ) == BT_FALSE ) )
	{
		startTime = x->getFirstTime( );
		endTime   = x->getLastTime( );
		timeDefined = BT_TRUE;
	}

	if ( ( xa != 0 ) && ( xa->isEmpty( ) == BT_FALSE ) )
	{
		if ( timeDefined == BT_FALSE )
		{
			startTime = xa->getFirstTime( );
			endTime   = xa->getLastTime( );
			timeDefined = BT_TRUE;
		}
		else
		{
			if ( ( acadoIsEqual( startTime,xa->getFirstTime( ) ) == BT_FALSE ) ||
				 ( acadoIsEqual( endTime,  xa->getLastTime(  ) ) == BT_FALSE ) )
				return BT_FALSE;
		}
	}

	if ( ( p != 0 ) && ( p->isEmpty( ) == BT_FALSE ) )
	{
		if ( timeDefined == BT_FALSE )
		{
			startTime = p->getFirstTime( );
			endTime   = p->getLastTime( );
			timeDefined = BT_TRUE;
		}
		else
		{
			if ( ( acadoIsEqual( startTime,p->getFirstTime( ) ) == BT_FALSE ) ||
				 ( acadoIsEqual( endTime,  p->getLastTime(  ) ) == BT_FALSE ) )
				return BT_FALSE;
		}
	}
	
	if ( ( u != 0 ) && ( u->isEmpty( ) == BT_FALSE ) )
	{
		if ( timeDefined == BT_FALSE )
		{
			startTime = u->getFirstTime( );
			endTime   = u->getLastTime( );
			timeDefined = BT_TRUE;
		}
		else
		{
			if ( ( acadoIsEqual( startTime,u->getFirstTime( ) ) == BT_FALSE ) ||
				 ( acadoIsEqual( endTime,  u->getLastTime(  ) ) == BT_FALSE ) )
				return BT_FALSE;
		}
	}
	
	if ( ( w != 0 ) && ( w->isEmpty( ) == BT_FALSE ) )
	{
		if ( timeDefined == BT_FALSE )
		{
			startTime = w->getFirstTime( );
			endTime   = w->getLastTime( );
			timeDefined = BT_TRUE;
		}
		else
		{
			if ( ( acadoIsEqual( startTime,w->getFirstTime( ) ) == BT_FALSE ) ||
				 ( acadoIsEqual( endTime,  w->getLastTime(  ) ) == BT_FALSE ) )
				return BT_FALSE;
		}
	}
	
	return BT_TRUE;
}



returnValue OCPiterate::getInitialData( DVector &x_, DVector &xa_, DVector &p_,
                                        DVector &u_, DVector &w_               ) const{

    if( x  != 0 ){ x_  = x ->getVector(0); } else { x_  = emptyVector; }
    if( xa != 0 ){ xa_ = xa->getVector(0); } else { xa_ = emptyVector; }
    if( p  != 0 ){ p_  = p ->getVector(0); } else { p_  = emptyVector; }
    if( u  != 0 ){ u_  = u ->getVector(0); } else { u_  = emptyVector; }
    if( w  != 0 ){ w_  = w ->getVector(0); } else { w_  = emptyVector; }

    return SUCCESSFUL_RETURN;
}


void OCPiterate::update( double t, VariablesGrid &z1, DVector &z2 ) const{

    if( z1.hasTime( t ) == BT_TRUE ){

        uint idx = z1.getFloorIndex( t );
        if( z1.getAutoInit(idx) == BT_FALSE ){ z2 = z1.getVector(idx); }
        else{

            DVector safeGuard = z2;
            safeGuard.setAll( BOUNDTOL );

            if( z1.hasUpperBounds() == BT_TRUE )
                if( z2 >= z1.getUpperBounds(idx) ) z2 = (DVector)z1.getUpperBounds(idx) - safeGuard;

            if( z1.hasLowerBounds() == BT_TRUE )
                if( z2 <= z1.getLowerBounds(idx) ) z2 = (DVector)z1.getLowerBounds(idx) + safeGuard;

            z1.setVector( idx, z2 );
        }
    }
}


returnValue OCPiterate::updateData(  double t  , DVector &x_, DVector &xa_,
                                     DVector &p_, DVector &u_, DVector &w_   ){

    if( x  != 0 ) update( t, *x , x_  );
    if( xa != 0 ) update( t, *xa, xa_ );
    if( p  != 0 ) update( t, *p , p_  );
    if( u  != 0 ) update( t, *u , u_  );
    if( w  != 0 ) update( t, *w , w_  );

    return SUCCESSFUL_RETURN;
}



returnValue OCPiterate::applyStep(	const BlockMatrix& bm,
									double alpha
									)
{
    uint run1, run2;
    DMatrix tmp;

    if( getNX() > 0 ){
        for( run1 = 0; run1 < getNumPoints(); run1++ ){
            bm.getSubBlock( run1, 0, tmp, getNX(), 1 );
            for( run2 = 0; run2 < getNX(); run2++ )
                x->operator()(run1,run2) += alpha*tmp(run2,0);
        }
    }

    if( getNXA() > 0 ){
        for( run1 = 0; run1 < getNumPoints(); run1++ ){
            bm.getSubBlock( getNumPoints()+run1, 0, tmp, getNXA(), 1 );
            for( run2 = 0; run2 < getNXA(); run2++ )
                xa->operator()(run1,run2) += alpha*tmp(run2,0);
        }
    }

	if( getNP() > 0 ){
		for( run1 = 0; run1 < getNumPoints(); run1++ ){
			bm.getSubBlock( 2*getNumPoints(), 0, tmp, getNP(), 1 );
			for( run2 = 0; run2 < getNP(); run2++ )
				p->operator()(run1,run2) += alpha*tmp(run2,0);
		}
    }

	if( getNU() > 0 ){
		for( run1 = 0; run1 < getNumPoints(); run1++ )
		{
			bm.getSubBlock( 3*getNumPoints()+run1, 0, tmp, getNU(), 1 );
			for( run2 = 0; run2 < getNU(); run2++ )
				u->operator()(run1,run2) += alpha*tmp(run2,0);
		}
	}    

	if( getNW() > 0 ){
        for( run1 = 0; run1 < getNumPoints(); run1++ ){
            bm.getSubBlock( 4*getNumPoints()+run1, 0, tmp, getNW(), 1 );
            for( run2 = 0; run2 < getNW(); run2++ )
                w->operator()(run1,run2) += alpha*tmp(run2,0);
        }
    }

	return SUCCESSFUL_RETURN;
}



returnValue OCPiterate::enableSimulationMode(){

    if( x  != 0 ) x ->enableAutoInit ();
    if( xa != 0 ) xa->enableAutoInit ();
    if( p  != 0 ) p ->disableAutoInit();
    if( u  != 0 ) u ->disableAutoInit();
    if( w  != 0 ) w ->disableAutoInit();

	inSimulationMode = BT_TRUE;
	
    return SUCCESSFUL_RETURN;
}



returnValue OCPiterate::shift(	double timeShift,
							    DVector  lastX,
							    DVector  lastXA,
								DVector  lastP,
								DVector  lastU,
								DVector  lastW ){
								
	if ( acadoIsNegative( timeShift ) == BT_TRUE )
		ACADOERROR( RET_INVALID_ARGUMENTS );

	double subintervalLength = x->getIntervalLength( 0 );

	if ( acadoIsEqual( timeShift,subintervalLength ) == BT_FALSE )
		return ACADOERROR( RET_NOT_YET_IMPLEMENTED );
	
// 	return ACADOERROR( RET_NOT_YET_IMPLEMENTED );
	
	if ( x != 0 )
		x->shiftBackwards( lastX );
	if ( xa != 0 )
		xa->shiftBackwards( lastXA );
	
	if ( p != 0 )
		p->shiftBackwards( lastP );
	
	if ( u != 0 )
		u->shiftBackwards( lastU );
	
	if ( w != 0 )
		w->shiftBackwards( lastW );
	
	return SUCCESSFUL_RETURN;
}



//
// PROTECTED MEMBER FUNCTIONS:
//

void OCPiterate::copy( const OCPiterate& rhs ){

    if( rhs.x  != 0 ) x  = new VariablesGrid(*rhs.x );
    else              x  = 0                         ;
    if( rhs.xa != 0 ) xa = new VariablesGrid(*rhs.xa);
    else              xa = 0                         ;
    if( rhs.p  != 0 ) p  = new VariablesGrid(*rhs.p );
    else              p  = 0                         ;
    if( rhs.u  != 0 ) u  = new VariablesGrid(*rhs.u );
    else              u  = 0                         ;
    if( rhs.w  != 0 ) w  = new VariablesGrid(*rhs.w );
    else              w  = 0                         ;

	inSimulationMode = rhs.inSimulationMode;
}



CLOSE_NAMESPACE_ACADO

// end of file.
