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
 *    \file src/conic_program/dense_cp.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *
 */

#include <acado/conic_program/dense_cp.hpp>

using namespace std;

BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

DenseCP::DenseCP( ){

    nS = 0;

    B   = 0;
    lbB = 0;
    ubB = 0;

    x   = 0;
    ylb = 0;
    yub = 0;

    ylbA = 0;
    yubA = 0;

    ylbB = 0;
    yubB = 0;
}


DenseCP::DenseCP( const DenseCP& rhs ){

    copy(rhs);
}


DenseCP::~DenseCP( ){

    clean();
}


DenseCP& DenseCP::operator=( const DenseCP& rhs ){

    if ( this != &rhs ){

        clean()  ;
        copy(rhs);
    }
    return *this;
}



void DenseCP::copy( const DenseCP& rhs ){

    uint run1, run2;

    nS = rhs.nS;

    H   = rhs.H;
    g   = rhs.g;

    lb  = rhs.lb;
    ub  = rhs.ub;

    A   = rhs.A;
    lbA = rhs.lbA;
    ubA = rhs.ubA;


    if( rhs.B != 0 ){
        B = (DMatrix**)calloc(nS,sizeof(DMatrix*));
        for( run1 = 0; run1 < nS; run1++ ){
            B[run1] = new DMatrix[ getNV() ];
            for( run2 = 0; run2 < getNV(); run2++ )
                B[run1][run2] = rhs.B[run1][run2];
        }
    }
    else B = 0;


    if( rhs.lbB != 0 ){
        lbB = (DVector*)calloc(nS,sizeof(DVector));
        for( run1 = 0; run1 < nS; run1++ )
            lbB[run1] = rhs.lbB[run1];
    }
    else lbB = 0;

    if( rhs.ubB != 0 ){
        ubB = (DVector*)calloc(nS,sizeof(DVector));
        for( run1 = 0; run1 < nS; run1++ )
            ubB[run1] = rhs.ubB[run1];
    }
    else ubB = 0;


    if( rhs.x != 0 ) x = new DVector(*rhs.x);
    else             x = 0                 ;

    if( rhs.ylb != 0 ) ylb = new DVector(*rhs.ylb);
    else               ylb = 0                   ;

    if( rhs.yub != 0 ) yub = new DVector(*rhs.yub);
    else               yub = 0                   ;

    if( rhs.ylbA != 0 ) ylbA = new DVector(*rhs.ylbA);
    else                ylbA = 0                    ;

    if( rhs.yubA != 0 ) yubA = new DVector(*rhs.yubA);
    else                yubA = 0                    ;

    if( nS > 0 ){

        ylbB = (DVector**)calloc(nS,sizeof(DVector*));
        yubB = (DVector**)calloc(nS,sizeof(DVector*));

        for( run1 = 0; run1 < nS; run1++ ){
            if( rhs.ylbB[run1] != 0 ) ylbB[run1] = new DVector(*rhs.ylbB[run1]);
            else                      ylbB[run1] = 0                          ;

            if( rhs.yubB[run1] != 0 ) yubB[run1] = new DVector(*rhs.yubB[run1]);
            else                      yubB[run1] = 0                          ;
        }
    }
    else{
        ylbB = 0;
        yubB = 0;
    }
}



void DenseCP::clean(){

    uint run1;

    for( run1 = 0; run1 < nS; run1++ ){

        if( B[run1]    != 0 ) delete[]    B[run1];
        if( ylbB[run1] != 0 ) delete   ylbB[run1];
        if( yubB[run1] != 0 ) delete   yubB[run1];
    }

    if( B    != 0 ) free(B)   ;
    if( ylbB != 0 ) free(ylbB);
    if( yubB != 0 ) free(yubB);

    if( lbB != 0 ) free(lbB);
    if( ubB != 0 ) free(ubB);

    if( ylbA != 0 ) delete ylbA;
    if( yubA != 0 ) delete yubA;

    if( ylb != 0 ) delete ylb;
    if( yub != 0 ) delete yub;

    if( x != 0 ) delete x;
}


returnValue DenseCP::init( uint nV_, uint nC_ ){

    H.init(nV_,nV_);
    A.init(nC_,nV_);

    return SUCCESSFUL_RETURN;
}


returnValue DenseCP::setQPsolution( const DVector &x_, const DVector &y_ ){

    uint run1;
    clean();

    ASSERT( x_.getDim() == getNV()           );
    ASSERT( y_.getDim() == getNV() + getNC() );


    // SET THE PRIMAL SOLUTION:
    // ------------------------
    x = new DVector(x_);


    // SET THE DUAL SOLUTION FOR THE BOUNDS:
    // -------------------------------------
    ylb = new DVector( getNV() );
    yub = new DVector( getNV() );

    for( run1 = 0; run1 < getNV(); run1++ ){
        if( fabs(x_(run1)-lb(run1)) <= BOUNDTOL ){
            ylb->operator()(run1) = y_(run1);
            yub->operator()(run1) = 0.0     ;
        }
        else{
            ylb->operator()(run1) = 0.0     ;
            yub->operator()(run1) = y_(run1);
        }
    }


    // SET THE DUAL SOLUTION FOR THE CONSTRAINTS:
    // ------------------------------------------
    DVector tmp = A*x_;
    ylbA = new DVector( getNC() );
    yubA = new DVector( getNC() );

    for( run1 = 0; run1 < getNC(); run1++ ){
        if( fabs(tmp(run1)-lbA(run1)) <= BOUNDTOL ){
            ylbA->operator()(run1) = y_(getNV()+run1);
            yubA->operator()(run1) = 0.0             ;
        }
        else{
            ylbA->operator()(run1) = 0.0             ;
            yubA->operator()(run1) = y_(getNV()+run1);
        }
    }


    return SUCCESSFUL_RETURN;
}


DVector DenseCP::getMergedDualSolution( ) const
{
	DVector dualSolution( getNV()+getNC() );

	uint i;
	for( i=0; i<getNV(); ++i )
	{
		if ( acadoIsGreater( fabs( (*ylb)(i) ),1e-10 ) == BT_TRUE )
			dualSolution(i) = (*ylb)(i);
		else
			dualSolution(i) = (*yub)(i);
	}

	for( i=0; i<getNC(); ++i )
	{
		if ( acadoIsGreater( fabs( (*ylbA)(i) ),1e-10 ) == BT_TRUE )
			dualSolution( getNV()+i ) = (*ylbA)(i);
		else
			dualSolution( getNV()+i ) = (*yubA)(i);
	}

	return dualSolution;
}



returnValue DenseCP::print(	const char* const name,
							const char* const startString,
							const char* const endString,
							uint width,
							uint precision,
							const char* const colSeparator,
							const char* const rowSeparator
							) const
{
	H.print  (cout, "H",  startString,endString,width,precision,colSeparator,rowSeparator );
	g.print  (cout, "g",  startString,endString,width,precision,colSeparator,rowSeparator );
	lb.print (cout, "lb", startString,endString,width,precision,colSeparator,rowSeparator );
	ub.print (cout, "ub", startString,endString,width,precision,colSeparator,rowSeparator );
	A.print  (cout, "A",  startString,endString,width,precision,colSeparator,rowSeparator );
	lbA.print(cout, "lbA",startString,endString,width,precision,colSeparator,rowSeparator );
	ubA.print(cout, "ubA",startString,endString,width,precision,colSeparator,rowSeparator );
	
	return SUCCESSFUL_RETURN;
}


returnValue DenseCP::print(	const char* const name,
							PrintScheme printScheme
							) const
{
	H.print  (cout, "H",  printScheme );
	g.print  (cout, "g",  printScheme );
	lb.print (cout, "lb", printScheme );
	ub.print (cout, "ub", printScheme );
	A.print  (cout, "A",  printScheme );
	lbA.print(cout, "lbA",printScheme );
	ubA.print(cout, "ubA",printScheme );
	
	return SUCCESSFUL_RETURN;
}


returnValue DenseCP::printToFile(	const char* const filename,
									const char* const name,
									const char* const startString,
									const char* const endString,
									uint width,
									uint precision,
									const char* const colSeparator,
									const char* const rowSeparator
									) const
{
	H.print  ( filename,"H",  startString,endString,width,precision,colSeparator,rowSeparator );
	g.print  ( filename,"g",  startString,endString,width,precision,colSeparator,rowSeparator );
	lb.print ( filename,"lb", startString,endString,width,precision,colSeparator,rowSeparator );
	ub.print ( filename,"ub", startString,endString,width,precision,colSeparator,rowSeparator );
	A.print  ( filename,"A",  startString,endString,width,precision,colSeparator,rowSeparator );
	lbA.print( filename,"lbA",startString,endString,width,precision,colSeparator,rowSeparator );
	ubA.print( filename,"ubA",startString,endString,width,precision,colSeparator,rowSeparator );
	
	return SUCCESSFUL_RETURN;
}


returnValue DenseCP::printToFile(	std::ostream& stream,
									const char* const name,
									const char* const startString,
									const char* const endString,
									uint width,
									uint precision,
									const char* const colSeparator,
									const char* const rowSeparator
									) const
{
	H.print  ( stream,"H",  startString,endString,width,precision,colSeparator,rowSeparator );
	g.print  ( stream,"g",  startString,endString,width,precision,colSeparator,rowSeparator );
	lb.print ( stream,"lb", startString,endString,width,precision,colSeparator,rowSeparator );
	ub.print ( stream,"ub", startString,endString,width,precision,colSeparator,rowSeparator );
	A.print  ( stream,"A",  startString,endString,width,precision,colSeparator,rowSeparator );
	lbA.print( stream,"lbA",startString,endString,width,precision,colSeparator,rowSeparator );
	ubA.print( stream,"ubA",startString,endString,width,precision,colSeparator,rowSeparator );
	
	return SUCCESSFUL_RETURN;
}


returnValue DenseCP::printToFile(	const char* const filename,
									const char* const name,
									PrintScheme printScheme
									) const
{
	H.print  ( filename,"H",  printScheme );
	g.print  ( filename,"g",  printScheme );
	lb.print ( filename,"lb", printScheme );
	ub.print ( filename,"ub", printScheme );
	A.print  ( filename,"A",  printScheme );
	lbA.print( filename,"lbA",printScheme );
	ubA.print( filename,"ubA",printScheme );
	
	return SUCCESSFUL_RETURN;
}


returnValue DenseCP::printToFile(	std::ostream& stream,
									const char* const name,
									PrintScheme printScheme
									) const
{
	H.print  ( stream,"H",  printScheme );
	g.print  ( stream,"g",  printScheme );
	lb.print ( stream,"lb", printScheme );
	ub.print ( stream,"ub", printScheme );
	A.print  ( stream,"A",  printScheme );
	lbA.print( stream,"lbA",printScheme );
	ubA.print( stream,"ubA",printScheme );
	
	return SUCCESSFUL_RETURN;
}



returnValue DenseCP::printSolution(	const char* const name,
									const char* const startString,
									const char* const endString,
									uint width,
									uint precision,
									const char* const colSeparator,
									const char* const rowSeparator
									) const
{
	if ( x != 0 )
		x->print(cout, "x", startString,endString,width,precision,colSeparator,rowSeparator );

	if ( ylb != 0 )
		ylb->print(cout,  "ylb",startString,endString,width,precision,colSeparator,rowSeparator );

	if ( yub != 0 )
		yub->print(cout,  "yub", startString,endString,width,precision,colSeparator,rowSeparator );
	
	if ( ylbA != 0 )
		ylbA->print(cout,  "ylbA", startString,endString,width,precision,colSeparator,rowSeparator );

	if ( yubA != 0 )
		yubA->print(cout,  "yubA", startString,endString,width,precision,colSeparator,rowSeparator );

	return SUCCESSFUL_RETURN;
}


returnValue DenseCP::printSolution(	const char* const name,
									PrintScheme printScheme
									) const
{
	if ( x != 0 )
		x->print(cout,  "x", printScheme );

	if ( ylb != 0 )
		ylb->print(cout,  "ylb",printScheme );

	if ( yub != 0 )
		yub->print(cout,  "yub", printScheme );
	
	if ( ylbA != 0 )
		ylbA->print(cout,  "ylbA", printScheme );

	if ( yubA != 0 )
		yubA->print(cout,  "yubA", printScheme );

	return SUCCESSFUL_RETURN;
}


returnValue DenseCP::printSolutionToFile(	const char* const filename,
											const char* const name,
											const char* const startString,
											const char* const endString,
											uint width,
											uint precision,
											const char* const colSeparator,
											const char* const rowSeparator
											) const
{
	if ( x != 0 )
		x->print( filename, "x", startString,endString,width,precision,colSeparator,rowSeparator );

	if ( ylb != 0 )
		ylb->print( filename, "ylb",startString,endString,width,precision,colSeparator,rowSeparator );

	if ( yub != 0 )
		yub->print( filename, "yub", startString,endString,width,precision,colSeparator,rowSeparator );
	
	if ( ylbA != 0 )
		ylbA->print( filename, "ylbA", startString,endString,width,precision,colSeparator,rowSeparator );

	if ( yubA != 0 )
		yubA->print( filename, "yubA", startString,endString,width,precision,colSeparator,rowSeparator );

	return SUCCESSFUL_RETURN;
}


returnValue DenseCP::printSolutionToFile(	std::ostream& stream,
											const char* const name,
											const char* const startString,
											const char* const endString,
											uint width,
											uint precision,
											const char* const colSeparator,
											const char* const rowSeparator
											) const
{
	if ( x != 0 )
		x->print( stream, "x", startString,endString,width,precision,colSeparator,rowSeparator );

	if ( ylb != 0 )
		ylb->print( stream, "ylb",startString,endString,width,precision,colSeparator,rowSeparator );

	if ( yub != 0 )
		yub->print( stream, "yub", startString,endString,width,precision,colSeparator,rowSeparator );
	
	if ( ylbA != 0 )
		ylbA->print( stream, "ylbA", startString,endString,width,precision,colSeparator,rowSeparator );

	if ( yubA != 0 )
		yubA->print( stream, "yubA", startString,endString,width,precision,colSeparator,rowSeparator );

	return SUCCESSFUL_RETURN;
}


returnValue DenseCP::printSolutionToFile(	const char* const filename,
											const char* const name,
											PrintScheme printScheme
											) const
{
	if ( x != 0 )
		x->print( filename, "x", printScheme );

	if ( ylb != 0 )
		ylb->print( filename, "ylb",printScheme );

	if ( yub != 0 )
		yub->print( filename, "yub", printScheme );
	
	if ( ylbA != 0 )
		ylbA->print( filename, "ylbA", printScheme );

	if ( yubA != 0 )
		yubA->print( filename, "yubA", printScheme );

	return SUCCESSFUL_RETURN;
}


returnValue DenseCP::printSolutionToFile(	std::ostream& stream,
											const char* const name,
											PrintScheme printScheme
											) const
{
	if ( x != 0 )
		x->print( stream, "x", printScheme );

	if ( ylb != 0 )
		ylb->print( stream, "ylb",printScheme );

	if ( yub != 0 )
		yub->print( stream, "yub", printScheme );
	
	if ( ylbA != 0 )
		ylbA->print( stream, "ylbA", printScheme );

	if ( yubA != 0 )
		yubA->print( stream, "yubA", printScheme );

	return SUCCESSFUL_RETURN;
}



CLOSE_NAMESPACE_ACADO

// end of file.
