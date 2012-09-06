/*
 *	This file is part of qpOASES.
 *
 *	qpOASES -- An Implementation of the Online Active Set Strategy.
 *	Copyright (C) 2007-2011 by Hans Joachim Ferreau, Andreas Potschka,
 *	Christian Kirches et al. All rights reserved.
 *
 *	qpOASES is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU Lesser General Public
 *	License as published by the Free Software Foundation; either
 *	version 2.1 of the License, or (at your option) any later version.
 *
 *	qpOASES is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *	See the GNU Lesser General Public License for more details.
 *
 *	You should have received a copy of the GNU Lesser General Public
 *	License along with qpOASES; if not, write to the Free Software
 *	Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */


/**
 *	\file interfaces/scilab/qpOASESroutines.cpp
 *	\author Holger Diedam, Hans Joachim Ferreau
 *	\version 3.0beta
 *	\date 2007-2011
 *
 *	Interface that enables to call qpOASES from scilab
 *  (C++ file to provide an interface between the files that
 *  have to be compiled with gcc and the qpOASES library).
 *
 */


#include <iostream>

#include <qpOASES/SQProblem.hpp>


using namespace qpOASES;

/* global pointers to qpOASES objects */
QProblem*  qp  = 0;
QProblemB* qpb = 0;
SQProblem* sqp = 0;


extern "C"
{
	void qpoases(	real_t* H, real_t* g, real_t* A, real_t* lb, real_t* ub, real_t* lbA, real_t* ubA,
					int *nV, int* nC, int* nWSR,
					real_t* x, real_t* obj, int* status, int* nWSRout, real_t* y
					);

	void init(		real_t* H, real_t* g, real_t* A, real_t* lb, real_t* ub, real_t* lbA, real_t* ubA,
					int* nV, int* nC, int* nWSR,
					real_t* x, real_t* obj, int* status, int* nWSRout, real_t* y
					);
	void initSB(	real_t* H, real_t* g, real_t* lb, real_t* ub,
					int* nV, int* nWSR,
					real_t* x, real_t* obj, int* status, int* nWSRout, real_t* y
					);
	void initVM(	real_t* H, real_t* g, real_t* A, real_t* lb, real_t* ub, real_t* lbA, real_t* ubA,
					int* nV, int* nC, int* nWSR,
					real_t* x, real_t* obj, int* status, int* nWSRout, real_t* y
					);

	void hotstart(		real_t* g, real_t* lb, real_t* ub, real_t* lbA, real_t* ubA,
						int* nWSR,
						real_t* x, real_t* obj, int* status, int* nWSRout, real_t* y
						);
	void hotstartSB(	real_t* g, real_t* lb, real_t* ub,
						int* nWSR,
						real_t* x, real_t* obj, int* status, int* nWSRout, real_t* y
						);
	void hotstartVM(	real_t* H, real_t* g, real_t* A, real_t* lb, real_t* ub, real_t* lbA, real_t* ubA,
						int* nWSR,
						real_t* x, real_t* obj, int* status, int* nWSRout, real_t* y
						);

	void cleanupp( );
	void cleanupSB( );
	void cleanupVM( );
}



/*
 *	t r a n s f o r m A
 */
void transformA( real_t* A, int nV, int nC )
{
	int i, j;

	real_t* A_tmp = new real_t[nC*nV];

	for( i=0; i<nV*nC; ++i )
		A_tmp[i] = A[i];

	for( i=0; i<nC; ++i )
		for( j=0; j<nV; ++j )
			A[i*nV + j] = A_tmp[j*nC + i];

	delete[] A_tmp;

	return;
}


/*
 *	g e t S t a t u s
 */
int getStatus( returnValue returnvalue )
{
	switch ( returnvalue )
	{
		case SUCCESSFUL_RETURN:
			return 0;

		case RET_MAX_NWSR_REACHED:
			return 1;

		case RET_INIT_FAILED_INFEASIBILITY:
		case RET_HOTSTART_STOPPED_INFEASIBILITY:
			return -2;
		
		case RET_INIT_FAILED_UNBOUNDEDNESS:
		case RET_HOTSTART_STOPPED_UNBOUNDEDNESS:
			return -3;
			
		default:
			return -1;
	}
}


/*
 *	q p o a s e s
 */
void qpoases(	real_t* H, real_t* g, real_t* A, real_t* lb, real_t* ub, real_t* lbA, real_t* ubA,
				int *nV, int* nC, int* nWSR,
				real_t* x, real_t* obj, int* status, int* nWSRout, real_t* y
				)
{
	/* transform A into C style matrix */
	transformA( A, *nV,*nC );

	/* setup and solve initial QP */
	QProblem single_qp( *nV,*nC );
	single_qp.setPrintLevel( PL_LOW );
	returnValue returnvalue = single_qp.init( H,g,A,lb,ub,lbA,ubA, *nWSR,0 );

	/* assign lhs arguments */
	single_qp.getPrimalSolution( x );
	*obj = single_qp.getObjVal( );
	*status = getStatus( returnvalue );
	*nWSRout = *nWSR;
	single_qp.getDualSolution( y );

	return;
}


/*
 *	i n i t
 */
void init(	real_t* H, real_t* g, real_t* A, real_t* lb, real_t* ub, real_t* lbA, real_t* ubA,
			int* nV, int* nC, int* nWSR,
			real_t* x, real_t* obj, int* status, int* nWSRout, real_t* y
			)
{
	cleanupp( );

	/* transform A into C style matrix */
	transformA( A, *nV,*nC );

	/* setup and solve initial QP */
	qp = new QProblem( *nV,*nC );
	qp->setPrintLevel( PL_LOW );
	returnValue returnvalue = qp->init( H,g,A,lb,ub,lbA,ubA, *nWSR,0 );

	/* assign lhs arguments */
	qp->getPrimalSolution( x );
	*obj = qp->getObjVal( );
	*status = getStatus( returnvalue );
	*nWSRout = *nWSR;
	qp->getDualSolution( y );

	return;
}


/*
 *	i n i t S B
 */
void initSB(	real_t* H, real_t* g, real_t* lb, real_t* ub,
				int* nV, int* nWSR,
				real_t* x, real_t* obj, int* status, int* nWSRout, real_t* y
				)
{
	cleanupSB( );

	/* setup and solve initial QP */
	qpb = new QProblemB( *nV );
	qpb->setPrintLevel( PL_LOW );
	returnValue returnvalue = qpb->init( H,g,lb,ub, *nWSR,0 );

	/* assign lhs arguments */
	qpb->getPrimalSolution( x );
	*obj = qpb->getObjVal( );
	*status = getStatus( returnvalue );
	*nWSRout = *nWSR;
	qpb->getDualSolution( y );

	return;
}


/*
 *	i n i t V M
 */
void initVM(	real_t* H, real_t* g, real_t* A, real_t* lb, real_t* ub, real_t* lbA, real_t* ubA,
				int* nV, int* nC, int* nWSR,
				real_t* x, real_t* obj, int* status, int* nWSRout, real_t* y
				)
{
	cleanupVM( );

	/* transform A into C style matrix */
	transformA( A, *nV,*nC );

	/* setup and solve initial QP */
	sqp = new SQProblem( *nV,*nC );
	sqp->setPrintLevel( PL_LOW );
	returnValue returnvalue = sqp->init( H,g,A,lb,ub,lbA,ubA, *nWSR,0 );

	/* assign lhs arguments */
	sqp->getPrimalSolution( x );
	*obj = sqp->getObjVal( );
	*status = getStatus( returnvalue );
	*nWSRout = *nWSR;
	sqp->getDualSolution( y );

	return;
}


/*
 *	h o t s t a r t
 */
void hotstart(	real_t* g, real_t* lb, real_t* ub, real_t* lbA, real_t* ubA,
				int* nWSR,
				real_t* x, real_t* obj, int* status, int* nWSRout, real_t* y
				)
{
	/* has QP been initialised? */
	if ( qp == 0 )
	{
		*status = -1;
		return;
	}

	/* solve QP */
	returnValue returnvalue = qp->hotstart( g,lb,ub,lbA,ubA, *nWSR,0 );

	/* assign lhs arguments */
	qp->getPrimalSolution( x );
	*obj = qp->getObjVal( );
	*status = getStatus( returnvalue );
	*nWSRout = *nWSR;
	qp->getDualSolution( y );

	return;
}


/*
 *	h o t s t a r t S B
 */
void hotstartSB(	real_t* g, real_t* lb, real_t* ub,
					int* nWSR,
					real_t* x, real_t* obj, int* status, int* nWSRout, real_t* y
					)
{
	/* has QP been initialised? */
	if ( qpb == 0 )
	{
		*status = -1;
		return;
	}

	/* solve QP */
	returnValue returnvalue = qpb->hotstart( g,lb,ub, *nWSR,0 );

	/* assign lhs arguments */
	qpb->getPrimalSolution( x );
	*obj = qpb->getObjVal( );
	*status = getStatus( returnvalue );
	*nWSRout = *nWSR;
	qpb->getDualSolution( y );

	return;
}


/*
 *	h o t s t a r t V M
 */
void hotstartVM(	real_t* H, real_t* g, real_t* A, real_t* lb, real_t* ub, real_t* lbA, real_t* ubA,
					int* nWSR,
					real_t* x, real_t* obj, int* status, int* nWSRout, real_t* y
					)
{
	/* has QP been initialised? */
	if ( sqp == 0 )
	{
		*status = -1;
		return;
	}

	/* transform A into C style matrix */
	transformA( A, sqp->getNV( ),sqp->getNC( ) );

	/* solve QP */
	returnValue returnvalue = sqp->hotstart( H,g,A,lb,ub,lbA,ubA, *nWSR,0 );

	/* assign lhs arguments */
	sqp->getPrimalSolution( x );
	*obj = sqp->getObjVal( );
	*status = getStatus( returnvalue );
	*nWSRout = *nWSR;
	sqp->getDualSolution( y );

	return;
}


/*
 *	c l e a n u p p
 */
void cleanupp( )
{
	/* Remark: A function cleanup already exists! */
	if ( qp != 0 )
	{
		delete qp;
		qp = 0;
	}

	return;
}


/*
 *	c l e a n u p S B
 */
void cleanupSB( )
{
	if ( qpb != 0 )
	{
		delete qpb;
		qpb = 0;
	}

	return;
}


/*
 *	c l e a n u p V M
 */
void cleanupVM( )
{
	if ( sqp != 0 )
	{
		delete sqp;
		sqp = 0;
	}

	return;
}


/*
 *	end of file
 */
