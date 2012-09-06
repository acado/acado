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
 *	\file interfaces/scilab/qpOASESinterface.c
 *	\author Holger Diedam, Hans Joachim Ferreau
 *	\version 3.0beta
 *	\date 2007-2011
 *
 *	Interface that enables to call qpOASES from scilab.
 *  (Please excuse a lot of copy and paste...)
 *
 */


#include <stdio.h>
#include <string.h>

#include <scilab/stack-c.h>
#include <scilab/machine.h>
#include <scilab/sciprint.h>
/*#include "os_specific/link.h"*/


extern int int_init(   char* fname );
extern int int_initSB( char* fname );
extern int int_initVM( char* fname );

extern int int_solve(   char* fname );
extern int int_solveSB( char* fname );
extern int int_solveVM( char* fname );

extern int int_cleanup(   char* fname );
extern int int_cleanupSB( char* fname );
extern int int_cleanupVM( char* fname );

extern int int_qpOASES( char* fname );


typedef int (*gate_function) ( char* );
extern int sci_gateway( char* name, gate_function f );
extern int C2F(qpOASESgateway)();


/* forward declaration of C++ routines */
void qpoases(	double* H, double* g, double* A, double* lb, double* ub, double* lbA, double* ubA,
				int *nV, int* nC, int* nWSR,
				double* x, double* obj, int* status, int* nWSRout, double* y
				);

void init(		double* H, double* g, double* A, double* lb, double* ub, double* lbA, double* ubA,
				int *nV, int* nC, int* nWSR,
				double* x, double* obj, int* status, int* nWSRout, double* y
				);
void initSB(	double* H, double* g, double* lb, double* ub,
				int *nV, int* nWSR,
				double* x, double* obj, int* status, int* nWSRout, double* y
				);
void initVM(	double* H, double* g, double* A, double* lb, double* ub, double* lbA, double* ubA,
				int *nV, int* nC, int* nWSR,
				double* x, double* obj, int* status, int* nWSRout, double* y
				);

void hotstart(		double* g, double* lb, double* ub, double* lbA, double* ubA,
					int* nWSR,
					double* x, double* obj, int* status, int* nWSRout, double* y
					);
void hotstartSB(	double* g, double* lb, double* ub,
					int* nWSR,
					double* x, double* obj, int* status, int* nWSRout, double* y
					);
void hotstartVM(	double* H, double* g, double* A, double* lb, double* ub, double* lbA, double* ubA,
					int* nWSR,
					double* x, double* obj, int* status, int* nWSRout, double* y
					);

void cleanupp( );
void cleanupSB( );
void cleanupVM( );


/* global variables containing dimensions of matrices
 * (also used to check whether qpOASES object were initialised) */
int qp_rowsH = -1;
int qp_rowsA = -1;
int qpb_rowsH = -1;
int sqp_rowsH = -1;
int sqp_rowsA = -1;


/*
 *	i n t _ q p O A S E S
 */
int int_qpOASES( char* fname )
{
	static int H, H_rows, H_cols;
	static int A, A_rows, A_cols;
	static int g, g_rows, g_cols;
	static int lb, lb_rows, lb_cols;
	static int ub, ub_rows, ub_cols;
	static int lbA, lbA_rows, lbA_cols;
	static int ubA, ubA_rows, ubA_cols;
	static int nWSR, nWSR_rows, nWSR_cols;

	static int obj, x, y, status, nWSRout;


	static int minlhs = 1, maxlhs = 5, minrhs = 8, maxrhs = 8, one = 1, y_size;

	CheckRhs( minrhs,maxrhs );
	CheckLhs( minlhs,maxlhs );


	/* check dimensions */
	GetRhsVar( 1,"d", &H_rows,&H_cols,&H );
	if ( ( H_rows != H_cols ) || ( H_rows < 1 ) )
	{
		sciprint( "ERROR (qpOASES): Dimension mismatch!\n" );
		Error( 999 );
		return 0;
	}

	GetRhsVar( 2,"d", &g_rows,&g_cols,&g );
	if ( !( ( ( g_rows == H_rows ) && ( g_cols == 1 ) ) || ( ( g_rows == 1 ) && ( g_cols == H_rows ) ) ) )
	{
		sciprint( "ERROR (qpOASES): Dimension mismatch!\n" );
		Error( 999 );
		return 0;
	}

	GetRhsVar( 3,"d", &A_rows,&A_cols,&A );
	if ( ( A_cols != H_rows ) || ( A_rows < 1 ) )
	{
		sciprint( "ERROR (qpOASES): Dimension mismatch!\n" );
		Error( 999 );
		return 0;
	}

	GetRhsVar( 4,"d", &lb_rows,&lb_cols,&lb);
	if ( !( ( ( lb_rows == H_rows ) && ( lb_cols == 1 ) ) || ( ( lb_rows == 0 ) && ( lb_cols == 0 ) ) ) )
	{
		sciprint( "ERROR (qpOASES): Dimension mismatch!\n" );
		Error( 999 );
		return 0;
	}

	GetRhsVar( 5,"d", &ub_rows,&ub_cols,&ub);
	if ( !( ( ( ub_rows == H_rows ) && ( ub_cols == 1 ) ) || ( ( ub_rows == 0 ) && ( ub_cols == 0 ) ) ) )
	{
		sciprint( "ERROR (qpOASES): Dimension mismatch!\n" );
		Error( 999 );
		return 0;
	}

	GetRhsVar( 6,"d", &lbA_rows,&lbA_cols,&lbA);
	if ( ( lbA_rows != A_rows ) || ( lbA_cols != 1 ) )
	{
		sciprint( "ERROR (qpOASES): Dimension mismatch!\n" );
		Error( 999 );
		return 0;
	}

	GetRhsVar( 7,"d", &ubA_rows,&ubA_cols,&ubA);
	if ( ( ubA_rows != A_rows ) || ( ubA_cols != 1 ) )
	{
		sciprint( "ERROR (qpOASES): Dimension mismatch!\n" );
		Error( 999 );
		return 0;
	}

	GetRhsVar( 8,"i", &nWSR_rows,&nWSR_cols,&nWSR);
	if ( ( nWSR_rows != nWSR_cols ) || ( nWSR_cols != 1 ) )
	{
		sciprint( "ERROR (qpOASES): Dimension mismatch!\n" );
		Error( 999 );
		return 0;
	}


	y_size = H_rows + A_rows;

	CreateVar(  9,"d", &H_rows,&one,&x );
	CreateVar( 10,"d", &one,&one,&obj );
	CreateVar( 11,"i", &one,&one,&status );
	CreateVar( 12,"i", &one,&one,&nWSRout );
	CreateVar( 13,"d", &y_size,&one,&y );


	/* call interfaced qpOASES routines with appropriate arguments */
	if ( ( lb_rows != 0 ) && ( ub_rows != 0 ) )
	{
		qpoases(	stk(H),stk(g),stk(A),stk(lb),stk(ub),stk(lbA),stk(ubA),
					&H_rows,&A_rows,istk(nWSR),
					stk(x),stk(obj),istk(status),istk(nWSRout),stk(y)
					);
	}
	else if ( ( lb_rows == 0 ) && ( ub_rows != 0 ) )
	{
		qpoases(	stk(H),stk(g),stk(A),0,stk(ub),stk(lbA),stk(ubA),
					&H_rows,&A_rows,istk(nWSR),
					stk(x),stk(obj),istk(status),istk(nWSRout),stk(y)
					);
	}
	else if ( ( lb_rows != 0 ) && ( ub_rows == 0 ) )
	{
		qpoases(	stk(H),stk(g),stk(A),stk(lb),0,stk(lbA),stk(ubA),
					&H_rows,&A_rows,istk(nWSR),
					stk(x),stk(obj),istk(status),istk(nWSRout),stk(y)
					);
	}
	else
	{
		qpoases(	stk(H),stk(g),stk(A),0,0,stk(lbA),stk(ubA),
					&H_rows,&A_rows,istk(nWSR),
					stk(x),stk(obj),istk(status),istk(nWSRout),stk(y)
					);
	}

	LhsVar(1) = 9;
	LhsVar(2) = 10;
	LhsVar(3) = 11;
	LhsVar(4) = 12;
	LhsVar(5) = 13;

	return 0;
}


/*
 *	i n t _ i n i t
 */
int int_init( char* fname )
{
	static int H, H_rows, H_cols;
	static int A, A_rows, A_cols;
	static int g, g_rows, g_cols;
	static int lb, lb_rows, lb_cols;
	static int ub, ub_rows, ub_cols;
	static int lbA, lbA_rows, lbA_cols;
	static int ubA, ubA_rows, ubA_cols;
	static int nWSR, nWSR_rows, nWSR_cols;

	static int obj, x, y, status, nWSRout;


	static int minlhs = 1, maxlhs = 5, minrhs = 8, maxrhs = 8, one = 1, y_size;

	CheckRhs( minrhs,maxrhs );
	CheckLhs( minlhs,maxlhs );


	/* check dimensions */
	GetRhsVar( 1,"d", &H_rows,&H_cols,&H );
	if ( ( H_rows != H_cols ) || ( H_rows < 1 ) )
	{
		sciprint( "ERROR (qpOASES): Dimension mismatch!\n" );
		Error( 999 );
		return 0;
	}

	GetRhsVar( 2,"d", &g_rows,&g_cols,&g );
	if ( !( ( ( g_rows == H_rows ) && ( g_cols == 1 ) ) || ( ( g_rows == 1 ) && ( g_cols == H_rows ) ) ) )
	{
		sciprint( "ERROR (qpOASES): Dimension mismatch!\n" );
		Error( 999 );
		return 0;
	}

	GetRhsVar( 3,"d", &A_rows,&A_cols,&A );
	if ( ( A_cols != H_rows ) || ( A_rows < 1 ) )
	{
		sciprint( "ERROR (qpOASES): Dimension mismatch!\n" );
		Error( 999 );
		return 0;
	}

	GetRhsVar( 4,"d", &lb_rows,&lb_cols,&lb);
	if ( !( ( ( lb_rows == H_rows ) && ( lb_cols == 1 ) ) || ( ( lb_rows == 0 ) && ( lb_cols == 0 ) ) ) )
	{
		sciprint( "ERROR (qpOASES): Dimension mismatch!\n" );
		Error( 999 );
		return 0;
	}

	GetRhsVar( 5,"d", &ub_rows,&ub_cols,&ub);
	if ( !( ( ( ub_rows == H_rows ) && ( ub_cols == 1 ) ) || ( ( ub_rows == 0 ) && ( ub_cols == 0 ) ) ) )
	{
		sciprint( "ERROR (qpOASES): Dimension mismatch!\n" );
		Error( 999 );
		return 0;
	}

	GetRhsVar( 6,"d", &lbA_rows,&lbA_cols,&lbA);
	if ( ( lbA_rows != A_rows ) || ( lbA_cols != 1 ) )
	{
		sciprint( "ERROR (qpOASES): Dimension mismatch!\n" );
		Error( 999 );
		return 0;
	}

	GetRhsVar( 7,"d", &ubA_rows,&ubA_cols,&ubA);
	if ( ( ubA_rows != A_rows ) || ( ubA_cols != 1 ) )
	{
		sciprint( "ERROR (qpOASES): Dimension mismatch!\n" );
		Error( 999 );
		return 0;
	}

	GetRhsVar( 8,"i", &nWSR_rows,&nWSR_cols,&nWSR);
	if ( ( nWSR_rows != nWSR_cols ) || ( nWSR_cols != 1 ) )
	{
		sciprint( "ERROR (qpOASES): Dimension mismatch!\n" );
		Error( 999 );
		return 0;
	}


	y_size = H_rows + A_rows;

	CreateVar(  9,"d", &H_rows,&one,&x );
	CreateVar( 10,"d", &one,&one,&obj );
	CreateVar( 11,"i", &one,&one,&status );
	CreateVar( 12,"i", &one,&one,&nWSRout );
	CreateVar( 13,"d", &y_size,&one,&y );


	qp_rowsH = H_rows;
	qp_rowsA = A_rows;


	/* call interfaced qpOASES routines with appropriate arguments */
	if ( ( lb_rows != 0 ) && ( ub_rows != 0 ) )
	{
		init(	stk(H),stk(g),stk(A),stk(lb),stk(ub),stk(lbA),stk(ubA),
				&H_rows,&A_rows,istk(nWSR),
				stk(x),stk(obj),istk(status),istk(nWSRout),stk(y)
				);
	}
	else if ( ( lb_rows == 0 ) && ( ub_rows != 0 ) )
	{
		init(	stk(H),stk(g),stk(A),0,stk(ub),stk(lbA),stk(ubA),
				&H_rows,&A_rows,istk(nWSR),
				stk(x),stk(obj),istk(status),istk(nWSRout),stk(y)
				);
	}
	else if ( ( lb_rows != 0 ) && ( ub_rows == 0 ) )
	{
		init(	stk(H),stk(g),stk(A),stk(lb),0,stk(lbA),stk(ubA),
				&H_rows,&A_rows,istk(nWSR),
				stk(x),stk(obj),istk(status),istk(nWSRout),stk(y)
				);
	}
	else
	{
		init(	stk(H),stk(g),stk(A),0,0,stk(lbA),stk(ubA),
				&H_rows,&A_rows,istk(nWSR),
				stk(x),stk(obj),istk(status),istk(nWSRout),stk(y)
				);
	}

	LhsVar(1) = 9;
	LhsVar(2) = 10;
	LhsVar(3) = 11;
	LhsVar(4) = 12;
	LhsVar(5) = 13;

	return 0;
}


/*
 *	i n t _ i n i t S B
 */
int int_initSB( char* fname )
{
	static int H, H_rows, H_cols;
	static int g, g_rows, g_cols;
	static int lb, lb_rows, lb_cols;
	static int ub, ub_rows, ub_cols;
	static int nWSR, nWSR_rows, nWSR_cols;

	static int obj, x, y, status, nWSRout;


	static int minlhs = 1, maxlhs = 5, minrhs = 5, maxrhs = 5, one = 1;

	CheckRhs( minrhs,maxrhs );
	CheckLhs( minlhs,maxlhs );


	/* check dimensions */
	GetRhsVar( 1,"d", &H_rows,&H_cols,&H );
	if ( ( H_rows != H_cols ) || ( H_rows < 1 ) )
	{
		sciprint( "ERROR (qpOASES): Dimension mismatch!\n" );
		Error( 999 );
		return 0;
	}

	GetRhsVar( 2,"d", &g_rows,&g_cols,&g );
	if ( !( ( ( g_rows == H_rows ) && ( g_cols == 1 ) ) || ( ( g_rows == 1 ) && ( g_cols == H_rows ) ) ) )
	{
		sciprint( "ERROR (qpOASES): Dimension mismatch!\n" );
		Error( 999 );
		return 0;
	}

	GetRhsVar( 3,"d", &lb_rows,&lb_cols,&lb);
	if ( !( ( ( lb_rows == H_rows ) && ( lb_cols == 1 ) ) || ( ( lb_rows == 0 ) && ( lb_cols == 0 ) ) ) )
	{
		sciprint( "ERROR (qpOASES): Dimension mismatch!\n" );
		Error( 999 );
		return 0;
	}

	GetRhsVar( 4,"d", &ub_rows,&ub_cols,&ub);
	if ( !( ( ( ub_rows == H_rows ) && ( ub_cols == 1 ) ) || ( ( ub_rows == 0 ) && ( ub_cols == 0 ) ) ) )
	{
		sciprint( "ERROR (qpOASES): Dimension mismatch!\n" );
		Error( 999 );
		return 0;
	}

	GetRhsVar( 5,"i", &nWSR_rows,&nWSR_cols,&nWSR);
	if ( ( nWSR_rows != nWSR_cols ) || ( nWSR_cols != 1 ) )
	{
		sciprint( "ERROR (qpOASES): Dimension mismatch!\n" );
		Error( 999 );
		return 0;
	}


	CreateVar(  9,"d", &H_rows,&one,&x );
	CreateVar( 10,"d", &one,&one,&obj );
	CreateVar( 11,"i", &one,&one,&status );
	CreateVar( 12,"i", &one,&one,&nWSRout );
	CreateVar( 13,"d", &H_rows,&one,&y );


	qpb_rowsH = H_rows;


	/* call interfaced qpOASES routines with appropriate arguments */
	if ( ( lb_rows != 0 ) && ( ub_rows != 0 ) )
	{
		initSB(	stk(H),stk(g),stk(lb),stk(ub),
				&H_rows,istk(nWSR),
				stk(x),stk(obj),istk(status),istk(nWSRout),stk(y)
				);
	}
	else if ( ( lb_rows == 0 ) && ( ub_rows != 0 ) )
	{
		initSB(	stk(H),stk(g),0,stk(ub),
				&H_rows,istk(nWSR),
				stk(x),stk(obj),istk(status),istk(nWSRout),stk(y)
				);
	}
	else if ( ( lb_rows != 0 ) && ( ub_rows == 0 ) )
	{
		initSB(	stk(H),stk(g),stk(lb),0,
				&H_rows,istk(nWSR),
				stk(x),stk(obj),istk(status),istk(nWSRout),stk(y)
				);
	}
	else
	{
		initSB(	stk(H),stk(g),0,0,
				&H_rows,istk(nWSR),
				stk(x),stk(obj),istk(status),istk(nWSRout),stk(y)
				);
	}

	LhsVar(1) = 9;
	LhsVar(2) = 10;
	LhsVar(3) = 11;
	LhsVar(4) = 12;
	LhsVar(5) = 13;

	return 0;
}


/*
 *	i n t _ i n i t V M
 */
int int_initVM( char* fname )
{
	static int H, H_rows, H_cols;
	static int A, A_rows, A_cols;
	static int g, g_rows, g_cols;
	static int lb, lb_rows, lb_cols;
	static int ub, ub_rows, ub_cols;
	static int lbA, lbA_rows, lbA_cols;
	static int ubA, ubA_rows, ubA_cols;
	static int nWSR, nWSR_rows, nWSR_cols;

	static int obj, x, y, status, nWSRout;


	static int minlhs = 1, maxlhs = 5, minrhs = 8, maxrhs = 8, one = 1, y_size;

	CheckRhs( minrhs,maxrhs );
	CheckLhs( minlhs,maxlhs );


	/* check dimensions */
	GetRhsVar( 1,"d", &H_rows,&H_cols,&H );
	if ( ( H_rows != H_cols ) || ( H_rows < 1 ) )
	{
		sciprint( "ERROR (qpOASES): Dimension mismatch!\n" );
		Error( 999 );
		return 0;
	}

	GetRhsVar( 2,"d", &g_rows,&g_cols,&g );
	if ( !( ( ( g_rows == H_rows ) && ( g_cols == 1 ) ) || ( ( g_rows == 1 ) && ( g_cols == H_rows ) ) ) )
	{
		sciprint( "ERROR (qpOASES): Dimension mismatch!\n" );
		Error( 999 );
		return 0;
	}

	GetRhsVar( 3,"d", &A_rows,&A_cols,&A );
	if ( ( A_cols != H_rows ) || ( A_rows < 1 ) )
	{
		sciprint( "ERROR (qpOASES): Dimension mismatch!\n" );
		Error( 999 );
		return 0;
	}

	GetRhsVar( 4,"d", &lb_rows,&lb_cols,&lb );
	if ( !( ( ( lb_rows == H_rows ) && ( lb_cols == 1 ) ) || ( ( lb_rows == 0 ) && ( lb_cols == 0 ) ) ) )
	{
		sciprint( "ERROR (qpOASES): Dimension mismatch!\n" );
		Error( 999 );
		return 0;
	}

	GetRhsVar( 5,"d", &ub_rows,&ub_cols,&ub );
	if ( !( ( ( ub_rows == H_rows ) && ( ub_cols == 1 ) ) || ( ( ub_rows == 0 ) && ( ub_cols == 0 ) ) ) )
	{
		sciprint( "ERROR (qpOASES): Dimension mismatch!\n" );
		Error( 999 );
		return 0;
	}

	GetRhsVar( 6,"d", &lbA_rows,&lbA_cols,&lbA );
	if ( ( lbA_rows != A_rows ) || ( lbA_cols != 1 ) )
	{
		sciprint( "ERROR (qpOASES): Dimension mismatch!\n" );
		Error( 999 );
		return 0;
	}

	GetRhsVar( 7,"d", &ubA_rows,&ubA_cols,&ubA );
	if ( ( ubA_rows != A_rows ) || ( ubA_cols != 1 ) )
	{
		sciprint( "ERROR (qpOASES): Dimension mismatch!\n" );
		Error( 999 );
		return 0;
	}

	GetRhsVar( 8,"i", &nWSR_rows,&nWSR_cols,&nWSR) ;
	if ( ( nWSR_rows != nWSR_cols ) || ( nWSR_cols != 1 ) )
	{
		sciprint( "ERROR (qpOASES): Dimension mismatch!\n" );
		Error( 999 );
		return 0;
	}


	y_size = H_rows + A_rows;

	CreateVar(  9,"d", &H_rows,&one,&x );
	CreateVar( 10,"d", &one,&one,&obj );
	CreateVar( 11,"i", &one,&one,&status );
	CreateVar( 12,"i", &one,&one,&nWSRout );
	CreateVar( 13,"d", &y_size,&one,&y );


	sqp_rowsH = H_rows;
	sqp_rowsA = A_rows;


	/* call interfaced qpOASES routines with appropriate arguments */
	if ( ( lb_rows != 0 ) && ( ub_rows != 0 ) )
	{
		initVM(	stk(H),stk(g),stk(A),stk(lb),stk(ub),stk(lbA),stk(ubA),
				&H_rows,&A_rows,istk(nWSR),
				stk(x),stk(obj),istk(status),istk(nWSRout),stk(y)
				);
	}
	else if ( ( lb_rows == 0 ) && ( ub_rows != 0 ) )
	{
		initVM(	stk(H),stk(g),stk(A),0,stk(ub),stk(lbA),stk(ubA),
				&H_rows,&A_rows,istk(nWSR),
				stk(x),stk(obj),istk(status),istk(nWSRout),stk(y)
				);
	}
	else if ( ( lb_rows != 0 ) && ( ub_rows == 0 ) )
	{
		initVM(	stk(H),stk(g),stk(A),stk(lb),0,stk(lbA),stk(ubA),
				&H_rows,&A_rows,istk(nWSR),
				stk(x),stk(obj),istk(status),istk(nWSRout),stk(y)
				);
	}
	else
	{
		initVM(	stk(H),stk(g),stk(A),0,0,stk(lbA),stk(ubA),
				&H_rows,&A_rows,istk(nWSR),
				stk(x),stk(obj),istk(status),istk(nWSRout),stk(y)
				);
	}

	LhsVar(1) = 9;
	LhsVar(2) = 10;
	LhsVar(3) = 11;
	LhsVar(4) = 12;
	LhsVar(5) = 13;

	return 0;
}


/*
 *	i n t _ h o t s t a r t
 */
int int_hotstart( char* fname )
{
	static int g, g_rows, g_cols;
	static int lb, lb_rows, lb_cols;
	static int ub, ub_rows, ub_cols;
	static int lbA, lbA_rows, lbA_cols;
	static int ubA, ubA_rows, ubA_cols;
	static int nWSR, nWSR_rows, nWSR_cols;

	static int obj, x, y, status, nWSRout;


	static int minlhs = 1, maxlhs = 5, minrhs = 6, maxrhs = 6, one = 1, y_size;

	CheckRhs( minrhs,maxrhs );
	CheckLhs( minlhs,maxlhs );


	if ( ( qp_rowsH == -1 ) || ( qp_rowsA == -1 ) )
	{
		sciprint( "ERROR (qpOASES): QP not initialised!\n" );
		Error( 999 );
		return 0;
	}

	/* check dimensions */
	GetRhsVar( 1,"d", &g_rows,&g_cols,&g );
	if ( !( ( ( g_rows == qp_rowsH ) && ( g_cols == 1 ) ) || ( ( g_rows == 1 ) && ( g_cols == qp_rowsH ) ) ) )
	{
		sciprint( "ERROR (qpOASES): Dimension mismatch!\n" );
		Error( 999 );
		return 0;
	}

	GetRhsVar( 2,"d", &lb_rows,&lb_cols,&lb );
	if ( !( ( ( lb_rows == qp_rowsH ) && ( lb_cols == 1 ) ) || ( ( lb_rows == 0 ) && ( lb_cols == 0 ) ) ) )
	{
		sciprint( "ERROR (qpOASES): Dimension mismatch!\n" );
		Error( 999 );
		return 0;
	}

	GetRhsVar( 3,"d", &ub_rows,&ub_cols,&ub );
	if ( !( ( ( ub_rows == qp_rowsH ) && ( ub_cols == 1 ) ) || ( ( ub_rows == 0 ) && ( ub_cols == 0 ) ) ) )
	{
		sciprint( "ERROR (qpOASES): Dimension mismatch!\n" );
		Error( 999 );
		return 0;
	}

	GetRhsVar( 4,"d", &lbA_rows,&lbA_cols,&lbA );
	if ( ( lbA_rows != qp_rowsA ) || ( lbA_cols != 1 ) )
	{
		sciprint( "ERROR (qpOASES): Dimension mismatch!\n" );
		Error( 999 );
		return 0;
	}

	GetRhsVar( 5,"d", &ubA_rows,&ubA_cols,&ubA );
	if ( ( ubA_rows != qp_rowsA ) || ( ubA_cols != 1 ) )
	{
		sciprint( "ERROR (qpOASES): Dimension mismatch!\n" );
		Error( 999 );
		return 0;
	}

	GetRhsVar( 6,"i", &nWSR_rows,&nWSR_cols,&nWSR );
	if ( ( nWSR_rows != nWSR_cols ) || ( nWSR_cols != 1 ) )
	{
		sciprint( "ERROR (qpOASES): Dimension mismatch!\n" );
		Error( 999 );
		return 0;
	}


	y_size = qp_rowsH + qp_rowsA;

	CreateVar(  7,"d", &qp_rowsH,&one,&x );
	CreateVar(  8,"d", &one,&one,&obj );
	CreateVar(  9,"i", &one,&one,&status );
	CreateVar( 10,"i", &one,&one,&nWSRout );
	CreateVar( 11,"d", &y_size,&one,&y );


	/* call interfaced qpOASES routines with appropriate arguments */
	if ( ( lb_rows != 0 ) && ( ub_rows != 0 ) )
	{
		hotstart(	stk(g),stk(lb),stk(ub),stk(lbA),stk(ubA),
					istk(nWSR),
					stk(x),stk(obj),istk(status),istk(nWSRout),stk(y)
					);
	}
	else if ( ( lb_rows == 0 ) && ( ub_rows != 0 ) )
	{
		hotstart(	stk(g),0,stk(ub),stk(lbA),stk(ubA),
					istk(nWSR),
					stk(x),stk(obj),istk(status),istk(nWSRout),stk(y)
					);
	}
	else if ( ( lb_rows != 0 ) && ( ub_rows == 0 ) )
	{
		hotstart(	stk(g),stk(lb),0,stk(lbA),stk(ubA),
					istk(nWSR),
					stk(x),stk(obj),istk(status),istk(nWSRout),stk(y)
					);
	}
	else
	{
		hotstart(	stk(g),0,0,stk(lbA),stk(ubA),
					istk(nWSR),
					stk(x),stk(obj),istk(status),istk(nWSRout),stk(y)
					);
	}


	LhsVar(1) = 7;
	LhsVar(2) = 8;
	LhsVar(3) = 9;
	LhsVar(4) = 10;
	LhsVar(5) = 11;

	return 0;
}


/*
 *	i n t _ h o t s t a r t S B
 */
int int_hotstartSB( char* fname )
{
	static int g, g_rows, g_cols;
	static int lb, lb_rows, lb_cols;
	static int ub, ub_rows, ub_cols;
	static int nWSR, nWSR_rows, nWSR_cols;

	static int obj, x, y, status, nWSRout;


	static int minlhs = 1, maxlhs = 5, minrhs = 4, maxrhs = 4, one = 1;

	CheckRhs( minrhs,maxrhs );
	CheckLhs( minlhs,maxlhs );


	if ( qpb_rowsH == -1 )
	{
		sciprint( "ERROR (qpOASES): QP not initialised!\n" );
		Error( 999 );
		return 0;
	}

	/* check dimensions */
	GetRhsVar( 1,"d", &g_rows,&g_cols,&g );
	if ( !( ( ( g_rows == qpb_rowsH ) && ( g_cols == 1 ) ) || ( ( g_rows == 1 ) && ( g_cols == qpb_rowsH ) ) ) )
	{
		sciprint( "ERROR (qpOASES): Dimension mismatch!\n" );
		Error( 999 );
		return 0;
	}

	GetRhsVar( 2,"d", &lb_rows,&lb_cols,&lb );
	if ( !( ( ( lb_rows == qpb_rowsH ) && ( lb_cols == 1 ) ) || ( ( lb_rows == 0 ) && ( lb_cols == 0 ) ) ) )
	{
		sciprint( "ERROR (qpOASES): Dimension mismatch!\n" );
		Error( 999 );
		return 0;
	}

	GetRhsVar( 3,"d", &ub_rows,&ub_cols,&ub );
	if ( !( ( ( ub_rows == qpb_rowsH ) && ( ub_cols == 1 ) ) || ( ( ub_rows == 0 ) && ( ub_cols == 0 ) ) ) )
	{
		sciprint( "ERROR (qpOASES): Dimension mismatch!\n" );
		Error( 999 );
		return 0;
	}

	GetRhsVar( 4,"i", &nWSR_rows,&nWSR_cols,&nWSR );
	if ( ( nWSR_rows != nWSR_cols ) || ( nWSR_cols != 1 ) )
	{
		sciprint( "ERROR (qpOASES): Dimension mismatch!\n" );
		Error( 999 );
		return 0;
	}


	CreateVar( 5,"d", &qpb_rowsH,&one,&x );
	CreateVar( 6,"d", &one,&one,&obj );
	CreateVar( 7,"i", &one,&one,&status );
	CreateVar( 8,"i", &one,&one,&nWSRout );
	CreateVar( 9,"d", &qpb_rowsH,&one,&y );


	/* call interfaced qpOASES routines with appropriate arguments */
	if ( ( lb_rows != 0 ) && ( ub_rows != 0 ) )
	{
		hotstartSB(	stk(g),stk(lb),stk(ub),
					istk(nWSR),
					stk(x),stk(obj),istk(status),istk(nWSRout),stk(y)
					);
	}
	else if ( ( lb_rows == 0 ) && ( ub_rows != 0 ) )
	{
		hotstartSB(	stk(g),0,stk(ub),
					istk(nWSR),
					stk(x),stk(obj),istk(status),istk(nWSRout),stk(y)
					);
	}
	else if ( ( lb_rows != 0 ) && ( ub_rows == 0 ) )
	{
		hotstartSB(	stk(g),stk(lb),0,
					istk(nWSR),
					stk(x),stk(obj),istk(status),istk(nWSRout),stk(y)
					);
	}
	else
	{
		hotstartSB(	stk(g),0,0,
					istk(nWSR),
					stk(x),stk(obj),istk(status),istk(nWSRout),stk(y)
					);
	}


	LhsVar(1) = 5;
	LhsVar(2) = 6;
	LhsVar(3) = 7;
	LhsVar(4) = 8;
	LhsVar(5) = 9;

	return 0;
}


/*
 *	i n t _ h o t s t a r t V M
 */
int int_hotstartVM( char* fname )
{
	static int H, H_rows, H_cols;
	static int A, A_rows, A_cols;
	static int g, g_rows, g_cols;
	static int lb, lb_rows, lb_cols;
	static int ub, ub_rows, ub_cols;
	static int lbA, lbA_rows, lbA_cols;
	static int ubA, ubA_rows, ubA_cols;
	static int nWSR, nWSR_rows, nWSR_cols;

	static int obj, x, y, status, nWSRout;


	static int minlhs = 1, maxlhs = 5, minrhs = 8, maxrhs = 8, one = 1, y_size;

	CheckRhs( minrhs,maxrhs );
	CheckLhs( minlhs,maxlhs );


	if ( ( sqp_rowsH == -1 ) || ( sqp_rowsA == -1 ) )
	{
		sciprint( "ERROR (qpOASES): QP not initialised!\n" );
		Error( 999 );
		return 0;
	}

	/* check dimensions */
	GetRhsVar( 1,"d", &H_rows,&H_cols,&H );
	if ( ( H_rows != H_cols ) || ( H_rows < 1 ) )
	{
		sciprint( "ERROR (qpOASES): Dimension mismatch!\n" );
		Error( 999 );
		return 0;
	}

	GetRhsVar( 2,"d", &g_rows,&g_cols,&g );
	if ( !( ( ( g_rows == H_rows ) && ( g_cols == 1 ) ) || ( ( g_rows == 1 ) && ( g_cols == H_rows ) ) ) )
	{
		sciprint( "ERROR (qpOASES): Dimension mismatch!\n" );
		Error( 999 );
		return 0;
	}

	GetRhsVar( 3,"d", &A_rows,&A_cols,&A );
	if ( ( A_cols != H_rows ) || ( A_rows < 1 ) )
	{
		sciprint( "ERROR (qpOASES): Dimension mismatch!\n" );
		Error( 999 );
		return 0;
	}

	GetRhsVar( 4,"d", &lb_rows,&lb_cols,&lb);
	if ( !( ( ( lb_rows == H_rows ) && ( lb_cols == 1 ) ) || ( ( lb_rows == 0 ) && ( lb_cols == 0 ) ) ) )
	{
		sciprint( "ERROR (qpOASES): Dimension mismatch!\n" );
		Error( 999 );
		return 0;
	}

	GetRhsVar( 5,"d", &ub_rows,&ub_cols,&ub);
	if ( !( ( ( ub_rows == H_rows ) && ( ub_cols == 1 ) ) || ( ( ub_rows == 0 ) && ( ub_cols == 0 ) ) ) )
	{
		sciprint( "ERROR (qpOASES): Dimension mismatch!\n" );
		Error( 999 );
		return 0;
	}

	GetRhsVar( 6,"d", &lbA_rows,&lbA_cols,&lbA);
	if ( ( lbA_rows != A_rows ) || ( lbA_cols != 1 ) )
	{
		sciprint( "ERROR (qpOASES): Dimension mismatch!\n" );
		Error( 999 );
		return 0;
	}

	GetRhsVar( 7,"d", &ubA_rows,&ubA_cols,&ubA);
	if ( ( ubA_rows != A_rows ) || ( ubA_cols != 1 ) )
	{
		sciprint( "ERROR (qpOASES): Dimension mismatch!\n" );
		Error( 999 );
		return 0;
	}

	GetRhsVar( 8,"i", &nWSR_rows,&nWSR_cols,&nWSR);
	if ( ( nWSR_rows != nWSR_cols ) || ( nWSR_cols != 1 ) )
	{
		sciprint( "ERROR (qpOASES): Dimension mismatch!\n" );
		Error( 999 );
		return 0;
	}

	/* have matrices same dimension as last QP? */
	if ( ( sqp_rowsH != H_rows ) || ( sqp_rowsA != A_rows ) )
	{
		sciprint( "ERROR (qpOASES): Incompatible matrix dimensions!\n" );
		Error( 999 );
		return 0;
	}


	y_size = H_rows + A_rows;

	CreateVar(  9,"d", &H_rows,&one,&x );
	CreateVar( 10,"d", &one,&one,&obj );
	CreateVar( 11,"i", &one,&one,&status );
	CreateVar( 12,"i", &one,&one,&nWSRout );
	CreateVar( 13,"d", &y_size,&one,&y );


	/* call interfaced qpOASES routines with appropriate arguments */
	if ( ( lb_rows != 0 ) && ( ub_rows != 0 ) )
	{
		hotstartVM(	stk(H),stk(g),stk(A),stk(lb),stk(ub),stk(lbA),stk(ubA),
					istk(nWSR),
					stk(x),stk(obj),istk(status),istk(nWSRout),stk(y)
					);
	}
	else if ( ( lb_rows == 0 ) && ( ub_rows != 0 ) )
	{
		hotstartVM(	stk(H),stk(g),stk(A),0,stk(ub),stk(lbA),stk(ubA),
					istk(nWSR),
					stk(x),stk(obj),istk(status),istk(nWSRout),stk(y)
					);
	}
	else if ( ( lb_rows != 0 ) && ( ub_rows == 0 ) )
	{
		hotstartVM(	stk(H),stk(g),stk(A),stk(lb),0,stk(lbA),stk(ubA),
					istk(nWSR),
					stk(x),stk(obj),istk(status),istk(nWSRout),stk(y)
					);
	}
	else
	{
		hotstartVM(	stk(H),stk(g),stk(A),0,0,stk(lbA),stk(ubA),
					istk(nWSR),
					stk(x),stk(obj),istk(status),istk(nWSRout),stk(y)
					);
	}


	LhsVar(1) = 9;
	LhsVar(2) = 10;
	LhsVar(3) = 11;
	LhsVar(4) = 12;
	LhsVar(5) = 13;

	return 0;
}


/*
 *	i n t _ c l e a n u p
 */
int int_cleanup( char* fname )
{
	const int minlhs = 1, maxlhs = 1, minrhs = 0, maxrhs = 0;

	CheckRhs( minrhs,maxrhs );
	CheckLhs( minlhs,maxlhs );

	cleanupp( );
	qp_rowsH = -1;
	qp_rowsA = -1;

	return 0;
}


/*
 *	i n t _ c l e a n u p S B
 */
int int_cleanupSB( char* fname )
{
	const int minlhs = 1, maxlhs = 1, minrhs = 0, maxrhs = 0;

	CheckRhs( minrhs,maxrhs );
	CheckLhs( minlhs,maxlhs );

	cleanupSB( );
	qpb_rowsH = -1;

	return 0;
}


/*
 *	i n t _ c l e a n u p V M
 */
int int_cleanupVM( char* fname )
{
	const int minlhs = 1, maxlhs = 1, minrhs = 0, maxrhs = 0;

	CheckRhs( minrhs,maxrhs );
	CheckLhs( minlhs,maxlhs );

	cleanupVM( );
	sqp_rowsH = -1;
	sqp_rowsA = -1;

	return 0;
}


/*
 *	q p O A S E S g a t e w a y
 */
int C2F(qpOASESgateway)( )
{
	gate_function function[] = {	int_qpOASES,
									int_init, int_initSB, int_initVM,
									int_hotstart, int_hotstartSB, int_hotstartVM,
									int_cleanup, int_cleanupSB, int_cleanupVM
									};
	char* name[] = {	"qpOASES",
						"qpOASES_init", "qpOASES_initSB", "qpOASES_initVM",
						"qpOASES_hotstart", "qpOASES_hotstartSB", "qpOASES_hotstartVM",
						"qpOASES_cleanup", "qpOASES_cleanupSB", "qpOASES_cleanupVM"
						};

	Rhs = Max( 0,Rhs );
	sci_gateway( name[Fin-1],function[Fin-1] );

	return 0;
}


/*
 *	end of file
 */
