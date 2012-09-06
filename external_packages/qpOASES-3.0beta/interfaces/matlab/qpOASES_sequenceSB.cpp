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
 *	\file interfaces/matlab/qpOASES_sequenceSB.cpp
 *	\author Hans Joachim Ferreau, Aude Perrin
 *	\version 3.0beta
 *	\date 2007-2011
 *
 *	Interface for Matlab(R) that enables to call qpOASES as a MEX function
 *  (variant for simply bounded QPs).
 *
 */



#include <qpOASES/QProblemB.hpp>


using namespace qpOASES;

#include <qpOASES_matlab_utils.cpp>


/* global pointer to QP object */
static QProblemB* globalQPB = 0;
static SymmetricMatrix* globalQPB_H = 0;
static long* globalQPB_Hdiag = 0;

/*
 *	a l l o c a t e G l o b a l Q P r o b l e m B I n s t a n c e
 */
void allocateGlobalQProblemBInstance(	int nV, Options* options
										)
{
	globalQPB = new QProblemB( nV );
	globalQPB->setOptions( *options );

	return;
}


/*
 *	d e l e t e G l o b a l Q P r o b l e m B I n s t a n c e
 */
void deleteGlobalQProblemBInstance( )
{
	if ( globalQPB != 0 )
	{
		delete globalQPB;
		globalQPB = 0;
	}

	return;
}


/*
 *	d e l e t e G l o b a l Q P r o b l e m B M a t r i c e s
 */
void deleteGlobalQProblemBMatrices( )
{
	if ( globalQPB_H != 0 )
	{
		delete globalQPB_H;
		globalQPB_H = 0;
	}

	if ( globalQPB_Hdiag != 0 )
	{
		delete[] globalQPB_Hdiag;
		globalQPB_Hdiag = 0;
	}

	return;
}


/*
 *	i n i t S B
 */
void initSB(	int nV,
				SymmetricMatrix *H, real_t* g,
				const real_t* const lb, const real_t* const ub,
				int nWSR, const real_t* const x0, Options* options,
				int nOutputs, mxArray* plhs[]
				)
{
	/* 1) Setup initial QP. */
	allocateGlobalQProblemBInstance( nV,options );

	/* 2) Solve initial QP. */
	returnValue returnvalue;

	if ( x0 == 0 )
	{
// 		myPrintf( "ccc\n" );
		returnvalue = globalQPB->init( H,g,lb,ub, nWSR,0 );
// 		myPrintf( "ccc\n" );
	}
	else
		returnvalue = globalQPB->init( H,g,lb,ub, nWSR,0, x0,0,0 );

	/* 3) Assign lhs arguments. */
	obtainOutputs(	0,globalQPB,returnvalue,nWSR,
					nOutputs,plhs,0 );

	return;
}



/*
 *	h o t s t a r t S B
 */
void hotstartSB(	const real_t* const g,
					const real_t* const lb, const real_t* const ub,
					int nWSR, Options* options,
					int nOutputs, mxArray* plhs[]
					)
{
	/* 1) Solve QP. */
	globalQPB->setOptions( *options );
	returnValue returnvalue = globalQPB->hotstart( g,lb,ub, nWSR,0 );

	/* 2) Assign lhs arguments. */
	obtainOutputs(	0,globalQPB,returnvalue,nWSR,
					nOutputs,plhs,0 );

	return;
}


/*
 *	m e x F u n c t i o n
 */
void mexFunction( int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[] )
{
	/* inputs */
	char* typeString;
	real_t *H_for=0, *H_mem=0, *g=0, *lb=0, *ub=0, *x0=0;

	Options options;
	options.printLevel = PL_LOW;
	#ifdef __DEBUG__
	options.printLevel = PL_HIGH;
	#endif
	#ifdef __SUPPRESSANYOUTPUT__
	options.printLevel = PL_NONE;
	#endif

	/* dimensions */
	unsigned int nV=0;


	/* I) CONSISTENCY CHECKS: */
	/* 1) Ensure that qpOASES is called with a feasible number of input arguments. */
	if ( ( nrhs < 4 ) || ( nrhs > 7 ) )
		if ( nrhs != 1 )
			mexErrMsgTxt( "ERROR (qpOASES): Invalid number of input arguments!\nType 'help qpOASES_sequenceSB' for further information." );

	/* 2) Ensure that first input is a string (and if so, read it). */
	if ( mxIsChar( prhs[0] ) != 1 )
		mexErrMsgTxt( "ERROR (qpOASES): First input argument must be a string!" );

	typeString = (char*) mxGetPr( prhs[0] );


	/* II) SELECT RESPECTIVE QPOASES FUNCTION CALL: */
	/* 1) Init (without or with initial guess for primal solution). */
	if ( ( strcmp( typeString,"i" ) == 0 ) || ( strcmp( typeString,"I" ) == 0 ) )
	{
		/* consistency checks */
		if ( ( nlhs < 1 ) || ( nlhs > 5 ) )
			mexErrMsgTxt( "ERROR (qpOASES): Invalid number of output arguments!\nType 'help qpOASES_sequenceSB' for further information." );

		if ( ( nrhs < 5 ) || ( nrhs > 7 ) )
			mexErrMsgTxt( "ERROR (qpOASES): Invalid number of input arguments!\nType 'help qpOASES_sequenceSB' for further information." );

		/* ensure that data is given in real_t precision */
		if ( ( mxIsDouble( prhs[1] ) == 0 ) ||
			 ( mxIsDouble( prhs[2] ) == 0 ) )
			mexErrMsgTxt( "ERROR (qpOASES): All data has to be provided in real_t precision!" );

		/* ensure that Hessian matrix is stored in dense format */
// 		if ( mxIsSparse( prhs[1] ) != 0 )
// 			mexErrMsgTxt( "ERROR (qpOASES): Matrices must not be stored in sparse format!" );


		/* Check inputs dimensions and assign pointers to inputs. */
		nV = mxGetM( prhs[1] ); /* row number of Hessian matrix */

		if ( mxGetN( prhs[1] ) != nV )
			mexErrMsgTxt( "ERROR (qpOASES): Input dimension mismatch!" );

		if ( smartDimensionCheck( &g,nV,1, BT_FALSE,prhs,2 ) != SUCCESSFUL_RETURN )
			return;

		if ( smartDimensionCheck( &lb,nV,1, BT_TRUE,prhs,3 ) != SUCCESSFUL_RETURN )
			return;

		if ( smartDimensionCheck( &ub,nV,1, BT_TRUE,prhs,4 ) != SUCCESSFUL_RETURN )
			return;

		/* default value for nWSR */
		int nWSRin = 5*nV;

		/* Check whether x0 and options are specified .*/
		if ( nrhs > 5 )
		{
			if ( smartDimensionCheck( &x0,nV,1, BT_TRUE,prhs,5 ) != SUCCESSFUL_RETURN )
				return;

			if ( nrhs > 6 )
				if ( ( !mxIsEmpty( prhs[6] ) ) && ( mxIsStruct( prhs[6] ) ) )
					setupOptions( &options,prhs[6],nWSRin );
		}

		deleteGlobalQProblemBInstance( );
		deleteGlobalQProblemBMatrices( );

		/* check for sparsity */
		if ( mxIsSparse( prhs[1] ) != 0 )
		{
			long *ir = (long *)mxGetIr(prhs[1]);
			long *jc = (long *)mxGetJc(prhs[1]);
			real_t *v = (real_t*)mxGetPr(prhs[1]);
			// mind pointer offsets due to 1-based indexing in Matlab
			SymSparseMat *sH;
			globalQPB_H = sH = new SymSparseMat(nV, nV, ir, jc, v);
			globalQPB_Hdiag = sH->createDiagInfo();
		}
		else
		{
			H_for = (real_t*) mxGetPr( prhs[1] );
			H_mem = new real_t[nV*nV];
			for( int i=0; i<nV*nV; ++i )
				H_mem[i] = H_for[i];
			globalQPB_H = new SymDenseMat( nV, nV, nV, H_mem );
			globalQPB_H->doFreeMemory();
		}

		/* Create output vectors and assign pointers to them. */
		allocateOutputs( nlhs,plhs, nV );

		/* call qpOASES */
		initSB(	nV,
				globalQPB_H,g,
				lb,ub,
				nWSRin,x0,&options,
				nlhs,plhs
				);

		return;
	}

	/* 2) Hotstart. */
	if ( ( strcmp( typeString,"h" ) == 0 ) || ( strcmp( typeString,"H" ) == 0 ) )
	{
		/* consistency checks */
		if ( ( nlhs < 1 ) || ( nlhs > 5 ) )
			mexErrMsgTxt( "ERROR (qpOASES): Invalid number of output arguments!\nType 'help qpOASES_sequenceSB' for further information." );

		if ( ( nrhs < 4 ) || ( nrhs > 5 ) )
			mexErrMsgTxt( "ERROR (qpOASES): Invalid number of input arguments!\nType 'help qpOASES_sequenceSB' for further information." );

		/* has QP been initialised? */
		if ( globalQPB == 0 )
			mexErrMsgTxt( "ERROR (qpOASES): QP sequence needs to be initialised first!" );

		/* Check inputs dimensions and assign pointers to inputs. */
		nV = globalQPB->getNV( );

		if ( smartDimensionCheck( &g,nV,1, BT_FALSE,prhs,1 ) != SUCCESSFUL_RETURN )
			return;

		if ( smartDimensionCheck( &lb,nV,1, BT_TRUE,prhs,2 ) != SUCCESSFUL_RETURN )
			return;

		if ( smartDimensionCheck( &ub,nV,1, BT_TRUE,prhs,3 ) != SUCCESSFUL_RETURN )
			return;

		/* default value for nWSR */
		int nWSRin = 5*nV;

		/* Check whether options are specified .*/
		if ( nrhs == 5 )
			if ( ( !mxIsEmpty( prhs[4] ) ) && ( mxIsStruct( prhs[4] ) ) )
				setupOptions( &options,prhs[4],nWSRin );

		/* Create output vectors and assign pointers to them. */
		allocateOutputs( nlhs,plhs, nV );

		/* call qpOASES */
		hotstartSB(	g,
					lb,ub,
					nWSRin,&options,
					nlhs,plhs
					);

		return;
	}

	/* 3) Cleanup. */
	if ( ( strcmp( typeString,"c" ) == 0 ) || ( strcmp( typeString,"C" ) == 0 ) )
	{
		/* consistency checks */
		if ( nlhs != 0 )
			mexErrMsgTxt( "ERROR (qpOASES): Invalid number of output arguments!\nType 'help qpOASES_sequenceSB' for further information." );

		if ( nrhs != 1 )
			mexErrMsgTxt( "ERROR (qpOASES): Invalid number of input arguments!\nType 'help qpOASES_sequenceSB' for further information." );

		/* Cleanup global QProblemB instance. */
		deleteGlobalQProblemBInstance( );
		deleteGlobalQProblemBMatrices( );
		return;
	}
}

/*
 *	end of file
 */
