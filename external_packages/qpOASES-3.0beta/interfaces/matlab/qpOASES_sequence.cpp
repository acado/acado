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
 *	\file interfaces/matlab/qpOASES_sequence.cpp
 *	\author Hans Joachim Ferreau, Aude Perrin
 *	\version 3.0beta
 *	\date 2007-2011
 *
 *	Interface for Matlab(R) that enables to call qpOASES as a MEX function
 *  (variant for QPs with fixed matrices).
 *
 */



#include <qpOASES/QProblem.hpp>


using namespace qpOASES;

#include <qpOASES_matlab_utils.cpp>


/* global pointer to QP object */
static QProblem* globalQP = 0;
static SymmetricMatrix* globalQP_H = 0;
static Matrix* globalQP_A = 0;
static long* globalQP_Hdiag = 0;


/*
 *	a l l o c a t e G l o b a l Q P r o b l e m I n s t a n c e
 */
void allocateGlobalQProblemInstance(	int nV, int nC, Options* options
										)
{
	globalQP = new QProblem( nV,nC );
	globalQP->setOptions( *options );

	return;
}


/*
 *	d e l e t e G l o b a l Q P r o b l e m I n s t a n c e
 */
void deleteGlobalQProblemInstance( )
{
	if ( globalQP != 0 )
	{
		delete globalQP;
		globalQP = 0;
	}

	return;
}


/*
 *	d e l e t e G l o b a l Q P r o b l e m M a t r i c e s
 */
void deleteGlobalQProblemMatrices( )
{
	if ( globalQP_H != 0 )
	{
		delete globalQP_H;
		globalQP_H = 0;
	}

	if ( globalQP_A != 0 )
	{
		delete globalQP_A;
		globalQP_A = 0;
	}

	if ( globalQP_Hdiag != 0 )
	{
		delete[] globalQP_Hdiag;
		globalQP_Hdiag = 0;
	}

	return;
}


/*
 *	i n i t
 */
void init(	int nV, int nC,
			SymmetricMatrix *H, real_t* g, Matrix *A,
			const real_t* const lb, const real_t* const ub, const real_t* const lbA, const real_t* const ubA,
			int nWSR, const real_t* const x0, Options* options,
			int nOutputs, mxArray* plhs[]
			)
{
	/* 1) Setup initial QP. */
	allocateGlobalQProblemInstance( nV,nC,options );

	/* 2) Solve initial QP. */
	returnValue returnvalue;

	if ( x0 == 0 )
		returnvalue = globalQP->init( H,g,A,lb,ub,lbA,ubA, nWSR,0 );
	else
		returnvalue = globalQP->init( H,g,A,lb,ub,lbA,ubA, nWSR,0, x0,0,0,0 );

	/* 3) Assign lhs arguments. */
	obtainOutputs(	0,globalQP,returnvalue,nWSR,
					nOutputs,plhs,0,0 );

	return;
}


/*
 *	h o t s t a r t
 */
void hotstart(	const real_t* const g,
				const real_t* const lb, const real_t* const ub,
				const real_t* const lbA, const real_t* const ubA,
				int nWSR, Options* options,
				int nOutputs, mxArray* plhs[]
				)
{
	/* 1) Solve QP with given options. */
	globalQP->setOptions( *options );
	returnValue returnvalue = globalQP->hotstart( g,lb,ub,lbA,ubA, nWSR,0 );

	/* 2) Assign lhs arguments. */
	obtainOutputs(	0,globalQP,returnvalue,nWSR,
					nOutputs,plhs,0,0 );

	return;
}


/*
 *	m e x F u n c t i o n
 */
void mexFunction( int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[] )
{
	unsigned int i,j;

	/* inputs */
	char* typeString;
	real_t *H_for=0, *H_mem=0, *g=0, *A_for=0, *A_mem=0, *lb=0, *ub=0, *lbA=0, *ubA=0, *x0=0;

	Options options;
	options.printLevel = PL_LOW;
	#ifdef __DEBUG__
	options.printLevel = PL_HIGH;
	#endif
	#ifdef __SUPPRESSANYOUTPUT__
	options.printLevel = PL_NONE;
	#endif

	/* dimensions */
	unsigned int nV=0, nC=0;


	/* I) CONSISTENCY CHECKS: */
	/* 1) Ensure that qpOASES is called with a feasible number of input arguments. */
	if ( ( nrhs < 6 ) || ( nrhs > 10 ) )
		if ( nrhs != 1 )
			mexErrMsgTxt( "ERROR (qpOASES): Invalid number of input arguments!\nType 'help qpOASES_sequence' for further information." );

	/* 2) Ensure that first input is a string ... */
	if ( mxIsChar( prhs[0] ) != 1 )
		mexErrMsgTxt( "ERROR (qpOASES): First input argument must be a string!" );

	typeString = (char*) mxGetPr( prhs[0] );

	/*    ... and if so, check if it is an allowed one. */
	if ( ( strcmp( typeString,"i" ) != 0 ) && ( strcmp( typeString,"I" ) != 0 ) &&
		 ( strcmp( typeString,"h" ) != 0 ) && ( strcmp( typeString,"H" ) != 0 ) &&
		 ( strcmp( typeString,"c" ) != 0 ) && ( strcmp( typeString,"C" ) != 0 ) )
	{
		mexErrMsgTxt( "ERROR (qpOASES): Undefined first input argument!\nType 'help qpOASES_sequence' for further information." );
	}


	/* II) SELECT RESPECTIVE QPOASES FUNCTION CALL: */
	/* 1) Init (without or with initial guess for primal solution). */
	if ( ( strcmp( typeString,"i" ) == 0 ) || ( strcmp( typeString,"I" ) == 0 ) )
	{
		/* consistency checks */
		if ( ( nlhs < 1 ) || ( nlhs > 5 ) )
			mexErrMsgTxt( "ERROR (qpOASES): Invalid number of output arguments!\nType 'help qpOASES_sequence' for further information." );

		if ( ( nrhs < 8 ) || ( nrhs > 10 ) )
			mexErrMsgTxt( "ERROR (qpOASES): Invalid number of input arguments!\nType 'help qpOASES_sequence' for further information." );

		/* ensure that data is given in real_t precision */
		if ( ( mxIsDouble( prhs[1] ) == 0 ) ||
			 ( mxIsDouble( prhs[2] ) == 0 ) ||
			 ( mxIsDouble( prhs[3] ) == 0 ) )
			mexErrMsgTxt( "ERROR (qpOASES): All data has to be provided in real_t precision!" );


		/* ensure that matrices are stored in dense format */
// 		if ( ( mxIsSparse( prhs[1] ) != 0 ) || ( mxIsSparse( prhs[3] ) != 0 ) )
// 			mexErrMsgTxt( "ERROR (qpOASES): Matrices must not be stored in sparse format!" );

		/* Check inputs dimensions and assign pointers to inputs. */
		nV = mxGetM( prhs[1] ); /* row number of Hessian matrix */
		nC = mxGetM( prhs[3] ); /* row number of constraint matrix */

		if ( ( mxGetN( prhs[1] ) != nV ) || ( ( mxGetN( prhs[3] ) != 0 ) && ( mxGetN( prhs[3] ) != nV ) ) )
			mexErrMsgTxt( "ERROR (qpOASES): Input dimension mismatch!" );


		if ( smartDimensionCheck( &g,nV,1, BT_FALSE,prhs,2 ) != SUCCESSFUL_RETURN )
			return;

		if ( smartDimensionCheck( &lb,nV,1, BT_TRUE,prhs,4 ) != SUCCESSFUL_RETURN )
			return;

		if ( smartDimensionCheck( &ub,nV,1, BT_TRUE,prhs,5 ) != SUCCESSFUL_RETURN )
			return;

		if ( smartDimensionCheck( &lbA,nC,1, BT_TRUE,prhs,6 ) != SUCCESSFUL_RETURN )
			return;

		if ( smartDimensionCheck( &ubA,nC,1, BT_TRUE,prhs,7 ) != SUCCESSFUL_RETURN )
			return;

		/* default value for nWSR */
		int nWSRin = 5*(nV+nC);


		/* Check whether x0 and options are specified .*/
		if ( nrhs > 8 )
		{
			if ( smartDimensionCheck( &x0,nV,1, BT_TRUE,prhs,8 ) != SUCCESSFUL_RETURN )
				return;

			if ( nrhs > 9 )
				if ( ( !mxIsEmpty( prhs[9] ) ) && ( mxIsStruct( prhs[9] ) ) )
					setupOptions( &options,prhs[9],nWSRin );
		}

		deleteGlobalQProblemInstance( );
		deleteGlobalQProblemMatrices( );

		/* check for sparsity */
		if ( mxIsSparse( prhs[1] ) != 0 )
		{
			long *ir = (long *)mxGetIr(prhs[1]);
			long *jc = (long *)mxGetJc(prhs[1]);
			real_t *v = (real_t*)mxGetPr(prhs[1]);
			/*
			for (long col = 0; col < nV; col++)
				for (long idx = jc[col]; idx < jc[col+1]; idx++)
					mexPrintf("   (%ld,%ld) %12.4f\n", ir[idx]+1, col+1, v[idx]);
					*/
			//mexPrintf( "%ld\n", ir[0] );
			SymSparseMat *sH;
			globalQP_H = sH = new SymSparseMat(nV, nV, ir, jc, v);
			globalQP_Hdiag = sH->createDiagInfo();
		}
		else
		{
			H_for = (real_t*) mxGetPr( prhs[1] );
			H_mem = new real_t[nV*nV];
			for( int i=0; i<nV*nV; ++i )
				H_mem[i] = H_for[i];
			globalQP_H = new SymDenseMat( nV, nV, nV, H_mem );
			globalQP_H->doFreeMemory();
		}

		/* Convert constraint matrix A from FORTRAN to C style
		 * (not necessary for H as it should be symmetric!). */
		if ( nC > 0 )
		{
			/* Check for sparsity. */
			if ( mxIsSparse( prhs[3] ) != 0 )
			{
				long *ir = (long *)mxGetIr(prhs[3]);
				long *jc = (long *)mxGetJc(prhs[3]);
				real_t *v = (real_t*)mxGetPr(prhs[3]);
				// mind pointer offsets due to 1-based indexing in Matlab
				globalQP_A = new SparseMatrix(nC, nV, ir, jc, v);
			}
			else
			{
				/* Convert constraint matrix A from FORTRAN to C style
				* (not necessary for H as it should be symmetric!). */
				A_for = (real_t*) mxGetPr( prhs[3] );
				A_mem = new real_t[nC*nV];
				convertFortranToC( A_for,nV,nC, A_mem );
				globalQP_A = new DenseMatrix(nC, nV, nV, A_mem );
				globalQP_A->doFreeMemory();
			}
		}

		/* Create output vectors and assign pointers to them. */
		allocateOutputs( nlhs,plhs, nV,nC );

		/* Call qpOASES. */
		init(	nV,nC,
				globalQP_H,g,globalQP_A,
				lb,ub,lbA,ubA,
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
			mexErrMsgTxt( "ERROR (qpOASES): Invalid number of output arguments!\nType 'help qpOASES_sequence' for further information." );

		if ( ( nrhs < 6 ) || ( nrhs > 7 ) )
			mexErrMsgTxt( "ERROR (qpOASES): Invalid number of input arguments!\nType 'help qpOASES_sequence' for further information." );

		/* has QP been initialised? */
		if ( globalQP == 0 )
			mexErrMsgTxt( "ERROR (qpOASES): QP sequence needs to be initialised first!" );


		/* Check inputs dimensions and assign pointers to inputs. */
		nV = globalQP->getNV( );
		nC = globalQP->getNC( );

		if ( smartDimensionCheck( &g,nV,1, BT_FALSE,prhs,1 ) != SUCCESSFUL_RETURN )
			return;

		if ( smartDimensionCheck( &lb,nV,1, BT_TRUE,prhs,2 ) != SUCCESSFUL_RETURN )
			return;

		if ( smartDimensionCheck( &ub,nV,1, BT_TRUE,prhs,3 ) != SUCCESSFUL_RETURN )
			return;

		if ( smartDimensionCheck( &lbA,nC,1, BT_TRUE,prhs,4 ) != SUCCESSFUL_RETURN )
			return;

		if ( smartDimensionCheck( &ubA,nC,1, BT_TRUE,prhs,5 ) != SUCCESSFUL_RETURN )
			return;

		/* default value for nWSR */
		int nWSRin = 5*(nV+nC);

		/* Check whether options are specified .*/
		if ( nrhs == 7 )
			if ( ( !mxIsEmpty( prhs[6] ) ) && ( mxIsStruct( prhs[6] ) ) )
				setupOptions( &options,prhs[6],nWSRin );

		/* Create output vectors and assign pointers to them. */
		allocateOutputs( nlhs,plhs, nV,nC );

		/* call qpOASES */
		hotstart(	g,
					lb,ub,lbA,ubA,
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
			mexErrMsgTxt( "ERROR (qpOASES): Invalid number of output arguments!\nType 'help qpOASES_sequence' for further information." );

		if ( nrhs != 1 )
			mexErrMsgTxt( "ERROR (qpOASES): Invalid number of input arguments!\nType 'help qpOASES_sequence' for further information." );

		/* Cleanup global QProblem instance. */
		deleteGlobalQProblemInstance( );
		deleteGlobalQProblemMatrices( );

		return;
	}
}

/*
 *	end of file
 */
