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
 *	\file interfaces/matlab/qpOASES_sequenceVM.cpp
 *	\author Hans Joachim Ferreau, Aude Perrin
 *	\version 3.0beta
 *	\date 2007-2011
 *
 *	Interface for Matlab(R) that enables to call qpOASES as a MEX function
 *  (variant for QPs with varying matrices).
 *
 */



#include <qpOASES/SQProblem.hpp>


using namespace qpOASES;

#include <qpOASES_matlab_utils.cpp>


/* global pointer to QP object */
static SQProblem* globalSQP = 0;
static SymmetricMatrix* globalSQP_H = 0;
static Matrix* globalSQP_A = 0;
static long* globalSQP_Hdiag = 0;


/*
 *	a l l o c a t e G l o b a l S Q P r o b l e m I n s t a n c e
 */
void allocateGlobalSQProblemInstance(	int nV, int nC, Options* options
										)
{
	globalSQP = new SQProblem( nV,nC );
	globalSQP->setOptions( *options );

	return;
}


/*
 *	d e l e t e G l o b a l S Q P r o b l e m I n s t a n c e
 */
void deleteGlobalSQProblemInstance( )
{
	if ( globalSQP != 0 )
	{
		delete globalSQP;
		globalSQP = 0;
	}

	return;
}


/*
 *	d e l e t e G l o b a l S Q P r o b l e m M a t r i c e s
 */
void deleteGlobalSQProblemMatrices( )
{
	if ( globalSQP_H != 0 )
	{
		delete globalSQP_H;
		globalSQP_H = 0;
	}

	if ( globalSQP_A != 0 )
	{
		delete globalSQP_A;
		globalSQP_A = 0;
	}

	if ( globalSQP_Hdiag != 0 )
	{
		delete[] globalSQP_Hdiag;
		globalSQP_Hdiag = 0;
	}

	return;
}


/*
 *	i n i t V M
 */
void initVM(	int nV, int nC,
				SymmetricMatrix *H, real_t* g, Matrix *A,
				const real_t* const lb, const real_t* const ub, const real_t* const lbA, const real_t* const ubA,
				int nWSR, const real_t* const x0, Options* options,
				int nOutputs, mxArray* plhs[]
				)
{
	/* 1) Setup initial QP. */
	allocateGlobalSQProblemInstance( nV,nC,options );

	/* 2) Solve initial QP. */
	returnValue returnvalue;

	if ( x0 == 0 )
		returnvalue = globalSQP->init( H,g,A,lb,ub,lbA,ubA, nWSR,0 );
	else
		returnvalue = globalSQP->init( H,g,A,lb,ub,lbA,ubA, nWSR,0, x0,0,0,0 );

	/* 3) Assign lhs arguments. */
	obtainOutputs(	0,globalSQP,returnvalue,nWSR,
					nOutputs,plhs,0,0 );

	return;
}


/*
 *	h o t s t a r t V M
 */
void hotstartVM(	SymmetricMatrix *H, real_t* g, Matrix *A,
					const real_t* const lb, const real_t* const ub, const real_t* const lbA, const real_t* const ubA,
					int nWSR, Options* options,
					int nOutputs, mxArray* plhs[]
					)
{
	/* 1) Solve QP. */
	globalSQP->setOptions( *options );
	returnValue returnvalue = globalSQP->hotstart( H,g,A,lb,ub,lbA,ubA, nWSR,0 );

	/* 2) Assign lhs arguments. */
	obtainOutputs(	0,globalSQP,returnvalue,nWSR,
					nOutputs,plhs,0,0 );

	return;
}



/*
 *	m e x F u n c t i o n
 */
void mexFunction( int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[] )
{
	unsigned int i, j;

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
	if ( ( nrhs < 8 ) || ( nrhs > 10 ) )
		if ( nrhs != 1 )
		mexErrMsgTxt( "ERROR (qpOASES): Invalid number of input arguments!\nType 'help qpOASES_sequenceVM' for further information." );

	/* 2) Ensure that first input is a string (and if so, read it). */
	if ( mxIsChar( prhs[0] ) != 1 )
		mexErrMsgTxt( "ERROR (qpOASES): First input argument must be a string!" );

	typeString = (char*) mxGetPr( prhs[0] );


	/* II) SELECT RESPECTIVE QPOASES FUNCTION CALL: */
	/* 1) Init (without or with initial guess for primal solution) OR
	 * 2) Hotstart. */
	if ( ( strcmp( typeString,"i" ) == 0 ) || ( strcmp( typeString,"I" ) == 0 ) ||
		 ( strcmp( typeString,"h" ) == 0 ) || ( strcmp( typeString,"H" ) == 0 ) )
	{
		/* consistency checks */
		if ( ( nlhs < 1 ) || ( nlhs > 5 ) )
			mexErrMsgTxt( "ERROR (qpOASES): Invalid number of output arguments!\nType 'help qpOASES_sequenceVM' for further information." );

		if ( ( nrhs < 8 ) || ( nrhs > 10 ) )
			mexErrMsgTxt( "ERROR (qpOASES): Invalid number of input arguments!\nType 'help qpOASES_sequenceVM' for further information." );

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
		if ( ( ( strcmp( typeString,"i" ) == 0 ) ) || ( strcmp( typeString,"I" ) == 0 ) )
		{
			if ( nrhs > 8 )
			{
				if ( smartDimensionCheck( &x0,nV,1, BT_TRUE,prhs,8 ) != SUCCESSFUL_RETURN )
					return;

				if ( nrhs > 9 )
					if ( ( !mxIsEmpty( prhs[9] ) ) && ( mxIsStruct( prhs[9] ) ) )
						setupOptions( &options,prhs[9],nWSRin );
			}
		}
		else /* hotstart */
		{
			if ( nrhs > 9 )
				mexErrMsgTxt( "ERROR (qpOASES): Invalid number of input arguments!\nType 'help qpOASES_sequenceVM' for further information." );

			if ( nrhs > 8 )
				if ( ( !mxIsEmpty( prhs[8] ) ) && ( mxIsStruct( prhs[8] ) ) )
					setupOptions( &options,prhs[8],nWSRin );
		}

		deleteGlobalSQProblemMatrices( );

		/* check for sparsity */
		if ( mxIsSparse( prhs[1] ) != 0 )
		{
			long *ir = (long *)mxGetIr(prhs[1]);
			long *jc = (long *)mxGetJc(prhs[1]);
			real_t *v = (real_t*)mxGetPr(prhs[1]);
			// mind pointer offsets due to 1-based indexing in Matlab
			SymSparseMat *sH;
			globalSQP_H = sH = new SymSparseMat(nV, nV, ir, jc, v);
			globalSQP_Hdiag = sH->createDiagInfo();
		}
		else
		{
			H_for = (real_t*) mxGetPr( prhs[1] );
			H_mem = new real_t[nV*nV];
			for( int i=0; i<nV*nV; ++i )
				H_mem[i] = H_for[i];
			globalSQP_H = new SymDenseMat( nV, nV, nV, H_mem );
			globalSQP_H->doFreeMemory();
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
				globalSQP_A = new SparseMatrix(nC, nV, ir, jc, v);
			}
			else
			{
				/* Convert constraint matrix A from FORTRAN to C style
				* (not necessary for H as it should be symmetric!). */
				A_for = (real_t*) mxGetPr( prhs[3] );
				A_mem = new real_t[nC*nV];
				convertFortranToC( A_for,nV,nC, A_mem );
				globalSQP_A = new DenseMatrix(nC, nV, nV, A_mem );
				globalSQP_A->doFreeMemory();
			}
		}

		/* Create output vectors and assign pointers to them. */
		allocateOutputs( nlhs,plhs, nV,nC );

		/* Call qpOASES */
		if ( ( ( strcmp( typeString,"i" ) == 0 ) ) || ( strcmp( typeString,"I" ) == 0 ) )
		{
			deleteGlobalSQProblemInstance( );

			initVM(	nV,nC,
					globalSQP_H,g,globalSQP_A,
					lb,ub,lbA,ubA,
					nWSRin,x0,&options,
					nlhs,plhs
					);
		}
		else /* hotstart */
		{
			if ( globalSQP == 0 )
				mexErrMsgTxt( "ERROR (qpOASES): QP needs to be initialised first!" );

			if ( ( (int)nV != globalSQP->getNV( ) ) || ( (int)nC != globalSQP->getNC( ) ) )
				mexErrMsgTxt( "ERROR (qpOASES): QP dimensions must be constant during a sequence!" );

			/* QUICK HACK TO BE REMOVED! */
			globalSQP->resetMatrixPointers( );

			hotstartVM(	globalSQP_H,g,globalSQP_A,
						lb,ub,lbA,ubA,
						nWSRin,&options,
						nlhs,plhs
						);
		}

		return;
	}

	/* 3) Cleanup. */
	if ( ( strcmp( typeString,"c" ) == 0 ) || ( strcmp( typeString,"C" ) == 0 ) )
	{
		/* consistency checks */
		if ( nlhs != 0 )
			mexErrMsgTxt( "ERROR (qpOASES): Invalid number of output arguments!\nType 'help qpOASES_sequenceVM' for further information." );

		if ( nrhs != 1 )
			mexErrMsgTxt( "ERROR (qpOASES): Invalid number of input arguments!\nType 'help qpOASES_sequenceVM' for further information." );

		/* Cleanup global SQProblem instance. */
		deleteGlobalSQProblemInstance( );
		deleteGlobalSQProblemMatrices( );
		return;
	}
}


/*
 *	end of file
 */
