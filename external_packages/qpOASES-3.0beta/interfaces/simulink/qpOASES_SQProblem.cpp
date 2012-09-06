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
 *	\file interfaces/simulink/qpOASES_SQProblem.cpp
 *	\author Aude Perrin, Hans Joachim Ferreau
 *	\version 3.0beta
 *	\date 2007-2011
 *
 *	Interface for Simulink(R) that enables to call qpOASES as a S function
 *  (variant for QPs with varying matrices).
 *
 */


#include <stdlib.h>

#include <qpOASES/SQProblem.hpp>


#ifdef __cplusplus
extern "C" {
#endif


#define S_FUNCTION_NAME   qpOASES_SQProblem		/**< Name of the S function. */
#define S_FUNCTION_LEVEL  2						/**< S function level. */

#define MDL_START								/**< Activate call to mdlStart. */

#include "simstruc.h"


/* SETTINGS: */
#define SAMPLINGTIME    0.1						/**< Sampling time. */
#define NCONTROLINPUTS  2						/**< Number of control inputs. */
#define NWSR            10						/**< Maximum number of working set recalculations. */


static void mdlInitializeSizes (SimStruct *S)   /* Init sizes array */
{
	int nU = NCONTROLINPUTS;

	/* Specify the number of continuous and discrete states */
	ssSetNumContStates(S, 0);
	ssSetNumDiscStates(S, 0);

	/* Specify the number of intput ports */
	if ( !ssSetNumInputPorts(S, 7) )
		return;

	/* Specify the number of output ports */
	if ( !ssSetNumOutputPorts(S, 4) )
		return;

	/* Specify dimension information for the input ports */
	ssSetInputPortVectorDimension(S, 0, DYNAMICALLY_SIZED);
	ssSetInputPortVectorDimension(S, 1, DYNAMICALLY_SIZED);
	ssSetInputPortVectorDimension(S, 2, DYNAMICALLY_SIZED);
	ssSetInputPortVectorDimension(S, 3, DYNAMICALLY_SIZED);
	ssSetInputPortVectorDimension(S, 4, DYNAMICALLY_SIZED);
	ssSetInputPortVectorDimension(S, 5, DYNAMICALLY_SIZED);
	ssSetInputPortVectorDimension(S, 6, DYNAMICALLY_SIZED);

	/* Specify dimension information for the output ports */
	ssSetOutputPortVectorDimension(S, 0, 1 );
	ssSetOutputPortVectorDimension(S, 1, nU );
	ssSetOutputPortVectorDimension(S, 2, 1 );
	ssSetOutputPortVectorDimension(S, 3, 1 );

	/* Specify the direct feedthrough status */
	ssSetInputPortDirectFeedThrough(S, 0, 1);
	ssSetInputPortDirectFeedThrough(S, 1, 1);
	ssSetInputPortDirectFeedThrough(S, 2, 1);
	ssSetInputPortDirectFeedThrough(S, 3, 1);
	ssSetInputPortDirectFeedThrough(S, 4, 1);
	ssSetInputPortDirectFeedThrough(S, 5, 1);
	ssSetInputPortDirectFeedThrough(S, 6, 1);

	/* One sample time */
	ssSetNumSampleTimes(S, 1);


	/* global variables:
     * 0: problem
     * 1: H
     * 2: g
     * 3: A
     * 4: lb
     * 5: ub
     * 6: lbA
     * 7: ubA
     * 8: count
     */

	/* Specify the size of the block's pointer work vector */
    ssSetNumPWork(S, 9);
}


#if defined(MATLAB_MEX_FILE)

#define MDL_SET_INPUT_PORT_DIMENSION_INFO
#define MDL_SET_OUTPUT_PORT_DIMENSION_INFO

static void mdlSetInputPortDimensionInfo(SimStruct *S, int_T port, const DimsInfo_T *dimsInfo)
{
	if ( !ssSetInputPortDimensionInfo(S, port, dimsInfo) )
		return;
}

static void mdlSetOutputPortDimensionInfo(SimStruct *S, int_T port, const DimsInfo_T *dimsInfo)
{
	if ( !ssSetOutputPortDimensionInfo(S, port, dimsInfo) )
		return;
}

#endif


static void mdlInitializeSampleTimes(SimStruct *S)
{
	ssSetSampleTime(S, 0, SAMPLINGTIME);
	ssSetOffsetTime(S, 0, 0.0);
}


static void mdlStart(SimStruct *S)
{
	#ifndef __DSPACE__
	using namespace qpOASES;
	#endif

	int nU = NCONTROLINPUTS;
	int size_H, size_g, size_A, size_lb, size_ub, size_lbA, size_ubA;
	int nV, nC;

	SQProblem* problem;
	real_t* count;


	/* get block inputs dimensions */
	size_H   = ssGetInputPortWidth(S, 0);
	size_g   = ssGetInputPortWidth(S, 1);
	size_A   = ssGetInputPortWidth(S, 2);
	size_lb  = ssGetInputPortWidth(S, 3);
	size_ub  = ssGetInputPortWidth(S, 4);
	size_lbA = ssGetInputPortWidth(S, 5);
	size_ubA = ssGetInputPortWidth(S, 6);


	/* dimension checks */
	nV = size_g;
	nC = (int) ( ((real_t) size_A) / ((real_t) nV) );

	if ( nV == 0 )
	{
		#ifndef __DSPACE__
		mexErrMsgTxt( "ERROR (qpOASES): Dimension mismatch!" );
		#endif
		return;
	}

	if ( size_H != nV*nV )
	{
		#ifndef __DSPACE__
		mexErrMsgTxt( "ERROR (qpOASES): Dimension mismatch!" );
		#endif
		return;
	}

	if ( nU > nV )
	{
		#ifndef __DSPACE__
		mexErrMsgTxt( "ERROR (qpOASES): Dimension mismatch!" );
		#endif
		return;
	}

	if ( size_lb != nV )
	{
		#ifndef __DSPACE__
		mexErrMsgTxt( "ERROR (qpOASES): Dimension mismatch!" );
		#endif
		return;
	}

	if ( size_ub != nV )
	{
		#ifndef __DSPACE__
		mexErrMsgTxt( "ERROR (qpOASES): Dimension mismatch!" );
		#endif
		return;
	}

	if ( size_lbA != nC )
	{
		#ifndef __DSPACE__
		mexErrMsgTxt( "ERROR (qpOASES): Dimension mismatch!" );
		#endif
		return;
	}

	if ( size_ubA != nC )
	{
		#ifndef __DSPACE__
		mexErrMsgTxt( "ERROR (qpOASES): Dimension mismatch!" );
		#endif
		return;
	}


	/* allocate QProblem object */
	problem = new SQProblem( nV,nC );
	if ( problem == 0 )
	{
		#ifndef __DSPACE__
		mexErrMsgTxt( "ERROR (qpOASES): Unable to create QProblem object!" );
		#endif
		return;
	}

	#ifndef __DEBUG__
	problem->setPrintLevel( PL_LOW );
	#endif
	#ifdef __SUPPRESSANYOUTPUT__
	problem->setPrintLevel( PL_NONE );
	#endif
	#ifdef __DSPACE__
	problem->setPrintLevel( PL_NONE );
	#endif

	ssGetPWork(S)[0] = (void *) problem;

	/* allocate memory for QP data ... */
	ssGetPWork(S)[1] = (void *) calloc( size_H, sizeof(real_t) );	/* H */
	ssGetPWork(S)[2] = (void *) calloc( size_g, sizeof(real_t) );	/* g */
	ssGetPWork(S)[3] = (void *) calloc( size_A, sizeof(real_t) );	/* A */
	ssGetPWork(S)[4] = (void *) calloc( size_lb, sizeof(real_t) );	/* lb */
	ssGetPWork(S)[5] = (void *) calloc( size_ub, sizeof(real_t) );	/* ub */
	ssGetPWork(S)[6] = (void *) calloc( size_lbA, sizeof(real_t) );	/* lbA */
	ssGetPWork(S)[7] = (void *) calloc( size_ubA, sizeof(real_t) );	/* ubA */
	ssGetPWork(S)[8] = (void *) calloc( 1, sizeof(real_t) ); /* count */

	/* reset counter */
	count = (real_t *) ssGetPWork(S)[8];
	count[0] = 0.0;
}



static void mdlOutputs(SimStruct *S, int_T tid)
{
	#ifndef __DSPACE__
	using namespace qpOASES;
	#endif

	int i;
	int nV, nC, status;

	int nWSR = NWSR;
	int nU   = NCONTROLINPUTS;

	InputRealPtrsType in_H, in_g, in_A, in_lb, in_ub, in_lbA, in_ubA;

	SQProblem* problem;
	real_t *H, *g, *A, *lb, *ub, *lbA, *ubA, *count;

	real_t *xOpt;

	real_T *out_objVal, *out_xOpt, *out_status, *out_nWSR;


	/* get pointers to block inputs ... */
	in_H   = ssGetInputPortRealSignalPtrs(S, 0);
	in_g   = ssGetInputPortRealSignalPtrs(S, 1);
	in_A   = ssGetInputPortRealSignalPtrs(S, 2);
	in_lb  = ssGetInputPortRealSignalPtrs(S, 3);
	in_ub  = ssGetInputPortRealSignalPtrs(S, 4);
	in_lbA = ssGetInputPortRealSignalPtrs(S, 5);
	in_ubA = ssGetInputPortRealSignalPtrs(S, 6);


	/* ... and to the QP data */
	problem = (SQProblem *) ssGetPWork(S)[0];

	H = (real_t *) ssGetPWork(S)[1];
	g = (real_t *) ssGetPWork(S)[2];
	A = (real_t *) ssGetPWork(S)[3];
	lb = (real_t *) ssGetPWork(S)[4];
	ub = (real_t *) ssGetPWork(S)[5];
	lbA = (real_t *) ssGetPWork(S)[6];
	ubA = (real_t *) ssGetPWork(S)[7];

	count = (real_t *) ssGetPWork(S)[8];


	/* setup QP data */
	nV = ssGetInputPortWidth(S, 1); /* nV = size_g */
	nC = (int) ( ((real_t) ssGetInputPortWidth(S, 2)) / ((real_t) nV) ); /* nC = size_A / size_g */

	for ( i=0; i<nV*nV; ++i )
		H[i] = (*in_H)[i];

	for ( i=0; i<nC*nV; ++i )
		A[i] = (*in_A)[i];

	for ( i=0; i<nV; ++i )
	{
		g[i] = (*in_g)[i];
		lb[i] = (*in_lb)[i];
		ub[i] = (*in_ub)[i];
	}

	for ( i=0; i<nC; ++i )
	{
		lbA[i] = (*in_lbA)[i];
		ubA[i] = (*in_ubA)[i];
	}

	xOpt = new real_t[nV];

	if ( count[0] == 0 )
	{
		/* initialise and solve first QP */
		status = problem->init( H,g,A,lb,ub,lbA,ubA, nWSR,0 );
		ssGetPWork(S)[0] = ( void* ) problem;
		problem->getPrimalSolution( xOpt );
	}
	else
	{
		/* solve neighbouring QP using hotstart technique */
		status = problem->hotstart( H,g,A,lb,ub,lbA,ubA, nWSR,0 );
		if ( ( status != SUCCESSFUL_RETURN ) && ( status != RET_MAX_NWSR_REACHED ) )
		{
			/* if an error occurs, reset problem data structures and initialise again */
			problem->reset( );
			problem->init( H,g,A,lb,ub,lbA,ubA, nWSR,0 );
		}
		else
		{
	        /* otherwise obtain optimal solution */
	        problem->getPrimalSolution( xOpt );
		}
	}

	/* generate block output: status information ... */
	out_objVal = ssGetOutputPortRealSignal(S, 0);
	out_xOpt   = ssGetOutputPortRealSignal(S, 1);
	out_status = ssGetOutputPortRealSignal(S, 2);
	out_nWSR   = ssGetOutputPortRealSignal(S, 3);

	out_objVal[0] = ((real_T) problem->getObjVal( ));

	for ( i=0; i<nU; ++i )
		out_xOpt[i] = ((real_T) xOpt[i]);

	switch ( status )
	{
		case SUCCESSFUL_RETURN:
			out_status[0] = 0.0;
			break;

		case RET_MAX_NWSR_REACHED:
			out_status[0] = 1.0;
			break;

		case RET_INIT_FAILED_INFEASIBILITY:
		case RET_HOTSTART_STOPPED_INFEASIBILITY:
			out_status[0] = -2.0;
			break;
		
		case RET_INIT_FAILED_UNBOUNDEDNESS:
		case RET_HOTSTART_STOPPED_UNBOUNDEDNESS:
			out_status[0] = -3.0;
			break;

		default:
			out_status[0] = -1.0;
			break;
	}

	out_nWSR[0] = ((real_T) nWSR);

	/* increase counter */
	count[0] = count[0] + 1;

	delete[] xOpt;
}


static void mdlTerminate(SimStruct *S)
{
	#ifndef __DSPACE__
	using namespace qpOASES;
	#endif

	/* reset global message handler */
	getGlobalMessageHandler( )->reset( );

	int i;
	for ( i=0; i<9; ++i )
	{
		if ( ssGetPWork(S)[i] != 0 )
			free( ssGetPWork(S)[i] );
	}
}


#ifdef  MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif


#ifdef __cplusplus
}
#endif


/*
 *	end of file
 */
