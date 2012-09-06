/*
 *    This file is part of ACADO Toolkit.
 *
 *    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
 *    Copyright (C) 2008-2009 by Boris Houska and Hans Joachim Ferreau, K.U.Leuven.
 *    Developed within the Optimization in Engineering Center (OPTEC) under
 *    supervision of Moritz Diehl. All rights reserved.
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


#define S_FUNCTION_NAME   sfunction
#define S_FUNCTION_LEVEL  2

#define MDL_START


#ifdef __cplusplus
extern "C" {
#endif

#include "acado.h"
#include "auxiliary_functions.c"
#include "simstruc.h"

ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;
Vars   vars;
Params params;

#define NX           6
#define NU           2
#define N            20
#define SAMPLINGTIME 0.1


static void mdlInitializeSizes (SimStruct *S)
{
    /* Specify the number of continuous and discrete states */
    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

    /* Specify the number of intput ports */
    if ( !ssSetNumInputPorts(S, 3) )
        return;

    /* Specify the number of output ports */
    if ( !ssSetNumOutputPorts(S, 2) )
        return;

// ssSetNumSFcnParams(S, 2)

    /* Specify dimension information for the input ports */
    ssSetInputPortVectorDimension(S, 0, NX);
    ssSetInputPortVectorDimension(S, 1, NX*N);
    ssSetInputPortVectorDimension(S, 2, NU*N);

    /* Specify dimension information for the output ports */
    ssSetOutputPortVectorDimension(S, 0, NU );
    ssSetOutputPortVectorDimension(S, 1, 1 );

    /* Specify the direct feedthrough status */
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetInputPortDirectFeedThrough(S, 1, 1);
    ssSetInputPortDirectFeedThrough(S, 2, 1);

    /* One sample time */
    ssSetNumSampleTimes(S, 1);
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

#endif /* MATLAB_MEX_FILE */


static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, SAMPLINGTIME);
    ssSetOffsetTime(S, 0, 0.0);
}


static void mdlStart(SimStruct *S)
{
    int i;
    double measurement[ NX ];

    for( i = 0; i < NX  ; ++i )  acadoVariables.x[i] = 0.0;
    for( i = 0; i < NU*N; ++i )  acadoVariables.u[i] = 0.0;

    for( i = 0; i < NX*N; ++i )  acadoVariables.xRef[i]  =  0.0;
    for( i = 0; i < NU*N; ++i )  acadoVariables.uRef[i]  =  0.0;
    
    preparationStep( );
}


static void mdlOutputs(SimStruct *S, int_T tid)
{
    int i;
    double measurement[NX];

    InputRealPtrsType in_x, in_xRef, in_uRef;
    real_t *out_u0, *out_kktTol;

    /* get inputs and perform feedback step */
    in_x    = ssGetInputPortRealSignalPtrs(S, 0);
    in_xRef = ssGetInputPortRealSignalPtrs(S, 1);
    in_uRef = ssGetInputPortRealSignalPtrs(S, 2);

    for( i = 0; i < NX; ++i )  measurement[i] = (*in_x)[i];
    for( i = 0; i < NX*N; ++i )  acadoVariables.xRef[i] = (*in_xRef)[i];
    for( i = 0; i < NU*N; ++i )  acadoVariables.uRef[i] = (*in_uRef)[i];

    feedbackStep( measurement );


    /* return outputs and prepare next iteration */
    out_u0     = ssGetOutputPortRealSignal(S, 0);
    out_kktTol = ssGetOutputPortRealSignal(S, 1);

    for( i = 0; i < NU; ++i )  out_u0[i] = acadoVariables.u[i];
    out_kktTol[0] = getKKT( );

    preparationStep( );
}


static void mdlTerminate(SimStruct *S)
{
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
 *    end of file
 */
