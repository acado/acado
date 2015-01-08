#define S_FUNCTION_NAME   sfunction_robot
#define S_FUNCTION_LEVEL  2

#define MDL_START


#ifdef __cplusplus
extern "C" {
#endif

#include "acado_common.h"
#include "acado_auxiliary_functions.h"
#include "simstruc.h"

ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;


static void mdlInitializeSizes (SimStruct *S)
{
    /* Specify the number of continuous and discrete states */
    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

    /* Specify the number of intput ports */
    if ( !ssSetNumInputPorts(S, 2) )
        return;

    /* Specify the number of output ports */
    if ( !ssSetNumOutputPorts(S, 2) )
        return;

    /* Specify the number of parameters */
    ssSetNumSFcnParams(S, 10);
    if ( ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S) )
        return;

    /* Specify dimension information for the input ports */
    ssSetInputPortVectorDimension(S, 0, ACADO_NX);
    ssSetInputPortVectorDimension(S, 1, ACADO_NY);

    /* Specify dimension information for the output ports */
    ssSetOutputPortVectorDimension(S, 0, ACADO_NU );
    ssSetOutputPortVectorDimension(S, 1, 1 );

    /* Specify the direct feedthrough status */
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetInputPortDirectFeedThrough(S, 1, 1);

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
    double SAMPLINGTIME = mxGetScalar( ssGetSFcnParam(S, 0) );
    
    ssSetSampleTime(S, 0, SAMPLINGTIME);
    ssSetOffsetTime(S, 0, 0.0);
}


static void mdlStart(SimStruct *S)
{
    int i, j, k;

    InputRealPtrsType in_ref;
    double *xInit, *uInit, *Smat, *SNmat, *numSteps, *lbV, *ubV, *lbAV, *ubAV;

    /* get inputs and perform feedback step */
    in_ref = ssGetInputPortRealSignalPtrs(S, 1);

    xInit = mxGetPr( ssGetSFcnParam(S, 1) );
    uInit = mxGetPr( ssGetSFcnParam(S, 2) );

    for( i=0; i < ACADO_N+1; ++i ) {
        for( j=0; j < ACADO_NX; ++j ) acadoVariables.x[i*ACADO_NX+j] = xInit[j];
    }
    for( i=0; i < ACADO_N; ++i ) {
        for( j=0; j < ACADO_NU; ++j ) acadoVariables.u[i*ACADO_NU+j] = uInit[j];
    }

    for( i=0; i < ACADO_N; ++i ) {
        for( j=0; j < ACADO_NY; ++j ) acadoVariables.y[i*ACADO_NY+j] = (double)(*in_ref[j]);
    }
    for( i=0; i < ACADO_NYN; ++i ) acadoVariables.yN[i] = (double)(*in_ref[i]);
    
    Smat = mxGetPr( ssGetSFcnParam(S, 3) );
    SNmat = mxGetPr( ssGetSFcnParam(S, 4) );
    numSteps = mxGetPr( ssGetSFcnParam(S, 5) );
    
    for( i = 0; i < (ACADO_NYN); ++i )  {
        for( j = 0; j < ACADO_NYN; ++j ) {
            acadoVariables.WN[i*ACADO_NYN+j] = SNmat[i*ACADO_NYN+j];
        }
    }
    for( i = 0; i < (ACADO_NY); ++i )  {
        for( j = 0; j < ACADO_NY; ++j ) {
            acadoVariables.W[i*ACADO_NY+j] = Smat[i*ACADO_NY+j];
        }
    }
    
    lbV = mxGetPr( ssGetSFcnParam(S, 6) );
    ubV = mxGetPr( ssGetSFcnParam(S, 7) );
    lbAV = mxGetPr( ssGetSFcnParam(S, 8) );
    ubAV = mxGetPr( ssGetSFcnParam(S, 9) );
    
    for( i=0; i < ACADO_N; ++i ) {
        for( j=0; j < ACADO_NU; ++j ) acadoVariables.lbValues[i*ACADO_NU+j] = lbV[j];
        for( j=0; j < ACADO_NU; ++j ) acadoVariables.ubValues[i*ACADO_NU+j] = ubV[j];
        
        for( j=0; j < ACADO_NU; ++j ) acadoVariables.lbAValues[i*ACADO_NU+j] = lbAV[j];
        for( j=0; j < ACADO_NU; ++j ) acadoVariables.ubAValues[i*ACADO_NU+j] = ubAV[j];
    }

    preparationStep( );
}


static void mdlOutputs(SimStruct *S, int_T tid)
{
    int i, j, iter, error;
    double measurement[ACADO_NX];
    timer t;
    double timeFdb, timePrep;

    InputRealPtrsType in_x, in_ref;
    real_t *out_u0, *out_kktTol;

    /* get inputs and perform feedback step */
    in_x    = ssGetInputPortRealSignalPtrs(S, 0);
    in_ref = ssGetInputPortRealSignalPtrs(S, 1);
    
    for( i=0; i < ACADO_NX; ++i ) acadoVariables.x0[i] = (double)(*in_x[i]);

    for( i=0; i < ACADO_N-1; ++i ) {
        for( j=0; j < ACADO_NY; ++j ) acadoVariables.y[i*ACADO_NY+j] = acadoVariables.y[ACADO_NY+i*ACADO_NY+j];
    }
    for( i=0; i < ACADO_NYN; ++i ) acadoVariables.y[(ACADO_N-1)*ACADO_NY+i] = acadoVariables.yN[i];
    for( i=0; i < ACADO_NYN; ++i ) acadoVariables.yN[i] = (double)(*in_ref[i]);
    
        
    tic( &t );
    error = feedbackStep( );
    timeFdb = toc( &t );
    
    tic( &t );
    preparationStep( );
    timePrep = toc( &t );
    
    printf("NMPC step (error value: %d):\n", error);
    printf("Timing RTI iteration:   %.3g ms   +   %.3g ms   =   %.3g ms \n", timeFdb*1e3, timePrep*1e3, (timeFdb+timePrep)*1e3);
    printf("---------------------------------------------------------------\n");

    /* return outputs and prepare next iteration */
    out_u0     = ssGetOutputPortRealSignal(S, 0);
    out_kktTol = ssGetOutputPortRealSignal(S, 1);

    for( i=0; i < ACADO_NU; ++i ) out_u0[i] = acadoVariables.u[i];
    out_kktTol[0] = getKKT( );
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
