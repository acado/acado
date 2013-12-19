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


 /**
 *    \file integrator/ACADOintegrators.cpp
 *    \author David Ariens, Boris Houska, Hans Joachim Ferreau, Niels Haverbeke
 *    \date 2009-2010
 */

#include <acado_integrators.hpp>
#include <acado/utils/matlab_acado_utils.hpp>


USING_NAMESPACE_ACADO


#include "model_include.hpp"

//#include "stdafx.h"


// global pointer to Matlab function handle,
// if dynamic model is given like that
mxArray* ModelFcn_f = NULL;
DifferentialEquation* modelFcn = NULL;
mxArray* ModelFcn_jac = NULL;

mxArray* ModelFcnT  = NULL;
mxArray* ModelFcnX  = NULL;
mxArray* ModelFcnXA = NULL;
mxArray* ModelFcnU  = NULL;
mxArray* ModelFcnP  = NULL;
mxArray* ModelFcnW  = NULL;
mxArray* ModelFcnDX = NULL;

unsigned int modelFcnNX  = 0;
unsigned int modelFcnNXA = 0;
unsigned int modelFcnNU  = 0;
unsigned int modelFcnNP  = 0;
unsigned int modelFcnNW  = 0;
unsigned int modelFcnNDX = 0;

unsigned int jacobianNumber = -1;
double* f_store 			= NULL;
double* J_store				= NULL;

void clearAllGlobals( )
{
	// first, clear all static counters within the toolkit
	clearAllStaticCounters( );

	// second, clear all global variables of the interface
	if ( modelFcn != NULL )
	{
		modelFcn = NULL;
		delete modelFcn;
	}

	if ( f_store != NULL )
	{
		f_store = NULL;
		delete f_store;
	}

	if ( J_store != NULL )
	{
		J_store = NULL;
		delete J_store;
	}

	if ( ModelFcn_f != NULL )
	{
		mxDestroyArray( ModelFcn_f );
		ModelFcn_f = NULL;
	}

	if ( ModelFcnT != NULL )
	{
		mxDestroyArray( ModelFcnT );
		ModelFcnT = NULL;
	}

	if ( ModelFcnX != NULL )
	{
		mxDestroyArray( ModelFcnX );
		ModelFcnX = NULL;
	}

	if ( ModelFcnXA != NULL )
	{
		mxDestroyArray( ModelFcnXA );
		ModelFcnXA = NULL;
	}

	if ( ModelFcnU != NULL )
	{
		mxDestroyArray( ModelFcnU );
		ModelFcnU = NULL;
	}

	if ( ModelFcnP != NULL )
	{
		mxDestroyArray( ModelFcnP );
		ModelFcnP = NULL;
	}

	if ( ModelFcnW != NULL )
	{
		mxDestroyArray( ModelFcnW );
		ModelFcnW = NULL;
	}

	if ( ModelFcnDX != NULL )
	{
		mxDestroyArray( ModelFcnDX );
		ModelFcnDX = NULL;
	}

	if ( ModelFcn_jac != NULL )
	{
		mxDestroyArray( ModelFcn_jac );
		ModelFcn_jac = NULL;
	}


	modelFcnNX  = 0;
	modelFcnNXA = 0;
	modelFcnNU  = 0;
	modelFcnNP  = 0;
	modelFcnNW  = 0;
	modelFcnNDX = 0;
	jacobianNumber = -1;
}


// generic dynamic model function for evaluating
// Matlab function handles from C
void genericODE( double* x, double* f, void *userData  )
{
	unsigned int i;

	double* tt = mxGetPr( ModelFcnT );
	tt[0] = x[0];

	double* xx = mxGetPr( ModelFcnX );
	for( i=0; i<modelFcnNX; ++i )
		xx[i] = x[i+1];

	double* uu = mxGetPr( ModelFcnU );
	for( i=0; i<modelFcnNU; ++i )
		uu[i] = x[i+1+modelFcnNX];

	double* pp = mxGetPr( ModelFcnP );
	for( i=0; i<modelFcnNP; ++i )
		pp[i] = x[i+1+modelFcnNX+modelFcnNU];

	double* ww = mxGetPr( ModelFcnW );
	for( i=0; i<modelFcnNW; ++i )
		ww[i] = x[i+1+modelFcnNX+modelFcnNU+modelFcnNP];

	mxArray* FF = NULL;

	mxArray* argIn[]  = { ModelFcn_f,ModelFcnT,ModelFcnX,ModelFcnU,ModelFcnP,ModelFcnW };
	mxArray* argOut[] = { FF };

	mexCallMATLAB( 1,argOut, 6,argIn,"generic_ode" );


	double* ff = mxGetPr( *argOut );

	for( i=0; i<modelFcnNX; ++i )
		f[i] = ff[i];

	mxDestroyArray( *argOut );

}


void genericJacobian ( int number, double* x, double* seed, double* f, double* df, void *userData )
{
	unsigned int i, j;
	double* ff;
	double* J;


	if (J_store == NULL){

		J_store = (double*) calloc ((modelFcnNX+modelFcnNXA+modelFcnNU+modelFcnNP+modelFcnNW)*(modelFcnNX+modelFcnNXA),sizeof(double));
		f_store = (double*) calloc (modelFcnNX,sizeof(double));

	}


	if ( (int) jacobianNumber == number){

		//mexPrintf("\n SAME JACOBIAN --- Number:%d", number);
		//ff = f_store;
		J = J_store;
		ff = f_store;

		for( i=0; i<modelFcnNX+modelFcnNXA; ++i ) {
			df[i] = 0;
			f[i] = 0; //F[i]
			for (j=0; j < modelFcnNX+modelFcnNXA+modelFcnNU+modelFcnNP+modelFcnNW; ++j){
				df[i] += J[(j*(modelFcnNX+modelFcnNXA))+i]*seed[j+1];  //  J+1 since we do not use nt here
				//mexPrintf ("\n jac %d, %d = %f -- seed: %f -- function : %f", i, j, J[(j*(modelFcnNX+modelFcnNXA))+i], seed[j+1], f[i]);
			}
		}


		for( i=0; i<modelFcnNX; ++i ){
			f[i] = ff[i];
			//mexPrintf("\n function EVAL %d --> %f", i, f[i]);
		}


	}else{
		//mexPrintf("\n DIFFERENT JACOBIAN --- Number:%d", number);

		jacobianNumber = number;   // Store current number in global.


		// Set the values to pass to generic_jacobian
		double* tt = mxGetPr( ModelFcnT );
		tt[0] = x[0];

		double* xx = mxGetPr( ModelFcnX );
		for( i=0; i<modelFcnNX; ++i )
			xx[i] = x[i+1];

		double* uu = mxGetPr( ModelFcnU );
		for( i=0; i<modelFcnNU; ++i )
			uu[i] = x[i+1+modelFcnNX];

		double* pp = mxGetPr( ModelFcnP );
		for( i=0; i<modelFcnNP; ++i )
			pp[i] = x[i+1+modelFcnNX+modelFcnNU];

		double* ww = mxGetPr( ModelFcnW );
		for( i=0; i<modelFcnNW; ++i )
			ww[i] = x[i+1+modelFcnNX+modelFcnNU+modelFcnNP];

		// Execute generic_jacobian
		mxArray* FF = NULL;
		mxArray* argIn[]  = { ModelFcn_jac,ModelFcnT,ModelFcnX,ModelFcnU,ModelFcnP,ModelFcnW };
		mxArray* argOut[] = { FF };

		//mexPrintf("\n EVAL JACOBIAN - JAC");
		mexCallMATLAB( 1,argOut, 6,argIn,"generic_jacobian" );

		// Examine output
		unsigned int rowLen = mxGetM(*argOut);
		unsigned int colLen = mxGetN(*argOut);


		if (rowLen != modelFcnNX+modelFcnNXA)
		{
			mexErrMsgTxt( "ERROR (ACADOintegrator): Jacobian matrix rows do not match (should be modelFcnNX+modelFcnNXA). " );
		}

		if (colLen != modelFcnNX+modelFcnNXA+modelFcnNU+modelFcnNP+modelFcnNW)
		{
			mexErrMsgTxt( "ERROR (ACADOintegrator): Jacobian matrix columns do not match (should be modelFcnNX+modelFcnNXA+modelFcnNU+modelFcnNP+modelFcnNW). " );
		}

		J = mxGetPr( *argOut );

		memcpy(J_store, J, (modelFcnNX+modelFcnNXA+modelFcnNU+modelFcnNP+modelFcnNW)*(modelFcnNX+modelFcnNXA) * sizeof ( double ));



		for( i=0; i<modelFcnNX+modelFcnNXA; ++i ) {
			df[i] = 0;
			f[i] = 0; //F[i]
			for (j=0; j < modelFcnNX+modelFcnNXA+modelFcnNU+modelFcnNP+modelFcnNW; ++j){
				df[i] += J[(j*(modelFcnNX+modelFcnNXA))+i]*seed[j+1];  //  J+1 since we do not use nt here
				//mexPrintf ("\n jac %d, %d = %f -- seed: %f -- function : %f", i, j, J[(j*(modelFcnNX+modelFcnNXA))+i], seed[j+1], f[i]);
			}
		}




		mxArray* FF2 = NULL;
		mxArray* argIn2[]  = { ModelFcn_f,ModelFcnT,ModelFcnX,ModelFcnU,ModelFcnP,ModelFcnW };
		mxArray* argOut2[] = { FF2 };

		mexCallMATLAB( 1,argOut2, 6,argIn2,"generic_ode" );


		ff = mxGetPr( *argOut2 );
		memcpy(f_store, ff, (modelFcnNX) * sizeof ( double ));


		for( i=0; i<modelFcnNX; ++i ){
			f[i] = ff[i];
			//mexPrintf("\n function EVAL %d --> %f", i, f[i]);
		}

		mxDestroyArray( *argOut );
		mxDestroyArray( *argOut2 );


	}



	//mexPrintf("\n\n");
}

// generic dynamic model function for evaluating
// Matlab function handles from C
void genericDAE( double* x, double* f, void *userData )
{
	unsigned int i;

	double* tt = mxGetPr( ModelFcnT );
	tt[0] = x[0];

	double* xx = mxGetPr( ModelFcnX );
	for( i=0; i<modelFcnNX; ++i )
		xx[i] = x[i+1];

	double* xxa = mxGetPr( ModelFcnXA );
	for( i=0; i<modelFcnNXA; ++i )
		xxa[i] = x[i+1+modelFcnNX];

	double* uu = mxGetPr( ModelFcnU );
	for( i=0; i<modelFcnNU; ++i )
		uu[i] = x[i+1+modelFcnNX+modelFcnNXA];

	double* pp = mxGetPr( ModelFcnP );
	for( i=0; i<modelFcnNP; ++i )
		pp[i] = x[i+1+modelFcnNX+modelFcnNXA+modelFcnNU];

	double* ww = mxGetPr( ModelFcnW );
	for( i=0; i<modelFcnNW; ++i )
		ww[i] = x[i+1+modelFcnNX+modelFcnNXA+modelFcnNU+modelFcnNP];
    
	// call matlab model via function handle and stack results
	mxArray* argIn_f[]  = { ModelFcn_f,ModelFcnT,ModelFcnX,ModelFcnXA,ModelFcnU,ModelFcnP,ModelFcnW }; 

	mxArray* FF = NULL;
	double*  ff = NULL;
	mxArray* argOut[] = { FF };
    
    ////printf("Before DAE call \n");

    mexCallMATLAB( 1,argOut, 7,argIn_f,"generic_dae" );
	ff = mxGetPr( *argOut );
	for( i=0; i<modelFcnNX+modelFcnNXA; ++i )
		f[i] = ff[i];

    //printf("after DAE call\n");
    
	mxDestroyArray( *argOut );
}


unsigned int determineNumberOfAlgebraicStates( )
{
	return 0;
}


void plotErrorMessage(	returnValue returnvalue,
						double* PrintLevel_
						)
{
	if ( ( PrintLevel_ != NULL ) && ( PrintLevel_[0] <= 0.0 ) )
	{
		switch ( returnvalue )
		{
			case RET_MISSING_INPUTS:
				mexPrintf( "ERROR (ACADOintegrator): The integrator misses some inputs.\n" );
	
			case RET_TO_SMALL_OR_NEGATIVE_TIME_INTERVAL:
				mexPrintf( "ERROR (ACADOintegrator): The integration interval is too small or negative.\n" );
	
			case RET_MAX_NUMBER_OF_STEPS_EXCEEDED:
				mexPrintf( "ERROR (ACADOintegrator): The maximum number of integration steps is exceeded.\n" );
	
			case RET_INPUT_HAS_WRONG_DIMENSION:
				mexPrintf( "ERROR (ACADOintegrator): At least one of the inputs has a wrong dimension.\n" );
	
			case SUCCESSFUL_RETURN:
				return;
	
			default:
				mexPrintf( "ERROR (ACADOintegrator): Unsuccessful return from the integrator.\n");
		}
	}
}

/**
 * Allocate differential equation, called in mexFunction()
 */
DifferentialEquation* allocateDifferentialEquation(	char *modelName,		// name of the model
													char *integratorName	// name of the integrator
													)
{
	DifferentialEquation* f = NULL;

	if( strcmp(integratorName,"DiscretizedODE") == 0 )
		f = new DiscretizedDifferentialEquation;
	else
		f = new DifferentialEquation;

	if ( modelName != NULL )
	{
		// function name is given as string
		void (*fcn)( DifferentialEquation* );
		fcn = allocateFunctionPointer( modelName );

		if( fcn == 0 )
		{
			delete f;
			clearAllGlobals( );
			mexErrMsgTxt( "ERROR (ACADOintegrator): A model with the specified name does not exist.\n" );
		}

		fcn( f );
	}

	return f;
}


//allocateJacobian


/**
 * Allocate integrator, called in mexFunction()
 */
Integrator* allocateIntegrator(	char *integratorName,		// name of the integrator
								DifferentialEquation* f		// allocated differential equation
								)
{
	Integrator* integrator = NULL;

	if ( f == NULL )
		mexErrMsgTxt( "bug!" );

	if( strcmp(integratorName,"RK12") == 0 )
		integrator = new IntegratorRK12( *f );

	if( strcmp(integratorName,"RK23") == 0 )
		integrator = new IntegratorRK23( *f );

	if( strcmp(integratorName,"RK45") == 0 )
		integrator = new IntegratorRK45( *f );

	if( strcmp(integratorName,"RK78") == 0 )
		integrator = new IntegratorRK78( *f );

	if( strcmp(integratorName,"DiscretizedODE") == 0 )
		integrator = new IntegratorDiscretizedODE( *f );

	if( strcmp(integratorName,"BDF") == 0 )
		integrator = new IntegratorBDF( *f );

	return integrator;
}


// --------------------------------------------------------------------------

              // ********************************//
              //                                 //
              //      m e x F u n c t i o n      //
              // ________________________________//

/**
 * mexFunction is not a routine you call. Rather, mexFunction is the name of a
 * function in C (subroutine in Fortran) that you must write in every MEX-file.
 * When you invoke a MEX-function, MATLAB software finds and loads the corresponding
 * MEX-file of the same name. MATLAB then searches for a symbol named mexFunction
 * within the MEX-file. If it finds one, it calls the MEX-function using the address
 * of the mexFunction symbol. If MATLAB cannot find a routine named mexFunction
 * inside the MEX-file, it issues an error message.
 *
 *
 * @param  int nlhs         Number of expected output mxArrays
 * @param  mxArray *plhs[]  Array of pointers to the expected output mxArrays
 * @param  int nrhs         Number of input mxArrays
 * @param  mxArray *prhs[]  Array of pointers to the input mxArrays. These mxArrays
 *                          are read only and should not be modified by your MEX-file.
 *
 */

void mexFunction(	int nlhs,       mxArray *plhs[],
					int nrhs, const mxArray *prhs[]
					)
{
	unsigned int i,j;


	// INPUT ARGUMENTS:
	// ---------------------------------------------------------------

	// integrator settings struct
	const mxArray *IntegratorSettings = NULL;

	// start point for differential states
	double *xStart = NULL;

	// initial value for algebraic states (only for DAEs)
	double *xaStart = NULL;

	// start of the integration interval
	double *tStart = NULL;
	int tStartIdx = -1;

	// end of the integration interval
	double *tEnd = NULL;
	int tEndIdx = -1;

	// is dynamic model an ODE (DAE otherwise)?
	BooleanType isODE = BT_TRUE;

	// end value of algebraic states is to be returned to plhs[xaEndIdx] (= -1 for none)
	int xaEndIdx = -1;

	// an output struct is to be returned to plhs[outputIdx] (= -1 for none)
	int outputIdx = -1;


	// SETTINGS:
	// ---------------------------------------------------------------

	// name of the model
	mxArray   *ModelName = NULL;
	char      *modelName = NULL;  // (if specified as string, otherwise global modelFcn is used)

	// name of the integrator
	mxArray   *IntegratorName = NULL;
	char      *integratorName = NULL;

	// tolerance of the integrator
	mxArray   *Tol = NULL;
	double    *tol = NULL;

	// tolerance of the integrator
	mxArray   *aTol = NULL;
	double    *atol = NULL;

	// maximum number of allowed steps
	mxArray   *MaxNumStep = NULL;
	double    *maxNumStep = NULL;

	// min step size
	mxArray   *MinStep = NULL;
	double    *minStep = NULL;
    
    // initial step size
	mxArray   *InitialStep = NULL;
	double    *initialStep = NULL;
    
	// max step size
	mxArray   *MaxStep = NULL;
	double    *maxStep = NULL;

	// tuning
	mxArray   *StepTuning = NULL;
	double    *stepTuning = NULL;

	// linear algebra solver
	mxArray   *LinearAlgebraSolver = NULL;
	char      *linearAlgebraSolver = NULL;

	// controls
	mxArray   *U = NULL;
	double    *u = NULL;

	// parameters
	mxArray   *P = NULL;
	double    *p = NULL;

	// disturbances
	mxArray   *W = NULL;
	double    *w = NULL;

	// initial value for xdot (only for DAEs)
	mxArray   *DxInit = NULL;
	double    *dxInit = NULL;

	// sensitivity mode
	mxArray   *SensitivityMode = NULL;
	char      *sensitivityMode = NULL;

	// backward seed
	mxArray   *Bseed = NULL;
	double    *bseed = NULL;

	// forward seed (w.r.t. x)
	mxArray   *Dx = NULL;
	double    *dx = NULL;

	// forward seed (w.r.t. p)
	mxArray   *Dp = NULL;
	double    *dp = NULL;

	// forward seed (w.r.t. u)
	mxArray   *Du = NULL;
	double    *du = NULL;

	// forward seed (w.r.t. w)
	mxArray   *Dw = NULL;
	double    *dw = NULL;

	// backward seed (2nd order)
	mxArray   *Bseed2 = NULL;
	double    *bseed2 = NULL;

	// forward seed (w.r.t. w) (2nd order)
	mxArray   *Dx2 = NULL;
	double    *dx2 = NULL;

	// forward seed (w.r.t. w) (2nd order)
	mxArray   *Dp2 = NULL;
	double    *dp2 = NULL;

	// forward seed (w.r.t. w) (2nd order)
	mxArray   *Du2 = NULL;
	double    *du2 = NULL;

	// forward seed (w.r.t. w) (2nd order)
	mxArray   *Dw2 = NULL;
	double    *dw2 = NULL;

	// defines if (error) messages shall be printed or not
	mxArray   *PrintLevel_ = NULL;
	double    *printLevel_  = NULL;

	// array containing the indices of the states whose trajectory shall be plotted
	mxArray   *PlotXTrajectory = NULL;
	mxArray   *PlotXaTrajectory = NULL;

	// defines if subplots are to be used
	mxArray   *UseSubplots = NULL;

	// tolerance of the integrator
	mxArray   *Jacobian = NULL;
	double    *jacobian = NULL;

    // OUTPUTS:
    // -----------------------------------------------------------------

	// end values of differential and algebraic states
	double  *xEnd = NULL;
	double  *xaEnd = NULL;

	// integrator status
	mxArray *Status = NULL;
	double  *statusPtr = NULL;

	// number of integrator steps
	mxArray *NumberOfSteps = NULL;
	double  *numberOfSteps = NULL;

	// number of rejected integrator steps
	mxArray *NumberOfRejectedSteps = NULL;
	double  *numberOfRejectedSteps = NULL;
    
    // step size
	mxArray *GetStepSize = NULL;
	double  *getStepSize = NULL;
    
    // corrector tollerance
	mxArray *CorrectorTolerance = NULL;
	double  *correctorTolerance = NULL;

	// x trajectory values 
	mxArray *XTrajectory = NULL;
	double  *xTrajectory = NULL;

	// xa trajectory values 
	mxArray *XaTrajectory = NULL;
	double  *xaTrajectory = NULL;

	// sensitivity w.r.t everything (in forward mode only)
	mxArray *JJ = NULL;
	double  *jj = NULL;

	// sensitivity w.r.t the states (in backward mode only)
	mxArray *Jx = NULL;
	double  *jx = NULL;

	// sensitivity w.r.t the parameters (in backward mode only)
	mxArray *Jp = NULL;
	double  *jp = NULL;

	// sensitivity w.r.t the controls (in backward mode only)
	mxArray *Ju = NULL;
	double  *ju = NULL;

	// sensitivity w.r.t the disturbances (in backward mode only)
	mxArray *Jw = NULL;
	double  *jw = NULL;

	// second order sensitivity w.r.t everything (in forward mode 2 only)
	mxArray *JJ2 = NULL;
	double  *jj2 = NULL;

	// second order sensitivity w.r.t the states (in backward mode only)
	mxArray *Jx2 = NULL;
	double  *jx2 = NULL;

	// second order sensitivity w.r.t the parameters (in backward mode only)
	mxArray *Jp2 = NULL;
	double  *jp2 = NULL;

	// second order sensitivity w.r.t the controls (in backward mode only)
	mxArray *Ju2 = NULL;
	double  *ju2 = NULL;

	// second order sensitivity w.r.t the disturbances (in backward mode only)
	mxArray *Jw2 = NULL;
	double  *jw2 = NULL;

	// second order sensitivity w.r.t the disturbances (in backward mode only)
	mxArray *StorageResolution = NULL;
	double  *storageResolution = NULL;


	// DIMENSIONS:
	// ----------------------------  -------------------------------------
	unsigned int nx     = 0;  // number of differential states
	unsigned int nxa    = 0;  // number of algebraic states
	unsigned int nu     = 0;  // number of controls
	unsigned int np     = 0;  // number of parameters
	unsigned int nw     = 0;  // number of disturbances
	unsigned int nbDir  = 0;  // number of backward directions
	unsigned int nfDir  = 0;  // number of forward directions
	unsigned int nbDir2 = 0;  // number of 2nd order backward directions
	unsigned int nfDir2 = 0;  // number of 2nd order forward directions


	// I) CONSISTENCY CHECKS:
	// ----------------------

	if ( ( nrhs < 4 ) || ( nrhs > 5 ) )
		mexErrMsgTxt( "ERROR (ACADOintegrator): Invalid number of input arguments!\n    Type 'help ACADOintegrators' for further information." );

	if ( ( nlhs < 1 ) || ( nlhs > 3 ) )
		mexErrMsgTxt( "ERROR (ACADOintegrator): Invalid number of output arguments!\n    Type 'help ACADOintegrators' for further information." );


	// II. READING THE INPUT ARGUMENTS:
	// --------------------------------

	if( !mxIsEmpty( prhs[0]) )
	{
		IntegratorSettings = prhs[0];
		if ( !mxIsStruct( IntegratorSettings ) )
			mexErrMsgTxt( "ERROR (ACADOintegrator): No integrator settings defined!" );
	}
	else
		mexErrMsgTxt( "ERROR (ACADOintegrator): No integrator settings defined!" );

	if( !mxIsEmpty( prhs[1]) )
	{
		xStart = mxGetPr( prhs[1] );
		nx     = mxGetM( prhs[1] );

		if ( mxGetN( prhs[1] ) != 1 )
			mexErrMsgTxt( "ERROR (ACADOintegrator): Start value for differential state needs to be given as column vector." );
	}
	else
		mexErrMsgTxt( "ERROR (ACADOintegrator): No initial value specified." );

	if ( nrhs == 4 )
	{
		// ODE system
		// An ODE system is supposed when ACADOintegrators() is called with 4 arguments
		isODE = BT_TRUE;
		tStartIdx = 2;
		tEndIdx   = 3;
	}
	else
	{
		// DAE system, xaStart optional
		isODE = BT_FALSE;
		tStartIdx = 3;
		tEndIdx   = 4;

		if( !mxIsEmpty( prhs[2]) )
		{
			xaStart = mxGetPr( prhs[2] );
			nxa     = mxGetM( prhs[2] );
	
			if ( mxGetN( prhs[2] ) != 1 )
				mexErrMsgTxt( "ERROR (ACADOintegrator): Start value for algebraic state needs to be given as column vector." );
		}
	}

	if( !mxIsEmpty( prhs[tStartIdx]) )
	{
		tStart = mxGetPr( prhs[tStartIdx] );

		if ( isScalar( prhs[tStartIdx] ) == 0 )
			mexErrMsgTxt( "ERROR (ACADOintegrator): Start time needs to be a scalar." );
	}
	else
		mexErrMsgTxt( "ERROR (ACADOintegrator): No start time defined." );


	if( !mxIsEmpty( prhs[tEndIdx]) )
	{
		tEnd = mxGetPr( prhs[tEndIdx] );

		if ( isScalar( prhs[tEndIdx] ) == 0 )
			mexErrMsgTxt( "ERROR (ACADOintegrator): End time needs to be a scalar." );
	}
	else
		mexErrMsgTxt( "ERROR (ACADOintegrator): No end time defined." );


	// III. READING THE SETTINGS STRUCT:
	// ---------------------------------

	ModelName           = mxGetField( IntegratorSettings,0,"Model" );
	IntegratorName      = mxGetField( IntegratorSettings,0,"Integrator" );
	Tol                 = mxGetField( IntegratorSettings,0,"Tolerance" );
	aTol                = mxGetField( IntegratorSettings,0,"AbsoluteTolerance" );
	MaxNumStep          = mxGetField( IntegratorSettings,0,"MaxNumberOfSteps" );
	MinStep             = mxGetField( IntegratorSettings,0,"MinimumStepSize" );
    InitialStep         = mxGetField( IntegratorSettings,0,"InitialStepSize" );
	MaxStep          	= mxGetField( IntegratorSettings,0,"MaximumStepSize" );
    CorrectorTolerance  = mxGetField( IntegratorSettings,0,"CorrectorTolerance" );
	StepTuning        	= mxGetField( IntegratorSettings,0,"StepSizeTuning" );
	LinearAlgebraSolver = mxGetField( IntegratorSettings,0,"LinearAlgebraSolver" );
	U                   = mxGetField( IntegratorSettings,0,"u" );
	P                   = mxGetField( IntegratorSettings,0,"p" );
	W                   = mxGetField( IntegratorSettings,0,"w" );
	DxInit              = mxGetField( IntegratorSettings,0,"dxInit" );
	SensitivityMode     = mxGetField( IntegratorSettings,0,"SensitivityMode" );
	Bseed               = mxGetField( IntegratorSettings,0,"mu" );
	Dx                  = mxGetField( IntegratorSettings,0,"lambdaX" );
	Du                  = mxGetField( IntegratorSettings,0,"lambdaU" );
	Dp                  = mxGetField( IntegratorSettings,0,"lambdaP" );
	Dw                  = mxGetField( IntegratorSettings,0,"lambdaW" );
	Bseed2              = mxGetField( IntegratorSettings,0,"mu2" );
	Dx2                 = mxGetField( IntegratorSettings,0,"lambda2X" );
	Du2                 = mxGetField( IntegratorSettings,0,"lambda2U" );
	Dp2                 = mxGetField( IntegratorSettings,0,"lambda2P" );
	Dw2                 = mxGetField( IntegratorSettings,0,"lambda2W" );
	PrintLevel_         = mxGetField( IntegratorSettings,0,"PrintLevel" );
	PlotXTrajectory     = mxGetField( IntegratorSettings,0,"PlotXTrajectory" );
	PlotXaTrajectory    = mxGetField( IntegratorSettings,0,"PlotXaTrajectory" );
	UseSubplots         = mxGetField( IntegratorSettings,0,"UseSubplots" );
	Jacobian         	= mxGetField( IntegratorSettings,0,"Jacobian" );
	StorageResolution	= mxGetField( IntegratorSettings,0,"StorageResolution" );

	if( !mxIsEmpty(IntegratorName) )
	{
		if ( !mxIsChar(IntegratorName) )
			mexErrMsgTxt( "ERROR (ACADOintegrator): No integrator specified." );

		integratorName = mxArrayToString( IntegratorName );
	}
	else
		mexErrMsgTxt( "ERROR (ACADOintegrator): No integrator specified." );

	if( !mxIsEmpty(MaxNumStep) )
	{
		maxNumStep = mxGetPr( MaxNumStep );

		if ( isScalar( MaxNumStep ) == 0 )
			mexErrMsgTxt( "ERROR (ACADOintegrator): Maximum number of steps needs to be a scalar." );
	}

	if( !mxIsEmpty(MinStep) )
	{
		minStep = mxGetPr( MinStep );

		if ( isScalar( MinStep ) == 0 )
			mexErrMsgTxt( "ERROR (ACADOintegrator): Minimum step size must be a scalar ." );
	}
    
    if( !mxIsEmpty(InitialStep) )
	{
		initialStep = mxGetPr( InitialStep );

		if ( isScalar( InitialStep ) == 0 )
			mexErrMsgTxt( "ERROR (ACADOintegrator): Initial step size must be a scalar ." );
	}
    
    if( !mxIsEmpty(CorrectorTolerance) )
	{
		correctorTolerance = mxGetPr( CorrectorTolerance );

		if ( isScalar( CorrectorTolerance ) == 0 )
			mexErrMsgTxt( "ERROR (ACADOintegrator): Corrector Tolerance must be a scalar ." );
	}
    
	if( !mxIsEmpty(MaxStep) )
	{
		maxStep = mxGetPr( MaxStep );

		if ( isScalar( MaxStep ) == 0 )
			mexErrMsgTxt( "ERROR (ACADOintegrator): Maximum step size must be a scalar ." );
	}

	if( !mxIsEmpty(StepTuning) )
	{
		stepTuning = mxGetPr( StepTuning );

		if ( isScalar( StepTuning ) == 0 )
			mexErrMsgTxt( "ERROR (ACADOintegrator): StepTuning size must be a scalar ." );
	}

	if( !mxIsEmpty(LinearAlgebraSolver) )
	{
		if ( !mxIsChar(LinearAlgebraSolver) )
			mexErrMsgTxt( "ERROR (ACADOintegrator): No linear algebra solver specified." );

		linearAlgebraSolver = mxArrayToString( LinearAlgebraSolver );
	}
	else
		mexErrMsgTxt( "ERROR (ACADOintegrator): No linear algebra solver specified." );

	if( !mxIsEmpty(Tol) )
	{
		tol = mxGetPr( Tol );

		if ( isScalar( Tol ) == 0 )
			mexErrMsgTxt( "ERROR (ACADOintegrator): Tolerance needs to be a scalar." );
	}

	if( !mxIsEmpty(aTol) )
	{
		atol = mxGetPr( aTol );

		if ( isScalar( aTol ) == 0 )
			mexErrMsgTxt( "ERROR (ACADOintegrator): Absolute Tolerance needs to be a scalar." );
	}

	if( !mxIsEmpty(U) )
	{
		u  = mxGetPr( U );
		nu = mxGetM( U );

		if ( mxGetN( U ) != 1 )
			mexErrMsgTxt( "ERROR (ACADOintegrator): Controls need to be given as column vector." );
	}

	if( !mxIsEmpty(P) )
	{
		p  = mxGetPr( P );
		np = mxGetM( P );

		if ( mxGetN( P ) != 1 )
			mexErrMsgTxt( "ERROR (ACADOintegrator): Parameters need to be given as column vector." );
	}

	if( !mxIsEmpty(W) )
	{
		w  = mxGetPr( W );
		nw = mxGetM( W );

		if ( mxGetN( W ) != 1 )
			mexErrMsgTxt( "ERROR (ACADOintegrator): Disturbances need to be given as column vector." );
	}

	if( !mxIsEmpty(DxInit) )
	{
		dxInit = mxGetPr( DxInit );
		if ( mxGetM( DxInit ) != nx )
		{
			clearAllGlobals( );
			mexErrMsgTxt( "ERROR (ACADOintegrator): Initial value for differential states derivatives has invalid dimensions." );
		}

		if ( mxGetN( DxInit ) != 1 )
		{
			clearAllGlobals( );
			mexErrMsgTxt( "ERROR (ACADOintegrator): Initial value for differential states derivatives needs to be given as column vector." );
		}
	}

	if( !mxIsEmpty(Bseed) )
	{
		bseed = mxGetPr( Bseed );

		if ( mxGetN( Bseed ) != nx )
			mexErrMsgTxt( "ERROR (ACADOintegrator): Backward seed has incompatible dimensions." );

		nbDir = mxGetM( Bseed );
	}

	if( !mxIsEmpty(Dx) )
	{
		dx = mxGetPr( Dx );

		if ( mxGetM( Dx ) != nx )
			mexErrMsgTxt( "ERROR (ACADOintegrator): Forward seed has incompatible dimensions." );

		nfDir = mxGetN( Dx );
	}

	if( !mxIsEmpty(Du) )
	{
		du = mxGetPr( Du );

		if ( mxGetM( Du ) != nx )
			mexErrMsgTxt( "ERROR (ACADOintegrator): Forward seed has incompatible dimensions." );

		if ( ( nfDir != 0 ) && ( nfDir != mxGetN( Du ) ) )
			mexErrMsgTxt( "ERROR (ACADOintegrator): Forward seed has incompatible dimensions." );

		nfDir = mxGetN( Du );
	}

	if( !mxIsEmpty(Dp) )
	{
		dp = mxGetPr( Dp );

		if ( mxGetM( Dp ) != nx )
			mexErrMsgTxt( "ERROR (ACADOintegrator): Forward seed has incompatible dimensions." );

		if ( ( nfDir != 0 ) && ( nfDir != mxGetN( Dp ) ) )
			mexErrMsgTxt( "ERROR (ACADOintegrator): Forward seed has incompatible dimensions." );

		nfDir = mxGetN( Dp );
	}

	if( !mxIsEmpty(Dw) )
	{
		dw = mxGetPr( Dw );

		if ( mxGetM( Dw ) != nx )
			mexErrMsgTxt( "ERROR (ACADOintegrator): Forward seed has incompatible dimensions." );

		if ( ( nfDir != 0 ) && ( nfDir != mxGetN( Dw ) ) )
			mexErrMsgTxt( "ERROR (ACADOintegrator): Forward seed has incompatible dimensions." );

		nfDir = mxGetN( Dw );
	}

	if( !mxIsEmpty(Bseed2) )
	{
		bseed2 = mxGetPr( Bseed2 );

		if ( mxGetN( Bseed2 ) != nx )
			mexErrMsgTxt( "ERROR (ACADOintegrator): Second backward seed has incompatible dimensions." );

		nbDir2 = mxGetM( Bseed2 );
	}

	if( !mxIsEmpty(Dx2) )
	{
		dx2 = mxGetPr( Dx2 );

		if ( mxGetM( Dx2 ) != nx )
			mexErrMsgTxt( "ERROR (ACADOintegrator): Second forward seed has incompatible dimensions." );

		nfDir2  = mxGetN( Dx2 );
	}

	if( !mxIsEmpty(Du2) )
	{
		du2 = mxGetPr( Du2 );

		if ( mxGetM( Du2 ) != nx )
			mexErrMsgTxt( "ERROR (ACADOintegrator): Second forward seed has incompatible dimensions." );

		if ( ( nfDir2 != 0 ) && ( nfDir2 != mxGetN( Du2 ) ) )
			mexErrMsgTxt( "ERROR (ACADOintegrator): Second forward seed has incompatible dimensions." );

		nfDir2  = mxGetN( Du2 );
	}

	if( !mxIsEmpty(Dp2) )
	{
		dp2 = mxGetPr( Dp2 );

		if ( mxGetM( Dp2 ) != nx )
			mexErrMsgTxt( "ERROR (ACADOintegrator): Second forward seed has incompatible dimensions." );

		if ( ( nfDir2 != 0 ) && ( nfDir2 != mxGetN( Dp2 ) ) )
			mexErrMsgTxt( "ERROR (ACADOintegrator): Second forward seed has incompatible dimensions." );

		nfDir2  = mxGetN( Dp2 );
	}

	if( !mxIsEmpty(Dw2) )
	{
		dw2 = mxGetPr( Dw2 );

		if ( mxGetM( Dw2 ) != nx )
			mexErrMsgTxt( "ERROR (ACADOintegrator): Second forward seed has incompatible dimensions." );

		if ( ( nfDir2 != 0 ) && ( nfDir2 != mxGetN( Dw2 ) ) )
			mexErrMsgTxt( "ERROR (ACADOintegrator): Second forward seed has incompatible dimensions." );

		nfDir2 = mxGetN( Dw2 );
	}

	if( !mxIsEmpty(PrintLevel_) )
		printLevel_ = mxGetPr( PrintLevel_ );


	// Set link to Jacobian
	if ( mxIsCell(Jacobian) )
	{
		if ( ( mxGetN( Jacobian ) == 1 ) && ( mxGetCell( Jacobian,0 ) != NULL ) )
		{
			ModelFcn_jac = mxDuplicateArray( mxGetCell( Jacobian,0 ) );

			if ( isFunctionHandle(ModelFcn_jac) == 0 )
				mexErrMsgTxt( "ERROR (ACADOintegrator): Jacobian: No valid dynamic model specified." );
		}else{
			mexErrMsgTxt( "ERROR (ACADOintegrator): Jacobian: Could not set Jacobian" );

		}
	}

	if( !mxIsEmpty(StorageResolution) )
	{
		storageResolution = mxGetPr( StorageResolution );

		if ( isScalar( StorageResolution ) == 0 )
			mexErrMsgTxt( "ERROR (ACADOintegrator): Maximum number of steps needs to be a scalar." );
	}



	DifferentialEquation* f;
    f = new DifferentialEquation();
    
    CFunction *cModel = NULL;
    

	if( !mxIsEmpty(ModelName) )
	{
		if ( mxGetM( ModelName ) != 1 )
			mexErrMsgTxt( "ERROR (ACADOintegrator): No valid dynamic model specified." );

		if ( mxIsChar(ModelName) )
		{
			modelName = mxArrayToString( ModelName );
			f = allocateDifferentialEquation( modelName,integratorName );

			if ( f == NULL )
				mexErrMsgTxt( "ERROR (ACADOintegrator): No valid dynamic model specified." );

			if ( f->getNumAlgebraicEquations( ) != (int) nxa )
				mexErrMsgTxt( "ERROR (ACADOintegrator): Start value for algebraic states has invalid dimension." );

			if ( f->getNumDynamicEquations( ) != (int) nx ){
                printf("Differentialstate dimension: %d , input dim: %d \n", f->getNumDynamicEquations( ), (int)nx );
                mexErrMsgTxt( "ERROR (ACADOintegrator): Start value for differential state has invalid dimension." );
            }
		}
		else
		{
			if ( mxIsCell(ModelName) )
			{
				// get f
				if ( ( mxGetN( ModelName ) > 0 ) && ( mxGetCell( ModelName,0 ) != NULL ) )
				{
					ModelFcn_f = mxDuplicateArray( mxGetCell( ModelName,0 ) );

					if ( isFunctionHandle(ModelFcn_f) == 0 )
						mexErrMsgTxt( "ERROR (ACADOintegrator): No valid dynamic model specified." );
				}
				else
					mexErrMsgTxt( "ERROR (ACADOintegrator): No valid dynamic model specified." );	

                

				// if dynamic model is given as Matlab function handle, allocate
				// dynamic model as C function with corresponding dimensions
		
				ModelFcnT  = mxCreateDoubleMatrix( 1,  1,mxREAL );
				ModelFcnX  = mxCreateDoubleMatrix( nx, 1,mxREAL );
				ModelFcnXA = mxCreateDoubleMatrix( nxa,1,mxREAL );
				ModelFcnDX = mxCreateDoubleMatrix( nx, 1,mxREAL );
				ModelFcnU  = mxCreateDoubleMatrix( nu, 1,mxREAL );
				ModelFcnP  = mxCreateDoubleMatrix( np, 1,mxREAL );
				ModelFcnW  = mxCreateDoubleMatrix( nw, 1,mxREAL );
		
				modelFcnNX  = nx;
				modelFcnNXA = nxa;
				modelFcnNDX = nx;
				modelFcnNP  = np;
				modelFcnNU  = nu;
				modelFcnNW  = nw;

                
                IntermediateState is( 1+modelFcnNX+modelFcnNXA+modelFcnNU+modelFcnNP+modelFcnNW);
                int j = 0;
                
                TIME t;
                is(j) = t; j++;
                
                if (modelFcnNX > 0){
                    DifferentialState x("",modelFcnNX,1);
                    for( i=0; i<modelFcnNX; ++i ){
                        is(j) = x(i); j++;
                    }
                }
                
                if (modelFcnNXA > 0){
                    AlgebraicState ax("",modelFcnNXA,1);
                    for( i=0; i<modelFcnNXA; ++i ){
                        is(j) = ax(i); j++;
                    }
                }
                
                if (modelFcnNU > 0){
                    Control u("",modelFcnNU,1);
                    for( i=0; i<modelFcnNU; ++i ){
                        is(j) = u(i); j++;
                    }
                }
                
                if (modelFcnNP > 0){
                    Parameter p("",modelFcnNP,1);
                    for( i=0; i<modelFcnNP; ++i ){
                        is(j) = p(i); j++;
                    }
                }
                
                if (modelFcnNW > 0){
                    Disturbance w("",modelFcnNW,1);
                    for( i=0; i<modelFcnNW; ++i ){
                        is(j) = w(i); j++;
                    }
                }

        
				//mexprintf ("nxn:%d nxa:%d np:%d nu:%d nw:%d -- j=%d\n", nx, nxa, np, nu, nw, j);


				if ( isODE == BT_TRUE )
				{
					if( !mxIsEmpty(Jacobian) )
					{
                        CFunction cLinkModel(nx, genericODE, genericJacobian, genericJacobian);
                        *f << cLinkModel(is);
					}
					else
					{
                        CFunction cLinkModel(nx, genericODE);
                        *f << cLinkModel(is);
					}
				}
				else
				{
                    CFunction cLinkModel(nx+nxa, genericDAE);
                    *f << cLinkModel(is);
				}
                
                
                
				//printf("after link model \n");
				
			}
			else
				mexErrMsgTxt( "ERROR (ACADOintegrator): No dynamic model specified." );
		}
	}
	else
		mexErrMsgTxt( "ERROR (ACADOintegrator): No dynamic model specified." );

	if( !mxIsEmpty(SensitivityMode) )
	{
		if ( !mxIsChar(SensitivityMode) )
		{
			delete f; delete cModel; clearAllGlobals( );
			mexErrMsgTxt( "ERROR (ACADOintegrator): No valid sensitivity mode specified." );
		}

		sensitivityMode = mxArrayToString( SensitivityMode );

		if ( ( modelName == NULL ) && ( strcmp(sensitivityMode,"AD_BACKWARD") == 0 ) )
		{
			delete f; delete cModel; clearAllGlobals( );
			mexErrMsgTxt( "ERROR (ACADOintegrator): Adjoint sensitivity generation via function handle not possible." );
		}
	}

    
	// set flags indicating of ODE/DAE is given and 
	// if output struct shall be setup
	if ( isODE == BT_TRUE )
	{
		if ( nlhs == 3 )
		{
			delete f; delete cModel; clearAllGlobals( );
			mexErrMsgTxt( "ERROR (ACADOintegrator): Invalid number of output arguments." );
		}

		if ( nlhs == 2 )
			outputIdx = 1;
	}
	else
	{
		if ( nlhs >= 2 )
			xaEndIdx = 1;

		if ( nlhs == 3 )
			outputIdx = 2;
	}


	// IV. CREATE OUTPUT VECTORS AND ASSIGN POINTERS:
	// ----------------------------------------------

	double status = 0.0;

	plhs[0] = mxCreateDoubleMatrix( nx,1,mxREAL );
	xEnd    = mxGetPr( plhs[0] );

	if ( xaEndIdx > 0 )
	{
		plhs[xaEndIdx] = mxCreateDoubleMatrix( nxa,1,mxREAL );
		xaEnd          = mxGetPr( plhs[xaEndIdx] );
	}

	if ( outputIdx > 0 )
	{
		// create outputs struct
		// DO NOT forget to document all changes done here within integratorsOutputs.m
		const char* outputFieldNames[] = {	"Status",
											"NumberOfSteps",
											"NumberOfRejectedSteps",
											"xTrajectory",
											"xaTrajectory",
											"J",
											"Jx",
											"Ju",
											"Jp",
											"Jw",
											"J2",
											"J2x",
											"J2u",
											"J2p",
											"J2w",
                                            "GetStepSize"
											};

		plhs[outputIdx] = mxCreateStructMatrix( 1,1,16,outputFieldNames );
	}


	// store trajectory either if output struct or if plot is desired
	BooleanType freezeTrajectory;

	if ( ( outputIdx > 0 ) || ( !mxIsEmpty( PlotXTrajectory ) ) || ( !mxIsEmpty( PlotXaTrajectory ) ) )
		freezeTrajectory = BT_TRUE;
	else
		freezeTrajectory = BT_FALSE;


	int *maxNumStepInt = 0;
	if ( !mxIsEmpty(MaxNumStep) )
	{
		maxNumStepInt = new int;
		maxNumStepInt[0] = (int) *maxNumStep;
	}


	// V. CALL THE INTEGRATION ROUTINE:
	// --------------------------------

	// allocate integrator object
	Integrator* integrator = allocateIntegrator( integratorName,f );

	if ( integrator == NULL )
	{
		delete maxNumStepInt; delete f; delete cModel; clearAllGlobals( );
		mexErrMsgTxt( "ERROR (ACADOintegrator): The specified integrator has not been found.\n");
	}

	clearAllStaticCounters( );


 	if( maxNumStep != 0 )
 		integrator->set( MAX_NUM_INTEGRATOR_STEPS,*maxNumStepInt );

    if( minStep != 0 )
        integrator->set( MIN_INTEGRATOR_STEPSIZE,*minStep );
    
    if( initialStep != 0 )
        integrator->set( INITIAL_INTEGRATOR_STEPSIZE,*initialStep );
            
    if( maxStep != 0 )
        integrator->set( MAX_INTEGRATOR_STEPSIZE,*maxStep );

 	if( tol != 0 )
 		integrator->set(INTEGRATOR_TOLERANCE,*tol );

 	if( atol != 0 )
 		integrator->set(ABSOLUTE_TOLERANCE,*atol );

    if( stepTuning != 0 )
        integrator->set( STEPSIZE_TUNING,*stepTuning );

    if( correctorTolerance != 0 )
        integrator->set( CORRECTOR_TOLERANCE,*correctorTolerance );
    
    

//	integrator->set( LINEAR_ALGEBRA_SOLVER,(int)LAS_HOUSEHOLDER_METHOD );
//	if( linearAlgebraSolver != 0 )
//	{
//		if ( strcmp(linearAlgebraSolver,"sparse") == 0 )
//		{
//			if ( strcmp(integratorName,"BDF") == 0 )
//			{
//				integrator->set( LINEAR_ALGEBRA_SOLVER,(int)LAS_SPARSE_CONJUGATE_GRADIENT_METHOD );
//			}
//			else
//			{
//				delete integrator; delete maxNumStepInt; delete f; delete cModel; clearAllGlobals( );
//				mexErrMsgTxt( "ERROR (ACADOintegrator): The specified integrator has not been found.\n");
//			}
//		}
//	}


	if( dxInit != NULL )
		integrator->setDxInitialization( dxInit );

	if( freezeTrajectory == BT_TRUE )
		integrator->freezeAll( );

    //printf("before integrate\n");

    Grid timeInterval( *tStart, *tEnd, (int) *storageResolution );
	returnValue returnvalue = integrator->integrate( timeInterval, xStart,xaStart,p,u,w );
    
    //printf("after integrate \n");
    
	if ( returnvalue != SUCCESSFUL_RETURN )
	{
		status = -1.0;
		plotErrorMessage( returnvalue,printLevel_ );
	}

	// assign end value for states
	//double* xEndFull = new double[nx+nxa];
    DVector xEnd_, xaEnd_;
	integrator->getX ( xEnd_  );
	integrator->getXA( xaEnd_ );

	for( i=0; i<nx; ++i )
		xEnd[i] = xEnd_(i);

	if ( xaEndIdx > 0 )
	{
		for( i=0; i<nxa; ++i )
			xaEnd[i] = xaEnd_(i);
	}

    //printf("after xaEnd, before sensitivities \n");

	// SENSITIVITIES:
	// -----------------------------------
	if ( ( !mxIsEmpty(SensitivityMode) ) && ( status >= 0.0 ) )
	{
        //printf("\n !! SensitivityMode is currently disabled !! \n\n");
		// forward mode
		if( strcmp(sensitivityMode,"AD_FORWARD") == 0  )
		{
			if ( ( dx == NULL ) && ( du == NULL ) && ( dp == NULL ) && ( dw == NULL ) )
			{
				delete integrator; delete maxNumStepInt; delete f; delete cModel; clearAllGlobals( );
				mexErrMsgTxt( "ERROR (ACADOintegrator): The forward seed is not defined." );
			}

			// setup seed and sensitvity matrices
			DMatrix D_x( nx,nfDir );
			DMatrix D_p( np,nfDir );
			DMatrix D_u( nu,nfDir );
			DMatrix D_w( nw,nfDir );

			if( dx != NULL ) {
                D_x = Eigen::Map<DMatrix>(dx,nx,nfDir);
                D_x.transposeInPlace();
            }
			else
				D_x.setZero();

			if( du != NULL ) {
                D_u = Eigen::Map<DMatrix>(du,nu,nfDir);
                D_u.transposeInPlace();
            }
			else
				D_u.setZero();

			if( dp != NULL ) {
                D_p = Eigen::Map<DMatrix>(dp,np,nfDir);
                D_p.transposeInPlace();
            }
			else
				D_p.setZero();

			if( dw != NULL ) {
                D_w = Eigen::Map<DMatrix>(dw,nw,nfDir);
                D_w.transposeInPlace();
            }
			else
				D_w.setZero();

			DMatrix J( nx+nxa,nfDir );

			// determine forward sensitivities

            int run1;

            for( run1 = 0; run1 < nfDir; run1++ ){

                DVector DXX = D_x.getCol( run1 );
                DVector DPP = D_p.getCol( run1 );
                DVector DUU = D_u.getCol( run1 );
                DVector DWW = D_w.getCol( run1 );

				integrator->setForwardSeed( 1, DXX, DPP, DUU, DWW );
				returnvalue = integrator->integrateSensitivities( );
				if( returnvalue != SUCCESSFUL_RETURN )
				{
					status = -1.0;
					plotErrorMessage( returnvalue,printLevel_ );
				}
				else
				{
                    DVector JJ(nx+nxa);
					integrator->getForwardSensitivities( JJ, 1 );
                    J.setCol( run1, JJ );
                }
            }

			// write sensitivity matrices into output struct (if given)
			if ( returnvalue == SUCCESSFUL_RETURN && outputIdx > 0 )
			{
				JJ = mxCreateDoubleMatrix( nx+nxa,nfDir,mxREAL );
				jj = mxGetPr( JJ );
                DMatrix jj_tmp = Eigen::Map<DMatrix>(jj, J.rows(), J.cols());  jj_tmp = J.transpose(); jj = jj_tmp.data();
				mxSetField( plhs[outputIdx],0,"J",JJ );
			}
		}

		// backward mode
		if( strcmp(sensitivityMode,"AD_BACKWARD") == 0 )
		{
			if( bseed == NULL )
			{
				delete integrator; delete maxNumStepInt; delete f; delete cModel; clearAllGlobals( );
				mexErrMsgTxt( "ERROR (ACADOintegrator): The backward seed is not defined." );
			}

			// setup seed and sensitvity matrices
			DMatrix xSeed( nbDir,nx );
            xSeed = Eigen::Map<DMatrix>(bseed,nbDir,nx);
            xSeed.transposeInPlace();

			DMatrix J_x( nbDir,nx+nxa );
			DMatrix J_u( nbDir,nu );
			DMatrix J_p( nbDir,np );
			DMatrix J_w( nbDir,nw );


            int run1;
            for( run1 = 0; run1 < nbDir; run1++ ){

                DVector XSEED = xSeed.getRow(run1);

				// determine backward sensitivities
				integrator->setBackwardSeed( 1, XSEED );
				returnvalue = integrator->integrateSensitivities( );
				if( returnvalue != SUCCESSFUL_RETURN )
				{
					status = -1.0;
					plotErrorMessage( returnvalue,printLevel_ );
				}
				else
				{
                    DVector JXX(nx+nxa), JPP(np), JUU(nu), JWW(nw);

					integrator->getBackwardSensitivities( JXX, JPP, JUU, JWW, 1 );

                    J_x.setRow( run1, JXX );
                    J_p.setRow( run1, JPP );
                    J_u.setRow( run1, JUU );
                    J_w.setRow( run1, JWW );
	            }
            }


            if( returnvalue == SUCCESSFUL_RETURN ){
				// write sensitivity matrices into output struct
				if ( outputIdx > 0 )
				{
					Jx = mxCreateDoubleMatrix( nbDir,nx+nxa,mxREAL );
					jx = mxGetPr( Jx );
                    DMatrix jx_tmp = Eigen::Map<DMatrix>(jx, J_x.rows(), J_x.cols());  jx_tmp = J_x.transpose(); jx = jx_tmp.data();
					mxSetField( plhs[outputIdx],0,"Jx",Jx );

					if ( nu > 0 )
					{
						Ju = mxCreateDoubleMatrix( nbDir,nu,mxREAL );
						ju = mxGetPr( Ju );
                        DMatrix ju_tmp = Eigen::Map<DMatrix>(ju, J_u.rows(), J_u.cols());  ju_tmp = J_u.transpose(); ju = ju_tmp.data();
						mxSetField( plhs[outputIdx],0,"Ju",Ju );
					}

					if( np > 0 )
					{
						Jp = mxCreateDoubleMatrix( nbDir,np,mxREAL );
						jp = mxGetPr( Jp );
                        DMatrix jp_tmp = Eigen::Map<DMatrix>(jp, J_p.rows(), J_p.cols());  jp_tmp = J_p.transpose(); jp = jp_tmp.data();
						mxSetField( plhs[outputIdx],0,"Jp",Jp );
					}

					if( nw > 0 )
					{
						Jw = mxCreateDoubleMatrix( nbDir,nw,mxREAL );
						jw = mxGetPr( Jw );
                        DMatrix jw_tmp = Eigen::Map<DMatrix>(jw, J_w.rows(), J_w.cols());  jw_tmp = J_w.transpose(); jw = jw_tmp.data();
						mxSetField( plhs[outputIdx],0,"Jw",Jw );
					}
				}
			}
		}


		// forward mode (2nd derivatives)
		if ( strcmp(sensitivityMode,"AD_FORWARD2") == 0  )
		{
			if ( ( dx == NULL ) && ( du == NULL ) && ( dp == NULL ) && ( dw == NULL ) )
			{
				delete integrator; delete maxNumStepInt; delete f; delete cModel; clearAllGlobals( );
				mexErrMsgTxt( "ERROR (integrator): The forward seed is not defined." );
			}

			if ( ( dx2 == NULL ) && ( du2 == NULL ) && ( dp2 == NULL ) && ( dw2 == NULL ) )
			{
				delete integrator; delete maxNumStepInt; delete f; delete cModel; clearAllGlobals( );
				mexErrMsgTxt( "ERROR (integrator): The second order forward seed is not defined." );
			}

			if ( nfDir > 1 )
			{
				delete integrator; delete maxNumStepInt; delete f; delete cModel; clearAllGlobals( );
				mexErrMsgTxt( "ERROR (integrator): More than one first order seed is not allowed in second order mode - please compute the required directions in a loop." );
			}

			// setup seed and sensitvity matrices (first order)
			DMatrix D_x( nx,nfDir );
			DMatrix D_p( np,nfDir );
			DMatrix D_u( nu,nfDir );
			DMatrix D_w( nw,nfDir );

			if( dx != NULL ) {
                D_x = Eigen::Map<DMatrix>(dx,nx,nfDir);
                D_x.transposeInPlace();
            }
			else
				D_x.setZero();

			if( du != NULL ) {
                D_u = Eigen::Map<DMatrix>(du,nu,nfDir);
                D_u.transposeInPlace();
            }
			else
				D_u.setZero();

			if( dp != NULL ) {
                D_p = Eigen::Map<DMatrix>(dp,np,nfDir);
                D_p.transposeInPlace();
            }
			else
				D_p.setZero();

			if( dw != NULL ) {
                D_w = Eigen::Map<DMatrix>(dw,nw,nfDir);
                D_w.transposeInPlace();
            }
			else
				D_w.setZero();

			DMatrix J( nx+nxa,nfDir );

			// setup seed and sensitvity matrices (second order)
			DMatrix D_x2( nx,nfDir2 );
			DMatrix D_p2( np,nfDir2 );
			DMatrix D_u2( nu,nfDir2 );
			DMatrix D_w2( nw,nfDir2 );

			if( dx != NULL ) {
                D_x2 = Eigen::Map<DMatrix>(dx2,nx,nfDir2);
                D_x2.transposeInPlace();
            }
			else
				D_x2.setZero();

			if( du != NULL ) {
                D_u2 = Eigen::Map<DMatrix>(du2,nu,nfDir2);
                D_u2.transposeInPlace();
            }
			else
				D_u2.setZero();

			if( dp2 != NULL ) {
                D_p2 = Eigen::Map<DMatrix>(dp2,np,nfDir2);
                D_p2.transposeInPlace();
            }
			else
				D_p2.setZero();

			if( dw2 != NULL ) {
                D_w2 = Eigen::Map<DMatrix>(dw2,nw,nfDir2);
                D_w2.transposeInPlace();
            }
			else
				D_w2.setZero();

			DMatrix J2( nx+nxa,nfDir2 );


            int run1, run2;

            for( run1 = 0; run1 < nfDir; run1++ ){

                DVector DXX = D_x.getCol( run1 );
                DVector DPP = D_p.getCol( run1 );
                DVector DUU = D_u.getCol( run1 );
                DVector DWW = D_w.getCol( run1 );

				integrator->setForwardSeed( 1, DXX, DPP, DUU, DWW );
				returnvalue = integrator->integrateSensitivities( );

				if( returnvalue != SUCCESSFUL_RETURN )
				{
					status = -1.0;
					plotErrorMessage( returnvalue,printLevel_ );
				}
				else
				{
                    DVector JJ(nx+nxa);
					integrator->getForwardSensitivities( JJ, 1 );
                    J.setCol( run1, JJ );
                }

                for( run2 = 0; run2 < nfDir2; run2++ ){

                    DVector DXX2 = D_x2.getCol( run2 );
                    DVector DPP2 = D_p2.getCol( run2 );
                    DVector DUU2 = D_u2.getCol( run2 );
                    DVector DWW2 = D_w2.getCol( run2 );

					// determine forward sensitivities
					integrator->setForwardSeed( 2, DXX2, DPP2, DUU2, DWW2 );
					returnvalue = integrator->integrateSensitivities( );

					if( returnvalue != SUCCESSFUL_RETURN )
					{
						status = -1.0;
						plotErrorMessage( returnvalue,printLevel_ );
					}
					else
					{
                        DVector JJ2(nx+nxa);
	    				integrator->getForwardSensitivities( JJ2, 2 );
                        J2.setCol( run2, JJ2 );
                    }
                }
            }

			// write sensitivity matrices into output struct (if given)
			if ( returnvalue == SUCCESSFUL_RETURN && outputIdx > 0 )
			{
				JJ = mxCreateDoubleMatrix( nx+nxa,nfDir,mxREAL );
				jj = mxGetPr( JJ );
                DMatrix jj_tmp = Eigen::Map<DMatrix>(jj, J.rows(), J.cols());  jj_tmp = J.transpose(); jj = jj_tmp.data();
				mxSetField( plhs[outputIdx],0,"J",JJ );

				JJ2 = mxCreateDoubleMatrix( nx+nxa,nfDir2,mxREAL );
				jj2 = mxGetPr( JJ2 );
                DMatrix jj2_tmp = Eigen::Map<DMatrix>(jj2, J2.rows(), J2.cols());  jj2_tmp = J2.transpose(); jj2 = jj2_tmp.data();
				mxSetField( plhs[outputIdx],0,"J2",JJ2 );
			}
		}


		// mixed forward-backward mode (2nd derivatives)
		if ( strcmp(sensitivityMode,"AD_FORWARD_BACKWARD") == 0 )
		{
			if ( ( dx == NULL ) && ( du == NULL ) && ( dp == NULL ) && ( dw == NULL ) )
			{
				delete integrator; delete maxNumStepInt; delete f; delete cModel; clearAllGlobals( );
				mexErrMsgTxt( "ERROR (integrator): The forward seed is not defined." );
			}

			if( bseed2 == NULL )
			{
				delete integrator; delete maxNumStepInt; delete f; delete cModel; clearAllGlobals( );
				mexErrMsgTxt( "ERROR (integrator): The second order backward seed is not defined." );
			}

			if( nfDir > 1 )
			{
				delete integrator; delete maxNumStepInt; delete f; delete cModel; clearAllGlobals( );
				mexErrMsgTxt( "ERROR (integrator): More than one first order seed is not allowed in second order mode - please compute the required directions in a loop." );
			}

			// setup seed and sensitvity matrices (first order)
			DMatrix D_x( nx,nfDir );
			DMatrix D_p( np,nfDir );
			DMatrix D_u( nu,nfDir );
			DMatrix D_w( nw,nfDir );

			if( dx != NULL ) {
                D_x = Eigen::Map<DMatrix>(dx,nx,nfDir);
                D_x.transposeInPlace();
            }
			else
				D_x.setZero();

			if( du != NULL ) {
                D_u = Eigen::Map<DMatrix>(du,nu,nfDir);
                D_u.transposeInPlace();
            }
			else
				D_u.setZero();

			if( dp != NULL ) {
                D_p = Eigen::Map<DMatrix>(dp,np,nfDir);
                D_p.transposeInPlace();
            }
			else
				D_p.setZero();

			if( dw != NULL ) {
                D_w = Eigen::Map<DMatrix>(dw,nw,nfDir);
                D_w.transposeInPlace();
            }
			else
				D_w.setZero();

			DMatrix J( nx+nxa,nfDir );

			// setup seed and sensitvity matrices
			DMatrix bSeed2( nbDir2,nx );
            bSeed2 = Eigen::Map<DMatrix>(bseed2,nbDir2,nx);
            bSeed2.transposeInPlace();

			DMatrix J_x2( nbDir2,nx+nxa );
			DMatrix J_u2( nbDir2,nu );
			DMatrix J_p2( nbDir2,np );
			DMatrix J_w2( nbDir2,nw );


            int run1, run2;

            for( run1 = 0; run1 < nfDir; run1++ ){

                DVector DXX = D_x.getCol( run1 );
                DVector DPP = D_p.getCol( run1 );
                DVector DUU = D_u.getCol( run1 );
                DVector DWW = D_w.getCol( run1 );

				integrator->setForwardSeed( 1, DXX, DPP, DUU, DWW );
				returnvalue = integrator->integrateSensitivities( );

				if( returnvalue != SUCCESSFUL_RETURN )
				{
					status = -1.0;
					plotErrorMessage( returnvalue,printLevel_ );
				}
				else
				{
                    DVector JJ(nx+nxa);
					integrator->getForwardSensitivities( JJ, 1 );
                    J.setCol( run1, JJ );
                }

                for( run2 = 0; run2 < nbDir2; run2++ ){

                    DVector XSEED = bSeed2.getRow(run2);

					// determine backward sensitivities
					integrator->setBackwardSeed( 2, XSEED );
					returnvalue = integrator->integrateSensitivities( );
					if( returnvalue != SUCCESSFUL_RETURN )
					{
						status = -1.0;
						plotErrorMessage( returnvalue,printLevel_ );
					}
					else
					{
        	            DVector JXX2(nx+nxa), JPP2(np), JUU2(nu), JWW2(nw);

						integrator->getBackwardSensitivities( JXX2, JPP2, JUU2, JWW2, 2 );

                	    J_x2.setRow( run2, JXX2 );
                    	J_p2.setRow( run2, JPP2 );
                    	J_u2.setRow( run2, JUU2 );
                    	J_w2.setRow( run2, JWW2 );
	            	}
                }
            }


			// write sensitivity matrices into output struct (if given)
			if ( returnvalue == SUCCESSFUL_RETURN && outputIdx > 0 )
			{
				JJ = mxCreateDoubleMatrix( nx+nxa,nfDir,mxREAL );
				jj = mxGetPr( JJ );
                DMatrix jj_tmp = Eigen::Map<DMatrix>(jj, J.rows(), J.cols());  jj_tmp = J.transpose(); jj = jj_tmp.data();
				mxSetField( plhs[outputIdx],0,"J",JJ );

				Jx2 = mxCreateDoubleMatrix( nbDir2,nx+nxa,mxREAL );
				jx2 = mxGetPr( Jx2 );
                DMatrix jx2_tmp = Eigen::Map<DMatrix>(jx2, J_x2.rows(), J_x2.cols());  jx2_tmp = J_x2.transpose(); jx2 = jx2_tmp.data();
				mxSetField( plhs[outputIdx],0,"J2x",Jx2 );

				if ( nu > 0 )
				{
					Ju2 = mxCreateDoubleMatrix( nbDir2,nu,mxREAL );
					ju2 = mxGetPr( Ju2 );
                    DMatrix ju2_tmp = Eigen::Map<DMatrix>(ju2, J_u2.rows(), J_u2.cols());  ju2_tmp = J_u2.transpose(); ju2 = ju2_tmp.data();
					mxSetField( plhs[outputIdx],0,"J2u",Ju2 );
				}

				if( np > 0 )
				{
					Jp2 = mxCreateDoubleMatrix( nbDir2,np,mxREAL );
					jp2 = mxGetPr( Jp2 );
                    DMatrix jp2_tmp = Eigen::Map<DMatrix>(jp2, J_p2.rows(), J_p2.cols());  jp2_tmp = J_p2.transpose(); jp2 = jp2_tmp.data();
					mxSetField( plhs[outputIdx],0,"J2p",Jp2 );
				}

				if( nw > 0 )
				{
					Jw2 = mxCreateDoubleMatrix( nbDir2,nw,mxREAL );
					jw2 = mxGetPr( Jw2 );
                    DMatrix jw2_tmp = Eigen::Map<DMatrix>(jw2, J_w2.rows(), J_w2.cols());  jw2_tmp = J_w2.transpose(); jw2 = jw2_tmp.data();
					mxSetField( plhs[outputIdx],0,"J2w",Jw2 );
				}
			}
		}
	}


	// VI. STORE OUTPUT STRUCT:
	// ------------------------

	if ( ( outputIdx > 0 ) || ( !mxIsEmpty( PlotXTrajectory ) ) || ( !mxIsEmpty( PlotXaTrajectory ) ) )
	{
		Status = mxCreateDoubleMatrix( 1,1,mxREAL );
		statusPtr = mxGetPr( Status );
		statusPtr[0] = status;

		if ( status >= 0.0 )
		{
            
            // Get x
            VariablesGrid out_x;
            integrator->getX(out_x);
            
            XTrajectory = mxCreateDoubleMatrix( out_x.getNumPoints(),1+out_x.getNumValues(),mxREAL );
            xTrajectory = mxGetPr( XTrajectory );

            for( int i=0; i<out_x.getNumPoints(); ++i ){ 
                xTrajectory[0*out_x.getNumPoints() + i] = out_x.getTime(i);
                for( int j=0; j<out_x.getNumValues(); ++j ){
                    xTrajectory[(1+j)*out_x.getNumPoints() + i] = out_x(i, j);
                }
            }

            
            // Get xa
            if ( isODE == BT_FALSE ){

                VariablesGrid out_xa;
                integrator->getXA(out_xa);

                XaTrajectory = mxCreateDoubleMatrix( out_xa.getNumPoints(),1+out_xa.getNumValues(),mxREAL );
                xaTrajectory = mxGetPr( XaTrajectory );

                for( int i=0; i<out_xa.getNumPoints(); ++i ){ 
                    xaTrajectory[0*out_xa.getNumPoints() + i] = out_xa.getTime(i);
                    for( int j=0; j<out_xa.getNumValues(); ++j ){
                        xaTrajectory[(1+j)*out_xa.getNumPoints() + i] = out_xa(i, j);
                    } 
                }
            }


			NumberOfSteps = mxCreateDoubleMatrix( 1,1,mxREAL );
			numberOfSteps = mxGetPr( NumberOfSteps );
			*numberOfSteps = (double) integrator->getNumberOfSteps();

			NumberOfRejectedSteps = mxCreateDoubleMatrix( 1,1,mxREAL );
			numberOfRejectedSteps = mxGetPr( NumberOfRejectedSteps );
			*numberOfRejectedSteps = (double) integrator->getNumberOfRejectedSteps();

            GetStepSize = mxCreateDoubleMatrix( 1,1,mxREAL );
			getStepSize = mxGetPr( GetStepSize );
			*getStepSize = (double) integrator->getStepSize();  

            
            
			// plot if desired
			mxArray *PlotType = mxCreateDoubleMatrix( 1,1,mxREAL );
			double  *plotType = mxGetPr( PlotType );

			mxArray *IsDiscretized = mxCreateDoubleMatrix( 1,1,mxREAL );
			double  *isDiscretized = mxGetPr( IsDiscretized );

			if( strcmp(integratorName,"DiscretizedODE") == 0 )
				isDiscretized[0] = 1.0;
			else
				isDiscretized[0] = 0.0;

            
			if ( !mxIsEmpty( PlotXTrajectory ) )
			{
				plotType[0] = 0.0;
				mxArray* plotArguments[] = {	XTrajectory,PlotXTrajectory,UseSubplots,
												PlotType,IsDiscretized };
				mexCallMATLAB( 0,0,5,plotArguments,"plot_trajectory" );
			}

			if ( ( isODE == BT_FALSE ) && ( !mxIsEmpty( PlotXaTrajectory ) ) )
			{
				plotType[0] = 1.0;
				mxArray* plotArguments[] = {	XaTrajectory,PlotXaTrajectory,UseSubplots,
												PlotType,IsDiscretized };
				mexCallMATLAB( 0,0,5,plotArguments,"plot_trajectory" );
			}

			mxDestroyArray( IsDiscretized );
			mxDestroyArray( PlotType );
			PlotType = NULL;
			plotType = NULL;
	

			// write output struct if given
			if ( outputIdx > 0 )
			{
				mxSetField( plhs[outputIdx],0,"Status",Status );
				mxSetField( plhs[outputIdx],0,"NumberOfSteps",NumberOfSteps );
				mxSetField( plhs[outputIdx],0,"NumberOfRejectedSteps",NumberOfRejectedSteps );
				mxSetField( plhs[outputIdx],0,"xTrajectory",XTrajectory );
				mxSetField( plhs[outputIdx],0,"GetStepSize",GetStepSize );
                
				if ( isODE == BT_FALSE )
					mxSetField( plhs[outputIdx],0,"xaTrajectory",XaTrajectory );
			}

		}
		else
		{
			if ( outputIdx > 0 )
				mxSetField( plhs[outputIdx],0,"Status",Status );
		}
	}
    
    
	// VII. FREE THE MEMORY:
	// ---------------------

	delete integrator;
	delete f;
    delete cModel; 
    
	if ( modelName != NULL )
		mxFree(modelName);

	if( !mxIsEmpty(IntegratorName) )
		mxFree(integratorName);

	if( !mxIsEmpty(SensitivityMode) )
		mxFree(sensitivityMode);

	if ( !mxIsEmpty(MaxNumStep) )
		delete maxNumStepInt;

	clearAllGlobals( );
}
