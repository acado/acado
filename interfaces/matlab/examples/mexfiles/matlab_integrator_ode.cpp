/**
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
 *    Author: David Ariens  --  http://www.acadotoolkit.org/matlab 
 *    Date: 2010
 *    
 *    LINK MATLAB ODE TO THE INTEGRATOR
 *
 *    Compilation:
 *     - Go to the folder <ACADOtoolkit-inst-dir>/interfaces/matlab/
 *     - Run: makemex('examples/mexfiles/matlab_integrator_ode.cpp', 'matlab_integrator_ode', 'examples/mexfiles/');
 *     - Run: cd ('examples/mexfiles/'); matlab_integrator_ode();
 */
 
#include <acado_toolkit.hpp>                    // Include the ACADO toolkit
#include <acado/utils/matlab_acado_utils.hpp>   // Include specific Matlab utils

USING_NAMESPACE_ACADO                           // Open the namespace

mxArray* ModelFcn_f = NULL;                     // Globals
mxArray* ModelFcn_T  = NULL;
mxArray* ModelFcn_X  = NULL;
mxArray* ModelFcn_XA = NULL;
mxArray* ModelFcn_U  = NULL;
mxArray* ModelFcn_P  = NULL;
mxArray* ModelFcn_W  = NULL;
unsigned int ModelFcn_NT  = 0;
unsigned int ModelFcn_NX  = 0;
unsigned int ModelFcn_NXA = 0;
unsigned int ModelFcn_NU  = 0;
unsigned int ModelFcn_NP  = 0;
unsigned int ModelFcn_NW  = 0;


void clearAllGlobals( ){                   // Helper method to clear all global variables. Don't modify this method.

    if ( ModelFcn_f != NULL ){
        mxDestroyArray( ModelFcn_f );
        ModelFcn_f = NULL;
    }

    if ( ModelFcn_T != NULL ){
        mxDestroyArray( ModelFcn_T );
        ModelFcn_T = NULL;
    }

    if ( ModelFcn_X != NULL ){
        mxDestroyArray( ModelFcn_X );
        ModelFcn_X = NULL;
    }

    if ( ModelFcn_XA != NULL ){
        mxDestroyArray( ModelFcn_XA );
        ModelFcn_XA = NULL;
    }

    if ( ModelFcn_U != NULL ){
        mxDestroyArray( ModelFcn_U );
        ModelFcn_U = NULL;
    }

    if ( ModelFcn_P != NULL ){
        mxDestroyArray( ModelFcn_P );
        ModelFcn_P = NULL;
    }

    if ( ModelFcn_W != NULL ){
        mxDestroyArray( ModelFcn_W );
        ModelFcn_W = NULL;
    }

    ModelFcn_NT  = 0;
    ModelFcn_NX  = 0;
    ModelFcn_NXA = 0;
    ModelFcn_NU  = 0;
    ModelFcn_NP  = 0;
    ModelFcn_NW  = 0;
}

void genericODE( double* x, double* f, void *userData ){   // Helper function to call ODE's. Don't modify this method.
    unsigned int i;
    double* tt = mxGetPr( ModelFcn_T );
    tt[0] = x[0];
    double* xx = mxGetPr( ModelFcn_X );
    for( i=0; i<ModelFcn_NX; ++i )
        xx[i] = x[i+1];
    double* uu = mxGetPr( ModelFcn_U );
    for( i=0; i<ModelFcn_NU; ++i )
        uu[i] = x[i+1+ModelFcn_NX];
    double* pp = mxGetPr( ModelFcn_P );
    for( i=0; i<ModelFcn_NP; ++i )
        pp[i] = x[i+1+ModelFcn_NX+ModelFcn_NU];
    double* ww = mxGetPr( ModelFcn_W );
    for( i=0; i<ModelFcn_NW; ++i )
        ww[i] = x[i+1+ModelFcn_NX+ModelFcn_NU+ModelFcn_NP];
    mxArray* FF = NULL;
    mxArray* argIn[]  = { ModelFcn_f,ModelFcn_T,ModelFcn_X,ModelFcn_U,ModelFcn_P,ModelFcn_W };
    mxArray* argOut[] = { FF };

    mexCallMATLAB( 1,argOut, 6,argIn,"generic_ode" );       // Call generic_ode
    double* ff = mxGetPr( *argOut );
    for( i=0; i<ModelFcn_NX; ++i ){
        f[i] = ff[i];
    }

    mxDestroyArray( *argOut );
}



void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] )  // Start the MEX function. Do NOT change the header of this function.
 { 
    clearAllStaticCounters( );                  // Clear software counters
    
    // Set sizes
    ModelFcn_NT  = 1;   ModelFcn_T  = mxCreateDoubleMatrix( ModelFcn_NT,  1,mxREAL );
    ModelFcn_NX  = 2;   ModelFcn_X  = mxCreateDoubleMatrix( ModelFcn_NX,  1,mxREAL );
    ModelFcn_NXA = 0;   ModelFcn_XA = mxCreateDoubleMatrix( ModelFcn_NXA, 1,mxREAL );
    ModelFcn_NP  = 1;   ModelFcn_U  = mxCreateDoubleMatrix( ModelFcn_NP,  1,mxREAL );
    ModelFcn_NU  = 1;   ModelFcn_P  = mxCreateDoubleMatrix( ModelFcn_NU,  1,mxREAL );
    ModelFcn_NW  = 0;   ModelFcn_W  = mxCreateDoubleMatrix( ModelFcn_NW,  1,mxREAL );
    
    // ODE name
    ModelFcn_f = mxCreateString("matlab_integrator_ode_pendulum");

    // Set expressions
    DifferentialState phi, dphi;
    Control u;
    Parameter p;
    TIME t;
    
    // Set intermediate state to be passed to the ode
    IntermediateState x(5);
    x(0) = t   ;
    x(1) = phi ;
    x(2) = dphi;
    x(3) = u   ;
    x(4) = p   ;
    
    // Link ODE
    CFunction pendulumModel( ModelFcn_NX, genericODE );

    
    
    // Define a Right-Hand-Side:
    // -------------------------

    DifferentialEquation f;
    f << pendulumModel(x);


    // DEFINE AN INTEGRATOR:
    // ---------------------
    IntegratorRK45 integrator( f );


    // DEFINE INITIAL VALUES:
    // ----------------------

    double x_start[2] = { 0.0, 0.0 };
    double u_     [1] = { 1.0      };
    double p_     [1] = { 1.0      };


    double t_start    =  0.0;
    double t_end      =  1.0;


    // START THE INTEGRATION:
    // ----------------------
    integrator.set(INTEGRATOR_PRINTLEVEL, HIGH );
    integrator.freezeAll();
    integrator.integrate( t_start, t_end, x_start, 0, p_, u_ );


    // DEFINE A SEED MATRIX:
    // ---------------------
    Vector seed1(2);
    Vector seed2(2);

    seed1(0) = 1.0;
    seed1(1) = 0.0;

    seed2(0) = 1.0;
    seed2(1) = 0.0;

    // COMPUTE FIRST ORDER DERIVATIVES:
    // --------------------------------
    integrator.setForwardSeed(1,seed1);
    integrator.integrateSensitivities();

    // COMPUTE SECOND ORDER DERIVATIVES:
    // ---------------------------------
    integrator.setForwardSeed(2,seed2);
    integrator.integrateSensitivities();
    
    
    
    clearAllGlobals( );                         // Clear all globals
    clearAllStaticCounters( );                  // Clear software counters
} 

