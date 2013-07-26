# ACADO for MATLAB {#md_matlab_interface}

[TOC]
 
ACADO for MATLAB is a MATLAB interface for the ACADO Toolkit. It brings the ACADO Integrators and algorithms for direct optimal control, model predictive control and parameter estimation to MATLAB. ACADO for MATLAB uses the ACADO Toolkit C++ code base and implements methods to communicate with this code base.

ACADO Toolkit is a software environment and algorithm collection for automatic control and dynamic optimization. It provides a general framework for using a great variety of algorithms for direct optimal control, including model predictive control, state and parameter estimation and robust optimization. The object-oriented design allows for convenient coupling of existing optimization packages and for extending it with user-written optimization routines.

## Getting started with the MATLAB interface ##
 
To install and use the Matlab interface you need to have a recent Matlab version and a C++ compiler installed. Follow [these steps](@ref md_matlab_interface_getting_started) to get you started in a few minutes.

## Key Features ##
 
Three available interfaces:
1.    Use the stand-alone Runge-Kutta and BDF's integrators in MATLAB
2.    Compose your optimization problem in a MATLAB environment with familiar MATLAB syntax using the generic optimal control interface
3.    Write your own C++ code as a MEX-function and compile it using ACADO for MATLAB build-in MEX-compiler

Link your models to ACADO:
1.    Link MATLAB ODE or DAE models
2.    Link C++ ODE or DAE models
3.    Provide optional Jacobians for faster calculations

## Documentation & Examples ##
 
Click [here](http://acado.sourceforge.net/matlab/doc/html/) to go to the class documentation or download the user [manual](@ref md_doc_manuals).

The folder \<ACADOtoolkit-inst-dir\>/interfaces/matlab/examples contain many examples explaining how to use the interface.
