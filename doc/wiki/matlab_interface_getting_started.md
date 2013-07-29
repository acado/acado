# ACADO for MATLAB - getting started {#md_matlab_interface_getting_started}

[TOC]
     
To install and use the MATLAB interface you need to have a recent MATLAB version and a C++ compiler installed. Follow these steps to get you started in a few minutes.
    
## Step 1 - Installing a compiler ##
     
** I'm a LINUX / Mac user **
    
Make sure you have installed a recent version of the GCC compiler (at least version 4.1 but 4.2 or later is advised).
    
** I'm a windows user **
    
Install the Microsoft Visual C++ 2010 Express Edition compiler. Click here to go to the download site. Continue this tutorial once you have installed this suite (you might be required to restart your PC first)
    
## Step 2 - Configuring MATLAB ##
     
Once a compiler is installed it needs to be linked to MATLAB. Open MATLAB (a recent version of MATLAB is required) and run in command window:
~~~
mex -setup;
~~~
MATLAB returns:
~~~    
Please choose your compiler for building external interface (MEX) files: 
Would you like mex to locate installed compilers [y]/n?
~~~     	  	   	   	      
Type "y" and hit enter.
    
** I'm a LINUX / Mac user **
    
MATLAB shows you a list of installed compilers. Enter the number corresponding to the GCC compiler (in this case 1) and hit enter.
~~~    
The options files available for mex are:
    
    1: /software/matlab/2009b/bin/gccopts.sh :
    Template Options file for building gcc MEX-files

    2: /software/matlab/2009b/bin/mexopts.sh :
    Template Options file for building MEX-files via the system ANSI compiler

    0: Exit with no changes

    Enter the number of the compiler (0-2):
~~~
** I'm a windows user **

MATLAB shows you a list of installed compilers. Enter the number corresponding to the Visual C++ compiler (in this case 2) and hit enter.
~~~
Select a compiler: 
[1] Lcc-win32 C 2.4.1 in C:\PROGRA~1\MATLAB\R2009a\sys\lcc 
[2] Microsoft Visual C++ 2008 Express in C:\Program Files\Microsoft Visual Studio 9.0 
                       
[0] None 
                       
Compiler:
~~~
Confirm the result by writing "y" and hitting enter:
~~~
Please verify your choices: 
 
Compiler: Microsoft Visual C++ 2008 Express  
Location: C:\Program Files\Microsoft Visual Studio 9.0 
 
Are these correct [y]/n?
~~~ 
##Step 3 - Building the ACADO interface ##
 
Follow instructions on the [main page](@ref mainpage) to **download** the ACADO code. Please note you do not need to build ACADO at this stage, you just need to download it. We will refer to the main ACADO folder as \<ACADOtoolkit-inst-dir\>. Open Matlab in this directory.

Navigate to the MATLAB installation directory by running:
~~~
cd interfaces/matlab/;
~~~
You are now ready to compile the ACADO interface. This compilation will take several minutes but needs to be done only once. Run `make` in your command window:
~~~
make clean all;
~~~
You will see:
~~~
Making ACADO... 
~~~
and after a while when the compilation is finished:
~~~
ACADO successfully compiled.
Needed to compile XXX file(s).

If you need to restart Matlab, run this make file again
to set all paths or run savepath in your console to
save the current search path for future sessions.
~~~
ACADO has now been compiled. As the text indicated every time you restart MATLAB you need to run `make` again to set all paths. When running `make` again no new files need to be compiled and the process will only take a few seconds. However, it is easier to save your paths for future Matlab session. Do so by running "savepath" in your command window (this step is optional).
~~~
savepath;
~~~
## Step 4 - Running your first example ##
 
We will now run the %OCP getting started example:
~~~
cd examples/ocp/getting_started/
~~~
The file getting_started.m contains the ACADO syntax to setup and execute a simple Optimal Control Problem. Run "getting_started" in your terminal to test the execution:
~~~
getting_started;
~~~
You should see
~~~
ACADO Toolkit::SCPmethod -- A Sequential Quadratic Programming Algorithm.
Copyright (C) 2008-2009 by Boris Houska and Hans Joachim Ferreau, K.U.Leuven.
Developed within the Optimization in Engineering Center (OPTEC) under
supervision of Moritz Diehl. All rights reserved.

[......]

     1:  KKT tolerance = 2.016e-001     line search parameter = 1.000e+000     objective value = 6.4478e-001
     2:  KKT tolerance = 2.074e+000     objective value = 4.3516e-001
     3:  KKT tolerance = 1.484e-001     objective value = -2.3787e+000
     4:  KKT tolerance = 9.130e-002     objective value = -2.3441e+000
     5:  KKT tolerance = 1.035e-001     objective value = -2.4338e+000
     6:  KKT tolerance = 5.587e-002     objective value = -2.5326e+000
     7:  KKT tolerance = 2.741e-002     objective value = -2.5766e+000
     8:  KKT tolerance = 1.839e-002     objective value = -2.5959e+000
     9:  KKT tolerance = 1.543e-002     objective value = -2.6105e+000
    10:  KKT tolerance = 1.494e-002     objective value = -2.6258e+000
    11:  KKT tolerance = 5.624e-003     objective value = -2.6404e+000
    12:  KKT tolerance = 1.584e-004     objective value = -2.6456e+000
    13:  KKT tolerance = 1.214e-008     objective value = -2.6456e+000

convergence achieved.
~~~ 
A graph will be drawn with the results who are stored in the variable 'out'.

You're done!
 
Everything OK? Then your done and you can start using the ACADO Toolkit MATLAB Interface. Please report any bugs at our forum.

If you would like to help us getting an idea on which platforms the MATLAB interface runs, please execute "\<ACADOtoolkit-inst-dir\>/interfaces/matlab/getversioninformation.m" in your command window and post the results at our forum .

Would you like to read more? Download the user [manual](@ref md_doc_manuals).
