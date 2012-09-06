%%
%%    This file is part of ACADO Toolkit.
%%
%%    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
%%    Copyright (C) 2008-2009 by Boris Houska and Hans Joachim Ferreau, K.U.Leuven.
%%Developed within the Optimization in Engineering Center (OPTEC) under
%%    supervision of Moritz Diehl. All rights reserved.
%%
%%    ACADO Toolkit is free software; you can redistribute it and/or
%%    modify it under the terms of the GNU Lesser General Public
%%    License as published by the Free Software Foundation; either
%%    version 3 of the License, or (at your option) any later version.
%%
%%    ACADO Toolkit is distributed in the hope that it will be useful,
%%    but WITHOUT ANY WARRANTY; without even the implied warranty of
%%    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
%%    Lesser General Public License for more details.
%%
%%    You should have received a copy of the GNU Lesser General Public
%%    License along with ACADO Toolkit; if not, write to the Free Software
%%    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
%%
%%

%%
%%    \file interfaces/matlab/examples/integrator/beachball.m
%%    \author David Ariens
%%    \date 2009
%%

clear all;

 %  DEFINE THE INTEGRATOR SETTINGS:
 %  -------------------------------
    settings = ACADOintegratorsSettings;


 %  DEFINE THE NAME OF THE MODEL:
 %  -----------------------------
     settings.Model = { @beachball_ode };
     settings.Jacobian = { @beachball_jacobian };      % THIS LINE IS OPTIONAL


 %  CHOOSE ONE OF THE FOLLOWING RUNGE-KUTTA INTEGRATORS:
 %  ----------------------------------------------------
   settings.Integrator = 'BDF'    ;


 %  DEFINE AN INITIAL POINT:
 %  ------------------------
    xStart = [0, 10]';


 %  DEFINE THE START AND THE END OF THE TIME HORIZON:
 %  -------------------------------------------------
    tStart = 0  ;
    tEnd   = 3;


 %  DEFINE THE PARAMETERS:
 %  -----------------------------------
 %   settings.p = [9.81, .02]';


 %  DEFINE PLOT OPTIONS:
 %  -----------------------
    settings.PlotXTrajectory = 1:length(xStart);
    settings.UseSubplots = 0;


 %  RUN THE INTEGRATION ROUTINE:
 %  ----------------------------
    tic
        [ xEnd, output ] = ACADOintegrators( settings,xStart,tStart,tEnd )
    toc                                             % 4 arguments => ODE
    
 %  END OF THE FILE.
 %  ----------------
