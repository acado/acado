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
%%    \file interfaces/matlab/models/sparse_dae.m
%%    \author Boris Houska, Hans Joachim Ferreau
%%    \date 2009
%%


 %  DEFINE THE INTEGRATOR SETTINGS:
 %  -------------------------------
    settings = ACADOintegratorsSettings;


 %  DEFINE THE NAME OF THE MODEL:
 %  -----------------------------
    settings.Model = 'sparse_dae';     % This file should also be included in ../../MODEL_INCLUDE.m
 

 %  USE THE BDF INTEGRATOR:
 %  -----------------------------
    settings.Integrator = 'BDF';


 %  ADJUST THE INTEGRATOR:
 %  -------------------------------------------
    settings.Tolerance = 1e-5;     % local error tolerance.
    
 
 %  SPECIFY LINEAR ALGEBRA SOLVER:
 %  -------------------------------------------
     settings.LinearAlgebraSolver = 'sparse';
    %settings.LinearAlgebraSolver = 'dense';


 %  DEFINE AN INITIAL POINT:
 %  ------------------------
    xStart  = ones(100,1);
    xaStart = [ 1.0 ]';


 %  DEFINE THE START AND THE END OF THE TIME HORIZON:
 %  -------------------------------------------------
    tStart = 0.0;
    tEnd   = 1.0;


 %  DEFINE THE PARAMETERS:
 %  ----------------------
    settings.p = [1.0]';


 %  DEFINE PLOT OPTIONS:
 %  -----------------------
%      settings.PlotXTrajectory  = 1:length(xStart);
%      settings.PlotXaTrajectory = 1:length(xaStart);
%      settings.UseSubplots = 0;


 %  RUN THE INTEGRATION ROUTINE:
 %  ----------------------------
    tic
        [ xEnd, xaEnd, outputs ] = ACADOintegrators( settings,xStart,xaStart,tStart,tEnd )
    toc


 %  END OF THE FILE.
 %  ----------------
