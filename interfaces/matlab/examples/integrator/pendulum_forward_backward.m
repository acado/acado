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
%%    \file interfaces/matlab/models/pendulum_forward_backward.m
%%    \author Boris Houska, Hans Joachim Ferreau
%%    \date 2008
%%


 %  DEFINE THE INTEGRATOR SETTINGS:
 %  -------------------------------
    settings = ACADOintegratorsSettings;


 %  DEFINE THE NAME OF THE MODEL:
 %  -----------------------------
    settings.Model = 'pendulum';


 %  CHOOSE ONE OF THE FOLLOWING RUNGE-KUTTA INTEGRATORS:
 %  ----------------------------------------------------
   %settings.Integrator = 'RK12'    ;
   %settings.Integrator = 'RK23'    ;
    settings.Integrator = 'RK45'    ;
   %settings.Integrator = 'RK78'    ;
   %settings.Integrator = 'BDF'     ;


 %  DEFINE AN INITIAL POINT:
 %  ------------------------
    xStart = [0.0, 0.0]';


 %  DEFINE THE START AND THE END OF THE TIME HORIZON:
 %  -------------------------------------------------
    tStart = 0;
    tEnd   = 2;


 %  DEFINE THE CONTROLS AND PARAMETERS:
 %  -----------------------------------
    settings.u = 1.0;
    settings.p = 1.0;


 %  DEFINE FORWARD SEED AND SECOND ORDER BACKWARD SEED (AUTOMATIC DIFFERENTIATION):
 %  -------------------------------------------------------------------------------
    settings.SensitivityMode = 'AD_FORWARD_BACKWARD';   % sensitivity mode
    settings.lambdaX         =  [1.0, 0.0]' ;           % 1st order forward seed
    settings.mu2             =  eye(length(xStart));    % 2nd order backward seed


 %  RUN THE INTEGRATION ROUTINE:
 %  ----------------------------

    tic
        [ xEnd,outputs ] = ACADOintegrators( settings,xStart,tStart,tEnd )
    toc


 %  END OF THE FILE.
 %  ----------------
