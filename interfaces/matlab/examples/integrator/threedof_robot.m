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
%%    \file interfaces/matlab/models/threedof_robot.m
%%    \author Niels Haverbeke, Boris Houska, Hans Joachim Ferreau
%%    \date 2008
%%


 %  DEFINE THE INTEGRATOR SETTINGS:
 %  -------------------------------
    settings = ACADOintegratorsSettings;


 %  DEFINE THE NAME OF THE MODEL:
 %  -----------------------------
    settings.Model = 'threedof_robot';         % This file should also be included in ../../MODEL_INCLUDE.m
    %settings.Model = {@threedof_robot_matlab};


 %  USE THE BDF INTEGRATOR:
 %  -----------------------------
    settings.Integrator = 'BDF';


 %  ADJUST THE INTEGRATOR:
 %  -------------------------------------------
    settings.Tolerance = 1e-5;     % local error tolerance.


 %  DEFINE AN INITIAL POINT:
 %  ------------------------
    xStart = [-4.8880e-05, 0.1841, 1.7084, 0.0, 0.0, 0.0 ]';


 %  DEFINE THE START AND THE END OF THE TIME HORIZON:
 %  -------------------------------------------------
    tStart = 0;
    tEnd   = 1;


 %  DEFINE THE CONTROLS AND PARAMETERS:
 %  -----------------------------------
    settings.u = [6.7479, -42.7850, -25.3565]';
    settings.p = [29.0130, -10.9085, -0.2629, 2.6403, -0.0421, 0.0509, 3.0221, 2.4426, 0.0531, -0.2577, -0.2627, 346.7819, -0.2651, 35.4666, 5.6321, 0.9935, 3.2192, 10.7823, 34.2604, 16.9902, 39.1594, 5.4501, 16.1392, -12.7243, -6.1094, 1.9590]';


 %  DEFINE THE DISTURBANCE:
 %  -----------------------
    settings.w = [0,0,0,0,0,0]';


 %  RUN THE INTEGRATION ROUTINE:
 %  ----------------------------
    tic
        [ xEnd, outputs ] = ACADOintegrators( settings,xStart,tStart,tEnd )
                                      % 4 arguments => ODE
    toc


 %  END OF THE FILE.
 %  ----------------
