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
%%    \file interfaces/matlab/models/simple_dae_sens.m
%%    \author Boris Houska, Hans Joachim Ferreau
%%    \date 2008
%%


 %  DEFINE THE INTEGRATOR SETTINGS:
 %  -------------------------------
    settings = ACADOintegratorsSettings;


 %  DEFINE THE NAME OF THE MODEL:
 %  -----------------------------
     settings.Model = 'simple_dae';
    %settings.Model = { @simple_dae_matlab_f,@simple_dae_matlab_g,1 };


 %  USE THE BDF INTEGRATOR:
 %  -----------------------------
    settings.Integrator = 'BDF';


 %  ADJUST THE INTEGRATOR:
 %  -------------------------------------------
    settings.Tolerance = 1e-5;     % local error tolerance.


 %  DEFINE AN INITIAL POINT:
 %  ------------------------
    xStart  = [ 1.0 ]';
    xaStart = [ 1.0 ]';


 %  DEFINE THE START AND THE END OF THE TIME HORIZON:
 %  -------------------------------------------------
    tStart = 0;
    tEnd   = 0.2;


 %  DEFINE THE PARAMETERS:
 %  ----------------------
    settings.p = [1.0 1.0]';


         % --------------------------------------------%
         % USE ONE OF THE FOLLOWING SENSITIVITY MODES: %
         % --------------------------------------------%


 %  DEFINE FORWARD SEED (FORWARD SENSITIVITY GENERATION):
 %  -----------------------------------------------------
    settings.SensitivityMode = 'AD_FORWARD';           % sensitivity mode
    settings.lambdaX         =  eye( length(xStart) );  % forward seed


 %  DEFINE BACKWARD SEED (ADJOINT SENSITIVITY GENERATION):
 %  ------------------------------------------------------
 %     settings.SensitivityMode = 'AD_BACKWARD';           % sensitivity mode
 %     settings.mu              =  eye( length(xStart) );  % backward seed


 %  DEFINE FORWARD SEED (2nd ORDER FORWARD SENSITIVITY GENERATION):
 %  ---------------------------------------------------------------
 %     settings.SensitivityMode = 'AD_FORWARD2'         ;     % sensitivity mode
 %     settings.lambdaX         =  [1.0]                ;     % forward seed
 %     settings.lambda2X        =  eye( length(xStart) );     % second order forward seed


 %  DEFINE FORWARD SEED AND SECOND ORDER BACKWARD SEED (AUTOMATIC FORWARD SENSITIVITIES):
 %  -------------------------------------------------------------------------------------
 %     settings.SensitivityMode = 'AD_FORWARD_BACKWARD';   % sensitivity mode
 %     settings.lambdaX         =  [1.0];                  % 1st order forward seed
 %     settings.mu2             =  eye(length(xStart));    % 2nd order backward seed



 %  RUN THE INTEGRATION ROUTINE:
 %  ----------------------------
    tic
        [ xEnd, xaEnd, outputsB ] = ACADOintegrators( settings,xStart,xaStart,tStart,tEnd )
    toc


 %  END OF THE FILE.
 %  ----------------
