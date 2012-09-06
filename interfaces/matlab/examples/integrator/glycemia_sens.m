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
%%    \file interfaces/matlab/models/glycemia_sens.m
%%    \author Niels Haverbeke, Boris Houska, Hans Joachim Ferreau
%%    \date 2008
%%


 %  DEFINE THE INTEGRATOR SETTINGS:
 %  -------------------------------
    settings = ACADOintegratorsSettings;


 %  DEFINE THE NAME OF THE MODEL:
 %  -----------------------------
    settings.Model = 'glycemia';
    %settings.Model = { @glycemia_matlab };


 %  CHOOSE ONE OF THE FOLLOWING RUNGE-KUTTA INTEGRATORS:
 %  ----------------------------------------------------
   %settings.Integrator = 'RK12'    ;
   %settings.Integrator = 'RK23'    ;
    settings.Integrator = 'RK45'    ;
   %settings.Integrator = 'RK78'    ;
   %settings.Integrator = 'BDF'     ;


 %  DEFINE AN INITIAL POINT:
 %  ------------------------
    xStart = [207.3, 57.9, 0.0005, 1.487]';


 %  DEFINE THE START AND THE END OF THE TIME HORIZON:
 %  -------------------------------------------------
    tStart = 0  ;
    tEnd   = 100;


 %  DEFINE THE CONTROLS AND PARAMETERS:
 %  -----------------------------------
    settings.u = 450000;
    settings.p = [158.3333;0.0171;0.0224;0.0025;95.1;116.8;8760;10.7;107.4;0.2623;0.35;0.00014];


 %  DEFINE THE DISTURBANCE:
 %  -----------------------
    settings.w = [0,0,0,0]';


%  DEFINE BACKWARD SEED (ADJOINT SENSITIVITY GENERATION):
%  ------------------------------------------------------
    settings.SensitivityMode = 'AD_BACKWARD';           % sensitivity mode
    settings.mu              =  eye( length(xStart) );  % backward seed


 %  CALL THE INTEGRATOR VIA THE MEX - INTERFACE:
 %  ------------------------------------------------------
    tic
        [ xEnd,outputB ] = ACADOintegrators( settings,xStart,tStart,tEnd )
    toc


 %  DEFINE FORWARD SEED (AUTOMATIC FORWARD SENSITIVITIES):
 %  ------------------------------------------------------
    settings.SensitivityMode = 'AD_FORWARD';                 % sensitivity mode
    settings.lambdaX         =  zeros( length(xStart), 3 );  % forward     seed

    settings.lambdaX(1,1) = 1;
    settings.lambdaX(2,2) = 1;
    settings.lambdaX(3,3) = 1;


%   %  CALL THE INTEGRATOR VIA THE MEX - INTERFACE:
%   %  ------------------------------------------------------
    tic
        [ xEnd,outputsF ] = ACADOintegrators( settings,xStart,tStart,tEnd )
    toc


 % --------------------------------------------------------------------
 % END OF THIS FILE.
