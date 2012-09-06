%Example
%
%  Licence:
%    This file is part of ACADO Toolkit  - (http://www.acadotoolkit.org/)
%
%    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
%    Copyright (C) 2008-2009 by Boris Houska and Hans Joachim Ferreau, K.U.Leuven.
%    Developed within the Optimization in Engineering Center (OPTEC) under
%    supervision of Moritz Diehl. All rights reserved.
%
%    ACADO Toolkit is free software; you can redistribute it and/or
%    modify it under the terms of the GNU Lesser General Public
%    License as published by the Free Software Foundation; either
%    version 3 of the License, or (at your option) any later version.
%
%    ACADO Toolkit is distributed in the hope that it will be useful,
%    but WITHOUT ANY WARRANTY; without even the implied warranty of
%    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
%    Lesser General Public License for more details.
%
%    You should have received a copy of the GNU Lesser General Public
%    License along with ACADO Toolkit; if not, write to the Free Software
%    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
%
%    \author David Ariens
%    \date 2010
% 


 %  DEFINE THE INTEGRATOR SETTINGS:
 %  -------------------------------
    settings = ACADOintegratorsSettings;


 %  DEFINE THE NAME OF THE MODEL:
 %  -----------------------------
    settings.Model = 'getting_started_othercppfilemodel';    % This file should also be included in ../../MODEL_INCLUDE.m
    % We also use the file getting_started_othercppfile.cpp This file is included in  ../../MODEL_INCLUDE.m
    

 %  CHOOSE ONE OF THE FOLLOWING RUNGE-KUTTA INTEGRATORS:
 %  ----------------------------------------------------
   %settings.Integrator = 'RK12'    ;
   %settings.Integrator = 'RK23'    ;
   %settings.Integrator = 'RK45'    ;
   %settings.Integrator = 'RK78'    ;
   settings.Integrator = 'BDF'     ;

   settings.Tolerance = 1e-5;
   settings.MinimumStepSize = 1e-10;


 %  DEFINE AN INITIAL POINT:
 %  ------------------------
    xStart = [ 1 ]';


 %  DEFINE THE START AND THE END OF THE TIME HORIZON:
 %  -------------------------------------------------
    tStart = 0;
    tEnd   = 1;


 %  DEFINE PLOT OPTIONS:
 %  -----------------------
    settings.PlotXTrajectory = [1:length(xStart)];
    settings.UseSubplots = 0;


 %  RUN THE INTEGRATION ROUTINE:
 %  ----------------------------

    tic
        [ xEnd, output ] = ACADOintegrators( settings,xStart,tStart,tEnd )
    toc


 %  END OF THE FILE.
 %  ----------------
