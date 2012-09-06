%ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
%Copyright (C) 2008-2009 by Boris Houska and Hans Joachim Ferreau, K.U.Leuven.
%Developed within the Optimization in Engineering Center (OPTEC) under
%supervision of Moritz Diehl. All rights reserved.
%
%ACADO is distributed under the terms of the GNU Lesser General 
%Public License 3 in the hope that it will be useful, but WITHOUT ANY 
%WARRANTY; without even the implied warranty of MERCHANTABILITY or 
%FITNESS FOR A PARTICULAR PURPOSE.
%See the GNU Lesser General Public License for more details.
%
%---------------------------------------------------------------------------------
%
%ACADOintegrators is a collection of different integrators for solving ODE and 
%DAE systems including efficient sensitivity generation.
%
% I) Call
%
%     [xEnd,outputs] = ACADOintegrators( settings,xStart,tStart,tEnd )
%
%for integrating an ordinary differential equation (ODE) with initial 
%differential state xStart from the start time tStart to the end time tEnd.
%ODE model, integrator and further settings are defined within the settings 
%struct (type "help ACADOintegratorsSettings" for further explanations). 
%It returns the value of the differential state xEnd at the end of the 
%integration interval as well as additional information within an optional 
%output struct (type "help ACADOintegratorsOutputs" for further explanation).
%
%II) Call
%
%     [xEnd,xaEnd,outputs] = ACADOintegrators( settings,xStart,xaStart,tStart,tEnd )
%
%for integrating an differential-algebraic equation (DAE) with initial 
%differential/algebraic states xStart/xaStart from the start time tStart to the 
%end time tEnd. DAE model, integrator and further settings are defined within the 
%settings struct (type "help ACADOintegratorsSettings" for further explanations). 
%It returns the value of the differential and algebraic state xEnd/xaEnd at the end 
%of the integration interval as well as additional information within an optional 
%output struct (type "help ACADOintegratorsOutputs" for further explanation).
%
%ODE/DAE models can be either passed as Matlab function handles or, much more 
%efficient, defined using the highly intiutive ACADO C++ syntax. See the 
%ACADOintegrators Tutorial and User's Manual for various examples and a detailed 
%explanation.
%
%See also ACADOINTEGRATORSSETTINGS, ACADOINTEGRATORSOUTPUTS
%
%
%For additional information see the ACADOintegrators Tutorial and User's Manual 
%or visit http://www.acadotoolkit.org.
%
%Please send remarks and questions to matlab-support@acadotoolkit.org!