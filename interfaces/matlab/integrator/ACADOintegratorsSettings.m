%ACADOintegrators is a collection of different integrators for solving ODE and 
%DAE systems including efficient sensitivity generation.
%ACADOintegratorsSettings returns a settings struct for calling ACADOintegrators.
%
%Call
%
%    settings = ACADOintegratorsSettings;
%
%for obtaining a settings struct comprising the following items:
%
%    Model               --  a) File name (without .cpp) containing the C++  
%                               implementation of the ODE/DAE,
%                               e.g. 'gettting_started' for getting_started.cpp
%                            b) a cell array of size one containing a Matlab  
%                               function handle to an ODE specified in an m-script,
%                               e.g. '{ @glycemia_matlab }' for an ODE given as: 
%                               dx = glycemia_matlab( t,x,u,p,w ) 
%                            c) a cell array of size one containing a Matlab  
%                               function handle to a DAE specified in an m-script,
%                               e.g. '{ @simple_dae_matlab }' for an DAE given as: 
%                               f = glycemia_matlab( t,x,xa,u,p,w ) 
%
%    Integrator          --  Name of one of the following integrators:
%                            a) 'RK12' for an error-controlled Runge-Kunga12
%                            b) 'RK23' for an error-controlled Runge-Kunga23
%                            c) 'RK45' for an error-controlled Runge-Kunga45
%                            d) 'RK78' for an error-controlled Runge-Kunga78
%                            e) 'BDF'  for an error-controlled, implicit integrator
%                                      based on the backward differentiation formula
% 
%    Tolerance           --  Integration tolerance 
%    AbsoluteTolerance   --  Integration absolute tolerance
%    MaxNumberOfSteps    --  Maximum number of integrator steps
%    MinimumStepSize     --  Minimum integrator step size
%    MaximumStepSize     --  Maximum integrator step size
%    InitialStepSize     --  Initial step size integrator
%    CorrectorTolerance  --  Corrector Tollerance
%    StepSizeTuning      --  Step size tuning parameter
%    LinearAlgebraSolver --  Specifies how the linear systems are solved:
%                            a) 'dense' for a dense solver
%                            b) 'sparse' for a sparse solver
%
%    u                   --  Constant value of controls affecting the ODE/DAE
%    p                   --  Constant value of parameters affecting the ODE/DAE
%    w                   --  Constant value of disturbances affecting the ODE/DAE
%    dxInit              --  Initial value for differential states derivatives
%                            (only for DAEs)
%
%    SensitivityMode     --  String specifying one of the following modes for 
%                            efficient sensitivity generation:
%                            a) '' (empty) for no sensitivity generation
%                            b) 'AD_FORWARD'  for 1st order forward sensitivities
%                            c) 'AD_BACKWARD' for 1st order backward sensitivities
%                            d) 'AD_FORWARD2' for 2nd order forward sensitivities
%                            e) 'AD_FORWARD_BACKWARD' for 2st order sensitivities,
%                               first forward then backward
%
%    mu                  --  Backward seed
%    lambdaX             --  Forward seed w.r.t. differential states x
%    lambdaU             --  Forward seed w.r.t. controls u
%    lambdaP             --  Forward seed w.r.t. parameters p
%    lambdaW             --  Forward seed w.r.t. disturbances w
%    mu2                 --  Backward seed (2nd order)
%    lambdaX2            --  Forward seed w.r.t. differential states x (2nd order)
%    lambdaU2            --  Forward seed w.r.t. controls u (2nd order)
%    lambdaP2            --  Forward seed w.r.t. parameters p (2nd order)
%    lambdaW2            --  Forward seed w.r.t. disturbances w (2nd order)
%
%    PrintLevel          --  Flag indicating if integrator shall print messages to
%                            Matlab command window: 0 for no, 1 for yes
%    PlotXTrajectory     --  Array containg a list of all indices of differential
%                            states that shall be printed, e.g. [] for none
%    PlotXaTrajectory    --  Array containg a list of all indices of algebraic 
%                            states that shall be plotted, e.g. [] for none
%    UseSubplots         --  Flag indicating if plot routine shall use subplots in 
%                            order to use fewer figures: 0 for no, 1 for yes
%    StorageResolution   --  Defines the number of time points at which the trajectory
%                            is stored and optionally plotted. By default these time
%                            points are equidistant and 101 in total.
%
%Except for Model and Integrator, all components are optional and default
%values might be used by the respective integrator. In particular, if 
%SensitivityMode is empty, no sensitivities are calculated and no seeds need 
%to be given (otherwise, at least on non-empty seed is expected).
%
%As a short-cut, you can also call
%
%    settings = ACADOintegratorsSettings( 'modelname','integratorname' );
%
%for obtaining a settings struct with Model set to 'modelname' and Integrator
%set to 'integratorname' and all other components empty. This might be directly 
%used (inlined) within a call to ACADOintegrators.
%
%
%See also ACADOINTEGRATORS, ACADOINTEGRATORSOUTPUTS
%
%
%For additional information see the ACADOintegrators Tutorial and User's Manual 
%or visit http://www.acadotoolkit.org.
%
%Please send remarks and questions to support@acadotoolkit.org!
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
%    Author: Niels Haverbeke, Boris Houska, Hans Joachim Ferreau, David Ariens
%    Date: 2008-2010

function [ settings ] = ACADOintegratorsSettings( model,integrator )

	settings = struct(	'Model',[],...
						'Integrator',[],...
						'Tolerance',[],...
						'AbsoluteTolerance',1e-6,...
						'MaxNumberOfSteps',1000,...
						'MinimumStepSize',1e-6,...
						'MaximumStepSize',1e8,...
                        'InitialStepSize',1e-3,...
                        'CorrectorTolerance', 1e-14,...
						'StepSizeTuning',0.5,...
						'LinearAlgebraSolver','dense',...
						'u',[],...
						'p',[],...
						'w',[],...
						'dxInit',[],...
						'SensitivityMode',[],...
						'mu',[],...
						'lambdaX',[],...
						'lambdaU',[],...
						'lambdaP',[],...
						'lambdaW',[],...
						'mu2',[],...
						'lambda2X',[],...
						'lambda2U',[],...
						'lambda2P',[],...
						'lambda2W',[],...
						'PrintLevel',1,...
						'PlotXTrajectory',[],...
						'PlotXaTrajectory',[],...
						'UseSubplots',0, ...
						'StorageResolution',101, ...
                        'Jacobian', []);

	switch ( nargin )
		case 0
			return;

		case 1
			settings.Model = model;
			return;

		case 2
			settings.Model = model;
			settings.Integrator = integrator;
			settings.Tolerance = determineDefaultTolerance( integrator );
			return;

		otherwise
			disp('ERROR (integratorSettings): At most two input arguments allowed!');
			return;
	end

end


function [ tolerance ] = determineDefaultTolerance( integrator )

	switch ( integrator )
		case 'BDF'
			tolerance = 1e-6;
			return;

		case 'RK12'
			tolerance = 1e-1;
			return;

		case 'RK23'
			tolerance = 1e-2;
			return;

		case 'RK45'
			tolerance = 1e-6;
			return;

		case 'RK78'
			tolerance = 1e-6;
			return;

		otherwise
			tolerance = [];
			return;
	end

end
