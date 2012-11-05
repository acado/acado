%Calculates the control inputs of the Process based on the Process outputs.
% The class Controller is one of the two main building-blocks within the SimulationEnvironment 
% and complements the Process. It contains an online control law (e.g. a DynamicFeedbackLaw 
% comprising a RealTimeAlgorithm) for obtaining the control inputs of the Process. 
% A state/parameter estimator as well as a ReferenceTrajectory can optionally be used 
% to provide estimated quatities and a reference values to the control law.
%
%  Usage:
%    >> Controller(controllaw[, reference]);
%
%  Parameters:
%    controllaw     Control law.             [acado.RealTimeAlgorithm]
%    reference      Reference generator.     [acado.ReferenceTrajectory]
%
%  Example:
%    algo = acado.RealTimeAlgorithm(ocp, 0.05);
%    zeroReference = acado.StaticReferenceTrajectory();
%    controller = acado.Controller( algo,zeroReference );
%
%  See also:
%    acado.RealTimeAlgorithm
%    acado.ReferenceTrajectory
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
%    Author: David Ariens, Rien Quirynen
%    Date: 2012
% 
classdef Controller < acado.SimulationBlock   
    properties (SetAccess='private')
        controllaw;
        reference;
        name = 'controller';
        
        init_is_set;
        init_startTime;
        init_x0;
        init_p;
        init_y_ref;
        
        step_startTime;
        step_x0;
        step_y_ref;
        
        do_one_step = 0;
    end
    
    methods
        function obj = Controller(varargin)
            
            checkActiveModel;
            global ACADO_;
            ACADO_.count_other = ACADO_.count_other+1;
            obj.name = strcat(obj.name, num2str(ACADO_.count_other));
            
            if (nargin == 2 && isa(varargin{1}, 'acado.ControlLaw') && isa(varargin{2}, 'acado.ReferenceTrajectory') ) 
                obj.controllaw = varargin{1};   
                obj.reference = varargin{2};
                
            elseif (nargin == 1 && isa(varargin{1}, 'acado.ControlLaw')) 
                obj.controllaw = varargin{1};   

            else
               error('ERROR: Invalid Controller call. <a href="matlab: help acado.Controller">help acado.Controller</a>'); 
            end
            
            ACADO_.helper.addInstruction(obj);  
            
            
        end
        
        getInstructions(obj, cppobj, get)
        init(obj, varargin)
        step(obj, varargin)
        
    end
    
end