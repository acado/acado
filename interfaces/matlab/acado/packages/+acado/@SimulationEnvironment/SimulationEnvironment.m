%Declaration of the Simulation Environment designed to test closed loop systems. 
% In a standard setup the Simulation Environment consists of a Process and a 
% Controller that are connected by signals.
% 
%  Usage:
%    >> SimulationEnvironment(startTime, endTime, process, controller);
%
%  Parameters:
%    startTime 	              [NUMERIC]
%    endTime                  [NUMERIC]
%    process                  [acado.Process]
%    controller               [acado.Controller]
%
%  Example:
%    >> sim = acado.SimulationEnvironment( 0.0,3.0,process,controller );
%
%  See also:
%    acado.Process
%    acado.Controller
%    acado.SimulationEnvironment.init     initialization values
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
classdef SimulationEnvironment < acado.SimulationBlock & acado.OptimizationAlgorithmBase    
    properties (SetAccess='private')
       
        startTime = 0;
        endTime = 0;
        process;
        controller;
        
        initvector;
    end
    
    methods
        function obj = SimulationEnvironment(varargin)

            global ACADO_;
            ACADO_.count_optalgo = ACADO_.count_optalgo+1;
            obj.name = strcat(obj.name, num2str(ACADO_.count_optalgo));
            
            if (nargin == 4 && isa(varargin{1}, 'numeric') && isa(varargin{2}, 'numeric') && isa(varargin{3}, 'acado.Process') && isa(varargin{4}, 'acado.Controller'))
                obj.startTime = varargin{1};   
                obj.endTime = varargin{2};
                obj.process = varargin{3};
                obj.controller = varargin{4};
                
            else
               error('ERROR: Invalid SimulationEnvironment call. <a href="matlab: help acado.SimulationEnvironment">help acado.SimulationEnvironment</a>'); 
            end
            
            ACADO_.helper.addInstruction(obj);
            
        end
        
        init(obj, varargin)
        getInstructions(obj, cppobj, get)
        
    end
end


