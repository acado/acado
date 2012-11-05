%The class Process is one of the two main building-blocks within the SimulationEnvironment and complements the Controller. 
% It simulates the process to be controlled based on a dynamic model.
%
%  Usage:
%    >> Process(dynamicSystem);
%    >> Process(dynamicSystem, integratorType);
%
%  Parameters:
%    dynamicSystem      [acado.DynamicSystem]
%    integratorType     Integrator for simulation of dynamical system.      [STRING]
%                             INT_RK12 	
%                             Explicit Runge-Kutta integrator of order 1/2
% 
%                             INT_RK23 	
%                             Explicit Runge-Kutta integrator of order 2/3
% 
%                             INT_RK45 	
%                             Explicit Runge-Kutta integrator of order 4/5
% 
%                             INT_RK78 	
%                             Explicit Runge-Kutta integrator of order 7/8
% 
%                             INT_BDF 	
%                             Implicit backward differentiation formula integrator. 
%              
%  Example:
%    >> process = acado.Process(dynamicSystem);
%    >> process = acado.Process(dynamicSystem, 'INT_RK45');
%
%  See also:
%    acado.Process.setProcessDisturbance 
%    acado.Process.initializeAlgebraicStates
%    acado.DynamicSystem
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
classdef Process < acado.SimulationBlock    
    properties(SetAccess='private')
        integratorType;
        dynamicSystem;
        disturbance;
        algebraicstates;
        name = 'process';
    end
    
    methods
        function obj = Process(varargin)
           
            global ACADO_;
            ACADO_.count_other = ACADO_.count_other+1;
            obj.name = strcat(obj.name, num2str(ACADO_.count_other));
            
            if (nargin == 1 && isa(varargin{1}, 'acado.DynamicSystem'))
               
                obj.dynamicSystem = varargin{1};
                obj.integratorType = 'INT_UNKNOWN';
            
            elseif(nargin == 2 && isa(varargin{1}, 'acado.DynamicSystem'))
                
                obj.dynamicSystem = varargin{1};
                obj.integratorType = varargin{2};
                
            else
                
                error('ERROR: Invalid Process call. <a href="matlab: help acado.Process">help acado.Process</a>');
                
            end
            
            ACADO_.helper.addInstruction(obj);
            
        end
        
        setProcessDisturbance(obj, varargin)
        initializeAlgebraicStates(obj, varargin)
        getInstructions(obj, cppobj, get)
        
    end
    
end



