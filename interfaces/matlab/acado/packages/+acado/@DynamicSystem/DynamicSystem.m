%Stores a DifferentialEquation together with an OutputFcn.
% The class DynamicSystem is a data class for storing a DifferentialEquation together 
% with an OutputFcn. The dynamic system might be of hybrid nature, i.e. differential 
% equation and output function might switch depending on a state-dependend
% switch function.
%
%  Usage:
%    >> Disturbance(differentialEquation, outputFunction);
%
%  Parameters:
%    differentialEquation 	   [acado.DifferentialEquation]
%    outputFunction 	       [acado.OutputFcn]
%
%  Example:
%    >> Disturbance w;
%
%  See also:
%    acado.OutputFcn
%    acado.DifferentialEquation
%
%  Licence:
%    This file is part of ACADO Toolkit  - (http://www.acadotoolkit.org/)
%
%    ACADO Toolkit -- A Toolkit for Automatic Disturbance and Dynamic Optimization.
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
classdef DynamicSystem < handle    
    properties(SetAccess='private')
        differentialEquation;
        outputFunction;
        name = 'dynamicsystem';
    end
    
    methods
        function obj = DynamicSystem(varargin)
           
            global ACADO_;
            ACADO_.count_other = ACADO_.count_other+1;
            obj.name = strcat(obj.name, num2str(ACADO_.count_other));
            
            
            if (nargin == 2 && isa(varargin{1}, 'acado.DifferentialEquation') && isa(varargin{2}, 'acado.OutputFcn'))
               
                obj.differentialEquation = varargin{1};
                obj.outputFunction = varargin{2};
                
            else
                
                error('ERROR: Invalid DynamicSystem call. <a href="matlab: help acado.DynamicSystem">help acado.DynamicSystem</a>');
                
            end
            
            ACADO_.helper.addInstruction(obj);
        end
        
        
        getInstructions(obj, cppobj, get)
        
    end
    
end


