%Constant
%
%  Usage:
%    >> DoubleConstant(value);
%
%  Parameters:
%    name 	   A numeric [NUMERIC or acado.MexInput]
%
%  Example:
%    >> acado.DoubleConstant(1.5);
%    >> acado.DoubleConstant(acado.MexInput());
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
%    Date: 2009-2012
%
classdef DoubleConstant < acado.Operator
    properties(SetAccess='private')
        val;
        callByValue = 0;
    end
    
    methods
        function obj = DoubleConstant(val)
            if nargin > 0
                global ACADO_;
                
                if (isa(val, 'numeric'))
                    
                    obj.callByValue = 1;
                    obj.val = val;
                    if val == 0
                        obj.zero = 1;
                    elseif val == 1
                        obj.one = 1;
                    end
                    %                 ACADO_.count_double = ACADO_.count_double+1;
                    %
                    %                 obj.val = val;
                    %                 obj.name = strcat('acadoconstant', num2str(ACADO_.count_double));
                    %
                    %                 ACADO_.helper.addInstruction(obj);
                    obj.name = num2str(val);
                    
                elseif (isa(val, 'acado.MexInput'))
                    
                    if (val.type ~= 1)
                        error('MexInput should be in this case a numeric value, not a vector or matrix.');
                    end
                    
                    obj.name = val.name;
                    
                else
                    error('DoubleConstant expects a numeric value or a acado.MexInput');
                end
                obj.singleTerm = 1;
            end
        end
        
        function out = copy(obj)
            out = acado.DoubleConstant(obj.val);
        end
        
        
        getInstructions(obj, cppobj, get)
        
        
        function s = toString(obj)
            % toString is used in epxressions (eg 2 + x -> DoubleConstant +
            % DifferentialState)
            global ACADO_;
            
            if obj.callByValue
                if ~isempty(ACADO_) && ACADO_.generatingCode
                    if obj.val < 0
                        s = ['(' sprintf('%0.20e', obj.val) ')'];
                    else
                        s = sprintf('%0.20e', obj.val);
                    end
                else
                    if obj.val < 0
                        s = ['(' num2str(obj.val) ')'];
                    else
                        s = num2str(obj.val);
                    end
                end
            else
                s = obj.name;
            end
            
        end
        
        function jac = jacobian(obj, var)
            if ~isvector(obj)
                error('A jacobian can only be computed of a vector function.');
            end
            jac = zeros(length(obj), length(var));
        end
        
        function setToAbsoluteValue(obj)
            obj.val = abs(obj.val);
            obj.name = num2str(obj.val);
            if obj.val == 1
               obj.one = 1; 
            end
        end
    end
end

