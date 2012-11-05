%Product of expressions.
%
%  Usage:
%    >> Product(obj1, obj2);
%
%  Parameters:
%    obj1 	    [Expression]
%    obj2       [Expression]
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
classdef Product < acado.BinaryOperator
    properties(SetAccess='private')
        
    end
    
    methods
        function obj = Product(obj1, obj2)
            if ~isempty(obj1) && ~isempty(obj2)
                if (isa(obj1, 'numeric'))
                    obj1 = acado.DoubleConstant(obj1);
                elseif (isa(obj2, 'numeric'))
                    obj2 = acado.DoubleConstant(obj2);
                end
                
                if obj1.zero || obj2.zero
                    obj.zero = 1;
                end
                obj.obj1 = obj1;
                obj.obj2 = obj2;
            end
        end
        
        function s = toString(obj)
            if obj.zero
                s = '0';
            elseif obj.obj1.one
                s = sprintf('%s', obj.obj2.toString);
            elseif obj.obj2.one
                s = sprintf('%s', obj.obj1.toString);
            else
                s = sprintf('%s*%s', obj.obj1.toString, obj.obj2.toString);
            end
        end
        
        function jac = jacobian(obj, var)
            if ~isvector(obj)
                error('A jacobian can only be computed of a vector function.');
            end
            for i = 1:length(obj)
                if obj(i).zero
                    jac(i,:) = zeros(1,length(var));
                elseif obj(i).obj1.one
                    jac(i,:) = jacobian(obj(i).obj2, var);
                elseif obj(i).obj2.one
                    jac(i,:) = jacobian(obj(i).obj1, var);
                else
                    jac(i,:) = obj(i).obj1*jacobian(obj(i).obj2, var) + obj(i).obj2*jacobian(obj(i).obj1, var);
                end
            end
        end
    end
    
end

