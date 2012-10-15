%Base class for all variables within the symbolic expressions family.
% The class Expression serves as a base class for all symbolic variables within the
% symbolic expressions family. Moreover, the Expression class defines all kind of
% matrix and vector operations on a symbolic level.
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
%    Author: David Ariens
%    Date: 2010
%
classdef Expression < handle
    properties
        name;
    end
    
    methods
        function obj = Expression()
            
        end
        
        function r = dot(obj1, b, dim)
            r = acado.Dot(obj1);
        end
        
        function r = next(obj1, b, dim)
            r = acado.Next(obj1);
        end
        
        %Matlab help: "Implementing Operators for Your Class"
        
        function r = mtimes(obj1,obj2)    % *
            r = acado.Product(obj1,obj2);
        end
        
        function r = plus(obj1,obj2)      % +
            r = acado.Addition(obj1,obj2);
        end
        
        function r = minus(obj1,obj2)     % -
            r = acado.Subtraction(obj1,obj2);
        end
        
        function r = mrdivide(obj1,obj2)  % /
            r = acado.Quotient(obj1,obj2);
        end
        
        function r = uminus(obj1)         % -
            r = acado.Subtraction(obj1);
        end
        
        function r = uplus(obj1)          % +
            r = acado.Addition(obj1);
        end
        
        function r = mpower(obj1, obj2)   % ^
            r = acado.Power(obj1, obj2);
        end
        
        function r = eq(obj1, obj2)       % ==
            r = acado.Equals(obj1, obj2);
        end
        
        function r = lt(obj1, obj2)       % <
            r = acado.LessThan(obj1, obj2);
        end
        
        function r = le(obj1, obj2)       % <=
            r = acado.LessThanEqual(obj1, obj2);
        end
        
        function r = gt(obj1, obj2)       % >
            r = acado.GreaterThan(obj1, obj2);
        end
        
        function r = ge(obj1, obj2)       % >=
            r = acado.GreaterThanEqual(obj1, obj2);
        end
        
        function r = exp(obj1)            % exp
            r = acado.Exp(obj1);
        end
        
        function r = acos(obj1)           % acos
            r = acado.Acos(obj1);
        end
        
        function r = asin(obj1)           % asin
            r = acado.Asin(obj1);
        end
        
        function r = atan(obj1)           % atan
            r = acado.Atan(obj1);
        end
        
        function r = cos(obj1)            % cos
            r = acado.Cos(obj1);
        end
        
        function r = sin(obj1)            % sin
            r = acado.Sin(obj1);
        end
        
        function r = tan(obj1)            % tan
            r = acado.Tan(obj1);
        end
        
        function r = log(obj1)            % log
            r = acado.Logarithm(obj1);
        end
        
        function s = toString(obj)
            s = obj.name;
        end
        
        function result = checkDoubleVectorMatrix(obj, r)
            
            if (isa(r, 'acado.MexInputVector'))
                result = acado.Vector(r);
                
            elseif(isa(r, 'acado.MexInputMatrix'))
                result = acado.Matrix(r);
                
            elseif (isa(r, 'acado.MexInput'))
                result = acado.DoubleConstant(r);
                
            elseif (isa(r, 'acado.Expression'))
                result = r;
                
            else
                [m n] = size(r);
                
                if( m == 1 && n == 1)
                    result = acado.DoubleConstant(r);
                elseif( (m == 1 && n >= 1) || (m >= 1 && n == 1) )
                    result = acado.Vector(r);
                else
                    result = acado.Matrix(r);
                end
            end
            
        end
        
        function C = vertcat(varargin)
            C = {};
            for k = 1 : nargin,
                C{k} = varargin{k};
            end
        end
        
        function C = horzcat(varargin)
            C = {};
            for k = 1 : nargin,
                C{k} = varargin{k};
            end
        end
    end
    
    
end

