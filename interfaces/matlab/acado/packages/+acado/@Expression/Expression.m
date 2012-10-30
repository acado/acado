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
        
        zero = 0;
        one = 0;
    end
    
    methods
        function obj = Expression( varargin )
            
        end
        
        function r = dot(obj1, b, dim)
            for i = 1:size(obj1,1)
                for j = 1:size(obj1,2)
                    r(i,j) = acado.Dot(obj1(i,j));
                end
            end
        end
        
        function r = next(obj1, b, dim)
            for i = 1:size(obj1,1)
                for j = 1:size(obj1,2)
                    r(i,j) = acado.Next(obj1(i,j));
                end
            end
        end
        
        %Matlab help: "Implementing Operators for Your Class"
        
        function r = mtimes(obj1,obj2)    % *
            if size(obj1,2) ~= size(obj2,1)
                error('ERROR: Invalid acado.Product. Check your dimensions..');
            end
            for i = 1:size(obj1,1)
                for j = 1:size(obj2,2)
                    r(i,j) = acado.Addition(acado.Product(obj1(i,1),obj2(1,j)));
                    for k = 2:size(obj1,2)
                        r(i,j) = acado.Addition(r(i,j), acado.Product(obj1(i,k),obj2(k,j)));
                    end
                end
            end
        end
        
        function r = times(obj1,obj2)    % .*
            if isa(obj2,'numeric')
                for i = 1:size(obj1,1)
                    for j = 1:size(obj1,2)
                        r(i,j) = acado.Addition(acado.Product(obj1(i,j),obj2));
                    end
                end
            elseif isa(obj1,'numeric')
                for i = 1:size(obj2,1)
                    for j = 1:size(obj2,2)
                        r(i,j) = acado.Addition(acado.Product(obj2(i,j),obj1));
                    end
                end
            else
                if size(obj1,1) ~= size(obj2,1) || size(obj1,2) ~= size(obj2,2)
                    error('ERROR: Invalid acado.Product. Check your dimensions..');
                end
                for i = 1:size(obj1,1)
                    for j = 1:size(obj1,2)
                        r(i,j) = acado.Addition(acado.Product(obj1(i,j),obj2(i,j)));
                    end
                end
            end
        end
        
        function r = plus(obj1,obj2)      % +
            if size(obj1,1) ~= size(obj2,1) || size(obj1,2) ~= size(obj2,2)
                error('ERROR: Invalid acado.Addition. Check your dimensions..');
            end
            for i = 1:size(obj1,1)
                for j = 1:size(obj1,2)
                    r(i,j) = acado.Addition(obj1(i,j),obj2(i,j));
                end
            end
        end
        
        function r = minus(obj1,obj2)     % -
            if size(obj1,1) ~= size(obj2,1) || size(obj1,2) ~= size(obj2,2)
                error('ERROR: Invalid acado.Addition. Check your dimensions..');
            end
            for i = 1:size(obj1,1)
                for j = 1:size(obj1,2)
                    r(i,j) = acado.Subtraction(obj1(i,j),obj2(i,j));
                end
            end
        end
        
        function r = mrdivide(obj1,obj2)  % /
            if numel(obj1) > 1 || numel(obj2) > 1
                error('ERROR: Invalid division !');
            end
            r = acado.Quotient(obj1,obj2);
        end
        
        function r = rdivide(obj1,obj2)    % .*
            if isa(obj2,'numeric')
                for i = 1:size(obj1,1)
                    for j = 1:size(obj1,2)
                        r(i,j) = acado.Addition(acado.Quotient(obj1(i,j),obj2));
                    end
                end
            else
                if size(obj1,1) ~= size(obj2,1) || size(obj1,2) ~= size(obj2,2)
                    error('ERROR: Invalid acado.Quotient. Check your dimensions..');
                end
                for i = 1:size(obj1,1)
                    for j = 1:size(obj1,2)
                        r(i,j) = acado.Addition(acado.Quotient(obj1(i,j),obj2(i,j)));
                    end
                end
            end
        end
        
        function r = uminus(obj1)         % -
            for i = 1:size(obj1,1)
                for j = 1:size(obj1,2)
                    r(i,j) = acado.Subtraction(obj1(i,j));
                end
            end
        end
        
        function r = uplus(obj1)          % +
            for i = 1:size(obj1,1)
                for j = 1:size(obj1,2)
                    r(i,j) = acado.Addition(obj1(i,j));
                end
            end
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
            for k = 1 : nargin,
                C(k,1) = acado.Addition(varargin{k});
            end
        end
        
        function C = horzcat(varargin)
            for k = 1 : nargin,
                C(1,k) = acado.Addition(varargin{k});
            end
        end
        
        function display(x)
            fprintf('\n%s = \n\n', inputname(1));
            for i = 1:size(x,1)
                for j = 1:size(x,2)
                    fprintf('%s ', toString(x(i,j)));
                end
                fprintf('\n');
            end
            fprintf('\n');
        end
        
        function out = ctranspose(in)
            out = transpose(in);
        end
        
        function out = transpose(in)
            for i = 1:size(in,1)
                for j = 1:size(in,2)
                    out(j,i) = in(i,j);
                end
            end
        end
    end
    
    
end

