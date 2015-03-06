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
%    Author: David Ariens, Rien Quirynen
%    Date: 2012
%
classdef Expression < handle
    properties
        name;
        
        zero = 0;
        one = 0;
        
        singleTerm = 0;
        
        expr;
    end
    
    methods
        function obj = Expression( in )
            if nargin > 0
                for i = 1:size(in,1)
                    for j = 1:size(in,2)
                        if isa(in(i,j), 'numeric')
                            obj(i,j).expr = acado.DoubleConstant(in(i,j)); 
                        else
                            obj(i,j).expr = in(i,j).getExpression;
                        end
                    end
                end
            end
        end
        
        function out = copy(obj)
            if strcmp(class(obj), 'acado.Expression')
                out = acado.Expression(copy(obj.expr));
            else
                error(['Undefined copy constructor for class ' class(obj) ' !']);
            end
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
            if length(obj2) == 1
                for i = 1:size(obj1,1)
                    for j = 1:size(obj1,2)
                        r(i,j) = acado.Expression(acado.Product(obj1(i,j),obj2));
                    end
                end
            elseif length(obj1) == 1
                for i = 1:size(obj2,1)
                    for j = 1:size(obj2,2)
                        r(i,j) = acado.Expression(acado.Product(obj2(i,j),obj1));
                    end
                end
            else
                if size(obj1,2) ~= size(obj2,1)
                    error('ERROR: Invalid acado.Product. Check your dimensions..');
                end
                for i = 1:size(obj1,1)
                    for j = 1:size(obj2,2)
                        r(i,j) = acado.Expression(acado.Product(obj1(i,1),obj2(1,j)));
                        for k = 2:size(obj1,2)
                            r(i,j) = acado.Expression(acado.Addition(r(i,j), acado.Product(obj1(i,k),obj2(k,j))));
                        end
                    end
                end
            end
        end
        
        function r = times(obj1,obj2)    % .*
            if length(obj2) == 1
                for i = 1:size(obj1,1)
                    for j = 1:size(obj1,2)
                        r(i,j) = acado.Expression(acado.Product(obj1(i,j),obj2));
                    end
                end
            elseif length(obj1) == 1
                for i = 1:size(obj2,1)
                    for j = 1:size(obj2,2)
                        r(i,j) = acado.Expression(acado.Product(obj2(i,j),obj1));
                    end
                end
            else
                if size(obj1,1) ~= size(obj2,1) || size(obj1,2) ~= size(obj2,2)
                    error('ERROR: Invalid acado.Product. Check your dimensions..');
                end
                for i = 1:size(obj1,1)
                    for j = 1:size(obj1,2)
                        r(i,j) = acado.Expression(acado.Product(obj1(i,j),obj2(i,j)));
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
                    r(i,j) = acado.Expression(acado.Addition(obj1(i,j),obj2(i,j)));
                end
            end
        end
        
        function r = minus(obj1,obj2)     % -
            if size(obj1,1) ~= size(obj2,1) || size(obj1,2) ~= size(obj2,2)
                error('ERROR: Invalid acado.Subtraction. Check your dimensions..');
            end
            for i = 1:size(obj1,1)
                for j = 1:size(obj1,2)
                    r(i,j) = acado.Expression(acado.Subtraction(obj1(i,j),obj2(i,j)));
                end
            end
        end
        
        function r = mrdivide(obj1,obj2)  % /
            if length(obj2) == 1
                for i = 1:size(obj1,1)
                    for j = 1:size(obj1,2)
                        r(i,j) = acado.Expression(acado.Quotient(obj1(i,j),obj2));
                    end
                end
            else
                if numel(obj1) > 1 || numel(obj2) > 1
                    error('ERROR: Invalid division !');
                end
                r = acado.Quotient(obj1,obj2);
            end
        end
        
        function r = rdivide(obj1,obj2)    % ./
            if length(obj2) == 1
                for i = 1:size(obj1,1)
                    for j = 1:size(obj1,2)
                        r(i,j) = acado.Expression(acado.Quotient(obj1(i,j),obj2));
                    end
                end
            else
                if size(obj1,1) ~= size(obj2,1) || size(obj1,2) ~= size(obj2,2)
                    error('ERROR: Invalid acado.Quotient. Check your dimensions..');
                end
                for i = 1:size(obj1,1)
                    for j = 1:size(obj1,2)
                        r(i,j) = acado.Expression(acado.Quotient(obj1(i,j),obj2(i,j)));
                    end
                end
            end
        end
        
        function r = uminus(obj1)         % -
            for i = 1:size(obj1,1)
                for j = 1:size(obj1,2)
                    r(i,j) = acado.Expression(acado.Subtraction(acado.DoubleConstant(0),obj1(i,j)));
                end
            end
        end
        
        function r = uplus(obj1)          % +
            for i = 1:size(obj1,1)
                for j = 1:size(obj1,2)
                    r(i,j) = acado.Expression(obj1(i,j));
                end
            end
        end
        
        function r = mpower(obj1, obj2)   % ^
            if numel(obj1) > 1 || numel(obj2) > 1
                error('ERROR: Invalid power !');
            end
            r = acado.Power(obj1,obj2);
        end
        
        function r = power(obj1, obj2)   % .^
            if length(obj2) == 1
                for i = 1:size(obj1,1)
                    for j = 1:size(obj1,2)
                        r(i,j) = acado.Expression(acado.Power(obj1(i,j),obj2));
                    end
                end
            else
                if size(obj1,1) ~= size(obj2,1) || size(obj1,2) ~= size(obj2,2)
                    error('ERROR: Invalid acado.Power. Check your dimensions..');
                end
                for i = 1:size(obj1,1)
                    for j = 1:size(obj1,2)
                        r(i,j) = acado.Expression(acado.Power(obj1(i,j),obj2(i,j)));
                    end
                end
            end
        end
        
        function r = eq(obj1, obj2)       % ==
            if isnumeric(obj1) && length(obj1) == 1
                obj1 = obj1*ones(size(obj2));
            elseif isnumeric(obj2) && length(obj2) == 1
                obj2 = obj2*ones(size(obj1));
            end
            if isa(obj1, 'acado.Variable') && length(obj1) == 1 && size(obj2,2) == 2  % special case of a VariablesGrid
                r = acado.Equals(obj1,obj2);
            else
                if size(obj1,1) ~= size(obj2,1) || size(obj1,2) ~= size(obj2,2)
                    error('ERROR: Invalid acado.Equals. Check your dimensions..');
                end
                for i = 1:size(obj1,1)
                    for j = 1:size(obj1,2)
                        r(i,j) = acado.Equals(obj1(i,j),obj2(i,j));
                    end
                end
            end
        end
        
        function r = lt(obj1, obj2)       % <
            if isnumeric(obj1) && length(obj1) == 1
                obj1 = obj1*ones(size(obj2));
            elseif isnumeric(obj2) && length(obj2) == 1
                obj2 = obj2*ones(size(obj1));
            end
            if size(obj1,1) ~= size(obj2,1) || size(obj1,2) ~= size(obj2,2)
                error('ERROR: Invalid acado.LessThan. Check your dimensions..');
            end
            for i = 1:size(obj1,1)
                for j = 1:size(obj1,2)
                    r(i,j) = acado.LessThan(obj1(i,j),obj2(i,j));
                end
            end
        end
        
        function r = le(obj1, obj2)       % <=
            if isnumeric(obj1) && length(obj1) == 1
                obj1 = obj1*ones(size(obj2));
            elseif isnumeric(obj2) && length(obj2) == 1
                obj2 = obj2*ones(size(obj1));
            end
            if size(obj1,1) ~= size(obj2,1) || size(obj1,2) ~= size(obj2,2)
                error('ERROR: Invalid acado.LessThanEqual. Check your dimensions..');
            end
            for i = 1:size(obj1,1)
                for j = 1:size(obj1,2)
                    r(i,j) = acado.LessThanEqual(obj1(i,j),obj2(i,j));
                end
            end
        end
        
        function r = gt(obj1, obj2)       % >
            if isnumeric(obj1) && length(obj1) == 1
                obj1 = obj1*ones(size(obj2));
            elseif isnumeric(obj2) && length(obj2) == 1
                obj2 = obj2*ones(size(obj1));
            end
            if size(obj1,1) ~= size(obj2,1) || size(obj1,2) ~= size(obj2,2)
                error('ERROR: Invalid acado.GreaterThan. Check your dimensions..');
            end
            for i = 1:size(obj1,1)
                for j = 1:size(obj1,2)
                    r(i,j) = acado.GreaterThan(obj1(i,j),obj2(i,j));
                end
            end
        end
        
        function r = ge(obj1, obj2)       % >=
            if isnumeric(obj1) && length(obj1) == 1
                obj1 = obj1*ones(size(obj2));
            elseif isnumeric(obj2) && length(obj2) == 1
                obj2 = obj2*ones(size(obj1));
            end
            if size(obj1,1) ~= size(obj2,1) || size(obj1,2) ~= size(obj2,2)
                error('ERROR: Invalid acado.GreaterThanEqual. Check your dimensions..');
            end
            for i = 1:size(obj1,1)
                for j = 1:size(obj1,2)
                    r(i,j) = acado.GreaterThanEqual(obj1(i,j),obj2(i,j));
                end
            end
        end
        
        function r = exp(obj1)            % exp
            for i = 1:size(obj1,1)
                for j = 1:size(obj1,2)
                    r(i,j) = acado.Exp(obj1(i,j));
                end
            end
        end
        
        function r = sqrt(obj1)            % sqrt
            for i = 1:size(obj1,1)
                for j = 1:size(obj1,2)
                    r(i,j) = acado.SquareRoot(obj1(i,j));
                end
            end
        end
        
        function r = acos(obj1)           % acos
            for i = 1:size(obj1,1)
                for j = 1:size(obj1,2)
                    r(i,j) = acado.Acos(obj1(i,j));
                end
            end
        end
        
        function r = asin(obj1)           % asin
            for i = 1:size(obj1,1)
                for j = 1:size(obj1,2)
                    r(i,j) = acado.Asin(obj1(i,j));
                end
            end
        end
        
        function r = atan(obj1)           % atan
            for i = 1:size(obj1,1)
                for j = 1:size(obj1,2)
                    r(i,j) = acado.Atan(obj1(i,j));
                end
            end
        end
        
        function r = cos(obj1)            % cos
            for i = 1:size(obj1,1)
                for j = 1:size(obj1,2)
                    r(i,j) = acado.Cos(obj1(i,j));
                end
            end
        end
        
        function r = sin(obj1)            % sin
            for i = 1:size(obj1,1)
                for j = 1:size(obj1,2)
                    r(i,j) = acado.Sin(obj1(i,j));
                end
            end
        end
        
        function r = tan(obj1)            % tan
            for i = 1:size(obj1,1)
                for j = 1:size(obj1,2)
                    r(i,j) = acado.Tan(obj1(i,j));
                end
            end
        end
        
        function r = log(obj1)            % log
            for i = 1:size(obj1,1)
                for j = 1:size(obj1,2)
                    r(i,j) = acado.Logarithm(obj1(i,j));
                end
            end
        end
        
        function s = toString(obj)
            if ~isempty(obj.expr)
                s = obj.expr.toString; 
            else
                s = obj.name;
            end
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
            temp = varargin{1};
            for i = 1:size(temp,1)
                for j = 1:size(temp,2)
                    if isa(temp(i,j), 'numeric')
                        C(i,j) = acado.Expression(acado.DoubleConstant(temp(i,j)));
                    else
                        C(i,j) = acado.Expression(temp(i,j));
                    end
                end
            end
            for k = 2:nargin,
                temp = varargin{k};
                if isempty(varargin{1})
                    base = 0;
                else
                    base = size(C,1);
                end
                for i = 1:size(temp,1)
                    for j = 1:size(temp,2)
                        if isa(temp(i,j), 'numeric')
                            C(base+i,j) = acado.Expression(acado.DoubleConstant(temp(i,j)));
                        else
                            C(base+i,j) = acado.Expression(temp(i,j));
                        end
                    end
                end
            end
        end
        
        function C = horzcat(varargin)
            temp = varargin{1};
            for i = 1:size(temp,1)
                for j = 1:size(temp,2)
                    if isa(temp(i,j), 'numeric')
                        C(i,j) = acado.Expression(acado.DoubleConstant(temp(i,j)));
                    else
                        C(i,j) = acado.Expression(temp(i,j));
                    end
                end
            end
            for k = 2:nargin,
                temp = varargin{k};
                base = size(C,2);
                for i = 1:size(temp,1)
                    for j = 1:size(temp,2)
                        if isa(temp(i,j), 'numeric')
                            C(i,base+j) = acado.Expression(acado.DoubleConstant(temp(i,j)));
                        else
                            C(i,base+j) = acado.Expression(temp(i,j));
                        end
                    end
                end
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
        
        function out = sum(in, varargin)
            if nargin == 1
                out = sum(in(:),1);
            elseif nargin == 2
                if varargin{1} == 1
                    for i = 1:size(in,2)
                        out(1,i) = acado.Expression(in(1,i));
                        for j = 2:size(in,1)
                            out(1,i) = acado.Expression(acado.Addition(out(1,i), in(j,i)));
                        end
                    end
                elseif varargin{1} == 2
                    for i = 1:size(in,1)
                        out(i,1) = acado.Expression(in(i,1));
                        for j = 2:size(in,2)
                            out(i,1) = acado.Expression(acado.Addition(out(i,1), in(i,j)));
                        end
                    end
                else
                    error('Unsupported use of the sum function in acado.Expression.');
                end
            else
                error('Unsupported use of the sum function in acado.Expression.');
            end
        end
        
        % vector 2-norm only
        function out = norm(in)
            if(size(in,1) > 1 && size(in,2) > 1)
                error('Unsupported use of the 2-norm function in acado.Expression.');
            end
            out = sqrt(sum(in.^2));
        end
        
        function out = MatrixDiff(in,var)
            [n,m] = size(var);
            if length(in) > 1
                error('Dimensions of the input expression not supported.');
            end
            for i = 1:n
                for j = 1:m
                    out(i,j) = jacobian(in,var(i,j));
                end
            end
        end
        
        function D = diag(in)
            if size(in,1) == 1 && size(in,2) == 1
                D = in;
            elseif size(in,1) == 1 || size(in,2) == 1
                for i = 1:length(in)
                    for j = 1:length(in)
                        D(i,j) = acado.Expression(acado.DoubleConstant(0));
                    end
                    D(i,i) = in(i);
                end
            elseif size(in,1) == size(in,2)
                for i = 1:size(in,1)
                    D(i,1) = in(i,i);
                end
            else
                error('Unsupported use of the diag function.')
            end
        end
        
        function out = trace(in)
            out = sum(diag(in));
        end
        
        function out = simplify(obj)
            for i = 1:size(obj,1)
                for j = 1:size(obj,2)
                    out(i,j) = simplifyOne(copy(obj(i,j)));
                end
            end
        end
        
        function out = simplifyOne(obj)
           if length(obj) ~= 1
              error('Unsupported use of the function simplifyOne !'); 
           end
           changed = 1;
           while(changed && ~obj.singleTerm)
               prevString = toString(obj);
               
               while isa(obj, 'acado.MultiOperator') && length(obj.objs) == 1 && ~obj.contra
                   obj = obj.objs{1};
               end
               obj = simplifyLocally(obj);
               if isa(obj, 'acado.UnaryOperator')
                   obj.obj1 = simplifyOne(obj.obj1);
                   
               elseif isa(obj, 'acado.BinaryOperator')
                   obj.obj1 = simplifyOne(obj.obj1);
                   obj.obj2 = simplifyOne(obj.obj2);
                   
               elseif isa(obj, 'acado.MultiOperator')
                   for k = 1:length(obj.objs)
                       obj.objs{k} = simplifyOne(obj.objs{k});
                   end
               elseif strcmp(class(obj), 'acado.Expression') || isa(obj, 'acado.IntermediateState')
                   obj.expr = simplifyOne(obj.expr);
               end
               
               changed = ~strcmp(toString(obj), prevString);
           end
           out = obj;
        end
        
        function out = simplifyLocally(obj)
            % NOTHING TO BE DONE AT THIS LEVEL
            out = obj;
        end
        
        function out = getExpression(obj)
            if strcmp(class(obj), 'acado.Expression')
                out = obj.expr;
            else
                out = obj;
            end
        end
        
        function jac = jacobian(obj, var)
            for i = 1:length(obj)
                for j = 1:length(var)
                    jac(i,j) = acado.Expression(jacobian(obj(i).getExpression, var(j).getExpression));
                end
            end
        end
        
        function out = is(obj)
            out = acado.Expression(acado.IntermediateState(obj));
        end
        
        function out = eval(obj, varargin) % x, u, z, dx, od, p, w, t
            global ACADO_;
            if ~isempty(ACADO_)
                t = []; x = []; z = []; dx = []; u = []; od = []; p = []; w = [];
                if nargin > 1
                    x = varargin{1};
                    if nargin > 2
                        u = varargin{2};
                        if nargin > 3
                            z = varargin{3};
                            if nargin > 4
                                dx = varargin{4};
                                if nargin > 5
                                    od = varargin{5};
                                    if nargin > 6
                                        p = varargin{6};
                                        if nargin > 7
                                            w = varargin{7};
                                            if nargin > 8
                                                t = varargin{8};
                                            end
                                        end
                                    end
                                end
                            end
                        end
                    end
                end
                ACADO_.helper.setValues(t, x, z, dx, u, od, p, w);
                for i = 1:size(obj,1)
                    for j = 1:size(obj,2)
                        out(i,j) = evalin('base', obj(i,j).toString);
                    end
                end
                ACADO_.helper.clearValues;
            else
                error('Unsupported use of the eval function.');
            end
        end
        
    end
    
end
