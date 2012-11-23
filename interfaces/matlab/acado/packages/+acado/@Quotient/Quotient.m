%Quotient of expressions.
%
%  Usage:
%    >> Quotient(obj1, obj2);
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
classdef Quotient < acado.Product
    properties(SetAccess='private')
        
    end
    
    methods
        function obj = Quotient(varargin)
            i = 1;
            while i <= nargin
               if isa(varargin{i}, 'numeric')
                   varargin{i} = acado.DoubleConstant(varargin{i});
               end
               old = length(obj.objs);
               varargin{i} = getExpression(varargin{i});
               obj.concatenate(varargin{i});
               if i > 1
                    obj.contra(old+1:end) = obj.contra(old+1:end)+ones(size(obj.contra(old+1:end)));
                    obj.contra = mod(obj.contra, 2);
               end
               
               if i == 1 && varargin{i}.zero
                    obj.zero = 1;
               elseif varargin{i}.zero
                    error('DIVISION BY ZERO !');
               end
               i = i+1;
            end
            if nargin == 1
                obj.singleTerm = varargin{1}.singleTerm;
            end
        end
    end
    
end

