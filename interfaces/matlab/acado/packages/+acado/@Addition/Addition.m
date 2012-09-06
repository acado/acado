%Addition of expressions.
%
%  Usage:
%    >> Addition(obj1, obj2);
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
%    Author: David Ariens
%    Date: 2009
% 
classdef Addition < acado.BinaryOperator
    properties(SetAccess='private')

    end
    
    methods
        function obj = Addition(varargin)
            if (nargin == 1)
                obj.obj2 = varargin{1};
                obj.obj1 = acado.EmptyWrapper();      
                
            elseif (nargin == 2)
                if (isa(varargin{1}, 'numeric'))
                    varargin{1} = acado.DoubleConstant(varargin{1});
                elseif (isa(varargin{2}, 'numeric'))
                    varargin{2} = acado.DoubleConstant(varargin{2});
                end

                obj.obj1 = varargin{1};
                obj.obj2 = varargin{2};
            else
                error('Invalid addition');
            end
        end
        
        function s = toString(obj)
            s = sprintf(' %s + %s ', obj.obj1.toString, obj.obj2.toString); 
        end
    end
    
end

