%Dot of an acado Expression (derivative)
%
%  Usage:
%    >> dot(e);
%
%  Parameters:
%    e          [Expression]
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
classdef Dot < acado.Expression
    properties(SetAccess='private')
        obj1;
    end
    
    methods
        function obj = Dot(obj1)
            if nargin > 0
                obj.obj1 = obj1;
                global ACADO_;
                if isa(obj1, 'acado.DifferentialState')
                    ACADO_.helper.addDX(obj1);
                end
            end
        end
        
        function out = copy(obj)
            out = obj;
        end
        
        function s = toString(obj)
            s = sprintf('dot(%s)', obj.obj1.toString);
        end
        
        function jac = jacobian(obj, var)
            error('Jacobian feature not supported for expressions with state derivatives.')
        end
    end
    
end
