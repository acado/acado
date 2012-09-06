function add(obj, varargin)
%Add an ordinary differential equation or differential algebraic equation
%in symbolic syntax.
%
%  Usage:
%    >> DifferentialEquation.add(expression)
%
%  Parameters:
%    expression  expression
%
%  Example:
%    >> f.add(-dot(x) -x*x + p + u*u + w); or f.add(dot(x) == -x*x + p + u*u + w);
%    >> f.add(0 ==  z + exp(z) - 1.0 + x);
%    >> f.add(dot(x) = sin(p)*cos(pi) + x);
%
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

if (~isempty(obj.matlabODE_fcnHandle) || ~isempty(obj.matlabDAE_fcnHandle) || ~isempty(obj.cfunction_file))
   error('Only _one_ Matlab DAE or ODE can be linked.');
end

    if (isa(varargin{1}, 'acado.Expression'))

        obj.differentialList{end+1} = varargin{1};

    else
        error('ERROR: Invalid DifferentialEquation.add call. <a href="matlab: help acado.DifferentialEquation.add">help acado.DifferentialEquation.add</a>'); 

    end
    
    

end