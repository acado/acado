function linkMatlabDAE(obj, varargin)
%Link a matlab DAE black box model to ACADO
%
%  Usage:
%    >> DifferentialEquation.linkMatlabDAE(fcnHandleDAE)
%
%  Parameters:
%    fcnHandleDAE       Reference to function handle            [STRING]
%
%  Example:
%    >> f = acado.DifferentialEquation();
%    >> f.linkMatlabDAE('myDAE');
%
%    The file fcnHandleDAE should have this header: 
%       [ f ] = fcnHandleDAE( t,x,xa,u,p,w,dx )
%
%    Here t is a numeric containing the current time, x is a vector
%    containing all states, xa is a vector containing all algebraic
%    states, u a vector with all controls (if any), p a vector
%    with all parameters (if any) and w a vector with all disturbances (if any).
%
%    f will be a vector of length "length(x)+length(xa)" containing 
%       f(1): dot(x1) == ...
%       f(n): dot(xn) == ...
%       f(n+1): zero1 == ...
%       f(n+m): zerom == ...
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
%    Date: 2009-2010
% 


%error('This function (linkMatlabDAE) is not yet implemented. We are currenlty working on it.');



if (nargin ~= 2)
     error('ERROR in linkMatlabDAE. See <a href="matlab: help acado.DifferentialEquation.linkMatlabDAE">help acado.DifferentialEquation.linkMatlabDAE</a>');         
end

if (~isempty(obj.differentialList) || ~isempty(obj.matlabODE_fcnHandle) || ~isempty(obj.matlabDAE_fcnHandle) || ~isempty(obj.cfunction_file))
   error('Only _one_ Matlab DAE or ODE or C++ file can be linked. Or use ACADO symbolic notation.');
end


if (nargin == 2)   %
    
    fcnHandle = varargin{1};
    
    if(isempty(fcnHandle) || isvarname(fcnHandle) ~= 1)
        error('ERROR in linkMatlabDAE. fcnHandle should be a string refering to the name of a function. See <a href="matlab: help acado.DifferentialEquation.linkMatlabDAE">help acado.DifferentialEquation.linkMatlabDAE</a>');         
    end
    

    obj.matlabDAE_fcnHandle = fcnHandle;
    
end   

    

end