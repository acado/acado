function linkMatlabODE(obj, varargin)
%Link a matlab ODE black box model to ACADO
%
%  Usage:
%    >> DifferentialEquation.linkMatlabODE(fcnHandleODE)
%    >> DifferentialEquation.linkMatlabODE(fcnHandleODE, fcnHandleJacobian)
%
%  Parameters:
%    fcnHandleODE       Reference to function handle            [STRING]
%    fcnHandleJacobian  Reference to Jacobian function handle   [STRING]
%
%  Example:
%    >> f = acado.DifferentialEquation();
%    >> f.linkMatlabODE('myODE');
%    or
%    >> f.linkMatlabODE('myODE', 'myJacobian');
%
%    The file fcnHandleODE should have this header: 
%       [ dx ] = fcnHandleODE(t,x,u,p,w )
%
%    Analogous, the file fcnHandleJacobian will have this header:
%       [ J ] = fcnHandleJacobian( t,x,u,p,w )
%
%    Here t is a numeric containing the current time, x is a vector
%    containing all states, u a vector with all controls (if any), p a
%    vector with all parameters (if any) and w a vector with all
%    disturbances (if any).
%
%    dx will be a vector of length "length(x)" containing dot(x). J will be
%    a matrix of size "length(x)" times "length(x)+length(u)+length(p)+length(w)"
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
    
if (nargin ~= 2 && nargin ~= 3)
     error('ERROR in linkMatlabODE. See <a href="matlab: help acado.DifferentialEquation.linkMatlabODE">help acado.DifferentialEquation.linkMatlabODE</a>');         
end

if (~isempty(obj.differentialList) || ~isempty(obj.matlabODE_fcnHandle) || ~isempty(obj.matlabDAE_fcnHandle) || ~isempty(obj.cfunction_file))
   error('Only _one_ Matlab DAE or ODE or C++ file can be linked. Or use ACADO symbolic notation.');
end

if (nargin == 2 || nargin == 3)   % ODE
    
    fcnHandle = varargin{1};
    
    if(isempty(fcnHandle) || isvarname(fcnHandle) ~= 1)
        error('ERROR in linkMatlabODE. fcnHandle should be a string refering to the name of a function. See <a href="matlab: help acado.DifferentialEquation.linkMatlabODE">help acado.DifferentialEquation.linkMatlabODE</a>');         
    end
    
    %test = feval( fcnHandle,t,x,u,p,w ); % This line should run without warnings.
    
    obj.matlabODE_fcnHandle = fcnHandle;
    
end   

    
if (nargin == 3)                  % JACOBIAN
    
    fcnHandle = varargin{2};
    
    if(isempty(fcnHandle) || isvarname(fcnHandle) ~= 1)
        error('ERROR in linkMatlabJacobian. fcnHandle should be a string refering to the name of a function. See <a href="matlab: help acado.DifferentialEquation.linkMatlabODE">help acado.DifferentialEquation.linkMatlabODE</a>');         
    end
    
    obj.matlabJacobian_fcnHandle = fcnHandle;

end    
    

end