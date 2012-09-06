function init(obj, varargin)
%Initialization
%
%  Usage:
%    >> init( r )
%
%  Parameters:
%    r   MATRIX with Initialization values     [NUMERIC 1 x n matrix]
%
%  Example:
%    >> sim = acado.SimulationEnvironment( 0.0,3.0,process,controller );
%    >> r = zeros(1,4);
%    >> sim.init( r );
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

if (length(varargin) == 1) 
    r = varargin{1};
    [rm, rn] = size(r);
    
    if (rm ~= 1)
        error('ERROR: Invalid init(r) call. <a href="matlab: help acado.SimulationEnvironment.init">help acado.SimulationEnvironment.init</a>'); 
    end
    
    obj.initvector = acado.Vector(r);

    
else 
   error('ERROR: Invalid init call. <a href="matlab: help acado.SimulationEnvironment.init">help acado.SimulationEnvironment.init</a>'); 
end

end