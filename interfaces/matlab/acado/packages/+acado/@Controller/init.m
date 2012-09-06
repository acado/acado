function init(obj, varargin)
%Initialization controller
%
%  Usage:
%    >> init( startTime )
%    >> init( startTime, x0 )
%    >> init( startTime, x0, p )
%    >> init( startTime, x0, p, y_ref )
%
%  Parameters:
%    startTime     [NUMERIC]
%    x0            [VECTOR]
%    p             [VECTOR]
%    y_ref         [MATRIX, first column are time points]
%
%
%  Example:
%    >> controller.init(0, x0);
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

obj.init_is_set = 1;

if (nargin == 2)
    obj.init_startTime = acado.DoubleConstant(varargin{1});

elseif (nargin == 3)
    obj.init_startTime = acado.DoubleConstant(varargin{1});
    obj.init_x0 = acado.Vector(varargin{2});
    
elseif (nargin == 4)
    obj.init_startTime = acado.DoubleConstant(varargin{1});
    obj.init_x0 = acado.Vector(varargin{2});
    obj.init_p = acado.Vector(varargin{3});
    
elseif (nargin == 5)
    obj.init_startTime = acado.DoubleConstant(varargin{1});
    obj.init_x0 = acado.Vector(varargin{2});
    obj.init_p = acado.Vector(varargin{3});
    obj.init_y_ref = acado.Matrix(varargin{4});
    
else
    error('ERROR: Invalid init(r) call. <a href="matlab: help acado.Controller.init">help acado.Controller.init</a>'); 

end



end