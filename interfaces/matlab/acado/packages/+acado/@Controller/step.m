function step(obj, varargin)
%Executes next single step. CPP code will be generated when this method is called
% When running your problem, the output struct will be {'U', 'P'} with U the
% optimized controls and P the optimized parameters.
%
%  Usage:
%    >> step( startTime )
%    >> step( startTime, x0 )
%    >> step( startTime, x0, y_ref )
%
%  Parameters:
%    startTime     [NUMERIC]
%    x0            [VECTOR]
%    y_ref         [MATRIX, first column are time points]
%
%  Example:
%    >> controller.step(0, x0);
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

obj.do_one_step = 1;


if (nargin == 2)
    obj.step_startTime = acado.DoubleConstant(varargin{1});

elseif (nargin == 3)
    obj.step_startTime = acado.DoubleConstant(varargin{1});
    obj.step_x0 = acado.Vector(varargin{2});
    
elseif (nargin == 4)
    obj.step_startTime = acado.DoubleConstant(varargin{1});
    obj.step_x0 = acado.Vector(varargin{2});
    obj.step_y_ref = acado.Matrix(varargin{3});
    
else
    error('ERROR: Invalid step(r) call. <a href="matlab: help acado.Controller.step">help acado.Controller.step</a>'); 

end



end