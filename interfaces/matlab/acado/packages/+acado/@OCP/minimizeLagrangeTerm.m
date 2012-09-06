function minimizeLagrangeTerm(obj, varargin)
% Adds an expression as a the Lagrange term to be minimized. 
%
%  IMPORTANT NOTICE: LAGRANGE TERMS ARE NOT YET IMPLEMENTED TO BE USED
%  TOGHETHER WITH MATLAB ODE CALLS. ONLY USE THEM WHEN DEFINING
%  A DIFFERENTIAL EQUATION INSIDE ACADO.
%
%  Usage:
%    >> ocp.minimizeLagrangeTerm(expression)
%
%  Parameters:
%    expression 	expression to be minimized     [  ]
%
%  Example:
%    >> ocp = acado.OCP(0.0, 1.0, 20);
%    >> ocp.minimizeLagrangeTerm(x);
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

    if (isa(varargin{1}, 'acado.Expression'))

        obj.minLagrangeTerms{end+1} = varargin{1};

    else
        error('ERROR: Invalid OCP.minimizeLagrangeTerm call. <a href="matlab: help acado.OCP.minimizeLagrangeTerm">help acado.OCP.minimizeLagrangeTerm</a>'); 

    end

end