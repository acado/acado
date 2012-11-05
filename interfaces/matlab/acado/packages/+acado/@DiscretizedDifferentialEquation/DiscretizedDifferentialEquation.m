%nonlinear autonomous discrete time system will be set up, i.e. the class 
% represtents a system of the form x_{k+1} = f( x_k, u_k, p, ... ) 
%
%  Usage:
%    >> DifferentialEquation();  If this constructor is used, the step length will be 1 by default. 
%    >> DifferentialEquation(stepLength);
%
%  Parameters:
%    stepLength 	step length     [NUMERIC]
%  
%  Example:
%    >> f = acado.DiscretizedDifferentialEquation();    % -> step length 1
%    >> f = acado.DiscretizedDifferentialEquation(2);
%
%  See also:
%    acado.DiscretizedDifferentialEquation.add             Adds a differential equation in symbolic syntax
%    acado.DiscretizedDifferentialEquation.linkMatlabODE   Links a matlab black box model
%    acado.DiscretizedDifferentialEquation.linkMatlabDAE
%    acado.DiscretizedDifferentialEquation.linkCFunction   Links a c function
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
%    Date: 2009-2012
% 
classdef DiscretizedDifferentialEquation  < acado.DifferentialEquation    
    properties
       
        stepLength;
        
    end
    
    methods
        function obj = DiscretizedDifferentialEquation(varargin)
            
            if (nargin == 1)

                if (isa(varargin{1}, 'acado.Expression'))
                    obj.stepLength = varargin{1};
                else
                    obj.stepLength = acado.DoubleConstant(varargin{1});    
                end
                
            elseif( nargin > 1)
                 error('ERROR in DiscretizedDifferentialEquation. See <a href="matlab: help acado.DiscretizedDifferentialEquation">help acado.DiscretizedDifferentialEquation</a>');         

            end
            
            
        end
        
        
        
        function r = getHeader(obj)
            
            if (~isempty(obj.stepLength))
                r = sprintf('DiscretizedDifferentialEquation %s(%s)',obj.name, obj.stepLength.name);

            else
                r = sprintf('DiscretizedDifferentialEquation %s',obj.name);
                
            end
            
        end
    end
    
end

