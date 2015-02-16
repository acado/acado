%Allows to setup and evaluate a general function based on SymbolicExpressions.
% The class Function allows to setup and evaluate general functions based on SymbolicExpressions.
%
%  Usage:
%    >> Function({x1, x2, ...});
%
%  Parameters:
%    {} Expression as a cell
%
%
%  Example:
%    >> f = acado.Function({x,u,p});
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
classdef Function < handle    
    properties
       name = 'f';
       items = {};
    end
    
    methods
        function obj = Function(varargin)
            % This constuctor is also called by OutputFcn and DifferentialEquation
            
            
            if (nargin == 1 && ischar(varargin{1}))
                
                obj.name = ['"' varargin{1} '"'];
                
            else
                
                global ACADO_;
                ACADO_.count_function = ACADO_.count_function+1;
                obj.name = strcat('acadodata_f', num2str(ACADO_.count_function));
                
                if (nargin == 1  )
                    f = varargin{1};
                    
                    for i=1:length(f)
                        obj.items{i} = f(i);
                    end
                end
                
                ACADO_.helper.addInstruction(obj);  %also called by OutputFcn and DifferentialEquation!!
                
            end
           
        end 
        
        function s = toString(obj)
            s = obj.name;
        end
        
        getInstructions(obj, cppobj, get)
       
    end
    
end
