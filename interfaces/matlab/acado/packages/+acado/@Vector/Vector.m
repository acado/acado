%A vector built from standard Matlab matrix notations
%
%  Usage:
%    >> Vector([]);
%
%  Parameters:
%    [] vector
%
%
%  Example:
%    >> f = acado.Vector([1,2,3]);
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
classdef Vector < acado.VectorspaceElement    
    properties
       dim = 0;
    end
    
    methods
        function obj = Vector(val)
            if nargin > 0
	            global ACADO_;

                if (isa(val, 'numeric'))
	                ACADO_.count_vector = ACADO_.count_vector+1;
	                obj.name = strcat('acadodata_v', num2str(ACADO_.count_vector));
                
	                [m n] = size(val);
                
	                if (m == 1)
	                    obj.dim = n;
	                    obj.items = val;
	                elseif (n == 1)
	                    obj.dim = m;
	                    obj.items = val';
	                else
	                    error('Input should be a vector');
	                end
                
	                ACADO_.helper.addInstruction(obj);
                
                elseif (isa(val, 'acado.MexInput'))
                
	                if (val.type ~= 2)
	                    error('MexInput should be in this case a vector, not a scalar or matrix.'); 
	                end
                
                	obj.name = val.name;
                
            	else
	                error('Vector expects a numeric value or a acado.MexInput'); 
            	end     
			end
        end  
        
        getInstructions(obj, cppobj, get)

    end
    
end