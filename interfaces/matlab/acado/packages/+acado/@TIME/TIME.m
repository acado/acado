%Implements the time within the family of Expressions. 
%
%  Usage:
%    >> TIME(t);
%    >> TIME t;                          SHORTHAND
%
%  Parameters:
%    name 	   A valid matlab variable name [STRING]
%
%  Example:
%    >> TIME t;
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
classdef TIME < acado.Expression   
    properties(SetAccess='protected')
        
    end
    
    methods
        function obj = TIME(varargin)
            global ACADO_;
            
            if (nargin == 2)
                if (isvarname(varargin{1}) ~= 1)
                    error( 'ERROR: The variable name you have set is not a valid matlab variable name. A valid variable name is a character string of letters, digits, and underscores, totaling not more than namelengthmax characters and beginning with a letter.' );
                end

                obj.name = varargin{1};
                ACADO_.helper.addT(obj);
                
            elseif (nargin == 1)
                if (isvarname(varargin{1}) ~= 1)
                    error( 'ERROR: The variable name you have set is not a valid matlab variable name. A valid variable name is a character string of letters, digits, and underscores, totaling not more than namelengthmax characters and beginning with a letter.' );
                end

                obj.name = varargin{1};
                ACADO_.helper.addT(obj);
                ACADO_.helper.addInstruction(obj);  
            
            else
                obj.name = 'autoTIME';
                ACADO_.helper.addT(obj);
                ACADO_.helper.addInstruction(obj);  
            end
            
            
        end
        
        getInstructions(obj, cppobj, get)

    end
    
end

