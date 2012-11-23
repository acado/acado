%Allows to setup and evaluate output functions based on SymbolicExpressions.
%
%  Usage:
%    >>
%
%  Parameters:
%
%  Example:
%    >>
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
classdef OutputFcn < acado.Function
    properties(SetAccess='private')
        
        % Output eq
        outputList = {};
    end
    
    methods
        
        function obj = OutputFcn(varargin)
           checkActiveModel;
           
        end
        
        
        function obj = subsasgn(obj, ind, rhs)
            indices = cell2mat(ind.subs);
            if(strcmp(ind.subs,':'))
                indices = 1:length(rhs);
            end
            
            if(~strcmp(ind.type,'()'))
                error('ERROR: only integer subscripts are currently supported.');
            end
            
            if isa(rhs, 'cell')
                for i = 1:length(indices)
                    if (isa(rhs{i}, 'acado.Expression'))
                        obj.outputList{indices(i)} = rhs{i};
                    else
                        error('ERROR: Invalid OutputFcn add call.');
                    end
                end
            elseif isa(rhs, 'acado.Expression')
                for i = 1:length(indices)
                    obj.outputList{indices(i)} = rhs(i);
                end
            end
        end
        
        
        function getInstructions(obj, cppobj, get)
            if (get == 'B')
%                 if (~isempty(obj.outputList))
                    
                    fprintf(cppobj.fileMEX,sprintf('    OutputFcn %s;\n', obj.name));
                    
                    for i=1:length(obj.outputList)
                        fprintf(cppobj.fileMEX,sprintf('    %s << %s;\n', obj.name, obj.outputList{i}.toString()));
                    end
                    
                    fprintf(cppobj.fileMEX,'\n');
%                 end
            end
        end
        
    end
    
end

