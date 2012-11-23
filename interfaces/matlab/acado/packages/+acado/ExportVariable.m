%ExportVariable
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
%    Author: Rien Quirynen
%    Date: 2012
%
classdef ExportVariable < handle
    properties(SetAccess='private')
        name;
        numRows = 1;
        numCols = 1;
    end
    
    methods
        function obj = ExportVariable(varargin)
            global ACADO_;
            if nargin > 0
                if ~ischar(varargin{1})
                    error('Unsupported use of the ExportVariable constructor: please specify a name.');
                end
                obj.name = varargin{1};
                if nargin == 2
                    obj.numRows = varargin{2};
                elseif nargin == 3
                    obj.numRows = varargin{2};
                    obj.numCols = varargin{3};
                elseif nargin > 3
                    error('Unsupported use of the ExportVariable constructor.');
                end
                
                ACADO_.helper.addExpV(obj);
                ACADO_.helper.addInstruction(obj);
            end
        end
        
        function s = toString(obj)
            s = obj.name;
        end
        
        function getInstructions(obj, cppobj, get)
            if (get == 'FB')
%                 if obj.numRows == 1 && obj.numCols == 1
%                     fprintf(cppobj.fileMEX,sprintf('    ExportVariable %s;\n', obj.name));
%                 else
                    fprintf(cppobj.fileMEX,sprintf('    ExportVariable %s( "%s", %s, %s );\n', obj.name, obj.name, num2str(obj.numRows), num2str(obj.numCols)));
%                 end
            end
            
        end
        
    end
    
end

