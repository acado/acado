%Short one line introduction
% Longer multiline
% text explaining all features
%
% Usage:
% >> class(obj1, obj2); Explain...
% >> class(obj1); Explain...
%
% Parameters:
% obj1 explain.... [NUMERIC]
% obj2 explain.... [NUMERIC/PARAMETER]
%
% Example:
% >> an example...
%
% See also:
% acado.class.method1
% acado.class.method2
%
% Licence:
% This file is part of ACADO Toolkit  (http://www.acadotoolkit.org/)
%
% ACADO Toolkit, toolkit for Automatic Control and Dynamic Optimization.
% Copyright (C) 20082009 by Boris Houska and Hans Joachim Ferreau,
% K.U.Leuven. Developed within the Optimization in Engineering Center
% (OPTEC) under supervision of Moritz Diehl. All rights reserved.
%
% ACADO Toolkit is free software; you can redistribute it and/or
% modify it under the terms of the GNU Lesser General Public
% License as published by the Free Software Foundation; either
% version 3 of the License, or (at your option) any later version.
%
% ACADO Toolkit is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
% Lesser General Public License for more details.
%
% You should have received a copy of the GNU Lesser General Public
% License along with ACADO Toolkit; if not, write to the Free Software
% Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
% MA 021101301 USA
%
% Author: Rien Quirynen
% Date: 2012

classdef ModelContainer < handle
    properties (SetAccess='protected')
        
        % Integration
        integrationGrid;
        
        % DifferentialEquation
        model;
        fileName;
        modelName;
        modelDiffsName;
        NX;
        NDX;
        NXA;
        NU;
        
        % NARX
        delay;
        params;
        
        % Linear input system
        M1;
        A1;
        B1;
        linearInput = 0;
        
        % Linear output system
        M3;
        A3;
        fun3;
        linearOutput = 0;
    end
    
    methods
        
        function obj = ModelContainer(varargin)
        end
        
        
        function setModel(obj, varargin)
            
            global ACADO_;
            if (nargin == 2 && isa(varargin{1}, 'acado.DifferentialEquation'))
                % SIMexport.setModel( f );
                obj.model = varargin{1};
                
            elseif (nargin == 2 && (isa(varargin{1}, 'cell') || isa(varargin{1}, 'acado.Expression')))
                if ~isvector(varargin{1})
                    error('ERROR: Please provide a vector of expressions instead of a matrix.');
                end
                obj.model = acado.DifferentialEquation();
                ACADO_.helper.removeInstruction(obj.model);
                obj.model(:) = varargin{1};
                
            elseif (nargin == 4 && isa(varargin{1}, 'char') && isa(varargin{2}, 'char') && isa(varargin{3}, 'char'))
                % SIMexport.setModel( fileName, modelName, modelDiffsName );
                obj.fileName = varargin{1};
                obj.modelName = varargin{2};
                obj.modelDiffsName = varargin{3};
                
            else
                error('ERROR: Invalid setModel.');
                
            end
        end
        
        function setNARXmodel(obj, varargin)
           if (nargin == 3 && isnumeric(varargin{1}) && varargin{1} > 0 && isnumeric(varargin{2}))
               obj.delay = varargin{1};
               obj.params = acado.Matrix(varargin{2});
           else
               error('ERROR: Invalid setNARXmodel.');
           end
        end
        
        
        function setLinearInput(obj, varargin)
            
            if (nargin == 3 && isa(varargin{1}, 'numeric') && isa(varargin{2}, 'numeric'))
                % setLinearInput( A1, B1 );
                obj.A1 = acado.Matrix(varargin{1});
                obj.B1 = acado.Matrix(varargin{2});
                
            elseif (nargin == 4 && isa(varargin{1}, 'numeric') && isa(varargin{2}, 'numeric') && isa(varargin{3}, 'numeric'))
                % setLinearInput( M1, A1, B1 );
                obj.M1 = acado.Matrix(varargin{1});
                obj.A1 = acado.Matrix(varargin{2});
                obj.B1 = acado.Matrix(varargin{3});
                
            else
                error('ERROR: Invalid setLinearInput.');
                
            end
            obj.linearInput = 1;
        end
        
        
        function setLinearOutput(obj, varargin)
            
            global ACADO_;
            if (nargin == 3 && isa(varargin{1}, 'numeric') && isa(varargin{2}, 'acado.Expression'))
                if ~isvector(varargin{2})
                    error('ERROR: Please provide a vector of expressions instead of a matrix.');
                end
                % setLinearOutput( A3, fun3 );
                obj.A3 = acado.Matrix(varargin{1});
                obj.fun3 = acado.OutputFcn();
                ACADO_.helper.removeInstruction(obj.fun3);
                obj.fun3(:) = varargin{2};
                
            elseif (nargin == 4 && isa(varargin{1}, 'numeric') && isa(varargin{2}, 'numeric') && isa(varargin{3}, 'acado.Expression'))
                if ~isvector(varargin{3})
                    error('ERROR: Please provide a vector of expressions instead of a matrix.');
                end
                % setLinearOutput( M3, A3, fun3 );
                obj.M3 = acado.Matrix(varargin{1});
                obj.A3 = acado.Matrix(varargin{2});
                obj.fun3 = acado.OutputFcn();
                ACADO_.helper.removeInstruction(obj.fun3);
                obj.fun3(:) = varargin{3};
                
            else
                error('ERROR: Invalid setLinearOutput.');
                
            end
            obj.linearOutput = 1;
        end
        
        
        function setDimensions(obj, varargin)
            
            if (nargin == 3 && isa(varargin{1}, 'numeric') && isa(varargin{2}, 'numeric'))
                % SIMexport.setDimensions( NX, NU );
                obj.NX = varargin{1};
                obj.NDX = 0;
                obj.NXA = 0;
                obj.NU = varargin{2};
                
            elseif (nargin == 5 && isa(varargin{1}, 'numeric') && isa(varargin{2}, 'numeric') && isa(varargin{3}, 'numeric') && isa(varargin{4}, 'numeric'))
                % SIMexport.setDimensions( NX, NDX, NXA, NU );
                obj.NX = varargin{1};
                obj.NDX = varargin{2};
                obj.NXA = varargin{3};
                obj.NU = varargin{4};
                
            else
                error('ERROR: Invalid call to setDimensions.');
                
            end
            
        end
        
        
        function setIntegrationGrid(obj, varargin)
           
            if (nargin == 2 && isa(varargin{1}, 'numeric'))
                obj.integrationGrid = acado.Vector( varargin{1} );
            else
                error('ERROR: Invalid call to setIntegrationGrid.');
            end
        end
        
        
        function getModelFormulation(obj, cppobj, get)
            
            if (get == 'B')
                
                % DIFFERENTIAL EQUATION
                if (~isempty(obj.model) && ~acadoDefined(obj.model))
                    obj.model.getInstructions(cppobj, get);
                end
                if (~isempty(obj.model) || obj.linearInput || obj.linearOutput)
                    if obj.linearInput
                        if ~isempty(obj.M1)
                            fprintf(cppobj.fileMEX,sprintf('    %s.setLinearInput( %s, %s, %s );\n', obj.name, obj.M1.name, obj.A1.name, obj.B1.name));
                        else
                            fprintf(cppobj.fileMEX,sprintf('    %s.setLinearInput( %s, %s );\n', obj.name, obj.A1.name, obj.B1.name));
                        end
                    end
                    if (~isempty(obj.model))
                        fprintf(cppobj.fileMEX,sprintf('    %s.setModel( %s );\n', obj.name, obj.model.name));
                    elseif (~isempty(obj.params))
                        fprintf(cppobj.fileMEX,sprintf('    %s.setNARXmodel( %d, %s );\n', obj.name, round(obj.delay), obj.params.name));
                    end
                    if obj.linearOutput
                        obj.fun3.getInstructions(cppobj, get);
                        if ~isempty(obj.M3)
                            fprintf(cppobj.fileMEX,sprintf('    %s.setLinearOutput( %s, %s, %s );\n', obj.name, obj.M3.name, obj.A3.name, obj.fun3.name));
                        else
                            fprintf(cppobj.fileMEX,sprintf('    %s.setLinearOutput( %s, %s );\n', obj.name, obj.A3.name, obj.fun3.name));
                        end
                    end
                    
                elseif (~isempty(obj.fileName) && ~isempty(obj.modelName) && ~isempty(obj.modelDiffsName))
                    fprintf(cppobj.fileMEX,sprintf('    %s.setModel( "%s", "%s", "%s" );\n', obj.name, obj.fileName, obj.modelName, obj.modelDiffsName));
                    if (isempty(obj.NX) || isempty(obj.NDX) || isempty(obj.NXA) || isempty(obj.NU))
                        error('ERROR: You need to provide the dimensions of the external model !\n');
                    else
                        fprintf(cppobj.fileMEX,sprintf('    %s.setDimensions( %s, %s, %s, %s );\n', obj.name, num2str(obj.NX), num2str(obj.NDX), num2str(obj.NXA), num2str(obj.NU)));
                    end
                else
                    % The model should then be defined elsewhere, e.g. with
                    % subjectTo in OCP
%                     error('ERROR: Invalid ModelContainer object in getInstructions !\n');
                end
                
                fprintf(cppobj.fileMEX,'\n');
            end
        end
        
    end
    
end

