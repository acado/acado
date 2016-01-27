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
        NX1;
        NX2;
        NX3;
        NDX;
        NXA;
        NU;
        NOD;
        NP;
        
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
        
        % Nonlinear feedback system
        C;
        phi;
        nonlinearFeedback = 0;
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
        
        
        function setNonlinearFeedback(obj, varargin)
            
            global ACADO_;
            if (nargin == 3 && isa(varargin{1}, 'numeric') && isa(varargin{2}, 'acado.Expression'))
                if ~isvector(varargin{2})
                    error('ERROR: Please provide a vector of expressions instead of a matrix.');
                end
                % setNonlinearFeedback( C, phi );
                obj.C = acado.Matrix(varargin{1});
                obj.phi = acado.OutputFcn();
                ACADO_.helper.removeInstruction(obj.phi);
                obj.phi(:) = varargin{2};
                
            else
                error('ERROR: Invalid setNonlinearFeedback.');
                
            end
            obj.nonlinearFeedback = 1;
        end
        
        
        function setDimensions(obj, varargin)
            
            if (nargin == 5 && isa(varargin{1}, 'numeric') && isa(varargin{2}, 'numeric') && isa(varargin{3}, 'numeric') && isa(varargin{4}, 'numeric'))
                % SIMexport.setDimensions( NX, NU, NOD, NP );
                obj.NX1 = 0;
                obj.NX2 = varargin{1};
                obj.NX3 = 0;
                obj.NDX = 0;
                obj.NXA = 0;
                obj.NU = varargin{2};
                obj.NOD = varargin{3};
                obj.NP = varargin{4};
                
            elseif (nargin == 7 && isa(varargin{1}, 'numeric') && isa(varargin{2}, 'numeric') && isa(varargin{3}, 'numeric') && isa(varargin{4}, 'numeric') && isa(varargin{5}, 'numeric') && isa(varargin{6}, 'numeric'))
                % SIMexport.setDimensions( NX, NDX, NXA, NU, NOD, NP );
                obj.NX1 = 0;
                obj.NX2 = varargin{1};
                obj.NX3 = 0;
                obj.NDX = varargin{2};
                obj.NXA = varargin{3};
                obj.NU = varargin{4};
                obj.NOD = varargin{5};
                obj.NP = varargin{6};
                
            elseif (nargin == 9 && isa(varargin{1}, 'numeric') && isa(varargin{2}, 'numeric') && isa(varargin{3}, 'numeric') && isa(varargin{4}, 'numeric') && isa(varargin{5}, 'numeric') && isa(varargin{6}, 'numeric') && isa(varargin{7}, 'numeric') && isa(varargin{8}, 'numeric'))
                % SIMexport.setDimensions( NX1, NX2, NX3, NDX, NXA, NU, NOD, NP );
                obj.NX1 = varargin{1};
                obj.NX2 = varargin{2};
                obj.NX3 = varargin{3};
                obj.NDX = varargin{4};
                obj.NXA = varargin{5};
                obj.NU = varargin{6};
                obj.NOD = varargin{7};
                obj.NP = varargin{8};
                
            else
                error('ERROR: Invalid call to setDimensions.');
                
            end
            
        end
        
        function setNOD(obj, NOD)
            if( ~isnumeric(NOD) || NOD < 0 )
                error('ERROR: Invalid call to setNOD.');
            end
           obj.NOD = NOD; 
        end
        
        function setNP(obj, NP)
            if( ~isnumeric(NP) || NP < 0 )
                error('ERROR: Invalid call to setNP.');
            end
           obj.NP = NP; 
        end
        
        function setNU(obj, NU)
            if( ~isnumeric(NU) || NU < 0 )
                error('ERROR: Invalid call to setNU.');
            end
           obj.NU = NU; 
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
                if obj.linearInput
                    if ~isempty(obj.M1)
                        fprintf(cppobj.fileMEX,sprintf('    %s.setLinearInput( %s, %s, %s );\n', obj.name, obj.M1.name, obj.A1.name, obj.B1.name));
                    else
                        fprintf(cppobj.fileMEX,sprintf('    %s.setLinearInput( %s, %s );\n', obj.name, obj.A1.name, obj.B1.name));
                    end
                end
                if (~isempty(obj.NOD))
                    fprintf(cppobj.fileMEX,sprintf('    %s.setNOD( %s );\n', obj.name, num2str(obj.NOD)));
                end
                if (~isempty(obj.NP))
                    fprintf(cppobj.fileMEX,sprintf('    %s.setNP( %s );\n', obj.name, num2str(obj.NP)));
                end
                if (~isempty(obj.NU))
                    fprintf(cppobj.fileMEX,sprintf('    %s.setNU( %s );\n', obj.name, num2str(obj.NU)));
                end
                if (~isempty(obj.model))
                    if (~isempty(obj.model))
                        fprintf(cppobj.fileMEX,sprintf('    %s.setModel( %s );\n', obj.name, obj.model.name));
                    elseif (~isempty(obj.params))
                        fprintf(cppobj.fileMEX,sprintf('    %s.setNARXmodel( %d, %s );\n', obj.name, round(obj.delay), obj.params.name));
                    end
                    
                elseif (~isempty(obj.fileName) && ~isempty(obj.modelName) && ~isempty(obj.modelDiffsName))
                    fprintf(cppobj.fileMEX,sprintf('    %s.setModel( "%s", "%s", "%s" );\n', obj.name, obj.fileName, obj.modelName, obj.modelDiffsName));
                    if (isempty(obj.NX1) || isempty(obj.NX2) || isempty(obj.NX3) || isempty(obj.NDX) || isempty(obj.NXA) || isempty(obj.NU) || isempty(obj.NOD) || isempty(obj.NP))
                        error('ERROR: You need to provide the dimensions of the external model !\n');
                    else
                        fprintf(cppobj.fileMEX,sprintf('    %s.setDimensions( %s, %s, %s, %s, %s, %s, %s, %s );\n', obj.name, num2str(obj.NX1), num2str(obj.NX2), num2str(obj.NX3), num2str(obj.NDX), num2str(obj.NXA), num2str(obj.NU), num2str(obj.NOD), num2str(obj.NP)));
                    end
                else
                    % The model should then be defined elsewhere, e.g. with
                    % subjectTo in OCP
%                     error('ERROR: Invalid ModelContainer object in getInstructions !\n');
                end
                if obj.linearOutput
                    obj.fun3.getInstructions(cppobj, get);
                    if ~isempty(obj.M3)
                        fprintf(cppobj.fileMEX,sprintf('    %s.setLinearOutput( %s, %s, %s );\n', obj.name, obj.M3.name, obj.A3.name, obj.fun3.name));
                    else
                        fprintf(cppobj.fileMEX,sprintf('    %s.setLinearOutput( %s, %s );\n', obj.name, obj.A3.name, obj.fun3.name));
                    end
                end
                if obj.nonlinearFeedback
                    obj.phi.getInstructions(cppobj, get);
                    fprintf(cppobj.fileMEX,sprintf('    %s.setNonlinearFeedback( %s, %s );\n', obj.name, obj.C.name, obj.phi.name));
                end
                
                fprintf(cppobj.fileMEX,'\n');
            end
        end
        
    end
    
end

