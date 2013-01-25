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

classdef MPCexport < acado.ExportOCP
    properties (GetAccess = 'public', SetAccess = 'protected')
        % Weighting matrices
        Q;
        R;
        S;
        K;
    end
    properties (SetAccess='protected')
        
        ocp;
        
        printDimQP = 0;
    end
    
    methods
        
        function obj = MPCexport(varargin)
            checkActiveModel;
            
            global ACADO_;
            ACADO_.count_other = ACADO_.count_other+1;
            obj.name = strcat(obj.name, num2str(ACADO_.count_other));
            
            if nargin == 1      % MPCexport( ocp )
                obj.ocp = varargin{1};
            else
                error('Unsupported use of the MPCexport constructor.')
            end
            
        end
        
        function printDimensionsQP(obj)
            obj.printDimQP = 1;
        end
        
        
        function getInstructions(obj, cppobj, get)
            
            if (get == 'B')
                
                % HEADER
                if ~isempty(obj.ocp)
                    if ~acadoDefined(obj.ocp)
                        obj.ocp.getInstructions(cppobj, get);
                    end
                    fprintf(cppobj.fileMEX,sprintf('    MPCexport %s( %s );\n', obj.name, obj.ocp.name));
                else
                    error('Unable to export a MPC algorithm without an OCP formulation.');
                end
                
                getOptions(obj, cppobj);
                
                % EXPORT
                if ~isempty(obj.dir)
                    fprintf(cppobj.fileMEX,sprintf('    %s.exportCode( "%s" );\n', obj.name, obj.dir));
                end
                
                % PRINT DIMENSIONS QP
                if obj.printDimQP
                    fprintf(cppobj.fileMEX,sprintf('    %s.printDimensionsQP( );\n', obj.name));
                end
                    
                fprintf(cppobj.fileMEX,'\n');
            end
        end
        
        
        function setMEXFiles(obj, dir)
            global ACADO_;
            if ~ischar(dir)
                error('Invalid directory name.');
            end
            ACADO_.helper.addMEX(dir, 'MPCstep.c')
        end
        
        
        function setMainFiles(obj, dir)
            global ACADO_;
            if ~ischar(dir)
                error('Invalid directory name.');
            end
            ACADO_.helper.addMain(dir, 'test.c', 'auxiliary_functions.c')
        end
        
        function setWeights(obj, varargin) % obj.setWeights(Q, R) or obj.setWeights(Q, R, S)
            if nargin > 4 || nargin < 3
                error('Unsupported use of the setWeights function.');
            end
            if length(diffStates) == size(varargin{1},1) && length(diffStates) == size(varargin{1},2)
                obj.Q = varargin{1};
            else
                error('Check the dimensions of the Q weighting matrix.');
            end
            if length(controls) == size(varargin{2},1) && length(controls) == size(varargin{2},2)
                obj.R = varargin{2};
            else
                error('Check the dimensions of the R weighting matrix.');
            end
            if nargin == 4
                if length(diffStates) == size(varargin{3},1) && length(diffStates) == size(varargin{3},2)
                    obj.S = varargin{3};
                else
                    error('Check the dimensions of the S weighting matrix.');
                end
            else
                computeTerminalCost(obj);
            end
        end
        
        function computeTerminalCost(obj)
            if isempty(obj.Xref) || isempty(obj.Uref)
               error('Unable to compute the terminal cost without the terminal values for the states and control inputs.'); 
            end
            if isempty(obj.ocp.model)
               error('Unable to compute the terminal cost without a system to linearize.');
            end
            global ACADO_;
            if ~isempty(ACADO_.helper.dx) || ~isempty(ACADO_.helper.z)
               error('This feature is currently only available for explicit ODE systems.'); 
            end
            jacX = jacobian(obj.ocp.model.getExpression, diffStates);
            jacU = jacobian(obj.ocp.model.getExpression, controls);
            A = jacX.eval(obj.Xref(end,:), obj.Uref(end,:));
            B = jacU.eval(obj.Xref(end,:), obj.Uref(end,:));
            
            [obj.K,obj.S,~] = lqr(A,B,obj.Q,obj.R);
        end
        
    end
    
end

