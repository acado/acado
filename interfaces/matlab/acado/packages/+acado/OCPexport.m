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

classdef OCPexport < acado.ExportModule
    properties (GetAccess = 'public', SetAccess = 'protected')
        
        userName;
        
        % public read access, but private write access.
        Tc;
        Ncvp;
        Ts;
        intSteps;
    end
    properties (SetAccess='protected')
        
        ocp;
        
        printDimQP = 0;
    end
    
    methods
        
        
        function obj = OCPexport(varargin)
            checkActiveModel;
            
            global ACADO_;
            ACADO_.count_other = ACADO_.count_other+1;
            obj.name = strcat(obj.name, num2str(ACADO_.count_other));
            
            if nargin == 1      % OCPexport( ocp )
                obj.ocp = varargin{1};
            else
                error('Unsupported use of the OCPexport constructor.')
            end
            
        end
        
        
        function setName(obj, name)
            obj.userName = name;
        end
        
        
        function init(obj, varargin) % obj.init(Tc, Ncvp, intSteps)
            if nargin > 1
                if nargin ~= 4 || (length(varargin{1})+length(varargin{2})+length(varargin{3})) ~= 3
                    error('Unsupported use of the init function.');
                end
                obj.Tc = varargin{1};
                obj.Ncvp = varargin{2};
                obj.Ts = obj.Tc/obj.Ncvp;
                obj.intSteps = varargin{3};
            end
        end
        
        
        function printDimensionsQP(obj)
            obj.printDimQP = 1;
        end
        
        
        function setDebugMode(obj)
            obj.debugMode = 1;
        end
        
        
        function getInstructions(obj, cppobj, get)
            
            global ACADO_;
            NU = numel(ACADO_.helper.u);
            NP = numel(ACADO_.helper.p);
            NOD = numel(ACADO_.helper.od);
                
            if (get == 'FB')
                % SET LOGGER TO LVL_DEBUG
                if obj.debugMode
                    fprintf(cppobj.fileMEX,'    Logger::instance().setLogLevel( LVL_DEBUG );\n');
                end
            elseif (get == 'B')
                
                % HEADER
                if ~isempty(obj.ocp)
                    if ~acadoDefined(obj.ocp)
                        obj.ocp.getInstructions(cppobj, get);
                    end
                    
                    % Set number of controls and online data:
                    fprintf(cppobj.fileMEX,sprintf('    %s.setNU( %s );\n', obj.ocp.name, num2str(NU)));
                    fprintf(cppobj.fileMEX,sprintf('    %s.setNP( %s );\n', obj.ocp.name, num2str(NP)));
                    fprintf(cppobj.fileMEX,sprintf('    %s.setNOD( %s );\n', obj.ocp.name, num2str(NOD)));
                    
                    fprintf(cppobj.fileMEX,sprintf('    OCPexport %s( %s );\n', obj.name, obj.ocp.name));
                else
                    error('Unable to export a RTI algorithm without an OCP formulation.');
                end
                
                % SET USER SPECIFIED NAME
                if ~isempty(obj.userName)
                    fprintf(cppobj.fileMEX,sprintf('    %s.setName( "%s" );\n', obj.name, obj.userName));
                end
                
                % OPTIONS
                fprintf(cppobj.fileMEX,sprintf('    %s.set( GENERATE_MATLAB_INTERFACE, 1 );\n', obj.name));
                getOptions(obj, cppobj);
                
                % EXPORT
                if ~isempty(obj.dir)
                    fprintf(cppobj.fileMEX,'    uint export_flag;\n');
                    fprintf(cppobj.fileMEX,sprintf('    export_flag = %s.exportCode( "%s" );\n', obj.name, obj.dir));
                    fprintf(cppobj.fileMEX,'    if(export_flag != 0) mexErrMsgTxt(\"ACADO export failed because of the above error(s)!\");\n');
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
            ACADO_.helper.addMEX(dir)
        end
        
    end
    
end

