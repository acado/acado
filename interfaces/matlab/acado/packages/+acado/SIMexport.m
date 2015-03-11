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

classdef SIMexport < acado.ExportModule & acado.ModelContainer
    properties (SetAccess='protected')
        
        simIntervals;
        totalTime;
        timingSteps = 0;
        
        % OutputEquation
        output = {};
        outputName = {};
        outputDiffsName = {};
        outputDim = {};
        outputColIndName = {};
        outputRowPtrName = {};
        outputDefined = [];
        meas = {};
    end
    
    methods
        
        function obj = SIMexport(varargin)
            checkActiveModel;
            
            global ACADO_;
            ACADO_.count_other = ACADO_.count_other+1;
            obj.name = strcat(obj.name, num2str(ACADO_.count_other));
            
            if (nargin == 2 )  % SIMexport( simIntervals, totalTime )
                obj.simIntervals = varargin{1};
                obj.totalTime = varargin{2};
                
            elseif (nargin == 1)  % SIMexport( totalTime )
                obj.simIntervals = 1;
                obj.totalTime = varargin{1};
                
            end
            
        end
        
        
        function exportAndRun(obj, varargin)
            
            if (nargin == 2 && isa(varargin{1}, 'char'))
                % SIMexport.exportAndRun( dirName );
                obj.dir = varargin{1};
                obj.run = 1;
                
            else
                error('ERROR: Invalid exportAndRun. <a href="matlab: help acado.SIMexport.exportAndRun">help acado.SIMexport.exportAndRun</a>');
                
            end
            
            GEN_ACADO; 
        end
        
        
        function addOutput(obj, varargin)
            
            global ACADO_;
            if (nargin == 3 && isa(varargin{1}, 'acado.OutputFcn') && (isnumeric(varargin{2}) || isa(varargin{2}, 'acado.Vector')))
                obj.outputDefined(end+1) = 1;
                % SIMexport.addOutput( h, meas );
                obj.output{end+1} = varargin{1};
                obj.meas{end+1} = varargin{2};
                
            elseif (nargin == 3 && (isa(varargin{1}, 'cell') || isa(varargin{1}, 'acado.Expression')) && (isnumeric(varargin{2}) || isa(varargin{2}, 'acado.Vector')))
                if ~isvector(varargin{1})
                    error('ERROR: Please provide a vector of expressions instead of a matrix.');
                end
                obj.outputDefined(end+1) = 0;
                tmp = acado.OutputFcn();
                ACADO_.helper.removeInstruction(tmp);
                tmp(:) = varargin{1};
                obj.output{end+1} = tmp;
                obj.meas{end+1} = varargin{2};
                
            elseif (nargin == 5 && isa(varargin{1}, 'char') && isa(varargin{2}, 'char') && isa(varargin{3}, 'numeric') && (isnumeric(varargin{4}) || isa(varargin{4}, 'acado.Vector')))
                obj.outputDefined(end+1) = 1;
                % SIMexport.addOutput( output, diffs_output, dim, meas );
                obj.outputName{end+1} = varargin{1};
                obj.outputDiffsName{end+1} = varargin{2};
                obj.outputDim{end+1} = varargin{3};
                obj.meas{end+1} = varargin{4};
                
            elseif (nargin == 7 && isa(varargin{1}, 'char') && isa(varargin{2}, 'char') && isa(varargin{3}, 'numeric')) && (isnumeric(varargin{4}) || isa(varargin{4}, 'acado.Vector')) && isa(varargin{5}, 'char')  && isa(varargin{6}, 'char') 
                obj.outputDefined(end+1) = 1;
                % SIMexport.addOutput( output, diffs_output, dim, meas, colInd, rowPtr );
                obj.outputName{end+1} = varargin{1};
                obj.outputDiffsName{end+1} = varargin{2};
                obj.outputDim{end+1} = varargin{3};
                obj.meas{end+1} = varargin{4};
                obj.outputColIndName{end+1} = varargin{5};
                obj.outputRowPtrName{end+1} = varargin{6};
                
            else
                error('ERROR: Invalid output added.');
            end
            
        end
        
        
        function setTimingSteps(obj, varargin)
            
            if (nargin == 2 && isa(varargin{1}, 'numeric'))
                % SIMexport.setTimingSteps( timingSteps );
                obj.timingSteps = acado.DoubleConstant(varargin{1});
                
            else
                error('ERROR: Invalid setTimingSteps. <a href="matlab: help acado.SIMexport.setTimingSteps">help acado.SIMexport.setTimingSteps</a>');
            end
        end
        
        
        function getInstructions(obj, cppobj, get)
            
            if (get == 'FB')
                % SET LOGGER TO LVL_DEBUG
                if obj.debugMode
                    fprintf(cppobj.fileMEX,'    Logger::instance().setLogLevel( LVL_DEBUG );\n');
                end
            elseif (get == 'B')
                
                % HEADER
                if (~isempty(obj.totalTime))
                    fprintf(cppobj.fileMEX,sprintf('    SIMexport %s( %s, %s );\n', obj.name, num2str(obj.simIntervals), num2str(obj.totalTime)));
                    
                elseif (~isempty(obj.simIntervals))
                    fprintf(cppobj.fileMEX,sprintf('    SIMexport %s( %s );\n', obj.name, num2str(obj.simIntervals)));
                    
                else
                    fprintf(cppobj.fileMEX,sprintf('    SIMexport %s( );\n', obj.name));
                    
                end
                
                % INTEGRATION GRID
                if (~isempty(obj.integrationGrid))
                    fprintf(cppobj.fileMEX,sprintf('    %s.setIntegrationGrid( %s );\n', obj.name, obj.integrationGrid.name));
                end
                
                % OPTIONS
                fprintf(cppobj.fileMEX,sprintf('    %s.set( GENERATE_MATLAB_INTERFACE, 1 );\n', obj.name));
                getOptions(obj, cppobj);
                
                
                % DIFFERENTIAL EQUATION
                obj.getModelFormulation(cppobj, get);
                
                
                % OUTPUT EQUATION
                numOutputs = 0;
                if (~isempty(obj.output))
                    numOutputs = length(obj.output);
                    if (numOutputs ~= length(obj.outputDefined))
                        error('INTERNAL ERROR: Invalid SIMexport object in getInstructions !\n');
                    end
                    for i = 1:numOutputs
                        if (~obj.outputDefined(i))
                           obj.output{i}.getInstructions(cppobj, get);
                        end
                        if isnumeric(obj.meas{i})
                            fprintf(cppobj.fileMEX,sprintf('    %s.addOutput( %s, %s );\n', obj.name, obj.output{i}.name, num2str(obj.meas{i})));
                        else
                            fprintf(cppobj.fileMEX,sprintf('    %s.addOutput( %s, %s );\n', obj.name, obj.output{i}.name, obj.meas{i}.name)); 
                        end
                    end
                    
                elseif (~isempty(obj.outputName))
                    if (length(obj.outputName) ~= length(obj.outputDiffsName) || length(obj.outputName) ~= length(obj.outputDim))
                        error('ERROR: Invalid SIMexport object in getInstructions !\n');
                    end
                    numOutputs = length(obj.outputName);
                    if (~isempty(obj.outputColIndName)) && (length(obj.outputColIndName) == length(obj.outputRowPtrName)) && (length(obj.outputColIndName) == numOutputs)
                        for i = 1:numOutputs
                            if isnumeric(obj.meas{i})
                                fprintf(cppobj.fileMEX,sprintf('    %s.addOutput( "%s", "%s", %s, %s, "%s", "%s" );\n', obj.name, obj.outputName{i}, obj.outputDiffsName{i}, num2str(obj.outputDim{i}), num2str(obj.meas{i}), obj.outputColIndName{i}, obj.outputRowPtrName{i}));
                            else
                                fprintf(cppobj.fileMEX,sprintf('    %s.addOutput( "%s", "%s", %s, %s, "%s", "%s" );\n', obj.name, obj.outputName{i}, obj.outputDiffsName{i}, num2str(obj.outputDim{i}), obj.meas{i}.name, obj.outputColIndName{i}, obj.outputRowPtrName{i}));
                            end
                        end
                    elseif (~isempty(obj.outputColIndName))
                        error('ERROR: Invalid SIMexport object in getInstructions !\n');
                    else
                        for i = 1:numOutputs
                            if isnumeric(obj.meas{i})
                                fprintf(cppobj.fileMEX,sprintf('    %s.addOutput( "%s", "%s", %s, %s );\n', obj.name, obj.outputName{i}, obj.outputDiffsName{i}, num2str(obj.outputDim{i}), num2str(obj.meas{i})));
                            else
                                fprintf(cppobj.fileMEX,sprintf('    %s.addOutput( "%s", "%s", %s, %s );\n', obj.name, obj.outputName{i}, obj.outputDiffsName{i}, num2str(obj.outputDim{i}), obj.meas{i}.name));
                            end
                        end
                    end
                end
                
                % EXPORT (AND RUN)
                fprintf(cppobj.fileMEX,'    uint export_flag = 0;\n');
                if (obj.run == 1 & ~isempty(obj.dir))
                    fprintf(cppobj.fileMEX,sprintf('    export_flag = %s.exportAndRun( "%s" );\n', obj.name, obj.dir));
                    
                elseif (~isempty(obj.dir))
                    % SET TIMING CALLS
                    fprintf(cppobj.fileMEX,sprintf('    %s.setTimingSteps( %s );\n', obj.name, num2str(obj.timingCalls)));
                    
                    fprintf(cppobj.fileMEX,sprintf('    export_flag = %s.exportCode( "%s" );\n', obj.name, obj.dir));
                    
                end
                fprintf(cppobj.fileMEX,'    if(export_flag != 0) mexErrMsgTxt(\"ACADO export failed because of the above error(s)!\");\n');
                
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
