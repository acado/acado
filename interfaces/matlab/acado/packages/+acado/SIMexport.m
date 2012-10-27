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

classdef SIMexport < acado.ExportModule
    properties (SetAccess='protected')
        
        simIntervals;
        totalTime;
        timingSteps;
        
        % DifferentialEquation
        model;
        modelDefined = 1;
        fileName;
        modelName;
        modelDiffsName;
        NX;
        NDX;
        NXA;
        NU;
        
        % OutputEquation
        output = {};
        outputName = {};
        outputDiffsName = {};
        outputDim = {};
        meas;
        outputDefined = [];
        
        
        % dirName
        dir;
        run;
    end
    
    methods
        
        function obj = SIMexport(varargin)
            checkActiveModel;
            
            global ACADO_;
            ACADO_.count_other = ACADO_.count_other+1;
            obj.name = strcat(obj.name, num2str(ACADO_.count_other));
            
            if (nargin == 2 )  % SIMexport( simIntervals, totalTime )
                obj.simIntervals = acado.DoubleConstant(varargin{1});
                obj.totalTime = acado.DoubleConstant(varargin{2});
                
            elseif (nargin == 1)  % SIMexport( totalTime )
                obj.simIntervals = acado.DoubleConstant(1);
                obj.totalTime = acado.DoubleConstant(varargin{1});
                
            end
            
            ACADO_.helper.addInstruction(obj);
            
        end
        
        
        function exportCode(obj, varargin)
            
            if (nargin == 2 && isa(varargin{1}, 'char'))
                % SIMexport.exportCode( dirName );
                obj.dir = varargin{1};
                obj.run = 0;
                
            elseif (nargin == 1)
                % SIMexport.exportCode( );
                obj.dir = './';
                obj.run = 0;
                
            else
                error('ERROR: Invalid exportCode. <a href="matlab: help acado.SIMexport.exportCode">help acado.SIMexport.exportCode</a>');
                
            end
            
            END_ACADO; 
        end
        
        
        function exportAndRun(obj, varargin)
            
            if (nargin == 2 && isa(varargin{1}, 'char'))
                % SIMexport.exportAndRun( dirName );
                obj.dir = varargin{1};
                obj.run = 1;
                
            else
                error('ERROR: Invalid exportAndRun. <a href="matlab: help acado.SIMexport.exportAndRun">help acado.SIMexport.exportAndRun</a>');
                
            end
            
            END_ACADO; 
        end
        
        
        function setModel(obj, varargin)
            
            if (nargin == 2 && isa(varargin{1}, 'acado.DifferentialEquation'))
                % SIMexport.setModel( f );
                obj.model = varargin{1};
                
            elseif (nargin == 2 && isa(varargin{1}, 'cell'))
                obj.modelDefined = 0;
                obj.model = acado.DifferentialEquation();
                global ACADO_;
                ACADO_.helper.removeLastInstruction();
                obj.model(:) = varargin{1};
                
            elseif (nargin == 2 && isa(varargin{1}, 'acado.Expression'))
                obj.setModel({varargin{1}});
                
            elseif (nargin == 4 && isa(varargin{1}, 'char') && isa(varargin{2}, 'char') && isa(varargin{3}, 'char'))
                % SIMexport.setModel( fileName, modelName, modelDiffsName );
                obj.fileName = varargin{1};
                obj.modelName = varargin{2};
                obj.modelDiffsName = varargin{3};
                
            else
                error('ERROR: Invalid setModel. <a href="matlab: help acado.SIMexport.setModel">help acado.SIMexport.setModel</a>');
                
            end
            
        end
        
        
        function addOutput(obj, varargin)
            
            if (nargin == 2 && isa(varargin{1}, 'acado.OutputFcn'))
                obj.outputDefined(end+1) = 1;
                % SIMexport.addOutput( h );
                obj.output{end+1} = varargin{1};
                
            elseif (nargin == 2 && isa(varargin{1}, 'cell'))
                obj.outputDefined(end+1) = 0;
                tmp = acado.OutputFcn();
                global ACADO_;
                ACADO_.helper.removeLastInstruction();
                tmp(:) = varargin{1};
                obj.output{end+1} = tmp;
                
            elseif (nargin == 2 && isa(varargin{1}, 'acado.Expression'))
                obj.addOutput({varargin{1}});
                
            elseif (nargin == 4 && isa(varargin{1}, 'char') && isa(varargin{2}, 'char') && isa(varargin{3}, 'numeric'))
                obj.outputDefined(end+1) = 1;
                % SIMexport.addOutput( output, diffs_output, dim );
                obj.outputName{end+1} = varargin{1};
                obj.outputDiffsName{end+1} = varargin{2};
                obj.outputDim{end+1} = varargin{3};
                
            else
                error('ERROR: Invalid output added.');
            end
            
        end
        
        
        function setMeasurements(obj, varargin)
            
            if (nargin == 2 && isa(varargin{1}, 'numeric'))
                obj.meas = acado.Vector(varargin{1});
                
            elseif (nargin == 2 && isa(varargin{1}, 'acado.Vector'))
                obj.meas = varargin{1};
                
            else
                error('ERROR: Invalid measurements provided.');
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
        
        
        function getInstructions(obj, cppobj, get)
            
            if (get == 'B')
                
                % HEADER
                if (~isempty(obj.totalTime))
                    fprintf(cppobj.fileMEX,sprintf('    SIMexport %s( %s, %s );\n', obj.name, obj.simIntervals.name, obj.totalTime.name));
                    
                elseif (~isempty(obj.simIntervals))
                    fprintf(cppobj.fileMEX,sprintf('    SIMexport %s( %s );\n', obj.name, obj.simIntervals.name));
                    
                else
                    fprintf(cppobj.fileMEX,sprintf('    SIMexport %s( );\n', obj.name));
                    
                end
                
                getOptions(obj, cppobj);
                
                
                % DIFFERENTIAL EQUATION
                if (~obj.modelDefined)
                    obj.model.getInstructions(cppobj, get);
                end
                if (~isempty(obj.model))
                    fprintf(cppobj.fileMEX,sprintf('    %s.setModel( %s );\n', obj.name, obj.model.name));
                elseif (~isempty(obj.fileName) && ~isempty(obj.modelName) && ~isempty(obj.modelDiffsName))
                    fprintf(cppobj.fileMEX,sprintf('    %s.setModel( "%s", "%s", "%s" );\n', obj.name, obj.fileName, obj.modelName, obj.modelDiffsName));
                    if (isempty(obj.NX) || isempty(obj.NDX) || isempty(obj.NXA) || isempty(obj.NU))
                        error('ERROR: You need to provide the dimensions of the external model !\n');
                    else
                        fprintf(cppobj.fileMEX,sprintf('    %s.setDimensions( %s, %s, %s, %s );\n', obj.name, num2str(obj.NX), num2str(obj.NDX), num2str(obj.NXA), num2str(obj.NU)));
                    end
                else
                    error('ERROR: Invalid SIMexport object in getInstructions !\n');
                end
                
                
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
                        fprintf(cppobj.fileMEX,sprintf('    %s.addOutput( %s );\n', obj.name, obj.output{i}.name));
                    end
                    
                elseif (~isempty(obj.outputName))
                    if (length(obj.outputName) ~= length(obj.outputDiffsName) || length(obj.outputName) ~= length(obj.outputDim))
                        error('ERROR: Invalid SIMexport object in getInstructions !\n');
                    end
                    numOutputs = length(obj.outputName);
                    for i = 1:numOutputs
                        fprintf(cppobj.fileMEX,sprintf('    %s.addOutput( "%s", "%s", %s );\n', obj.name, obj.outputName{i}, obj.outputDiffsName{i}, num2str(obj.outputDim{i})));
                    end
                end
                if (numOutputs > 0 && numOutputs ~= obj.meas.dim)
                    error('ERROR: Invalid SIMexport object in getInstructions !\n');
                end
                if (~isempty(obj.meas))
                    fprintf(cppobj.fileMEX,sprintf('    %s.setMeasurements( %s );\n', obj.name, obj.meas.name));
                end
                
                
                % EXPORT (AND RUN)
                if (obj.run == 1 & ~isempty(obj.dir))
                    fprintf(cppobj.fileMEX,sprintf('    %s.exportAndRun( "%s" );\n', obj.name, obj.dir));
                    
                elseif (~isempty(obj.dir))
                    fprintf(cppobj.fileMEX,sprintf('    %s.exportCode( "%s" );\n', obj.name, obj.dir));
                    
                end
                
                fprintf(cppobj.fileMEX,'\n');
            end
        end
        
    end
    
end

