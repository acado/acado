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

classdef SIMexport < handle
    properties (SetAccess='private')
        name = 'SIMexport';
        
        simIntervals;
        totalTime;
        timingSteps;
        
        % DifferentialEquation
        model;
        
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
                
            elseif (nargin == 1)  % SIMexport( simIntervals )
                obj.simIntervals = acado.DoubleConstant(varargin{1});
                
            end
            
            ACADO_.helper.addInstruction(obj);
            
        end
        
        
        function exportCode(obj, varargin)
            
            if (nargin == 2 && isa(varargin{1}, 'char'))
                % SIMexport.exportCode( dirName );
                obj.dir = varargin{1};
                obj.run = 0;
                
            else
                error('ERROR: Invalid exportCode. <a href="matlab: help acado.SIMexport.exportCode">help acado.SIMexport.exportCode</a>');
                
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
            
        end
        
        
        function setModel(obj, varargin)
            
            if (nargin == 2 && isa(varargin{1}, 'acado.DifferentialEquation'))
                % SIMexport.setModel( f );
                obj.model = sprintf('%s', varargin{1}.name);
            else
                error('ERROR: Invalid setModel. <a href="matlab: help acado.SIMexport.setModel">help acado.SIMexport.setModel</a>');
                
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
            
            if (get == 'B')
                
                % HEADER
                if (~isempty(obj.totalTime))
                    fprintf(cppobj.fileMEX,sprintf('    SIMexport %s( %s, %s );\n', obj.name, obj.simIntervals.name, obj.totalTime.name));
                    
                elseif (~isempty(obj.simIntervals))
                    fprintf(cppobj.fileMEX,sprintf('    SIMexport %s( %s );\n', obj.name, obj.simIntervals.name));
                    
                else
                    fprintf(cppobj.fileMEX,sprintf('    SIMexport %s( );\n', obj.name));
                    
                end
                
                
                % DIFFERENTIAL EQUATION
                if (~isempty(obj.model))
                    fprintf(cppobj.fileMEX,sprintf('    %s.setModel( %s );\n', obj.name, obj.model));
                else
                    error('ERROR: Invalid SIMexport object in getInstructions !\n');
                end
                
                
                % EXPORT (AND RUN)
                
                if (obj.run == 1 & ~isempty(obj.dir))
                    fprintf(cppobj.fileMEX,sprintf('    %s.exportAndRun("%s");\n', obj.name, obj.dir));
                    
                elseif (~isempty(obj.dir))
                    fprintf(cppobj.fileMEX,sprintf('    %s.exportCode("%s");\n', obj.name, obj.dir));
                    
                end
                
                fprintf(cppobj.fileMEX,'\n');
            end
        end
        
    end
    
end

