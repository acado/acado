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

classdef Options < handle
    properties (SetAccess='protected')
        
        %set
        set_n = {};
        set_v = {};
    end
    
    methods
        
        function obj = Options(varargin)
            checkActiveModel;
            
        end
        
        
        function set(obj, varargin)
            
            if (nargin == 3)
                % Options.set( name, val );
                obj.set_n{end+1} = varargin{1};
                obj.set_v{end+1} = varargin{2};
            else
                error('ERROR: Invalid set option. <a href="matlab: help acado.Options.set">help acado.Options.set</a>');
                
            end
            
        end
        
        
        function getOptions(obj, cppobj)
            
            fprintf(cppobj.fileMEX,'    uint options_flag;\n');
            for i=1:length(obj.set_n)
                
                if(ischar(obj.set_n{i}) && ischar(obj.set_v{i}))
                    fprintf(cppobj.fileMEX,sprintf('    options_flag = %s.set( %s, %s );\n', obj.name, obj.set_n{i}, obj.set_v{i}));
                    
                elseif(ischar(obj.set_n{i}) && isnumeric(obj.set_v{i}))
                    fprintf(cppobj.fileMEX,sprintf('    options_flag = %s.set( %s, %s );\n', obj.name, obj.set_n{i}, stringIntDouble(obj.set_v{i})));
                    
                elseif(ischar(obj.set_n{i}) && isa(obj.set_v{i}, 'acado.MexInput'))
                    fprintf(cppobj.fileMEX,sprintf('    options_flag = %s.set( %s, (int)%s );\n', obj.name, obj.set_n{i}, obj.set_v{i}.name));
                    
                else
                    error('ERROR: Invalid option given.');
                end
                fprintf(cppobj.fileMEX,['    if(options_flag != 0) mexErrMsgTxt(\"ACADO export failed when setting the following option: ' obj.set_n{i} '\");\n']);
            end
        end
        
    end
    
end
