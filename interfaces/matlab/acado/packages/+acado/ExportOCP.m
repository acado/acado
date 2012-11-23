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

classdef ExportOCP < acado.ExportModule
    properties (GetAccess = 'public', SetAccess = 'protected')
        % public read access, but private write access.
        Tc;
        Ncvp;
        Ts;
        intSteps;
        
        % Initial condition
        X0;
        U0;
        
        % References
        Tref;
        Xref;
        Uref;
        
        % Initial guess
        Tguess;
        Xguess;
        Uguess;
    end
    
    methods
        
        function obj = ExportOCP(varargin)
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
        
        function setInit(obj, varargin) % obj.setInit(X0) or obj.setInit(X0, U0)
            if nargin > 3
                error('Unsupported use of the setInit function.');
            end
            if nargin > 1 && length(diffStates) == length(varargin{1})
                obj.X0 = varargin{1};
                if nargin > 2 && length(controls) == length(varargin{2})
                    obj.U0 = varargin{2};
                elseif nargin > 2
                    error('Unsupported use of the setInit function.');
                end
            elseif nargin > 1
                error('Unsupported use of the setInit function.');
            end
        end
        
        function setRef(obj, varargin) % obj.setRef(), obj.setRef(Xref) or obj.setRef(Xref, Uref)
            if nargin > 3
                error('Unsupported use of the setRef function.');
            end
            obj.Tref = [0:obj.Ts:obj.Tc]';
            if nargin > 1 && length(diffStates) == size(varargin{1},2)
                if size(varargin{1},1) == 1
                    obj.Xref = repmat(varargin{1}, obj.Ncvp+1, 1);
                elseif size(varargin{1},1) == obj.Ncvp+1
                    obj.Xref = varargin{1};
                else
                    error('Unsupported use of the setRef function.');
                end
                if nargin > 2 && length(controls) == size(varargin{2},2)
                    if size(varargin{2},1) == 1
                        obj.Uref = repmat(varargin{2}, obj.Ncvp+1, 1);
                    elseif size(varargin{2},1) == obj.Ncvp+1
                        obj.Uref = varargin{2};
                    else
                        error('Unsupported use of the setRef function.');
                    end
                elseif nargin > 2
                    error('Unsupported use of the setRef function.');
                else
                    obj.Uref = repmat(obj.U0, obj.Ncvp+1, 1);
                end
            elseif nargin > 1
                error('Unsupported use of the setRef function.');
            else
                obj.Xref = repmat(obj.X0, obj.Ncvp+1, 1);
                obj.Uref = repmat(obj.U0, obj.Ncvp+1, 1);
            end
        end
        
        function setGuess(obj, varargin) % obj.setGuess(), obj.setGuess(Xguess) or obj.setGuess(Xguess, Uguess)
            if nargin > 3
                error('Unsupported use of the setGuess function.');
            end
            obj.Tguess = [0:obj.Ts:(obj.Tc-2*eps)]';
            if nargin > 1 && length(diffStates) == size(varargin{1},2)
                if size(varargin{1},1) == 1
                    obj.Xguess = repmat(varargin{1}, obj.Ncvp+1, 1);
                elseif size(varargin{1},1) == obj.Ncvp+1
                    obj.Xguess = varargin{1};
                else
                    error('Unsupported use of the setGuess function.');
                end
                if nargin > 2 && length(controls) == size(varargin{2},2)
                    if size(varargin{2},1) == 1
                        obj.Uguess = repmat(varargin{2}, obj.Ncvp, 1);
                    elseif size(varargin{2},1) == obj.Ncvp
                        obj.Uguess = varargin{2};
                    else
                        error('Unsupported use of the setGuess function.');
                    end
                elseif nargin > 2
                    error('Unsupported use of the setGuess function.');
                else
                    obj.Uguess = repmat(obj.U0, obj.Ncvp, 1);
                end
            elseif nargin > 1
                error('Unsupported use of the setGuess function.');
            else
                obj.Xguess = repmat(obj.X0, obj.Ncvp+1, 1);
                obj.Uguess = repmat(obj.U0, obj.Ncvp, 1);
            end
        end
        
    end
    
end

