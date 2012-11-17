%Start a new empty ACADO problem
%
%  Example:
%    >> BEGIN_ACADO
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
%    Author: David Ariens, Rien Quirynen
%    Date: 2009-2012
% 

function BEGIN_ACADO( varargin )

if ~iscellstr( varargin ) || nargin > 1
        error( 'Syntax is: ''BEGIN_ACADO'' or ''BEGIN_ACADO  problemName''' );
else

% Check if acadoglobals function can be called.
if ~exist('acadoglobals','file'),
    error( 'ERROR: Run make script first. Go to <ACADOtoolkit-inst-dir>/interfaces/matlab/ and run <a href="matlab: help make">help make</a>' );
end

% Set globals
acadoglobals;
if ~exist(ACADO_.pwd)
     error( 'ERROR: Run make script first. Go to <ACADOtoolkit-inst-dir>/interfaces/matlab/ and run <a href="matlab: help make">help make</a>' );
end


% % Add OCP folder to path
% addpath( genpath([ACADO_.pwd filesep 'acado']) );

% A model is now active
ACADO_.modelactive = 1;
ACADO_.generatingCode = 0;

% Set current problemname
if nargin == 0
ACADO_.problemname = 'test';
else
ACADO_.problemname = varargin{1}; 
end

% Write results to file?
ACADO_.results_to_file = false;

% Helper
ACADO_.helper = acado.AcadoMatlab();

% Set counts (used to auto name variables)
ACADO_.count_ocp = 0;
ACADO_.count_optalgo = 0;
ACADO_.count_function = 0;
ACADO_.count_matrix = 0;
ACADO_.count_vector = 0;
ACADO_.count_sim = 0;
ACADO_.count_other = 0;
ACADO_.count_mexin = 0;
ACADO_.count_double = 0;
ACADO_.count_generation = 0;


end
end