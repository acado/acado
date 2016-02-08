function [HEADER_PATHS, SRC, BIN, BINFOLDER, SRCMEX, BINMEX, BINFOLDERMEX] = objects(returnlist)
% This function returns all files to be used in the make function.
% Adapt this file whenever new files need to be compiled when updates are
% made to ACADO.
%
%  Usage:
%   >>[LOCAL_PATH_PREFIX, HEADER_PATHS, SRC, BIN, BINFOLDER, SRCMEX, BINMEX] = objects(0) :
%   Get all objects (for integrators + ocp)  
%
%   >>[LOCAL_PATH_PREFIX, HEADER_PATHS, SRC, BIN, BINFOLDER, SRCMEX, BINMEX] = objects(1) :
%   Get only objects for integrator
%
%   >>[LOCAL_PATH_PREFIX, HEADER_PATHS, SRC, BIN, BINFOLDER, SRCMEX, BINMEX] = objects(2) :
%   Get only objects for ocp
%
%   >>[LOCAL_PATH_PREFIX, HEADER_PATHS, SRC, BIN, BINFOLDER, SRCMEX, BINMEX] = objects(3) :
%   Get only objects for simulation
%
%
%  How to add new CPP files (deprecated, this is going to be updated):
%   * When new files from the ACADO c++ source files needs to be added, add a new line 
%     "SRC{k} = '../../src/xxxxxx'; BIN{k} ='xxx';  BINFOLDER{k} = {'src/'}; k=k+1;". 
%     Set the path in the "SRC{k}" variable, eg "src/FOLDER/FILE.cpp". Set a
%     _unique_ name for the compiled result in "BIN{k}" and set an output
%     folder for the bin file in "BINFOLDER{k}".
%
%   * When adding new MEX files (located in subdirectories of
%     <ACADOtoolkit-inst-dir>/interfaces/matlab/) add a new line at the  bottom  
%     "SRCMEX{k} = 'FOLDER/FILE.cpp'; BINMEX{k} = 'FILE'; BINFOLDERMEX{k} = 'ocp/'; k=k+1;" 
%     where "SRCMEX" is the path to the mex file, "BINMEX" is a unique name
%     and "BINFOLDERMEX" is the folder where to store the resulting mex
%     file.
%
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
%    \author David Ariens
%    \date 2009-2010
% 

%% SETTINGS

HEADER_PATHS = sprintf('-I''%s/bin'' -I''%s/../../'' -I''%s/../../acado'' -I''%s/../../external_packages'' -I''%s/../../external_packages/qpOASES-3.2.0/include'' -I''%s/../../external_packages/eigen3'' -I''%s''', pwd,pwd,pwd,pwd,pwd,pwd,pwd);
%% C++ SRC ACADO OBJECTS

SRC = {};
BIN = {};
BINFOLDER = {};

%% INTEGRATOR

[fl, ol] = getFilesAndObjectNames('../../acado/utils');

SRC = [SRC; fl];
BIN = [BIN; ol'];
BINFOLDER = [BINFOLDER repmat({'acado/'}, length( fl ), 1)];

[fl, ol] = getFilesAndObjectNames('../../acado/clock');

SRC = [SRC; fl];
BIN = [BIN; ol'];
BINFOLDER = [BINFOLDER; repmat({'acado/'}, length( fl ), 1)];

[fl, ol] = getFilesAndObjectNames('../../acado/user_interaction');

SRC = [SRC; fl];
BIN = [BIN; ol'];
BINFOLDER = [BINFOLDER; repmat({'acado/'}, length( fl ), 1)];

[fl, ol] = getFilesAndObjectNames('../../acado/variables_grid');

SRC = [SRC; fl];
BIN = [BIN; ol'];
BINFOLDER = [BINFOLDER; repmat({'acado/'}, length( fl ), 1)];


% TODO remove this file and patch acado_csparse.cpp
[fl, ol] = getFilesAndObjectNames('../../acado/bindings/acado_csparse/');
SRC = [SRC; fl];
BIN = [BIN; ol'];
BINFOLDER = [BINFOLDER; repmat({'acado/'}, length( fl ), 1)];

[fl, ol] = getFilesAndObjectNames('../../acado/matrix_vector');

SRC = [SRC; fl];
BIN = [BIN; ol'];
BINFOLDER = [BINFOLDER; repmat({'acado/'}, length( fl ), 1)];

[fl, ol] = getFilesAndObjectNames('../../acado/sparse_solver');

SRC = [SRC; fl];
BIN = [BIN; ol'];
BINFOLDER = [BINFOLDER; repmat({'acado/'}, length( fl ), 1)];

[fl, ol] = getFilesAndObjectNames('../../acado/symbolic_operator');

SRC = [SRC; fl];
BIN = [BIN; ol'];
BINFOLDER = [BINFOLDER; repmat({'acado/'}, length( fl ), 1)];

[fl, ol] = getFilesAndObjectNames('../../acado/symbolic_expression');

SRC = [SRC; fl];
BIN = [BIN; ol'];
BINFOLDER = [BINFOLDER; repmat({'acado/'}, length( fl ), 1)];

[fl, ol] = getFilesAndObjectNames('../../acado/function');

SRC = [SRC; fl];
BIN = [BIN; ol'];
BINFOLDER = [BINFOLDER; repmat({'acado/'}, length( fl ), 1)];

[fl, ol] = getFilesAndObjectNames('../../acado/modeling_tools');

SRC = [SRC; fl];
BIN = [BIN; ol'];
BINFOLDER = [BINFOLDER; repmat({'acado/'}, length( fl ), 1)];

[fl, ol] = getFilesAndObjectNames('../../acado/integrator');

SRC = [SRC; fl];
BIN = [BIN; ol'];
BINFOLDER = [BINFOLDER; repmat({'acado/'}, length( fl ), 1)];

[fl, ol] = getFilesAndObjectNames('../../acado/curve');

SRC = [SRC; fl];
BIN = [BIN; ol'];
BINFOLDER = [BINFOLDER; repmat({'acado/'}, length( fl ), 1)];


%% OPTIMAL CONTROL
if (returnlist == 0 || returnlist == 2 || returnlist == 3)
	[fl, ol] = getFilesAndObjectNames('../../acado/code_generation');

	SRC = [SRC; fl];
	BIN = [BIN; ol'];
	BINFOLDER = [BINFOLDER; repmat({'acado/'}, length( fl ), 1)];
	
	[fl, ol] = getFilesAndObjectNames('../../acado/dynamic_system');

	SRC = [SRC; fl];
	BIN = [BIN; ol'];
	BINFOLDER = [BINFOLDER; repmat({'acado/'}, length( fl ), 1)];
	
	[fl, ol] = getFilesAndObjectNames('../../acado/objective');

	SRC = [SRC; fl];
	BIN = [BIN; ol'];
	BINFOLDER = [BINFOLDER; repmat({'acado/'}, length( fl ), 1)];
	
	[fl, ol] = getFilesAndObjectNames('../../acado/constraint');

	SRC = [SRC; fl];
	BIN = [BIN; ol'];
	BINFOLDER = [BINFOLDER; repmat({'acado/'}, length( fl ), 1)];
	
	[fl, ol] = getFilesAndObjectNames('../../acado/conic_program');

	SRC = [SRC; fl];
	BIN = [BIN; ol'];
	BINFOLDER = [BINFOLDER; repmat({'acado/'}, length( fl ), 1)];
	
	[fl, ol] = getFilesAndObjectNames('../../acado/conic_solver');

	SRC = [SRC; fl];
	BIN = [BIN; ol'];
	BINFOLDER = [BINFOLDER; repmat({'acado/'}, length( fl ), 1)];
	
	[fl, ol] = getFilesAndObjectNames('../../acado/ocp');

	SRC = [SRC; fl];
	BIN = [BIN; ol'];
	BINFOLDER = [BINFOLDER; repmat({'acado/'}, length( fl ), 1)];
	
	[fl, ol] = getFilesAndObjectNames('../../acado/dynamic_discretization');

	SRC = [SRC; fl];
	BIN = [BIN; ol'];
	BINFOLDER = [BINFOLDER; repmat({'acado/'}, length( fl ), 1)];
	
	[fl, ol] = getFilesAndObjectNames('../../external_packages/qpOASES-3.2.0/src');

	SRC = [SRC; fl];
	BIN = [BIN; ol'];
	BINFOLDER = [BINFOLDER; repmat({'acado/qpOASES/'}, length( fl ), 1)];

	[fl, ol] = getFilesAndObjectNames('../../acado/nlp_solver');

	SRC = [SRC; fl];
	BIN = [BIN; ol'];
	BINFOLDER = [BINFOLDER; repmat({'acado/'}, length( fl ), 1)];
	
	[fl, ol] = getFilesAndObjectNames('../../acado/bindings/acado_qpoases');

	SRC = [SRC; fl];
	BIN = [BIN; ol'];
	BINFOLDER = [BINFOLDER; repmat({'acado/'}, length( fl ), 1)];
	
	[fl, ol] = getFilesAndObjectNames('../../acado/nlp_derivative_approximation');

	SRC = [SRC; fl];
	BIN = [BIN; ol'];
	BINFOLDER = [BINFOLDER; repmat({'acado/'}, length( fl ), 1)];
	
	[fl, ol] = getFilesAndObjectNames('../../acado/optimization_algorithm');

	SRC = [SRC; fl];
	BIN = [BIN; ol'];
	BINFOLDER = [BINFOLDER; repmat({'acado/'}, length( fl ), 1)];
    
	[fl, ol] = getFilesAndObjectNames('../../external_packages/casadi/symbolic');

	SRC = [SRC; fl];
	BIN = [BIN; ol'];
	BINFOLDER = [BINFOLDER; repmat({'acado/casadi/'}, length( fl ), 1)];
	
end;


%% Simulation
if (returnlist == 0 || returnlist == 3)  
    % curve already set in ocp
	
	[fl, ol] = getFilesAndObjectNames('../../acado/controller');

	SRC = [SRC; fl];
	BIN = [BIN; ol'];
	BINFOLDER = [BINFOLDER; repmat({'acado/'}, length( fl ), 1)];
	
	[fl, ol] = getFilesAndObjectNames('../../acado/reference_trajectory');

	SRC = [SRC; fl];
	BIN = [BIN; ol'];
	BINFOLDER = [BINFOLDER; repmat({'acado/'}, length( fl ), 1)];
	
	[fl, ol] = getFilesAndObjectNames('../../acado/estimator');

	SRC = [SRC; fl];
	BIN = [BIN; ol'];
	BINFOLDER = [BINFOLDER; repmat({'acado/'}, length( fl ), 1)];
	
	[fl, ol] = getFilesAndObjectNames('../../acado/control_law');

	SRC = [SRC; fl];
	BIN = [BIN; ol'];
	BINFOLDER = [BINFOLDER; repmat({'acado/'}, length( fl ), 1)];
	
	[fl, ol] = getFilesAndObjectNames('../../acado/noise');

	SRC = [SRC; fl];
	BIN = [BIN; ol'];
	BINFOLDER = [BINFOLDER; repmat({'acado/'}, length( fl ), 1)];
	
	[fl, ol] = getFilesAndObjectNames('../../acado/transfer_device');

	SRC = [SRC; fl];
	BIN = [BIN; ol'];
	BINFOLDER = [BINFOLDER; repmat({'acado/'}, length( fl ), 1)];
	
	[fl, ol] = getFilesAndObjectNames('../../acado/process');

	SRC = [SRC; fl];
	BIN = [BIN; ol'];
	BINFOLDER = [BINFOLDER; repmat({'acado/'}, length( fl ), 1)];
	
	[fl, ol] = getFilesAndObjectNames('../../acado/simulation_environment');

	SRC = [SRC; fl];
	BIN = [BIN; ol'];
	BINFOLDER = [BINFOLDER; repmat({'acado/'}, length( fl ), 1)];
end;
    

%% MEX
k = 1;
%% OCP MEX
if (returnlist == 0 || returnlist == 2 || returnlist == 3)
    SRCMEX = []; BINMEX = [];BINFOLDERMEX = [];

%     SRCMEX{k} = 'examples/ocp/getting_started/gettingstarted_ACADO.cpp';     
%     BINMEX{k} = 'gettingstarted_ACADO';            
%     BINFOLDERMEX{k} = 'examples/ocp/getting_started/'; k=k+1;
%     
%     SRCMEX{k} = 'examples/ocp/invertedpendulum/pendswingup_ACADO.cpp';     
%     BINMEX{k} = 'pendswingup_ACADO';            
%     BINFOLDERMEX{k} = 'examples/ocp/invertedpendulum/'; 
%     k=k+1;

%     SRCMEX{k} = 'examples/ocp/invertedpendulum/pendswingup_ACADO.cpp';     
%     BINMEX{k} = 'pendswingup_ACADO';            
%     BINFOLDERMEX{k} = 'examples/ocp/invertedpendulum/'; 
%     k=k+1;
    %SRCMEX{k} = 'ocp/ACADOocpCALL.cpp';                 BINMEX{k} = 'ACADOocpCALL';        BINFOLDERMEX{k} = 'ocp/'; k=k+1;
end

%% INTEGRATOR MEX
if (returnlist == 0 || returnlist == 1)
    SRCMEX{k} = 'integrator/ACADOintegrators.cpp';      BINMEX{k} = 'ACADOintegrators';    BINFOLDERMEX{k} = 'integrator/'; k=k+1;
end


end
