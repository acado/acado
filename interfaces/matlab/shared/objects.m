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

% HEADER_PATHS = '-I../../include -I../../external_packages -I../../external_packages/include  -I../../external_packages/qpOASES-2.0/INCLUDE';
HEADER_PATHS = sprintf('-I''%s/bin/include'' -I''%s/../../include'' -I''%s/../../external_packages'' -I''%s/../../external_packages/include''  -I''%s/../../external_packages/qpOASES-3.0beta/include'' -I''%s''', pwd,pwd,pwd,pwd,pwd,pwd);
%% C++ SRC ACADO OBJECTS

SRC = {};
BIN = {};
BINFOLDER = {};

%% INTEGRATOR

[fl, ol] = getFilesAndObjectNames('../../src/utils');

SRC = [SRC; fl];
BIN = [BIN; ol'];
BINFOLDER = [BINFOLDER repmat({'src/'}, length( fl ), 1)];

[fl, ol] = getFilesAndObjectNames('../../src/clock');

SRC = [SRC; fl];
BIN = [BIN; ol'];
BINFOLDER = [BINFOLDER; repmat({'src/'}, length( fl ), 1)];

[fl, ol] = getFilesAndObjectNames('../../src/user_interaction');

SRC = [SRC; fl];
BIN = [BIN; ol'];
BINFOLDER = [BINFOLDER; repmat({'src/'}, length( fl ), 1)];

[fl, ol] = getFilesAndObjectNames('../../src/variables_grid');

SRC = [SRC; fl];
BIN = [BIN; ol'];
BINFOLDER = [BINFOLDER; repmat({'src/'}, length( fl ), 1)];


% TODO remove this file and patch acado_csparse.cpp
[fl, ol] = getFilesAndObjectNames('../../external_packages/src/acado_csparse/');
SRC = [SRC; fl];
BIN = [BIN; ol'];
BINFOLDER = [BINFOLDER; repmat({'src/'}, length( fl ), 1)];

[fl, ol] = getFilesAndObjectNames('../../src/matrix_vector');

SRC = [SRC; fl];
BIN = [BIN; ol'];
BINFOLDER = [BINFOLDER; repmat({'src/'}, length( fl ), 1)];

[fl, ol] = getFilesAndObjectNames('../../src/sparse_solver');

SRC = [SRC; fl];
BIN = [BIN; ol'];
BINFOLDER = [BINFOLDER; repmat({'src/'}, length( fl ), 1)];

[fl, ol] = getFilesAndObjectNames('../../src/symbolic_operator');

SRC = [SRC; fl];
BIN = [BIN; ol'];
BINFOLDER = [BINFOLDER; repmat({'src/'}, length( fl ), 1)];

[fl, ol] = getFilesAndObjectNames('../../src/symbolic_expression');

SRC = [SRC; fl];
BIN = [BIN; ol'];
BINFOLDER = [BINFOLDER; repmat({'src/'}, length( fl ), 1)];

[fl, ol] = getFilesAndObjectNames('../../src/function');

SRC = [SRC; fl];
BIN = [BIN; ol'];
BINFOLDER = [BINFOLDER; repmat({'src/'}, length( fl ), 1)];

[fl, ol] = getFilesAndObjectNames('../../src/modeling_tools');

SRC = [SRC; fl];
BIN = [BIN; ol'];
BINFOLDER = [BINFOLDER; repmat({'src/'}, length( fl ), 1)];

[fl, ol] = getFilesAndObjectNames('../../src/integrator');

SRC = [SRC; fl];
BIN = [BIN; ol'];
BINFOLDER = [BINFOLDER; repmat({'src/'}, length( fl ), 1)];

[fl, ol] = getFilesAndObjectNames('../../src/curve');

SRC = [SRC; fl];
BIN = [BIN; ol'];
BINFOLDER = [BINFOLDER; repmat({'src/'}, length( fl ), 1)];


%% OPTIMAL CONTROL
if (returnlist == 0 || returnlist == 2 || returnlist == 3)
	[fl, ol] = getFilesAndObjectNames('../../src/code_generation');

	SRC = [SRC; fl];
	BIN = [BIN; ol'];
	BINFOLDER = [BINFOLDER; repmat({'src/'}, length( fl ), 1)];
	
	[fl, ol] = getFilesAndObjectNames('../../src/dynamic_system');

	SRC = [SRC; fl];
	BIN = [BIN; ol'];
	BINFOLDER = [BINFOLDER; repmat({'src/'}, length( fl ), 1)];
	
	[fl, ol] = getFilesAndObjectNames('../../src/objective');

	SRC = [SRC; fl];
	BIN = [BIN; ol'];
	BINFOLDER = [BINFOLDER; repmat({'src/'}, length( fl ), 1)];
	
	[fl, ol] = getFilesAndObjectNames('../../src/constraint');

	SRC = [SRC; fl];
	BIN = [BIN; ol'];
	BINFOLDER = [BINFOLDER; repmat({'src/'}, length( fl ), 1)];
	
	[fl, ol] = getFilesAndObjectNames('../../src/conic_program');

	SRC = [SRC; fl];
	BIN = [BIN; ol'];
	BINFOLDER = [BINFOLDER; repmat({'src/'}, length( fl ), 1)];
	
	[fl, ol] = getFilesAndObjectNames('../../src/conic_solver');

	SRC = [SRC; fl];
	BIN = [BIN; ol'];
	BINFOLDER = [BINFOLDER; repmat({'src/'}, length( fl ), 1)];
	
	[fl, ol] = getFilesAndObjectNames('../../src/ocp');

	SRC = [SRC; fl];
	BIN = [BIN; ol'];
	BINFOLDER = [BINFOLDER; repmat({'src/'}, length( fl ), 1)];
	
	[fl, ol] = getFilesAndObjectNames('../../src/dynamic_discretization');

	SRC = [SRC; fl];
	BIN = [BIN; ol'];
	BINFOLDER = [BINFOLDER; repmat({'src/'}, length( fl ), 1)];
	
	[fl, ol] = getFilesAndObjectNames('../../external_packages/qpOASES-3.0beta/src');

	SRC = [SRC; fl];
	BIN = [BIN; ol'];
	BINFOLDER = [BINFOLDER; repmat({'qpOASES/'}, length( fl ), 1)];

	[fl, ol] = getFilesAndObjectNames('../../src/nlp_solver');

	SRC = [SRC; fl];
	BIN = [BIN; ol'];
	BINFOLDER = [BINFOLDER; repmat({'src/'}, length( fl ), 1)];
	
	[fl, ol] = getFilesAndObjectNames('../../external_packages/src/acado_qpoases');

	SRC = [SRC; fl];
	BIN = [BIN; ol'];
	BINFOLDER = [BINFOLDER; repmat({'src/'}, length( fl ), 1)];
	
	[fl, ol] = getFilesAndObjectNames('../../src/nlp_derivative_approximation');

	SRC = [SRC; fl];
	BIN = [BIN; ol'];
	BINFOLDER = [BINFOLDER; repmat({'src/'}, length( fl ), 1)];
	
	[fl, ol] = getFilesAndObjectNames('../../src/optimization_algorithm');

	SRC = [SRC; fl];
	BIN = [BIN; ol'];
	BINFOLDER = [BINFOLDER; repmat({'src/'}, length( fl ), 1)];
    
	[fl, ol] = getFilesAndObjectNames('../../external_packages/casadi/symbolic');

	SRC = [SRC; fl];
	BIN = [BIN; ol'];
	BINFOLDER = [BINFOLDER; repmat({'src/'}, length( fl ), 1)];
	
end;


%% Simulation
if (returnlist == 0 || returnlist == 3)  
    % curve already set in ocp
	
	[fl, ol] = getFilesAndObjectNames('../../src/controller');

	SRC = [SRC; fl];
	BIN = [BIN; ol'];
	BINFOLDER = [BINFOLDER; repmat({'src/'}, length( fl ), 1)];
	
	[fl, ol] = getFilesAndObjectNames('../../src/reference_trajectory');

	SRC = [SRC; fl];
	BIN = [BIN; ol'];
	BINFOLDER = [BINFOLDER; repmat({'src/'}, length( fl ), 1)];
	
	[fl, ol] = getFilesAndObjectNames('../../src/estimator');

	SRC = [SRC; fl];
	BIN = [BIN; ol'];
	BINFOLDER = [BINFOLDER; repmat({'src/'}, length( fl ), 1)];
	
	[fl, ol] = getFilesAndObjectNames('../../src/control_law');

	SRC = [SRC; fl];
	BIN = [BIN; ol'];
	BINFOLDER = [BINFOLDER; repmat({'src/'}, length( fl ), 1)];
	
	[fl, ol] = getFilesAndObjectNames('../../src/noise');

	SRC = [SRC; fl];
	BIN = [BIN; ol'];
	BINFOLDER = [BINFOLDER; repmat({'src/'}, length( fl ), 1)];
	
	[fl, ol] = getFilesAndObjectNames('../../src/transfer_device');

	SRC = [SRC; fl];
	BIN = [BIN; ol'];
	BINFOLDER = [BINFOLDER; repmat({'src/'}, length( fl ), 1)];
	
	[fl, ol] = getFilesAndObjectNames('../../src/process');

	SRC = [SRC; fl];
	BIN = [BIN; ol'];
	BINFOLDER = [BINFOLDER; repmat({'src/'}, length( fl ), 1)];
	
	[fl, ol] = getFilesAndObjectNames('../../src/simulation_environment');

	SRC = [SRC; fl];
	BIN = [BIN; ol'];
	BINFOLDER = [BINFOLDER; repmat({'src/'}, length( fl ), 1)];
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


    % SRC{k} = '../../external_packages/src/acado_csparse/acado_csparse.cpp';  BIN{k} ='acado_csparse';                      BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_add.c';                  BIN{k} ='cs_add';                               BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_amd.c';                  BIN{k} ='cs_amd';                               BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_chol.c';                 BIN{k} ='cs_chol';                              BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_cholsol.c';              BIN{k} ='cs_cholsol';                           BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_counts.c';               BIN{k} ='cs_counts';                            BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_cumsum.c';               BIN{k} ='cs_cumsum';                            BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_droptol.c';                  BIN{k} ='cs_droptol';                               BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_dropzeros.c';                  BIN{k} ='cs_dropzeros';                               BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_dupl.c';                 BIN{k} ='cs_dupl';                              BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_entry.c';              BIN{k} ='cs_entry';                           BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_etree.c';               BIN{k} ='cs_etree';                            BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_fkeep.c';               BIN{k} ='cs_fkeep';                            BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_gaxpy.c';                 BIN{k} ='cs_gaxpy';                              BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_happly.c';              BIN{k} ='cs_happly';                           BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_house.c';               BIN{k} ='cs_house';                            BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_ipvec.c';               BIN{k} ='cs_ipvec';                            BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_lsolve.c';               BIN{k} ='cs_lsolve';                            BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_ltsolve.c';               BIN{k} ='cs_ltsolve';                            BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_lu.c';                 BIN{k} ='cs_lu';                              BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_lusol.c';              BIN{k} ='cs_lusol';                           BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_util.c';               BIN{k} ='cs_util';                            BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_multiply.c';               BIN{k} ='cs_multiply';                            BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_permute.c';               BIN{k} ='cs_permute';                            BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_pinv.c';               BIN{k} ='cs_pinv';                            BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_post.c';                 BIN{k} ='cs_post';                              BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_pvec.c';              BIN{k} ='cs_pvec';                           BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_qr.c';               BIN{k} ='cs_qr';                            BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_qrsol.c';               BIN{k} ='cs_qrsol';                            BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_scatter.c';               BIN{k} ='cs_scatter';                            BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_schol.c';               BIN{k} ='cs_schol';                            BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_sqr.c';                 BIN{k} ='cs_sqr';                              BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_symperm.c';              BIN{k} ='cs_symperm';                           BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_tdfs.c';               BIN{k} ='cs_tdfs';                            BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_malloc.c';               BIN{k} ='cs_malloc';                            BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_transpose.c';               BIN{k} ='cs_transpose';                            BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_compress.c';               BIN{k} ='cs_compress';                            BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_usolve.c';                 BIN{k} ='cs_usolve';                              BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_utsolve.c';              BIN{k} ='cs_utsolve';                           BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_scc.c';               BIN{k} ='cs_scc';                            BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_maxtrans.c';               BIN{k} ='cs_maxtrans';                            BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_dmperm.c';               BIN{k} ='cs_dmperm';                            BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_updown.c';               BIN{k} ='cs_updown';                            BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_print.c';                 BIN{k} ='cs_print';                              BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_norm.c';              BIN{k} ='cs_norm';                           BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_load.c';               BIN{k} ='cs_load';                            BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_dfs.c';               BIN{k} ='cs_dfs';                            BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_reach.c';               BIN{k} ='cs_reach';                            BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_spsolve.c';                 BIN{k} ='cs_spsolve';                              BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_ereach.c';              BIN{k} ='cs_ereach';                           BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_leaf.c';               BIN{k} ='cs_leaf';                            BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_randperm.c';               BIN{k} ='cs_randperm';                           BINFOLDER{k} = 'csparse/'; k=k+1;
