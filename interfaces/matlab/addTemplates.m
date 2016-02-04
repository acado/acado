% Licence:
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
%    Author: Rien Quirynen
%    Date: 2012
% 

templateFiles = {
    'integrator_mex.c.in', 'rhs_mex.c.in', 'forces_interface.in', 'acado_forces_generator.m.in', 'acado_forces_generator.py.in', ...
    'qpoases_interface.hpp.in', 'qpoases_interface.cpp.in', 'qpoases3_interface.h.in', 'qpoases3_interface.c.in', ...
    'qpdunes_split_interface.in', 'qpdunes_interface.in', ...
    'acado_auxiliary_functions.h.in', 'acado_auxiliary_functions.c.in', 'acado_auxiliary_sim_functions.h.in', 'acado_auxiliary_sim_functions.c.in', ...
    'acado_solver_mex.c.in', 'acado_EH_solver_mex.c.in', 'make_acado_solver_forces.m.in', ...
    'make_acado_solver_qpoases.m.in', 'make_acado_EH_solver_qpoases.m.in', 'make_acado_solver_qpoases3.m.in', 'make_acado_EH_solver_qpoases3.m.in', ...
    'make_acado_integrator.m.in', 'make_acado_model.m.in', ...
    'makefile.forces.in', 'makefile.qpoases.in', 'makefile.qpoases3.in', 'makefile.EH_qpoases.in', 'makefile.EH_qpoases3.in', 'makefile.EH_qpdunes.in', ...
    'makefile.integrator.in', 'make_acado_solver_sfunction.m.in', 'make_acado_solver_sfunction.m.in', ...
    'acado_solver_sfunction.c.in', 'acado_solver_sfunction.h.in', 'dummy_test_file.in', 'acado_common_header.h.in', ...
    'acado_hpmpc_interface.c.in', 'makefile.qpdunes.in', 'makefile.hpmpc.in', 'make_acado_solver_qpdunes.m.in', 'make_acado_EH_solver_qpdunes.m.in', 'make_acado_block_solver_qpdunes.m.in', 'acado_hessian_regularization.c.in'};

templates = { ...
    'INTEGRATOR_MEX_TEMPLATE', 'RHS_MEX_TEMPLATE', 'FORCES_TEMPLATE', 'FORCES_GENERATOR', 'FORCES_GENERATOR_PYTHON', ...
    'QPOASES_HEADER', 'QPOASES_SOURCE', 'QPOASES3_HEADER', 'QPOASES3_SOURCE', ...
    'QPDUNES_SPLIT_TEMPLATE', 'QPDUNES_TEMPLATE', ...
    'AUXILIARY_FUNCTIONS_HEADER', 'AUXILIARY_FUNCTIONS_SOURCE', 'AUXILIARY_SIM_FUNCTIONS_HEADER', 'AUXILIARY_SIM_FUNCTIONS_SOURCE', ...
    'SOLVER_MEX', 'EH_SOLVER_MEX', 'MAKE_MEX_FORCES', ...
    'MAKE_MEX_QPOASES', 'MAKE_MEX_EH_QPOASES', 'MAKE_MEX_QPOASES3', 'MAKE_MEX_EH_QPOASES3', ...
    'MAKE_MEX_INTEGRATOR', 'MAKE_MEX_MODEL', ...
    'MAKEFILE_FORCES', 'MAKEFILE_QPOASES', 'MAKEFILE_QPOASES3' 'MAKEFILE_EH_QPOASES', 'MAKEFILE_EH_QPOASES3', 'MAKEFILE_EH_QPDUNES', ...
    'MAKEFILE_INTEGRATOR', 'MAKEFILE_SFUN_QPOASES', 'MAKEFILE_SFUN_QPOASES3', ...
    'SOLVER_SFUN_SOURCE', 'SOLVER_SFUN_HEADER', 'DUMMY_TEST_FILE', 'COMMON_HEADER_TEMPLATE', ...
    'HPMPC_INTERFACE', 'MAKEFILE_QPDUNES', 'MAKEFILE_HPMPC', 'MAKE_MEX_QPDUNES', 'MAKE_MEX_EH_QPDUNES', 'MAKE_MEX_BLOCK_QPDUNES', 'HESSIAN_REG_SOURCE'};

fid = fopen('./bin/acado/code_generation/templates/templates.hpp', 'w+');

fprintf(fid, '%s \n', ' #ifndef ACADO_TOOLKIT_TEMPLATES_HPP');
fprintf(fid, '%s \n\n', ' #define ACADO_TOOLKIT_TEMPLATES_HPP');
tempString = sprintf('%s "%s/../../acado/code_generation/templates" \n\n', ' #define TEMPLATE_PATHS', pwd);
if ispc
    tempString = regexprep( tempString, '\', '/' );
end
fprintf(fid, '%s', tempString);

for i = 1:length(templates)
    tempString = sprintf(' #define %s "%s" \n\n', char(templates(i)), char(templateFiles(i)));
    if ispc
        tempString = regexprep( tempString, '\', '/' );
    end
    fprintf(fid, '%s', tempString);
end

fprintf(fid, '%s \n', ' #endif // ACADO_TOOLKIT_TEMPLATES_HPP');

fclose(fid);

