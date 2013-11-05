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

templateFiles = {'integrator_mex.c.in', 'rhs_mex.c.in', 'forces_interface.in', ...
    'acado_forces_generator.m.in', 'qpoases_interface.hpp.in', 'qpoases_interface.cpp.in', ...
    'qpdunes_interface.in', 'acado_auxiliary_functions.h.in', 'acado_auxiliary_functions.c.in', ...
    'acado_solver_mex.c.in', 'make_acado_solver_forces.m.in', 'make_acado_solver_qpoases.m.in', ...
    'make_acado_integrator.m.in', 'make_acado_model.m.in', ...
    'makefile.forces.in', 'makefile.qpoases.in', 'make_acado_solver_sfunction.m.in', 'acado_solver_sfunction.c.in', ...
    'acado_solver_sfunction.h.in', 'dummy_test_file.in'};

templates = {'INTEGRATOR_MEX_TEMPLATE', 'RHS_MEX_TEMPLATE', 'FORCES_TEMPLATE', ...
    'FORCES_GENERATOR', 'QPOASES_HEADER', 'QPOASES_SOURCE', 'QPDUNES_TEMPLATE', ...
    'AUXILIARY_FUNCTIONS_HEADER', 'AUXILIARY_FUNCTIONS_SOURCE', 'SOLVER_MEX', ...
    'MAKE_MEX_FORCES', 'MAKE_MEX_QPOASES', 'MAKE_MEX_INTEGRATOR', 'MAKE_MEX_MODEL', ...
    'MAKEFILE_FORCES', 'MAKEFILE_QPOASES', ...
    'MAKEFILE_SFUN_QPOASES', 'SOLVER_SFUN_SOURCE', 'SOLVER_SFUN_HEADER', 'DUMMY_TEST_FILE'};

fid = fopen('./bin/include/acado/code_generation/templates/templates.hpp', 'w+');

fprintf(fid, '%s \n', ' #ifndef ACADO_TOOLKIT_TEMPLATES_HPP');
fprintf(fid, '%s \n\n', ' #define ACADO_TOOLKIT_TEMPLATES_HPP');
tempString = sprintf('%s "%s/../../src/code_generation/templates" \n\n', ' #define TEMPLATE_PATHS', pwd);
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

