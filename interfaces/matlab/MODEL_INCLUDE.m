function [ INCLUDELIST ] = MODEL_INCLUDE( )
%Add all files needed to be compiled when using the ACADO integrators. 
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
%    \date 2010
% 
    INCLUDELIST.MODELFILES1 = {};       % Do not change this line
    INCLUDELIST.MODELFILES2 = {};       % Do not change this line
    INCLUDELIST.OTHERFILES  = {};       % Do not change this line
    
    
    
    % Add Models. 
    %
    % These models always have the header 
    % void FUNCTION_NAME( DifferentialEquation *f ){ ... }
    %
    % resulting in this call:
    % addModelFile('FILENAME.cpp', 'FUNCTION_NAME');
    %
    % FILENAME.cpp is relative w.r.t. the folder matlab/integrator (the
    % folder where the mex file is stored)
    
    addModelFile('../examples/integrator/cstr.cpp', 'cstr');
    addModelFile('../examples/integrator/getting_started.cpp', 'getting_started');
    addModelFile('../examples/integrator/getting_started_discretized.cpp', 'getting_started_discretized');
    addModelFile('../examples/integrator/glycemia.cpp', 'glycemia');
    addModelFile('../examples/integrator/harmonic_oscillator.cpp', 'harmonic_oscillator');
    addModelFile('../examples/integrator/liebouds_system.cpp', 'liebouds_system');
    addModelFile('../examples/integrator/pendulum.cpp', 'pendulum');
    addModelFile('../examples/integrator/simple_cstr.cpp', 'simple_cstr');
    addModelFile('../examples/integrator/simple_dae.cpp', 'simple_dae');
    addModelFile('../examples/integrator/sparse_dae.cpp', 'sparse_dae');
    addModelFile('../examples/integrator/threedof_robot.cpp', 'threedof_robot');
    addModelFile('../examples/integrator/getting_started_othercppfilemodel.cpp', 'getting_started_othercppfilemodel');
    
    
    
    % Add other CPP files you need to include to run your model. These
    % files don't need a specific header and can only be called from a
    % model file. You cannot use these files directly with the integrator.
    %
    % Use this call: addOtherCppFile('FILENAME.cpp');
    %
    % FILENAME.cpp is relative w.r.t. the folder matlab/integrator (the
    % folder where the mex file is stored)
    %
    % See also the example
    % examples/integrator/getting_started_othercppfile.m
    
    addOtherCppFile('../examples/integrator/getting_started_othercppfile.cpp');
   
    
    
    
    
    
    
    % Do not change below this line
    
    function addModelFile(filename, functionname)
        
        INCLUDELIST.MODELFILES1{length(INCLUDELIST.MODELFILES1) + 1} = filename;
        INCLUDELIST.MODELFILES2{length(INCLUDELIST.MODELFILES2) + 1} = functionname;
        
    end
    

    function addOtherCppFile(filename)

        INCLUDELIST.OTHERFILES{length(INCLUDELIST.OTHERFILES) + 1} = filename;

    end

end