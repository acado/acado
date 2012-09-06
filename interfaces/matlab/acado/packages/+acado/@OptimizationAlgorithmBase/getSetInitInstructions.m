function getSetInitInstructions(obj, cppobj)
%Used to generate CPP file (called by getInstructions of child classes of
%optimizationalgorithmbase
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
%    Author: David Ariens
%    Date: 2010
% 

    
    for i=1:length(obj.set_n)
        fprintf(cppobj.fileMEX,sprintf('    %s.set( %s, %s );\n', obj.name, obj.set_n{i}, stringIntDouble(obj.set_v{i})));               
    end

    if (~isempty(obj.initdiffstates)) 	%init diff states
        fprintf(cppobj.fileMEX,sprintf('    %s.initializeDifferentialStates( %s );\n', obj.name, obj.initdiffstates.name));  
    end
    
    if (~isempty(obj.initalgstates)) 	%init alg states
        fprintf(cppobj.fileMEX,sprintf('    %s.initializeAlgebraicStates( %s );\n', obj.name, obj.initalgstates.name));  
    end
    
    if (~isempty(obj.initcontrols)) 	%init controls
        fprintf(cppobj.fileMEX,sprintf('    %s.initializeControls( %s );\n', obj.name, obj.initcontrols.name));  
    end

    if (~isempty(obj.initparameters)) 	%init parameters
        fprintf(cppobj.fileMEX,sprintf('    %s.initializeParameters( %s );\n', obj.name, obj.initparameters.name));  
    end

    if (~isempty(obj.initdisturbances))	%init disturbances
        fprintf(cppobj.fileMEX,sprintf('    %s.initializeDisturbances( %s );\n', obj.name, obj.initdisturbances.name));  
    end


end