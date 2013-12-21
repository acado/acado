%Acado for Matlab helper. Internal use only.
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
%    Date: 2012
% 
classdef AcadoMatlab < handle
    properties (SetAccess='protected')
        %setVariables
        t = {};     % time (can only be one element)
        x = {};     % diff states
        dx = {};    % diff state derivatives
        u = {};     % controls
        od = {};    % online data
        p = {};     % parameters
        w = {};     % disturbances
        z = {};     % alg states
        ints = {};  % intermediate states
        in = {};    % mexInput
        expV = {};  % export variables
    
        exportModule;
        
        %instructions
        instructionList = {};
        
        %mex files
        mexList = {};
    
        problemname = '';
        fileMEX = '';

    end
    
    methods
        function obj = AcadoMatlab(varargin)

        end
        
        % Add instruction (differentialequation, ocp,... to list)
        function addInstruction(obj, set)
            obj.instructionList{length(obj.instructionList)+1} = set;
        end
        
        % Remove last instruction
        function removeInstruction(obj, set)
            found = 0;
            i = 1;
            while i <= length(obj.instructionList) && ~found
                if strcmp(class(obj.instructionList{i}), class(set)) && strcmp(obj.instructionList{i}.toString, set.toString)
                    found = 1;
                else 
                    i = i+1;
                end
            end
            if found
                obj.instructionList = {obj.instructionList{1:i-1}, obj.instructionList{i+1:end}};
            end
        end
        
        % Add time
        function addT(obj, set)
            if (~isempty(obj.t))
                error('You can only add TIME once.');
            end
            
            obj.t{1} = set;
        end
        
        % Add differential state
        function addX(obj, set)
            
            for i=1:length(obj.x)
                if (strcmp(obj.x{i}.name, set.name))
                   error('The differential state you are trying to add already exists.'); 
                end
            end
            
            obj.x{length(obj.x)+1} = set;
        end
        function clearX(obj)
            for i = 1:length(obj.x)
            	obj.removeInstruction(obj.x{i});
            end
            obj.x = {};
        end
        
        % Add differential state derivative
        function addDX(obj, set)
            
            if(~isa(set, 'acado.DifferentialState'))
                error('ERROR: A differential state derivative must be created using an existing differential state.');
            end
            foundX = 0;
            for i = 1:length(obj.x)
                if (strcmp(obj.x{i}.name, set.name))
                   foundX = 1;
                end
            end
            if ~foundX
                error('The differential state derivative, you are trying to add, has no corresponding differential state.');
            end
            foundDX = 0;
            for i = 1:length(obj.dx)
                if (strcmp(obj.dx{i}.name, set.name))
                   foundDX = 1;
                end
            end
            
            if ~foundDX
                obj.dx{length(obj.dx)+1} = set;
            end
        end
        function clearDX(obj)
            obj.dx = {};
        end
        
        % Add algebraic state
        function addZ(obj, set)
            
            for i=1:length(obj.z)
                if (strcmp(obj.z{i}.name, set.name))
                   error('The algebraic state you are trying to add already exists.'); 
                end
            end
            
            obj.z{length(obj.z)+1} = set;
        end
        function clearZ(obj)
            for i = 1:length(obj.z)
            	obj.removeInstruction(obj.z{i});
            end
            obj.z = {};
        end
        
        % Add control
        function addU(obj, set)
            
            for i=1:length(obj.u)
                if (strcmp(obj.u{i}.name, set.name))
                   error('The control you are trying to add already exists.'); 
                end
            end
            
            obj.u{length(obj.u)+1} = set;
        end
        function clearU(obj)
            for i = 1:length(obj.u)
            	obj.removeInstruction(obj.u{i});
            end
            obj.u = {};
        end
        
        % Add online data
        function addOD(obj, set)
            
            for i=1:length(obj.od)
                if (strcmp(obj.od{i}.name, set.name))
                   error('The online data you are trying to add already exists.'); 
                end
            end
            
            obj.od{length(obj.od)+1} = set;
        end
        function clearOD(obj)
            for i = 1:length(obj.od)
            	obj.removeInstruction(obj.od{i});
            end
            obj.od = {};
        end
        
        % Add parameter
        function addP(obj, set)
            
            for i=1:length(obj.p)
                if (strcmp(obj.p{i}.name, set.name))
                   error('The parameter you are trying to add already exists.'); 
                end
            end
            
            obj.p{length(obj.p)+1} = set;
        end
        function clearP(obj)
            for i = 1:length(obj.p)
            	obj.removeInstruction(obj.p{i});
            end
            obj.p = {};
        end
        
        % Add disturbance
        function addW(obj, set)
            
            for i=1:length(obj.w)
                if (strcmp(obj.w{i}.name, set.name))
                   error('The disturbance you are trying to add already exists.'); 
                end
            end
            
            obj.w{length(obj.w)+1} = set;
        end
        function clearW(obj)
            for i = 1:length(obj.w)
            	obj.removeInstruction(obj.w{i});
            end
            obj.w = {};
        end
        
        % Add export module
        function addModule(obj, set)
            if isa(set, 'acado.ExportModule')
                obj.exportModule = set;
            else
                error('This is not an instance of acado.ExportModule.');
            end
        end
        
        % Add mex input
        function addIn(obj, set)
            
            for i=1:length(obj.in)
                if (strcmp(obj.in{i}.name, set.name))
                   error('The mex input you are trying to add already exists.'); 
                end
            end
            
            obj.in{length(obj.in)+1} = set;
        end
        function clearIn(obj)
            for i = 1:length(obj.in)
            	obj.removeInstruction(obj.in{i});
            end
            obj.in = {};
            global ACADO_;
            ACADO_.count_mexin = 0;
        end
        
        % Add intermediate state
        function addIntS(obj, set)
            
            for i=1:length(obj.ints)
                if (strcmp(obj.ints{i}.name, set.name))
                   error('The intermediate state you are trying to add already exists.'); 
                end
            end
            
            obj.ints{length(obj.ints)+1} = set;
        end
        function clearIntS(obj)
            for i = 1:length(obj.ints)
            	obj.removeInstruction(obj.ints{i});
            end
            obj.ints = {};
        end
        
        % Get number of intermediate states
        function num = getNumIntS(obj)
            num = length(obj.ints);
        end
        
        % Add export variables
        function addExpV(obj, set)
            
            for i=1:length(obj.expV)
                if (strcmp(obj.expV{i}.name, set.name))
                   error('The export variable you are trying to add already exists.'); 
                end
            end
            
            obj.expV{length(obj.expV)+1} = set;
        end
        function clearExpV(obj)
            for i = 1:length(obj.expV)
            	obj.removeInstruction(obj.expV{i});
            end
            obj.expV = {};
        end
        
        % Add mex files to be compiled
        function addMEX(obj, dir, varargin)
            if nargin > 1 && ischar(dir)
                obj.mexList{end+1} = dir;
            end
        end
        function clearMEX(obj)
            obj.mexList = {};
        end
        
        generateCPP(obj);
        getCPPheader(obj);
        getCPPbody(obj);
        getCPPfooter(obj);
        getCPPlefthandout(obj, nameB, name, out);
        
        function setValues(obj, t, x, z, dx, u, od, p, w)
            if ~isempty(t) 
                obj.t{1}.setValue(t);
            end
            for i = 1:length(x)
                obj.x{i}.setValue(x(i));
            end
            for i = 1:length(z)
                obj.z{i}.setValue(z(i));
            end
            for i = 1:length(dx)
                obj.dx{i}.setValue(dx(i));
            end
            for i = 1:length(u)
                obj.u{i}.setValue(u(i));
            end
            for i = 1:length(od)
                obj.od{i}.setValue(od(i));
            end
            for i = 1:length(p)
                obj.p{i}.setValue(p(i));
            end
            for i = 1:length(w)
                obj.w{i}.setValue(w(i));
            end
        end
        
        function clearValues(obj)
            if ~isempty(obj.t)
                obj.t{1}.setValue([]);
            end
            for i = 1:length(obj.x)
                obj.x{i}.setValue([]);
            end
            for i = 1:length(obj.z)
                obj.z{i}.setValue([]);
            end
            for i = 1:length(obj.dx)
                obj.dx{i}.setValue([]);
            end
            for i = 1:length(obj.u)
                obj.u{i}.setValue([]);
            end
            for i = 1:length(obj.od)
                obj.od{i}.setValue([]);
            end
            for i = 1:length(obj.p)
                obj.p{i}.setValue([]);
            end
            for i = 1:length(obj.w)
                obj.w{i}.setValue([]);
            end
        end
        
    end
    
end