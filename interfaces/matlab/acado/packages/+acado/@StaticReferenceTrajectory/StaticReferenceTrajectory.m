%Allows to define a static reference trajectory that the ControlLaw aims to track.
% The class StaticReferenceTrajectory allows to define a static reference trajectory 
% (given beforehand) that the ControlLaw aims to track while computing its output.
%
%  Usage:
%    >> StaticReferenceTrajectory();   (zero reference)
%    >> StaticReferenceTrajectory(x);
%
%  Parameters:
%    x      matrix
%
%  Example:
%    >> traject = acado.StaticReferenceTrajectory(x)
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
classdef StaticReferenceTrajectory < acado.ReferenceTrajectory    
    properties(SetAccess='protected')
        ref;
    end
    
    methods
        function obj = StaticReferenceTrajectory(varargin)
            
            if (nargin == 1)
                obj.ref = acado.Matrix(varargin{1});
            end
            
            global ACADO_;
            ACADO_.helper.addInstruction(obj);
            
        end
        
        getInstructions(obj, cppobj, get)

    end
end
