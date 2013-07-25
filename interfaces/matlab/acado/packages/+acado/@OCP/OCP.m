%The class OCP is a data wrapper for defining optimal control problems.
%
%  Usage:
%    >> OCP();                 Define an OCP without start and end time. Do
%                              this only when setting start and end times
%                              in the differential equation object.
%
%    >> OCP(timepoints);       Define a grid of timepoints (used in
%                              parameter estimation)
%
%    >> OCP(tStart, tEnd);     Normal OCP formulation with start and end time
%
%    >> OCP(tStart, tEnd, N);  Normal OCP formulation with, start and end
%                              time + number of intervals
%
%  Parameters:
%    tStart 	start of the time horizon of the OCP       [NUMERIC]
%    tEnd       end of the time horizon of the OCP         [NUMERIC/PARAMETER]
%    N          number of discretization intervals         [NUMERIC]
%    timepoints vector with timepoints                     [1xn NUMERIC VECTOR]
%
%
%  Example:
%    >> ocp = acado.OCP(0.0, 1.0, 20);
%    >> ocp = acado.OCP(0.0, T, 20);
%    >> ocp = acado.OCP(0.0, T);
%    >> M = [0 1 5 6 7 10];
%    >> ocp = acado.OCP(M);
%
%
%  See also:
%    acado.OCP.minimizeLSQ              Least squares term
%    acado.OCP.minimizeLSQEndTerm
%    acado.OCP.minimizeMayerTerm        Mayer Term
%    acado.OCP.maximizeMayerTerm
%    acado.OCP.minimizeLagrangeTerm     Lagrange Term
%    acado.OCP.maximizeLagrangeTerm
%    acado.OCP.subjectTo                Bounds, constraints
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
%    Author: David Ariens, Rien Quirynen
%    Date: 2009-2012
% 
classdef OCP < acado.MultiObjectiveFunctionality & acado.ModelContainer    
    properties (SetAccess='private')
        name = 'ocp';
       
        % Constructor
        tStart;
        tEnd;
        N;
        grid = {};
        
        % Objective
        minMayerTerms = {};
        maxMayerTerms = {};
        
        minLagrangeTerms = {};
        maxLagrangeTerms = {};
        
        minLSQTermS = {};
        minLSQTermh = {};
        minLSQTermr = {};
        
        minLSQTermQ;
        minLSQTermR;

        minLSQEndTermS = {};
        minLSQEndTermh = {};
        minLSQEndTermr = {};
        
        minLSQEndTermQ;
        minLSQEndTermR;

        % Subject to
        subjectoItems = {};
    end
    
    methods
        function obj = OCP(varargin)
            checkActiveModel;
            
            global ACADO_;
            ACADO_.count_ocp = ACADO_.count_ocp+1;
            obj.name = strcat(obj.name, num2str(ACADO_.count_ocp));
            
            if (nargin == 2 )  %OCP(tStart, tEnd)
                obj.tStart = acado.DoubleConstant(varargin{1});   
                
                if (isa(varargin{2}, 'acado.Expression'))
                    obj.tEnd = varargin{2};
                else
                    obj.tEnd = acado.DoubleConstant(varargin{2});    
                end
                
            elseif (nargin == 3)  %OCP(tStart, tEnd, N)
                obj.tStart = acado.DoubleConstant(varargin{1});
                
                if (isa(varargin{2}, 'acado.Expression'))
                    obj.tEnd = varargin{2};
                else
                    obj.tEnd = acado.DoubleConstant(varargin{2});    
                end
                
                obj.N = acado.DoubleConstant(varargin{3});
            
            elseif (nargin == 1)  %OCP(timepoints)
                obj.grid = acado.Vector(varargin{1});
                
            end
            

            ACADO_.helper.addInstruction(obj);
            
        end
        
        
        function result = checkVectorMatrix(obj, r)
            
            if (isa(r, 'acado.MexInputVector'))
                result = acado.Vector(r);
                
            elseif(isa(r, 'acado.MexInputMatrix'))
                result = acado.Matrix(r);
                
            elseif isnumeric(r)
                [m n] = size(r);
                
                if( (m == 1 && n >= 1) || (m >= 1 && n == 1) )
                    result = acado.Vector(r);
                else
                    result = acado.Matrix(r);
                end
            else
                result = r;
            end
    
        end
        
        function s = toString(obj)
            s = obj.name;
        end
    
        
        maximizeMayerTerm(obj, varargin)
        minimizeMayerTerm(obj, varargin)
        
        minimizeLagrangeTerm(obj, varargin)
        maximizeLagrangeTerm(obj, varargin)
        
        minimizeLSQ(obj, varargin)
        minimizeLSQEndTerm(obj,varargin)
        
        subjectTo(obj, varargin)
        
        getInstructions(obj, cppobj, get)

    end
    
end
