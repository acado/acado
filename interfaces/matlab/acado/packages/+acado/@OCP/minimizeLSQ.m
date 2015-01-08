function minimizeLSQ(obj, varargin)
%Adds an Least Square term minimized. 
% Adds an Least Square term of the form
% 0.5* sum_i || ( h(t_i,x(t_i),u(t_i),p(t_i),...) - r ) ||^2_S_i
% Here the sum is over all grid points of the objective grid. The
% Matrix S is assumed to be symmetric and positive (semi-) definite.
% The Function r is called reference and can be specified by the user. 
%
%  Usage:
%    >> ocp.minimizeLSQ(h)
%    >> ocp.minimizeLSQ(h, r)
%    >> ocp.minimizeLSQ(S, h, r)
%
%  Parameters: 
%    h 	  the LSQ-Function    (1 x n  FUNCTION)
%    S 	  a weighting matrix  (n x n  MATRIX)
%    r 	  the reference       
%         -either a 1 x n VECTOR  with references for each expression in h
%         -or a q x (n+1) MATRIX with the first column timepoints and the next
%          columns references for each expression in h (the difference
%          being the fact that you can set a time dependant reference)
%
%  Example:
%    >> S = [10 0; 0 1];
%    >> h = {x1, x2};
%    >> r = [0.2, 1.4];
%    >> ocp.minimizeLSQ(S, h, r);
%
%    >> r = [0    0.2  1.4;
%            1    0.4  1.5;
%            1.5  0.4  1.3];
%           %time x1   x2
%    >> ocp.minimizeLSQ(S, h, r);
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

index = length(obj.minLSQTermh);

if (length(varargin) == 1) % ocp.minimizeLSQ(h)
    h = varargin{1};

    obj.minLSQTermh{index+1} = acado.Function(h);
    obj.minLSQTermr{index+1} = {};
    obj.minLSQTermS{index+1} = {};
    
elseif (length(varargin) == 2)  
    if (isnumeric(varargin{1}) || isa(varargin{1}, 'acado.BMatrix')) && ...
            (isnumeric(varargin{2}) || isa(varargin{2}, 'acado.BMatrix')) %ocp.minimizeLSQ(Q, R)
        Q = varargin{1};
        R = varargin{2};
        
        obj.minLSQTermQ = obj.checkVectorMatrix(Q);
        obj.minLSQTermR = obj.checkVectorMatrix(R);
    elseif isnumeric(varargin{1}) || isa(varargin{1}, 'acado.BMatrix') %ocp.minimizeLSQ(Q, r)
        Q = varargin{1};
        r = varargin{2};
        
        obj.minLSQTermQ = obj.checkVectorMatrix(Q);
        obj.minLSQTermR = acado.Function(r);
    else %ocp.minimizeLSQ(h, r)
        h = varargin{1};
        r = varargin{2};
        
        obj.minLSQTermh{index+1} = acado.Function(h);
        if(isa(obj.checkVectorMatrix(r), 'acado.Matrix'))
           obj.minLSQTermr{index+1} = acado.VariablesGrid(obj.checkVectorMatrix(r));
        else
           obj.minLSQTermr{index+1} = obj.checkVectorMatrix(r);
        end
        obj.minLSQTermS{index+1} = {};
    end
    
elseif (length(varargin) == 3)  %ocp.minimizeLSQ(S, h, r)
    h = varargin{2};
    r = varargin{3};
    S = varargin{1};

    obj.minLSQTermh{index+1} = acado.Function(h);
    obj.minLSQTermS{index+1} = acado.Matrix(S);
    if(isa(obj.checkVectorMatrix(r), 'acado.Matrix'))
        obj.minLSQTermr{index+1} = acado.VariablesGrid(obj.checkVectorMatrix(r));
    else
        obj.minLSQTermr{index+1} = obj.checkVectorMatrix(r);
    end

    
else %error
   error('ERROR: Invalid minimizeLSQ call. <a href="matlab: help acado.OCP.minimizeLSQ">help acado.OCP.minimizeLSQ</a>'); 
end


end