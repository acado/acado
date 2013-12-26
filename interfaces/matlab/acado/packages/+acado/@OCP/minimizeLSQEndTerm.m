function minimizeLSQEndTerm(obj, varargin)
%Adds an Least Square term that is only evaluated at the end:
% 0.5* || ( m(T,x(T),u(T),p(T),...) - r ) ||^2_S
% where S is a weighting matrix, r a reference vector and T the time
% at the last objective grid point. 
%
%  Usage:
%    >> ocp.minimizeLSQEndTerm(m, r)
%    >> ocp.minimizeLSQEndTerm(S, m, r)
%
%  Parameters:
%    m 	  the LSQ-Function    (1 x n  FUNCTION)
%    S 	  a weighting matrix  (n x n  MATRIX)
%    r 	  the reference       (1 x n  VECTOR)
%
%  Example:
%    >> S = [10 0; 0 1];
%    >> m = {x1, x2};
%    >> r = [0, 0];
%    >> ocp.minimizeLSQEndTerm(S, m, r);
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

index = length(obj.minLSQEndTermh);

if length(varargin) == 1        %ocp.minimizeLSQEndTerm(QT)
    QT = varargin{1};
    
    obj.minLSQEndTermQ = obj.checkVectorMatrix(QT);
    
elseif (length(varargin) == 2)  %ocp.minimizeLSQEndTerm(h, r)
    if isnumeric(varargin{1}) || isa(varargin{1}, 'acado.BMatrix')
        Q = varargin{1};
        r = varargin{2};
        
        obj.minLSQEndTermQ = obj.checkVectorMatrix(Q);
        obj.minLSQEndTermR = acado.Function(r);
    else
        h = varargin{1};
        r = varargin{2};
        
        obj.minLSQEndTermh{index+1} = acado.Function(h);
        obj.minLSQEndTermr{index+1} = obj.checkVectorMatrix(r);
        obj.minLSQEndTermS{index+1} = {};
    end
    
elseif (length(varargin) == 3)  %ocp.minimizeLSQEndTerm(S, h, r)
    h = varargin{2};
    r = varargin{3};
    S = varargin{1};

    obj.minLSQEndTermh{index+1} = acado.Function(h);
    obj.minLSQEndTermr{index+1} = obj.checkVectorMatrix(r);
    obj.minLSQEndTermS{index+1} = acado.Matrix(S);

    
else %error
   error('ERROR: Invalid minimizeLSQEndTerm call. <a href="matlab: help acado.OCP.minimizeLSQEndTerm">help acado.OCP.minimizeLSQEndTerm</a>'); 
end


end