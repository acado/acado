function setProcessDisturbance(obj, varargin)
%Sets a disturbance to a process, read from a matrix
%
%  Usage:
%    >> ocp.setProcessDisturbance(d)
%
%  Parameters:
%    d 	  disturbance matrix    (n x m  MATRIX)
%         where n = number of time steps
%               m = (number of disturbances) - 1
%         the first column represents the time points
%         the second column represents the first disturbance
%         [the thirth column represents the second disturbance]
%
%  Example:
%    >> Disturbance R;
%    >> Disturbance W;
%    >> disturbance = [
%         0.0       0.00   0.00
%         0.5       0.00   1.00 
%         1.0       0.00   1.50
%         1.5       1.00   0.00
%        ];
%       % TIME       R      W
%    >>
%    >> process = acado.Process();
%    >> process.setProcessDisturbance(disturbance);
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


 %http://www.mathworks.com/access/helpdesk/help/techdoc/ref/dlmwrite.html 

if (length(varargin) == 1) 
    
    obj.disturbance = acado.Matrix(varargin{1});
    
else %error
   error('ERROR: Invalid setProcessDisturbance call. <a href="matlab: help acado.Process.setProcessDisturbance">help acado.Process.setProcessDisturbance</a>'); 
end


end