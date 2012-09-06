function linkCFunction(obj, fcnfile, fcnname)
%Link a black box c function to an ACADO problem.
%
%  Usage:
%    >> linkCFunction(fcnfile, fcnname)
%
%  Parameters:
%    fcnfile  link to a c function. This c function should have at least
%             this method called fcnname:
%               void [fcnname]( double *x, double *f, void *user_data ){
%                   // x -> input vector,   f -> output vector
%                   // sequence in x:  t,x,u,p,w  (analogous to ode calls)
%               }
%
%  Example:
%    >> f = acado.DifferentialEquation();
%    >> f.linkCFunction('cfunction.cpp', 'AcadoDifferentialEquation') or
%       f.linkCFunction('mysubfolder/cfunction.cpp', 'AcadoDifferentialEquation');
%  
%       File: cfunction.cpp:
%               void AcadoDifferentialEquation( double *x, double *f, void *user_data ){
%                   f[0] = x[1];
%               }
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

if (~isempty(obj.matlabODE_fcnHandle) || ~isempty(obj.matlabDAE_fcnHandle) || ~isempty(obj.cfunction_file))
   error('Only _one_ Matlab DAE or ODE can be linked.');
end

    obj.cfunction_file = fcnfile;
    obj.cfunction_function = fcnname;

end