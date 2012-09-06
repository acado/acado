function [ f ] = simple_dae_matlab( t,x,xa,u,p,w ) 
%Blackbox DAE example file. 
%
%  Every DAE f should have the header
%  [ f ] = FILE_NAME( t,x,xa,u,p,w,dx )
%  with  t: time
%       x: differential states
%       xa: algebraic states (if any)
%       u: controls (if any)
%       p: parameters (if any)
%       w: disturbances (if any)
%
%  Write first all differential states (dot(x) == ...)
%  afterwards all algebraic states (0 == ...)
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
%    \author David Ariens, Hans Joachim Ferreau, Boris Houska, Niels Haverbeke
%    \date 2008-2009-2010
% 

	f(1) =  -p(1)*x(1)*x(1)*xa(1);          %ODE 1 ->   dot(x) ==  -p*x*x*z
    f(2) =  p(2)*p(2) - xa(1)*xa(1);        %DAE 1 ->        0 == q*q - z*z

end
